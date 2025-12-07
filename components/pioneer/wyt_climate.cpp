#ifdef USE_REMOTE_TRANSMITTER
#include "wyt_remote.h"
#include "esphome/components/remote_base/pioneer_wyt_protocol.h"
#endif

#include "wyt_climate.h"
#include "esphome/core/log.h"

#include <cfloat>
#include <cmath>
#include <stdint.h>

namespace esphome {
namespace pioneer {
namespace wyt {

static const char *const TAG = "pioneer.climate";

void WytClimate::setup() {
  if (!this->query_state_()) {
    ESP_LOGE(TAG, "Status query timed out");
    return;
  }

  // Set the current mode
  this->mode = this->get_mode();
  this->action = this->get_action();

  auto f_mode = this->get_pioneer_fan_mode();
  auto c_f_mode = this->get_pioneer_custom_fan_mode();
  if (f_mode.has_value()) {
    this->set_fan_mode_(*f_mode);
  } else if (c_f_mode.has_value()) {
    this->set_custom_fan_mode_(c_f_mode->c_str());
  }

  this->swing_mode = this->get_swing_mode();
  this->target_temperature = this->get_setpoint();
  this->current_temperature = this->get_temperature();
}

bool WytClimate::query_state_(bool read_only) {
  if (this->is_busy()) {
    ESP_LOGD(TAG, "Waiting on busy state to clear before querying");
    return true;
  }
  this->busy_++;

  if (!read_only) {
    ESP_LOGV(TAG, "Sending query command");
    this->write_array(WYT_QUERY_COMMAND, WYT_QUERY_COMMAND_SIZE);
    this->flush();
  }

  static uint8_t buf_state_[WYT_QUERY_RESPONSE_SIZE];
  bool result = this->read_array(buf_state_, WYT_QUERY_RESPONSE_SIZE);
  this->busy_--;
  if (!result)
    return false;

  if (buf_state_[0] != 0xBB) {
    ESP_LOGW(TAG, "Invalid response header: 0x%x", buf_state_[0]);
    if (!this->is_busy()) {
      this->busy_++;
      // Clear the buffer
      uint8_t byte;
      while (this->available()) {
        this->read_byte(&byte);
      }
      this->busy_--;
    }
    return false;
  }

  bool state_changed = false;                          // FIXME: Cleanup
  for (int i = 0; i < WYT_QUERY_RESPONSE_SIZE; i++) {  // FIXME: Copy the whole array at once later instead of looping
    if (i != 18 && i != 30 && i != 34 && i != 35 && i != 36 && i != 37 && i != 38 && i != 39 && i != 45 && i != 46 &&
        i != 60 && this->raw_state_[i] != buf_state_[i]) {
      ESP_LOGI(TAG, "State changed at %d: 0x%x -> 0x%x", i, this->raw_state_[i], buf_state_[i]);
      state_changed = true;
    }
    this->raw_state_[i] = buf_state_[i];
  }
  if (state_changed) {
    ESP_LOGI(TAG, "Response: %s", format_hex_pretty(buf_state_, WYT_QUERY_RESPONSE_SIZE).c_str());  // FIXME: Cleanup
  }

  uint8_t checksum = this->raw_state_[WYT_QUERY_RESPONSE_SIZE - 1];
  if (checksum != this->response_checksum(this->raw_state_)) {
    ESP_LOGW(TAG, "Checksum mismatch: 0x%x != 0x%x", checksum, this->response_checksum(this->raw_state_));
    return false;
  }

  this->state_ = response_from_bytes(this->raw_state_);

  bool changed{false};
  this->update_property_(this->action, this->get_action(), changed);
  this->update_property_(this->current_temperature, this->get_temperature(), changed);
  if (changed)
    this->publish_state();

  return true;
}

void WytClimate::update() {
  if (this->is_busy()) {
    ESP_LOGD(TAG, "Waiting on busy state to clear, %d cycles left", this->busy_--);
    return;
  }

  if (!this->query_state_())
    ESP_LOGE(TAG, "Status query timed out");

  // Publish updates for the ancillary sensors
  this->update_sensors_();

  bool changed = false;
  this->update_property_(this->current_temperature, this->get_temperature(), changed);
  this->update_property_(this->target_temperature, this->get_setpoint(), changed);
  this->update_property_(this->swing_mode, this->get_swing_mode(), changed);
  this->update_property_(this->mode, this->get_mode(), changed);
  this->update_property_(this->fan_mode, this->get_pioneer_fan_mode(), changed);

  auto c_f_mode = this->get_pioneer_custom_fan_mode();
  if (c_f_mode.has_value()) {
    if (!this->has_custom_fan_mode() || (c_f_mode.value() != this->get_custom_fan_mode())) {
      this->set_custom_fan_mode_(c_f_mode->c_str());
      changed = true;
    }
  } else {
    if (this->has_custom_fan_mode()) {
      // If we had a custom mode but now don't, we probably switched to a standard fan mode
      // which is handled by update_property_(fan_mode) above IF get_pioneer_fan_mode() returns a value.
      // However, if both are empty (unlikely?), we might need to clear.
      // Since standard fan mode setter clears custom, we rely on that.
    }
  }

  if (changed) {
    this->publish_state();
  }
}

template<typename T> void WytClimate::update_property_(T &property, const T &value, bool &flag) {
  if (property != value) {
    property = value;
    flag = true;
  }
}

void WytClimate::update_sensors_() {
  // FIXME: Add deduplicators
  if (this->defrost_binary_sensor_ != nullptr) {
    this->defrosting_ = this->is_defrosting();
    this->defrost_binary_sensor_->publish_state(this->defrosting_);
  }
  if (this->indoor_fan_speed_sensor_ != nullptr && this->indoor_fan_speed_ != this->get_indoor_fan_speed()) {
    this->indoor_fan_speed_ = this->get_indoor_fan_speed();
    this->indoor_fan_speed_sensor_->publish_state(this->indoor_fan_speed_);
  }
  if (this->outdoor_fan_speed_sensor_ != nullptr && this->outdoor_fan_speed_ != this->get_outdoor_fan_speed()) {
    this->outdoor_fan_speed_ = this->get_outdoor_fan_speed();
    this->outdoor_fan_speed_sensor_->publish_state(this->outdoor_fan_speed_);
  }
  if (this->outdoor_sensor_ != nullptr && this->outdoor_temperature_ != this->get_outdoor_temperature()) {
    this->outdoor_temperature_ = this->get_outdoor_temperature();
    this->outdoor_sensor_->publish_state(this->outdoor_temperature_);
  }
  if (this->power_sensor_ != nullptr && this->power_usage_ != this->get_power_usage()) {
    this->power_usage_ = this->get_power_usage();
    this->power_sensor_->publish_state(this->get_power_usage());
  }
}

void WytClimate::refresh() {
  this->command = this->command_from_response(this->state_);

  this->switch_to_mode_(this->mode);
  this->switch_to_action_(this->action);
  if (this->has_custom_fan_mode())
    this->switch_to_custom_fan_mode_(this->get_custom_fan_mode());
  else if (this->fan_mode.has_value())
    this->switch_to_fan_mode_(this->fan_mode.value());
  this->switch_to_swing_mode_(this->swing_mode);
  this->validate_target_temperature();
  this->switch_to_setpoint_temperature_();

  this->send_command(this->command);
}

void WytClimate::validate_target_temperature() {
  if (std::isnan(this->target_temperature)) {
    this->target_temperature =
        ((this->get_traits().get_visual_max_temperature() - this->get_traits().get_visual_min_temperature()) / 2) +
        this->get_traits().get_visual_min_temperature();
  } else {
    // target_temperature must be between the visual minimum and the visual maximum
    if (this->target_temperature < this->get_traits().get_visual_min_temperature())
      this->target_temperature = this->get_traits().get_visual_min_temperature();
    if (this->target_temperature > this->get_traits().get_visual_max_temperature())
      this->target_temperature = this->get_traits().get_visual_max_temperature();
  }
}

void WytClimate::control(const climate::ClimateCall &call) {
  /* FIXME: Implement this
  if (call.get_preset().has_value()) {
    // setup_complete_ blocks modifying/resetting the temps immediately after boot
    if (this->setup_complete_) {
      this->change_preset_(*call.get_preset());
    } else {
      this->preset = *call.get_preset();
    }
  }
  */

  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_fan_mode().has_value()) {
    this->set_fan_mode_(*call.get_fan_mode());
  }
  if (call.has_custom_fan_mode()) {
    this->set_custom_fan_mode_(call.get_custom_fan_mode());
  }
  if (call.get_swing_mode().has_value())
    this->swing_mode = *call.get_swing_mode();
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    validate_target_temperature();
  }

  // make any changes happen
  this->refresh();
}

climate::ClimateTraits WytClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::ClimateSupports::CLIMATE_SUPPORTS_ACTION |
                           climate::ClimateSupports::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);

  traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_DRY);
  traits.add_supported_mode(climate::CLIMATE_MODE_FAN_ONLY);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);

  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_QUIET);
  traits.set_supported_custom_fan_modes({"Medium-Low", "Medium-High", "Turbo"});

  traits.add_supported_swing_mode(climate::CLIMATE_SWING_BOTH);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_HORIZONTAL);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);

  traits.set_visual_target_temperature_step(1.0);
  traits.set_visual_current_temperature_step(1.0);

  return traits;
}

void WytClimate::switch_to_action_(climate::ClimateAction action) {
  if (action == this->get_action()) {
    ESP_LOGI(TAG, "Already showing target action %s", climate::climate_action_to_string(action));
    return;
  }
  this->action = this->get_action();
}

void WytClimate::switch_to_fan_mode_(climate::ClimateFanMode fan_mode) {
  if (fan_mode == this->get_pioneer_fan_mode()) {
    ESP_LOGI(TAG, "Already in target fan mode %s", climate::climate_fan_mode_to_string(fan_mode));
    return;
  }

  this->command.mute = false;
  this->command.turbo = false;
  switch (fan_mode) {
    case climate::CLIMATE_FAN_LOW:
      this->command.fan_speed = CmdFanSpeed::Low;
      break;
    case climate::CLIMATE_FAN_MEDIUM:
      this->command.fan_speed = CmdFanSpeed::Medium;
      break;
    case climate::CLIMATE_FAN_HIGH:
      this->command.fan_speed = CmdFanSpeed::High;
      break;
    case climate::CLIMATE_FAN_QUIET:
      this->command.fan_speed = CmdFanSpeed::Low;
      this->command.mute = true;
      break;
    case climate::CLIMATE_FAN_AUTO:
    default:
      // we cannot report an invalid mode back to HA (even if it asked for one)
      //  and must assume some valid value
      this->command.fan_speed = CmdFanSpeed::Auto;
      fan_mode = climate::CLIMATE_FAN_AUTO;
  }

  // Clear any custom fan modes, this is highlander rules
  this->set_fan_mode_(fan_mode);
}

void WytClimate::switch_to_custom_fan_mode_(std::string custom_fan_mode) {
  if (custom_fan_mode == this->get_pioneer_custom_fan_mode()) {
    ESP_LOGI(TAG, "Already in target custom fan mode %s", custom_fan_mode);
    return;
  }

  this->command.mute = false;
  this->command.turbo = false;
  if (custom_fan_mode == "Medium-Low") {
    this->command.fan_speed = CmdFanSpeed::MediumLow;
  } else if (custom_fan_mode == "Medium-High") {
    command.fan_speed = CmdFanSpeed::MediumHigh;
  } else {
    // we cannot report an invalid mode back to HA (even if it asked for one)
    //  and must assume some valid value
    command.fan_speed = CmdFanSpeed::High;
    this->command.turbo = true;
    custom_fan_mode = "Turbo";
  }

  // Clear any fan modes, this is highlander rules
  this->set_custom_fan_mode_(custom_fan_mode.c_str());
}

void WytClimate::switch_to_mode_(climate::ClimateMode mode) {
  if (mode == this->get_mode()) {
    ESP_LOGI(TAG, "Already in target mode %s", climate::climate_mode_to_string(mode));
    return;
  }

  this->command.power = true;
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      this->command.mode = CmdMode::Off;
      this->command.power = false;
      break;
    case climate::CLIMATE_MODE_COOL:
      this->command.mode = CmdMode::Cool;
      break;
    case climate::CLIMATE_MODE_HEAT:
      this->command.mode = CmdMode::Heat;
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      this->command.mode = CmdMode::Fan;
      break;
    case climate::CLIMATE_MODE_DRY:
      this->command.mode = CmdMode::Dry;
      break;
    case climate::CLIMATE_MODE_AUTO:
    default:
      // we cannot report an invalid mode back to HA (even if it asked for one)
      //  and must assume some valid value
      this->command.mode = CmdMode::Auto;
      mode = climate::CLIMATE_MODE_AUTO;
  }

  this->mode = mode;
}

void WytClimate::switch_to_swing_mode_(climate::ClimateSwingMode swing_mode) {
  if (swing_mode == this->get_swing_mode()) {
    ESP_LOGI(TAG, "Already in target swing mode %s", climate::climate_swing_mode_to_string(swing_mode));
    return;
  }

  switch (swing_mode) {
    case climate::CLIMATE_SWING_BOTH:
      this->command.horizontal_flow = true;
      this->command.left_right_flow = LeftRightFlow::LeftRightFlow;
      this->command.up_down_flow = UpDownFlow::UpDownFlow;
      this->command.vertical_flow = VerticalFlow::On;
      break;
    case climate::CLIMATE_SWING_HORIZONTAL:
      this->command.horizontal_flow = true;
      this->command.left_right_flow = LeftRightFlow::LeftRightFlow;
      this->command.up_down_flow = UpDownFlow::Auto;
      this->command.vertical_flow = VerticalFlow::Off;
      break;
    case climate::CLIMATE_SWING_VERTICAL:
      this->command.horizontal_flow = false;
      this->command.left_right_flow = LeftRightFlow::Auto;
      this->command.up_down_flow = UpDownFlow::UpDownFlow;
      this->command.vertical_flow = VerticalFlow::On;
      break;
    case climate::CLIMATE_SWING_OFF:
    default:
      // we cannot report an invalid mode back to HA (even if it asked for one)
      //  and must assume some valid value
      this->command.horizontal_flow = false;
      this->command.left_right_flow = LeftRightFlow::Auto;
      this->command.up_down_flow = UpDownFlow::Auto;
      this->command.vertical_flow = VerticalFlow::Off;
      swing_mode = climate::CLIMATE_SWING_OFF;
  }
  this->swing_mode = swing_mode;
}

void WytClimate::switch_to_setpoint_temperature_() {
  if (this->target_temperature == this->get_setpoint()) {
    ESP_LOGD(TAG, "Already set to target temperature %.1f", this->target_temperature);
    return;
  }
  this->set_temperature_(command, this->target_temperature);
}

void WytClimate::dump_config() { LOG_CLIMATE("", "WytClimate", this); }

uint8_t WytClimate::checksum(const SetCommand &command) {
  uint8_t result = 0;
  // Skip the magic byte
  for (size_t i = 0; i < WYT_STATE_COMMAND_SIZE - 1; ++i) {
    result ^= command.bytes[i];
  }
  return result;
}

Header WytClimate::new_header(const Source &source, const Dest &dest, const Command &command, const uint8_t size) {
  Header header;
  header.magic = 0xbb;
  header.source = source;
  header.dest = dest;
  header.command = command;
  header.length = size;
  return header;
}

SetCommand WytClimate::command_from_bytes(const uint8_t buffer[WYT_STATE_COMMAND_SIZE]) {
  SetCommand command = {};
  memcpy(command.bytes, buffer, WYT_STATE_COMMAND_SIZE);
  return command;
}

SetCommand WytClimate::command_from_response(const StateResponse &response) {
  SetCommand command = {};

  Header header = this->new_header(Source::Controller, Dest::Appliance, Command::Set, 0x1d);
  command.header = header;
  command.eco = response.eco;
  command.display = this->enable_display_;
  command.beeper = this->enable_beeper_;
  command.gen_mode = 0x0;  // FIXME: Always disable gen_mode for now
  command.power = response.power;
  command.mute = response.mute;
  command.turbo = response.turbo;
  command.health = response.health;
  switch (response.mode) {
    case Mode::Auto:
      command.mode = CmdMode::Auto;
      break;
    case Mode::Cool:
      command.mode = CmdMode::Cool;
      break;
    case Mode::Dry:
      command.mode = CmdMode::Dry;
      break;
    case Mode::Fan:
      command.mode = CmdMode::Fan;
      break;
    case Mode::Heat:
      command.mode = CmdMode::Heat;
      break;
  }
  command.freeze_protection = response.freeze_protection;
  switch (response.fan_speed) {
    case FanSpeed::Auto:
      command.fan_speed = CmdFanSpeed::Auto;
      break;
    case FanSpeed::Low:
      command.fan_speed = CmdFanSpeed::Low;
      break;
    case FanSpeed::MediumLow:
      command.fan_speed = CmdFanSpeed::MediumLow;
      break;
    case FanSpeed::Medium:
      command.fan_speed = CmdFanSpeed::Medium;
      break;
    case FanSpeed::MediumHigh:
      command.fan_speed = CmdFanSpeed::MediumHigh;
      break;
    case FanSpeed::High:
      command.fan_speed = CmdFanSpeed::High;
      break;
  }
  this->set_temperature_(command, this->get_setpoint());
  command.unknown9[0] = 0x80;
  // command.unknown11[0-9] = 0x15;  // FIXME: This value is returned in byte 20+offset, no changes
  command.sleep = response.sleep;
  // command.timer_enabled = response.timer_enabled; // FIXME
  // command.timer_enabled = false;
  // command.unknown9[1] = 0x05;  // FIXME: Testing if this is timer hours
  // command.unknown9[2] = 0x30;  // FIXME: Testing if this is timer minutes
  command.horizontal_flow = response.horizontal_flow;
  command.left_right_flow = static_cast<LeftRightFlow>(static_cast<uint8_t>(response.left_right_flow) | 0x80);
  command.up_down_flow = response.up_down_flow;
  command.vertical_flow = response.vertical_flow ? VerticalFlow::On : VerticalFlow::Off;
  command.checksum = checksum(command);
  return command;
}

void WytClimate::send_command(SetCommand &command) {
  command.left_right_flow = static_cast<LeftRightFlow>(static_cast<uint8_t>(command.left_right_flow) | 0x80);
  command.checksum = this->checksum(command);
  ESP_LOGI(TAG, "Send: %s", format_hex_pretty(command.bytes, WYT_STATE_COMMAND_SIZE).c_str());
  this->write_array(command.bytes, WYT_STATE_COMMAND_SIZE);
  this->flush();
  /*
  Wait N cycles to publish state after sending a command to reduce flip-flopping in the UI
  when changing multiple options in quick succession.
  */
  this->busy_ = this->command_delay_;
}

StateResponse WytClimate::response_from_bytes(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]) {
  StateResponse response = {};
  memcpy(response.bytes, buffer, WYT_QUERY_RESPONSE_SIZE);
  return response;
}

uint8_t WytClimate::response_checksum(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]) {
  uint8_t result = 0;
  // Skip the magic byte
  for (size_t i = 0; i < WYT_QUERY_RESPONSE_SIZE - 1; ++i) {
    result ^= buffer[i];
  }
  return result;
}

climate::ClimateAction WytClimate::get_action() {
  if (!this->state_.power)
    return climate::CLIMATE_ACTION_OFF;
  if (this->state_.outdoor_fan_speed == 0 || this->state_.outdoor_unit_status != OutdoorStatus::Running)
    return climate::CLIMATE_ACTION_IDLE;
  switch (this->get_mode()) {
    case climate::CLIMATE_MODE_HEAT:
      return climate::CLIMATE_ACTION_HEATING;
    case climate::CLIMATE_MODE_COOL:
      return climate::CLIMATE_ACTION_COOLING;
    case climate::CLIMATE_MODE_DRY:
      return climate::CLIMATE_ACTION_DRYING;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return climate::CLIMATE_ACTION_FAN;
    default:
      ESP_LOGE(TAG, "Unknown mode: %s", climate::climate_mode_to_string(this->get_mode()));
      return climate::CLIMATE_ACTION_OFF;
  }
}

climate::ClimateMode WytClimate::get_mode() {
  if (!this->state_.power)
    return climate::CLIMATE_MODE_OFF;
  switch (this->state_.mode) {
    case Mode::Auto:
      return climate::CLIMATE_MODE_AUTO;
    case Mode::Cool:
      return climate::CLIMATE_MODE_COOL;
    case Mode::Dry:
      return climate::CLIMATE_MODE_DRY;
    case Mode::Fan:
      return climate::CLIMATE_MODE_FAN_ONLY;
    case Mode::Heat:
      return climate::CLIMATE_MODE_HEAT;
    default:
      ESP_LOGE(TAG, "Unknown mode: %d", this->state_.mode);
      return climate::CLIMATE_MODE_OFF;
  }
}

optional<std::string> WytClimate::get_pioneer_custom_fan_mode() {
  switch (this->state_.fan_speed) {
    case FanSpeed::MediumLow:
      return optional<std::string>("Medium-Low");
    case FanSpeed::MediumHigh:
      return optional<std::string>("Medium-High");
    case FanSpeed::High:
      if (this->state_.turbo)
        return optional<std::string>("Turbo");
    default:
      return optional<std::string>();
  }
}

optional<climate::ClimateFanMode> WytClimate::get_pioneer_fan_mode() {
  switch (this->state_.fan_speed) {
    case FanSpeed::Low:
      if (this->state_.mute)
        return climate::CLIMATE_FAN_QUIET;
      return climate::CLIMATE_FAN_LOW;
    case FanSpeed::Medium:
      return climate::CLIMATE_FAN_MEDIUM;
    case FanSpeed::High:
      return climate::CLIMATE_FAN_HIGH;
    case FanSpeed::Auto:
      return climate::CLIMATE_FAN_AUTO;
    default:
      return optional<climate::ClimateFanMode>();
  }
}

climate::ClimateSwingMode WytClimate::get_swing_mode() {
  bool left_right = (this->state_.left_right_flow == LeftRightFlow::LeftRightFlow);
  bool up_down = (this->state_.up_down_flow == UpDownFlow::UpDownFlow);

  if (left_right && up_down)
    return climate::CLIMATE_SWING_BOTH;
  else if (left_right)
    return climate::CLIMATE_SWING_HORIZONTAL;
  else if (up_down)
    return climate::CLIMATE_SWING_VERTICAL;
  return climate::CLIMATE_SWING_OFF;
}

float WytClimate::get_setpoint() {
  return 16 + this->state_.setpoint_whole + (this->state_.setpoint_half_digit ? 0.5 : 0);
}

/*
Calibration points:
Reading, App Temp, Converted, Rounding Range, Calculated Result
0x69: 105, 72F, 22.22C, 21.94 < x < 22.49, 22.0
0x6b: 107, 73F, 22.78C, 22.50 < x < 23.05, 22.8
0x6e: 110, 75F, 23.89C, 23.61 < x < 24.16, 24.0
0x70: 112, 77F, 25.00C, 24.72 < x < 25.27, 24.8
0x73: 115, 79F, 26.11C, 25.83 < x < 26.38, 26.0
*/
float WytClimate::get_temperature() { return this->state_.indoor_temp_base * 0.4 - 20; }

void WytClimate::set_temperature_(SetCommand &command, const float temp_c) {
  // Double before rounding because 77F/24.94°C * 2 = 49.88, which is cast to 49, adding extra setpoint bumps
  float temp_double_float = temp_c * 2;
  uint8_t temp_double = round(temp_double_float);
  uint8_t temp_whole = temp_double / 2;
  bool temp_half = temp_double % 2;
  command.setpoint_whole = 0x6f - temp_whole;
  command.setpoint_half_digit = temp_half;
}

#ifdef USE_REMOTE_TRANSMITTER
IrFanCommand WytClimate::get_fan_command_from_state() {
  IrFanCommand command = {IrHeader{
      WYT_REMOTE_HEADER_MAGIC[0],
      WYT_REMOTE_HEADER_MAGIC[1],
      WYT_REMOTE_HEADER_MAGIC[2],
      uint8_t(IrCommandType::Fan),
  }};

  switch (this->state_.fan_speed) {
    case FanSpeed::Low:
      command.fan_speed = IrFanSpeed::Low;
      break;
    case FanSpeed::MediumLow:
      command.fan_speed = IrFanSpeed::MediumLow;
      break;
    case FanSpeed::Medium:
      command.fan_speed = IrFanSpeed::Medium;
      break;
    case FanSpeed::MediumHigh:
      command.fan_speed = IrFanSpeed::MediumHigh;
      break;
    case FanSpeed::High:
      command.fan_speed = IrFanSpeed::High;
      break;
    case FanSpeed::Auto:
    default:
      command.fan_speed = IrFanSpeed::Auto;
  }
  command.unknown2 = 1;
  command.mute = this->state_.mute;  // AKA silent mode
  command.horizontal_swing = this->state_.horizontal_flow ? IrSwing::On : IrSwing::Off;
  command.vertical_swing = this->state_.vertical_flow ? IrSwing::On : IrSwing::Off;
  return command;
}

IrGeneralCommand WytClimate::get_general_command_from_state() {
  IrGeneralCommand command = {IrHeader{
      WYT_REMOTE_HEADER_MAGIC[0],
      WYT_REMOTE_HEADER_MAGIC[1],
      WYT_REMOTE_HEADER_MAGIC[2],
      uint8_t(IrCommandType::General),
  }};

  command.power = this->state_.power;
  command.timer = this->state_.timer_enabled;
  command.beeper = this->enable_beeper_;
  command.disable_display = !this->enable_display_;
  command.eco = this->state_.eco;

  switch (this->mode) {
    case climate::CLIMATE_MODE_COOL:
      command.mode = IrMode::Cool;
      break;
    case climate::CLIMATE_MODE_DRY:
      command.mode = IrMode::Dry;
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      command.mode = IrMode::Fan;
      break;
    case climate::CLIMATE_MODE_HEAT:
      command.mode = IrMode::Heat;
      break;
    case climate::CLIMATE_MODE_AUTO:
    default:
      command.mode = IrMode::Auto;
  }

  command.health = this->state_.health;
  command.turbo = this->state_.turbo;
  if (this->state_.turbo)
    command.aux_mode = IrAuxMode::Turbo;
  else if (this->state_.sleep != SleepMode::Off)
    command.aux_mode = IrAuxMode::Sleep;
  else if (this->mode == climate::CLIMATE_MODE_DRY)
    command.aux_mode = IrAuxMode::Dry;

  if (this->state_.up_down_flow == UpDownFlow::UpDownFlow)
    command.up_down_flow = IrUpDownFlow::UpDownFlow;
  if (this->state_.left_right_flow == LeftRightFlow::LeftRightFlow)
    command.left_right_flow = IrLeftRightFlow::LeftRightFlow;

  command.setpoint_whole_number = 31 - static_cast<uint8_t>(this->target_temperature);
  command.setpoint_half_digit = this->state_.setpoint_half_digit;
  return command;
}
#endif

void WytClimate::do_remote_temp(float temp_c, bool beeper) {
#ifdef USE_REMOTE_TRANSMITTER
  IrFanCommand fan_command = this->get_fan_command_from_state();
  IrGeneralCommand general_command = this->get_general_command_from_state();

  general_command.remote_temp = static_cast<uint8_t>(round(temp_c));
  general_command.beeper = this->enable_beeper_ || beeper;

  remote_base::PioneerWytData fan_data(
      std::vector<uint8_t>(fan_command.bytes, fan_command.bytes + WYT_REMOTE_COMMAND_SIZE));
  remote_base::PioneerWytData general_data(
      std::vector<uint8_t>(general_command.bytes, general_command.bytes + WYT_REMOTE_COMMAND_SIZE));

  // Transmit the fan command first (always required)
  auto transmit = this->transmitter_->transmit();
  remote_base::PioneerWytProtocol().encode(transmit.get_data(), fan_data);
  transmit.perform();

  // Pause for 70ms between first and second command
  delay_microseconds_safe(70000);

  // Transmit the general command
  transmit = this->transmitter_->transmit();
  remote_base::PioneerWytProtocol().encode(transmit.get_data(), general_data);
  transmit.perform();

  ESP_LOGD(TAG, "Remote temp action called with: %0.1f °C, rounded to: %u °C", temp_c, general_command.remote_temp);
#else
  ESP_LOGW(TAG, "Action needs remote_transmitter component");
#endif
}

}  // namespace wyt
}  // namespace pioneer
}  // namespace esphome