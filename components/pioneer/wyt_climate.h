#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_REMOTE_TRANSMITTER
#include "wyt_remote.h"
#include "esphome/components/remote_base/remote_base.h"
#endif

namespace esphome {
namespace pioneer {
namespace wyt {

static const uint8_t WYT_QUERY_COMMAND[8] = {0xBB, 0x00, 0x01, 0x04, 0x02, 0x01, 0x00, 0xBD};

static const uint8_t WYT_HEADER_SIZE = 5;
static const uint8_t WYT_QUERY_COMMAND_SIZE = 8;
static const uint8_t WYT_QUERY_RESPONSE_SIZE = 61;
static const uint8_t WYT_STATE_COMMAND_SIZE = 35;

enum class Source : uint8_t {
  Controller = 0x00,
  Appliance = 0x01,
};

enum class Dest : uint8_t {
  Controller = 0x00,
  Appliance = 0x01,
};

enum class Command : uint8_t {
  Set = 0x03,
  Get = 0x04,
};

enum class Mode : uint8_t {
  Cool = 0x01,
  Fan = 0x02,
  Dry = 0x03,
  Heat = 0x04,
  Auto = 0x05,
};

enum class CmdMode : uint8_t {
  Off = 0x00,
  Heat = 0x01,
  Dry = 0x02,
  Cool = 0x03,
  Fan = 0x07,
  Auto = 0x08,
};

enum class FanSpeed : uint8_t {
  Auto = 0x00,
  Low = 0x01,
  MediumLow = 0x04,
  Medium = 0x02,
  MediumHigh = 0x05,
  High = 0x03,
};

enum class CmdFanSpeed : uint8_t {
  Auto = 0x00,
  Low = 0x02,
  MediumLow = 0x06,
  Medium = 0x03,
  MediumHigh = 0x07,
  High = 0x05,
};

enum class SleepMode : uint8_t {
  Off = 0x00,
  Standard = 0x01,
  Elderly = 0x02,
  Child = 0x03,
};

enum class VerticalFlow : uint8_t {
  Off = 0x00,
  On = 0x07,
};

enum class UpDownFlow : uint8_t {
  Auto = 0x00,
  TopFix = 0x01,
  UpperFix = 0x02,
  MiddleFix = 0x03,
  LowerFix = 0x04,
  BottomFix = 0x05,
  UpDownFlow = 0x08,
  UpFlow = 0x10,
  DownFlow = 0x18,
};

enum class LeftRightFlow : uint8_t {
  Auto = 0x00,
  LeftFix = 0x01,
  MiddleLeftFix = 0x02,
  MiddleFix = 0x03,
  MiddleRightFix = 0x04,
  RightFix = 0x05,
  LeftRightFlow = 0x08,
  LeftFlow = 0x10,
  MiddleFlow = 0x18,
  RightFlow = 0x20,
};

enum class OutdoorStatus : uint8_t {
  Idle = 0x00,
  Running = 0x0a,
};

/** All commands sent to the WYT MCU begin with this header */
union Header {
  struct {
    // 00
    uint8_t magic;
    // 01
    Source source;
    // 02
    Dest dest;
    // 03
    Command command;
    // 04
    uint8_t length;
  } __attribute__((packed));
  uint8_t bytes[WYT_HEADER_SIZE];
};

/** A request to update the MCU state to the one in this object. */
typedef union {
  struct {
    // 00..04
    Header header;
    // 05..06
    uint8_t unknown1[2];
    // 07
    uint8_t unknown2 : 2;
    bool power : 1;
    bool unknown3 : 2;
    bool beeper : 1;
    bool display : 1;
    bool eco : 1;
    // 08
    CmdMode mode : 4;
    bool health : 1;  // FIXME: Verify
    bool unknown4 : 1;
    bool turbo : 1;
    bool mute : 1;
    // 09
    uint8_t setpoint_whole;
    // 10
    CmdFanSpeed fan_speed : 3;
    VerticalFlow vertical_flow : 3;
    bool unknown5 : 1;
    bool freeze_protection : 1;
    // 11
    uint8_t unknown6 : 2;
    bool setpoint_half_digit : 1;
    bool horizontal_flow : 1;
    uint8_t unknown7 : 4;
    // uint8_t unknown8 : 2;
    // 12..17
    uint8_t unknown9[6];
    // 18 Reduced output for use with a generator (0: 100%, 1: 30%, 2: 50%, 3: 80%)
    uint8_t gen_mode : 2;
    uint8_t unknown10 : 6;
    // 19
    SleepMode sleep;
    // 20..31
    uint8_t unknown11[12];
    // 32
    UpDownFlow up_down_flow;
    // 33
    LeftRightFlow left_right_flow;
    // 34
    uint8_t checksum;
  } __attribute__((packed));
  uint8_t bytes[WYT_STATE_COMMAND_SIZE];
} SetCommand;

/** Holds a response from the WYT MCU describing its current operating state. */
typedef union {
  struct {
    // 00
    uint8_t magic;
    // 01
    Source source;
    // 02
    Dest dest;
    // 03
    Command command;
    // 04
    uint8_t command_length;
    // 05..06
    uint8_t unknown1[2];
    // 07
    Mode mode : 3;
    bool unknown2 : 1;
    bool power : 1;
    bool display : 1;
    bool eco : 1;
    bool turbo : 1;
    // 08
    uint8_t setpoint_whole : 4;
    FanSpeed fan_speed : 3;
    bool unknown3 : 1;
    // 09
    bool setpoint_half_digit : 1;
    bool unknown4 : 1;
    bool health : 1;
    uint8_t unknown5 : 3;
    bool timer_enabled : 1;
    uint8_t unknown6 : 1;
    // 10
    uint8_t unknown7 : 5;
    bool horizontal_flow : 1;
    bool vertical_flow : 1;
    bool unknown8 : 1;
    // 11
    uint8_t timer_hours_remaining;
    // 12
    uint8_t timer_minutes_remaining;
    // 13..16
    uint8_t unknown9[4];
    // 17
    uint8_t indoor_temp_base;
    // 18
    uint8_t unknown10;
    // 19
    SleepMode sleep : 2;
    uint8_t unknown11 : 5;
    bool four_way_valve_on : 1;
    // 20..29
    uint8_t unknown12[10];
    // 30
    uint8_t indoor_heat_exchanger_temp;
    // 31
    uint8_t unknown13;
    // 32
    uint8_t unknown14 : 7;
    bool freeze_protection : 1;
    // 33
    uint8_t unknown15 : 7;
    bool mute : 1;
    // 34
    uint8_t indoor_fan_speed;
    // 35
    uint8_t outdoor_temp;
    // 36
    uint8_t condenser_coil_temp;
    // 37
    uint8_t compressor_discharge_temp;
    // 38
    uint8_t compressor_frequency;
    // 39
    uint8_t outdoor_fan_speed;
    // 40
    OutdoorStatus outdoor_unit_status : 4;
    uint8_t unknown16 : 2;
    bool heat_mode : 1;
    bool unknown17 : 1;
    // 41..44
    uint8_t unknown18[4];
    // 45
    uint8_t supply_voltage;
    // 46
    uint8_t current_used_amps;
    // 47
    uint8_t gen_mode;
    // 48..50
    uint8_t unknown19[3];
    // 51
    UpDownFlow up_down_flow;
    // 52
    LeftRightFlow left_right_flow;
    // 53..60
    uint8_t unknown20[8];
  } __attribute__((packed));
  uint8_t bytes[WYT_QUERY_RESPONSE_SIZE];
} StateResponse;

class WytClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

  void set_defrost_binary_sensor(binary_sensor::BinarySensor *sensor) { this->defrost_binary_sensor_ = sensor; }
  void set_indoor_fan_speed_sensor(sensor::Sensor *sensor) { this->indoor_fan_speed_sensor_ = sensor; }
  void set_outdoor_fan_speed_sensor(sensor::Sensor *sensor) { this->outdoor_fan_speed_sensor_ = sensor; }
  void set_outdoor_temperature_sensor(sensor::Sensor *sensor) { this->outdoor_sensor_ = sensor; }
  void set_power_sensor(sensor::Sensor *sensor) { this->power_sensor_ = sensor; }
#ifdef USE_REMOTE_TRANSMITTER
  void set_transmitter(remote_base::RemoteTransmitterBase *transmitter) { this->transmitter_ = transmitter; }
#endif

  // Send commands based on updated climate settings
  void refresh();
  void validate_target_temperature();

  bool is_busy() const { return this->busy_ > 0; }
  bool is_defrosting() const { return (this->state_.mode == Mode::Heat && !this->state_.heat_mode); }
  climate::ClimateAction get_action();
  optional<std::string> get_device_custom_fan_mode();
  optional<climate::ClimateFanMode> get_device_fan_mode();
  climate::ClimateMode get_mode();
  climate::ClimateSwingMode get_swing_mode();
  float get_setpoint();
  float get_temperature();

  int get_indoor_fan_speed() const { return static_cast<int>(this->state_.indoor_fan_speed); }
  int get_outdoor_fan_speed() const { return static_cast<int>(this->state_.outdoor_fan_speed); }
  int get_outdoor_temperature() const { return static_cast<int>(this->state_.outdoor_temp) - 20; }
  int get_power_usage() const { return this->state_.supply_voltage * this->state_.current_used_amps; }

  /* ############### */
  /* ### ACTIONS ### */
  /* ############### */

  void do_remote_temp(float temp_c, bool beeper = false);
  void do_display_toggle() {
    this->set_display(!this->enable_display_);
    this->refresh();
  }
  void do_beeper_on() {
    this->set_beeper(true);
    this->refresh();
  }
  void do_beeper_off() {
    this->set_beeper(false);
    this->refresh();
  }

#ifdef USE_REMOTE_TRANSMITTER
  // Get an IR command to override the internal temperature sensor with a remote sensor reading that better represents
  // the room temperature. Referred to in the manual as "I Feel" or "Follow Me" mode.
  // This command is generated based on the current state to avoid inadvertent settings changes.
  IrFanCommand get_fan_command_from_state();
  IrGeneralCommand get_general_command_from_state();
#endif

  void set_beeper(bool enable_beeper) { this->enable_beeper_ = enable_beeper; }
  void set_display(bool enable_display) { this->enable_display_ = enable_display; }

 protected:
  // The current state of the climate device
  uint8_t raw_state_[WYT_QUERY_RESPONSE_SIZE];
  StateResponse state_;
  bool enable_beeper_{false};
  bool enable_display_{true};

  uint8_t busy_{0};
  uint8_t command_delay_{2};

  // The current state of the extra sensors
  bool defrosting_{false};
  uint8_t indoor_fan_speed_{0};
  uint8_t outdoor_fan_speed_{0};
  int outdoor_temperature_{0};
  int power_usage_{0};

  // The new command to send to the WYT MCU
  SetCommand command;

#ifdef USE_REMOTE_TRANSMITTER
  RemoteTransmitterBase *transmitter_{nullptr};
#endif

  // Update the property if it has changed from previous and set the flag to true
  template<typename T> void update_property_(T &property, const T &value, bool &flag);

  // Update the ancillary sensors
  void update_sensors_();

  // Override control to change settings of the climate device.
  void control(const climate::ClimateCall &call) override;

  // Return the traits of this climate device.
  climate::ClimateTraits traits() override;

  // Switch the climate device to the given climate action.
  void switch_to_action_(climate::ClimateAction action);

  // Switch the climate device to the given climate fan mode.
  void switch_to_fan_mode_(climate::ClimateFanMode fan_mode);
  void switch_to_custom_fan_mode_(std::string custom_fan_mode);

  // Switch the climate device to the given climate mode.
  void switch_to_mode_(climate::ClimateMode mode);

  // Switch the climate device to the given climate swing mode.
  void switch_to_swing_mode_(climate::ClimateSwingMode swing_mode);

  // Check if the temperature change trigger should be called.
  void switch_to_setpoint_temperature_();
  void set_temperature_(SetCommand &command, const float temp_c);

  // Get the current state of the climate device
  bool query_state_(bool read_only = false);

  StateResponse response_from_bytes(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]);
  uint8_t response_checksum(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]);

  // Checksum message data with XOR
  uint8_t checksum(const SetCommand &command);
  Header new_header(const Source &source, const Dest &dest, const Command &command, const uint8_t size);
  SetCommand command_from_bytes(const uint8_t buffer[WYT_STATE_COMMAND_SIZE]);
  SetCommand command_from_response(const StateResponse &response);

  // Send a command to the WYT MCU
  void send_command(SetCommand &command);

  binary_sensor::BinarySensor *defrost_binary_sensor_{nullptr};
  sensor::Sensor *indoor_fan_speed_sensor_{nullptr};
  sensor::Sensor *outdoor_fan_speed_sensor_{nullptr};
  sensor::Sensor *outdoor_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};

  /* FIXME: Implement or cleanup
  // The set of standard preset configurations this thermostat supports (Eg. AWAY, ECO, etc)
  std::map<climate::ClimatePreset, WytClimateTargetTempConfig> preset_config_{};
  // The set of custom preset configurations this thermostat supports (eg. "My Custom Preset")
  std::map<std::string, WytClimateTargetTempConfig> custom_preset_config_{};

  // Default standard preset to use on start up
  climate::ClimatePreset default_preset_{};
  // Default custom preset to use on start up
  std::string default_custom_preset_{};

  // If set to DEFAULT_PRESET then the default preset is always used. When MEMORY prior
  // state will attempt to be restored if possible
  thermostat::OnBootRestoreFrom on_boot_restore_from_{thermostat::OnBootRestoreFrom::MEMORY};
  */
};

}  // namespace wyt
}  // namespace pioneer
}  // namespace esphome
