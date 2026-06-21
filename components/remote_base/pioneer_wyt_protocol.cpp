#include "pioneer_wyt_protocol.h"

#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.pioneer_wyt";

/* These timings are from the tcl112 component, but they're awfully close to what I measured.
 * The protocols are pretty similar as well, probably related. Similar protocol to Mitsubishi
 * also, though with different timing. */
constexpr uint32_t HEADER_MARK_US = 3100;
constexpr uint32_t HEADER_SPACE_US = 1650;
constexpr uint32_t BIT_MARK_US = 500;
constexpr uint32_t BIT_ONE_SPACE_US = 1100;
constexpr uint32_t BIT_ZERO_SPACE_US = 350;

constexpr unsigned int PIONEER_WYT_IR_PACKET_BIT_SIZE = 112;

uint8_t PioneerWytData::calc_cs_(uint8_t checksum_offset) const {
  if (this->type() == PIONEER_WYT_TYPE_FAN) {
    checksum_offset = 0x0F;
  }
  for (uint8_t i = 0; i < WYT_REMOTE_COMMAND_SIZE - 1; i++) {
    checksum_offset += this->data_[i];
  }
  return checksum_offset;
}

void PioneerWytProtocol::encode(RemoteTransmitData *dst, const PioneerWytData &data) {
  dst->set_carrier_frequency(38000);
  dst->reserve(2 + (data.size() * 8 * 2));
  dst->mark(HEADER_MARK_US);
  dst->space(HEADER_SPACE_US);
  dst->mark(BIT_MARK_US);

  uint8_t checksum = 0;
  if (data.type() == PioneerWytData::PIONEER_WYT_TYPE_FAN) {
    checksum = 0x0F;
  }
  for (size_t i = 0; i < data.size() - 1; i++) {
    uint8_t item = data[i];
    this->encode_byte_(dst, item);
    checksum += item;
  }
  ESP_LOGI(TAG, "Transmit PioneerWyt: %s cs: %0X", data.to_string().c_str(), checksum);
  this->encode_byte_(dst, checksum);
}

void PioneerWytProtocol::encode_byte_(RemoteTransmitData *dst, uint8_t item) {
  for (uint8_t b = 0; b < 8; b++) {
    if (item & (1UL << b)) {
      dst->space(BIT_ONE_SPACE_US);
    } else {
      dst->space(BIT_ZERO_SPACE_US);
    }
    dst->mark(BIT_MARK_US);
  }
}

optional<PioneerWytData> PioneerWytProtocol::decode(RemoteReceiveData src) {
  if (!src.expect_item(HEADER_MARK_US, HEADER_SPACE_US)) {
    return {};
  }
  if (!src.expect_mark(BIT_MARK_US)) {
    return {};
  }

  size_t size = src.size() - src.get_index() - 1;
  if (size < PIONEER_WYT_IR_PACKET_BIT_SIZE * 2)
    return {};
  size = PIONEER_WYT_IR_PACKET_BIT_SIZE * 2;
  uint8_t checksum = 0;
  PioneerWytData out;
  size_t idx = 0;
  while (size > 0) {
    uint8_t data = 0;
    for (uint8_t b = 0; b < 8; b++) {
      if (src.expect_space(BIT_ONE_SPACE_US)) {
        data |= (1UL << b);
      } else if (!src.expect_space(BIT_ZERO_SPACE_US)) {
        return {};
      }
      if (!src.expect_mark(BIT_MARK_US)) {
        return {};
      }
      size -= 2;
    }

    if (idx < WYT_REMOTE_COMMAND_SIZE - 1) {
      checksum += data;
      out[idx++] = data;
    } else {
      if (out[3] == PioneerWytData::PIONEER_WYT_TYPE_FAN) {
        checksum += 0x0F;
      }
      if (checksum != data) {
        return {};
      }
      out[idx++] = data;
    }
  }
  return out;
}

void PioneerWytProtocol::dump(const PioneerWytData &data) {
  ESP_LOGI(TAG, "Received PioneerWyt: %s", data.to_string().c_str());
}

}  // namespace remote_base
}  // namespace esphome
