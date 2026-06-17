#pragma once

#include <array>
#include <vector>

#include "esphome/core/helpers.h"
#include "remote_base.h"

namespace esphome {
namespace remote_base {

static const uint8_t WYT_REMOTE_COMMAND_SIZE = 14;

class PioneerWytData {
 public:
  PioneerWytData() {}
  PioneerWytData(std::array<uint8_t, WYT_REMOTE_COMMAND_SIZE> data) {
    std::copy_n(data.begin(), WYT_REMOTE_COMMAND_SIZE, this->data_.begin());
  }
  PioneerWytData(const std::vector<uint8_t> &data) {
    std::copy_n(data.begin(), std::min(data.size(), this->data_.size()), this->data_.begin());
  }

  uint8_t size() const { return this->data_.size(); }
  bool is_valid() const { return this->data_[WYT_REMOTE_COMMAND_SIZE - 1] == this->calc_cs_(this->checksum_offset_); }
  void finalize() { this->data_[WYT_REMOTE_COMMAND_SIZE - 1] = this->calc_cs_(this->checksum_offset_); }
  std::string to_string() const { return format_hex_pretty(this->data_.data(), this->data_.size()); }

  bool operator==(const PioneerWytData &rhs) const {
    return std::equal(this->data_.begin(), this->data_.begin() + WYT_REMOTE_COMMAND_SIZE - 1, rhs.data_.begin());
  }
  enum PioneerWytDataType : uint8_t {
    PIONEER_WYT_TYPE_GENERAL = 0x01,
    PIONEER_WYT_TYPE_FAN = 0x02,
  };
  PioneerWytDataType type() const { return static_cast<PioneerWytDataType>(this->data_[3]); }
  uint8_t &operator[](size_t idx) { return this->data_[idx]; }
  const uint8_t &operator[](size_t idx) const { return this->data_[idx]; }

 protected:
  uint8_t checksum_offset_ = 0;
  std::array<uint8_t, WYT_REMOTE_COMMAND_SIZE> data_{0};

  // Calculate checksum
  uint8_t calc_cs_(uint8_t checksum_offset = 0x00) const;
};

class PioneerWytProtocol : public RemoteProtocol<PioneerWytData> {
 public:
  void encode(RemoteTransmitData *dst, const PioneerWytData &data);
  optional<PioneerWytData> decode(RemoteReceiveData src);
  void dump(const PioneerWytData &data) override;

 protected:
  void encode_byte_(RemoteTransmitData *dst, uint8_t item);
};

DECLARE_REMOTE_PROTOCOL(PioneerWyt)

template<typename... Ts> class PioneerWytAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(std::vector<uint8_t>, code)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    PioneerWytData data(this->code_.value(x...));
    data.finalize();
    PioneerWytProtocol().encode(dst, data);
  }
};

}  // namespace remote_base
}  // namespace esphome
