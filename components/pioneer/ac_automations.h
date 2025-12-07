#pragma once

#include "esphome/core/automation.h"
#include "wyt_climate.h"

namespace esphome {
namespace pioneer {
namespace wyt {

template<typename... Ts> class PioneerWytActionBase : public Action<Ts...> {
 public:
  void set_parent(WytClimate *parent) { this->parent_ = parent; }

 protected:
  WytClimate *parent_;
};

template<typename... Ts> class RemoteTempAction : public PioneerWytActionBase<Ts...> {
  TEMPLATABLE_VALUE(float, temperature)
  TEMPLATABLE_VALUE(bool, beeper)

  void play(const Ts &...x) override {
    this->parent_->do_remote_temp(this->temperature_.value(x...), this->beeper_.value(x...));
  }
};

template<typename... Ts> class DisplayToggleAction : public PioneerWytActionBase<Ts...> {
 public:
  void play(const Ts &...x) override { this->parent_->do_display_toggle(); }
};

template<typename... Ts> class BeeperOnAction : public PioneerWytActionBase<Ts...> {
 public:
  void play(const Ts &...x) override { this->parent_->do_beeper_on(); }
};

template<typename... Ts> class BeeperOffAction : public PioneerWytActionBase<Ts...> {
 public:
  void play(const Ts &...x) override { this->parent_->do_beeper_off(); }
};

}  // namespace wyt
}  // namespace pioneer
}  // namespace esphome
