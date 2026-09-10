//
// Created by guanlin on 25-9-4.
//

#include "bipedal_wheel_controller/controller_mode/mode_base.h"

namespace rm_chassis_controllers
{
void ModeBase::updateUnstick(const bool& left_unstick, const bool& right_unstick)
{
  left_unstick_ = left_unstick;
  right_unstick_ = right_unstick;
}

}  // namespace rm_chassis_controllers
