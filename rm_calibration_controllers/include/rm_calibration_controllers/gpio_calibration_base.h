//
// Created by guanlin on 23-3-16.
//

#pragma once

#include <rm_common/hardware_interface/gpio_interface.h>
#include <rm_calibration_controllers/calibration_base.h>
#include <rm_msgs/GpioData.h>

namespace rm_calibration_controllers
{
class GpioCalibrationBase : public CalibrationBase<rm_control::RobotStateInterface, rm_control::GpioStateInterface,
                                                   hardware_interface::EffortJointInterface>
{
public:
  GpioCalibrationBase() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

protected:
  double vel_threshold_{};
  bool last_gpio_state_ = false;
  std::vector<rm_control::GpioStateHandle> gpio_state_handles_;
  std::vector<bool> initial_gpio_states_;
};
}  // namespace rm_calibration_controllers
