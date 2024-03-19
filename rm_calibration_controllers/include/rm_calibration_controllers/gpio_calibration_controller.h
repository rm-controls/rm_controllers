//
// Created by guanlin on 22-11-7.
//

#pragma once

#include "rm_calibration_controllers/calibration_base.h"
#include <effort_controllers/joint_position_controller.h>

namespace rm_calibration_controllers
{
class GpioCalibrationController
  : public CalibrationBase<rm_control::ActuatorExtraInterface, rm_control::GpioStateInterface,
                           hardware_interface::EffortJointInterface>
{
public:
  GpioCalibrationController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  enum State
  {
    FAST_FORWARD = 3,
    RETREAT,
    SLOW_FORWARD
  };
  double position_threshold_{}, backward_angle_{}, start_retreat_position_{}, slow_forward_velocity_{};
  rm_control::GpioStateHandle gpio_state_handle_;
  bool initial_gpio_state_;
};
}  // namespace rm_calibration_controllers
