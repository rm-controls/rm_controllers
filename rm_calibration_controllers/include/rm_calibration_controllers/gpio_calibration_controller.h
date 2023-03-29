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
    RETREAT = 3,
    FORWARD
  };
  double position_threshold_{}, backward_radius, start_retreat_pos_{}, forward_velocity_{};
  std::vector<rm_control::GpioStateHandle> gpio_state_handles_;
  std::vector<bool> initial_gpio_states_;

  effort_controllers::JointPositionController position_ctrl_;
};
}  // namespace rm_calibration_controllers
