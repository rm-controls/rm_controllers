//
// Created by guanlin on 22-11-7.
//

#pragma once

#include "rm_calibration_controllers/gpio_calibration_base.h"
#include <effort_controllers/joint_position_controller.h>

namespace rm_calibration_controllers
{
class HallSwitchCalibrationController : public GpioCalibrationBase
{
public:
  HallSwitchCalibrationController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  enum State
  {
    MOVING_AROUND = 3,
    RETURN
  };
  double position_threshold_{}, enter_pos_{}, exit_pos_{};
  bool is_returned_ = false;

  effort_controllers::JointPositionController position_ctrl_;
};
}  // namespace rm_calibration_controllers
