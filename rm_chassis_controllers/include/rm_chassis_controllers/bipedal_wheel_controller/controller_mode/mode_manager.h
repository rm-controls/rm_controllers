//
// Created by guanlin on 25-9-4.
//

#pragma once

#include <control_toolbox/pid.h>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/controller_mode/sit_down.h"
#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller_mode/upstairs.h"

namespace rm_chassis_controllers
{
class ModeManager
{
public:
  ModeManager(ros::NodeHandle& controller_nh, const std::vector<hardware_interface::JointHandle*>& joint_handles);
  virtual ~ModeManager() = default;
  void switchMode(int mode)
  {
    mode_impl = mode_map_[mode];
  }
  const std::shared_ptr<ModeBase>& getModeImpl()
  {
    return mode_impl;
  }

private:
  std::shared_ptr<ModeBase> mode_impl;
  std::map<int, std::shared_ptr<ModeBase>> mode_map_;

  control_toolbox::Pid pid_yaw_vel_, pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_roll_;
  control_toolbox::Pid pid_left_leg_stand_up_, pid_right_leg_stand_up_;
  control_toolbox::Pid pid_left_leg_theta_, pid_right_leg_theta_, pid_left_leg_theta_vel_, pid_right_leg_theta_vel_;
  control_toolbox::Pid pid_left_wheel_vel_, pid_right_wheel_vel_;
  std::vector<control_toolbox::Pid*> pid_wheels_, pid_legs_, pid_thetas_, pid_legs_stand_up_;
};
}  // namespace rm_chassis_controllers
