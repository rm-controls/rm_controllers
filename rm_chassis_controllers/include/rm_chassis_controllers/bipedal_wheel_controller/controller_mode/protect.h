//
// Created by wk on 2026/5/13.
//

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <rm_common/filters/filters.h>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
class Protect : public ModeBase
{
public:
  explicit Protect(BipedalControllerInterface* controller_,
                   const std::vector<hardware_interface::JointHandle*>& joint_handles,
                   const std::vector<control_toolbox::Pid*>& pid_legs,
                   const std::vector<control_toolbox::Pid*>& pid_thetas,
                   const std::vector<control_toolbox::Pid*>& pid_wheels, control_toolbox::Pid* pid_theta_diff,
                   control_toolbox::Pid* pid_yaw_vel);
  void execute(const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "PROTECT";
  }

private:
  double theta_des_l, theta_des_r, length_des_l, length_des_r;
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_, pid_thetas_;
  std::vector<control_toolbox::Pid*> pid_wheels_;
  control_toolbox::Pid *pid_theta_diff_, *pid_yaw_vel_;
  std::shared_ptr<RampFilter<double>> ramp_length_des_l_, ramp_length_des_r_, ramp_angle_des_l_, ramp_angle_des_r_;
};
}  // namespace rm_chassis_controllers
