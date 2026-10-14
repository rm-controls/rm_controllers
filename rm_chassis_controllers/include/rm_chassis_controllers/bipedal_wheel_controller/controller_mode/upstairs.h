//
// Created by wk on 2025/11/1.
//
#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include "rm_common/filters/filters.h"

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/vmc/VMC.h"

namespace rm_chassis_controllers
{
class Upstairs : public ModeBase
{
public:
  Upstairs(BipedalControllerInterface* controller_, const std::vector<hardware_interface::JointHandle*>& joint_handles,
           const std::vector<control_toolbox::Pid*>& pid_legs, const std::vector<control_toolbox::Pid*>& pid_thetas);
  void execute(const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "Upstairs";
  }

private:
  /**
   * Detect the leg state before stand up: UNDER, FRONT, BEHIND
   * @param x
   * @param leg_state
   */
  void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_state);
  inline LegCommand computePidLegCommand(double desired_length, double desired_angle, const VMCPtr& vmc_,
                                         control_toolbox::Pid& length_pid, control_toolbox::Pid& angle_pid,
                                         control_toolbox::Pid& angle_vel_pid, const LegOrientation& leg_state,
                                         const ros::Duration& period, double& feedforward_force);

  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_, pid_thetas_;
  LegOrientation left_leg_orientation, right_leg_orientation;
  std::shared_ptr<LegStateThresholdParams> leg_state_threshold_;
  VMCPtr vmcPtr_;
};
}  // namespace rm_chassis_controllers
