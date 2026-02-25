//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
class StandUp : public ModeBase
{
public:
  StandUp(const std::vector<hardware_interface::JointHandle*>& joint_handles,
          const std::vector<control_toolbox::Pid*>& pid_legs, const std::vector<control_toolbox::Pid*>& pid_thetas);
  void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "STAND_UP";
  }

private:
  void setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const int& other_leg_state,
                      const double& leg_length, const double& leg_theta, int& leg_state, double& theta_des,
                      double& length_des);
  /**
   * Detect the leg state before stand up: UNDER, FRONT, BEHIND
   * @param x
   * @param leg_state
   */
  void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, int& leg_state);
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_, pid_thetas_;
  int left_leg_state, right_leg_state;
  double theta_des_l, theta_des_r, length_des_l, length_des_r;
  double spring_force_{};
  std::shared_ptr<LegStateThresholdParams> leg_state_threshold_;
};
}  // namespace rm_chassis_controllers
