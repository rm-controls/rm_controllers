//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/vmc/VMC.h"

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
                      double& length_des, bool& stop_flag);
  /**
   * Detect the leg state before stand up: UNDER, FRONT, BEHIND
   * @param x
   * @param leg_state
   */
  void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, int& leg_state);
  /**
   * Compute the leg command using PID controllers
   * @param desired_length
   * @param desired_angle
   * @param current_length
   * @param current_angle
   * @param length_pid
   * @param angle_pid
   * @param leg_angle
   * @param period
   * @param feedforward_force
   * @return
   */
  inline LegCommand computePidLegCommand(double desired_length, double desired_angle, double leg_pos[2],
                                         double leg_spd[2], control_toolbox::Pid& length_pid,
                                         control_toolbox::Pid& angle_pid, control_toolbox::Pid& angle_vel_pid,
                                         const double* leg_angle, const int& leg_state, const ros::Duration& period,
                                         double feedforward_force = 0.0f);
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_, pid_thetas_;
  int left_leg_state, right_leg_state;
  double theta_des_l, theta_des_r, length_des_l, length_des_r;
  double spring_force_{};
  bool left_stop_{ false }, right_stop_{ false };
  std::shared_ptr<LegStateThresholdParams> leg_state_threshold_;
  VMCPtr vmcPtr_;
};
}  // namespace rm_chassis_controllers
