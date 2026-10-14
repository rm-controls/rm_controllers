//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <rm_common/filters/filters.h>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/vmc/VMC.h"

namespace rm_chassis_controllers
{
class StandUp : public ModeBase
{
  struct StandUpLegCommand
  {
    double desired_length;
    double desired_angle;
    double desired_angle_vel;
  };

public:
  StandUp(BipedalControllerInterface* controller_, const std::vector<hardware_interface::JointHandle*>& joint_handles,
          const std::vector<control_toolbox::Pid*>& pid_legs, const std::vector<control_toolbox::Pid*>& pid_thetas);
  void execute(const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "STAND_UP";
  }

private:
  void setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const LegOrientation& other_leg_orientation,
                      const double& leg_length, const double& leg_theta, LegOrientation& leg_orientation,
                      StandUpLegCommand& legCommand, bool& stop_flag, bool& arrive_flag, ros::Time arrive_time);
  /**
   * Detect the leg state before stand up: UNDER, FRONT, BEHIND
   * @param x
   * @param leg_orientation
   */
  void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_orientation);
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
  inline LegCommand computePidLegCommand(const StandUpLegCommand& leg_command, const VMCPtr& vmc_,
                                         control_toolbox::Pid& length_pid, control_toolbox::Pid& angle_pid,
                                         control_toolbox::Pid& angle_vel_pid, const LegOrientation& leg_orientation,
                                         const ros::Duration& period, double feedforward_force = 0.0f);
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_, pid_thetas_;
  LegOrientation left_leg_orientation, right_leg_orientation;
  double theta_des_l, theta_des_r, length_des_l, length_des_r;
  StandUpLegCommand left_leg_command_, right_leg_command_;
  bool left_stop_{ false }, right_stop_{ false };
  std::shared_ptr<LegStateThresholdParams> leg_state_threshold_;
  ros::Time left_arrive_time_, right_arrive_time_;
  bool left_arrive_flag_{ false }, right_arrive_flag_{ false };
  VMCPtr vmcPtr_;
  std::shared_ptr<RampFilter<double>> ramp_length_des_l_, ramp_length_des_r_, ramp_angle_des_l_, ramp_angle_des_r_;
};
}  // namespace rm_chassis_controllers
