//
// Created by wk on 2026/5/13.
//
#include "bipedal_wheel_controller/controller_mode/protect.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"

namespace rm_chassis_controllers
{
Protect::Protect(BipedalControllerInterface* controller_,
                 const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas,
                 const std::vector<control_toolbox::Pid*>& pid_wheels, control_toolbox::Pid* pid_theta_diff,
                 control_toolbox::Pid* pid_yaw_vel)
  : ModeBase(controller_)
  , joint_handles_(joint_handles)
  , pid_legs_(pid_legs)
  , pid_thetas_(pid_thetas)
  , pid_wheels_(pid_wheels)
  , pid_theta_diff_(pid_theta_diff)
  , pid_yaw_vel_(pid_yaw_vel)
{
  double leg_len_acc = 20, leg_theta_acc = 10;
  ramp_length_des_l_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_length_des_r_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_angle_des_l_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
  ramp_angle_des_r_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
}

void Protect::execute(const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter PROTECT");
    controller->setStateChange(true);
  }
  auto& left_leg_state = controller->getLegState(LEFT);
  auto& right_leg_state = controller->getLegState(RIGHT);
  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& chassis_geometry_params = controller->getChassisGeometryParams();
  const auto& chassis_state = controller->getChassisState();

  double left_wheel_desired_vel{}, right_wheel_desired_vel{};

  auto vel_cmd_ = controller->getVelCmd();
  left_wheel_desired_vel = vel_cmd_.x - vel_cmd_.z * chassis_geometry_params->wheel_track;
  right_wheel_desired_vel = vel_cmd_.x + vel_cmd_.z * chassis_geometry_params->wheel_track;

  length_des_l = length_des_r = 0.12f;
  theta_des_l = theta_des_r = 0.0f;
  ramp_length_des_l_->input(length_des_l);
  ramp_angle_des_l_->input(theta_des_l);
  ramp_length_des_r_->input(length_des_r);
  ramp_angle_des_r_->input(theta_des_r);
  length_des_l = ramp_length_des_l_->output();
  theta_des_l = ramp_angle_des_l_->output();
  length_des_r = ramp_length_des_r_->output();
  theta_des_r = ramp_angle_des_r_->output();

  LegCommand left_cmd{}, right_cmd{};
  double F_pid_left{}, F_pid_right{};
  F_pid_left = pid_legs_[LEFT]->computeCommand(length_des_l - left_pos.L0, period);
  F_pid_right = pid_legs_[RIGHT]->computeCommand(length_des_r - right_pos.L0, period);
  F_pid_left = abs(F_pid_left) > 75 ? std::copysign(1, F_pid_left) * 75 : F_pid_left;
  F_pid_right = abs(F_pid_right) > 75 ? std::copysign(1, F_pid_right) * 75 : F_pid_right;
  left_cmd.force = F_pid_left - controller->f_spring_force(left_pos.L0);
  right_cmd.force = F_pid_right - controller->f_spring_force(right_pos.L0);
  double T_theta_diff = pid_theta_diff_->computeCommand(right_pos.theta - left_pos.theta, period);
  left_cmd.torque = pid_thetas_[0]->computeCommand(theta_des_l - left_pos.theta, period) + T_theta_diff;
  right_cmd.torque = pid_thetas_[1]->computeCommand(theta_des_r - right_pos.theta, period) - T_theta_diff;
  double T_yaw = pid_yaw_vel_->computeCommand(vel_cmd_.z - chassis_state.angular_vel.z, period);
  double left_wheel_cmd =
      pid_wheels_[0]->computeCommand(left_wheel_desired_vel - joint_handles_[0]->getVelocity(), period) - T_yaw;
  double right_wheel_cmd =
      pid_wheels_[1]->computeCommand(right_wheel_desired_vel - joint_handles_[1]->getVelocity(), period) + T_yaw;

  left_leg_state.vmc->leg_conv(left_cmd.force, left_cmd.torque, left_cmd.input);
  right_leg_state.vmc->leg_conv(right_cmd.force, right_cmd.torque, right_cmd.input);

  setJointCommands(joint_handles_, left_cmd, right_cmd, left_wheel_cmd, right_wheel_cmd);
  // Exit
  if (abs(chassis_state.pitch) < 0.3f && abs(chassis_state.angular_vel.y) < 0.2f &&
      abs(left_pos.theta + right_pos.theta) / 2.0f < 0.2f)
  {
    controller->setMode(BalanceMode::NORMAL);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit PROTECT");
  }
  else if (controller->getOverturn() && controller->getBaseState() != rm_msgs::ChassisCmd::FALLEN)
  {
    controller->setStateChange(false);
    controller->setMode(BalanceMode::RECOVER);
    ROS_INFO("[balance] Exit PROTECT");
  }
}

}  // namespace rm_chassis_controllers
