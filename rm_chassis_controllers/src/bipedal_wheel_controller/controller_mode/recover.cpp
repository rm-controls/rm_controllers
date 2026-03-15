//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
Recover::Recover(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas, control_toolbox::Pid* pid_theta_diff)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas), pid_theta_diff_(pid_theta_diff)
{
}

void Recover::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter RECOVER");
    detectd_flag = false;
    controller->setStateChange(true);
  }

  // until chassis
  if (!detectd_flag && abs(x_left_[1]) < 0.1 && abs(x_left_[5]) < 0.1)
  {
    detectChassisStateToRecover();
    detectd_flag = true;
    leg_recovery_velocity_ =
        recovery_chassis_state_ == BackwardSlip ? -leg_recovery_velocity_const_ : leg_recovery_velocity_const_;
  }

  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  //  leg_theta_diff_ = angles::shortest_angular_distance(left_pos_[1], right_pos_[1]);
  leg_theta_diff_ = right_pos_[1] - left_pos_[1];
  double T_theta_diff{ 0.0 }, feedforward_force{ 0.0 };
  if (controller->getBaseState() != 4)
  {
    const auto& model_params_ = controller->getModelParams();
    feedforward_force = model_params_->f_spring;
    T_theta_diff = pid_theta_diff_->computeCommand(leg_theta_diff_, period);
    left_cmd.force = pid_legs_[0]->computeCommand(desired_leg_length_ - left_pos_[0], period) + feedforward_force;
    right_cmd.force = pid_legs_[1]->computeCommand(desired_leg_length_ - right_pos_[0], period) + feedforward_force;
    controller->getVMCPtr()->leg_conv(left_cmd.force, T_theta_diff, left_angle_[0], left_angle_[1], left_cmd.input);
    controller->getVMCPtr()->leg_conv(right_cmd.force, -T_theta_diff, right_angle_[0], right_angle_[1], right_cmd.input);
    if (abs(leg_theta_diff_) < 0.3)
    {
      left_cmd.torque = pid_thetas_[2]->computeCommand(leg_recovery_velocity_ - left_spd_[1], period);
      right_cmd.torque = pid_thetas_[3]->computeCommand(leg_recovery_velocity_ - right_spd_[1], period);
      controller->getVMCPtr()->leg_conv(left_cmd.force, 5 * leg_recovery_velocity_ + left_cmd.torque + T_theta_diff,
                                        left_angle_[0], left_angle_[1], left_cmd.input);
      controller->getVMCPtr()->leg_conv(right_cmd.force, 5 * leg_recovery_velocity_ + right_cmd.torque - T_theta_diff,
                                        right_angle_[0], right_angle_[1], right_cmd.input);
    }
  }
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (abs(pitch_) < 0.2 && linear_acc_base_.z > 5.0 && !controller->getOverturn())
  {
    controller->setMode(BalanceMode::STAND_UP);
    controller->setStateChange(false);
    controller->clearRecoveryFlag();
    ROS_INFO("[balance] Exit RECOVER");
  }
}

void Recover::detectChassisStateToRecover()
{
  // pitch_ is base_link pitch not model pitch
  if (pitch_ > 0.45 && pitch_ < M_PI)
  {
    ROS_INFO("forward");
    recovery_chassis_state_ = RecoveryChassisState::ForwardSlip;
  }
  else if (pitch_ < -0.45 && pitch_ > -M_PI)
  {
    ROS_INFO("back");
    recovery_chassis_state_ = RecoveryChassisState::BackwardSlip;
  }
}
}  // namespace rm_chassis_controllers
