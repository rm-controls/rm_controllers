//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
Recover::Recover(BipedalControllerInterface* controller_,
                 const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas, control_toolbox::Pid* pid_theta_diff)
  : ModeBase(controller_)
  , joint_handles_(joint_handles)
  , pid_legs_(pid_legs)
  , pid_thetas_(pid_thetas)
  , pid_theta_diff_(pid_theta_diff)
{
}

void Recover::execute(const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter RECOVER");
    detectd_flag = false;
    controller->setStateChange(true);
  }
  chassis_state_ = controller->getChassisState();
  auto& left_leg_state = controller->getLegState(LEFT);
  auto& right_leg_state = controller->getLegState(RIGHT);
  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& left_spd = left_leg_state.vmc->getSpd();
  const auto& right_spd = right_leg_state.vmc->getSpd();

  // until chassis static
  if (!detectd_flag && abs(left_leg_state.x[1]) < 0.2 && abs(right_leg_state.x[5]) < 0.2 &&
      abs(chassis_state_.angular_vel.y) < 0.1)
  {
    detectChassisStateToRecover();
    detectLegRecoveryState(left_recovery_leg, left_pos.theta);
    detectLegRecoveryState(right_recovery_leg, right_pos.theta);
    detectd_flag = true;
    controller->setRecoveryLegSpdTurnback(false);
    leg_recovery_velocity_ =
        recovery_chassis_state_ == BackwardSlip ? -leg_recovery_velocity_const_ : leg_recovery_velocity_const_;
  }

  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  leg_theta_diff_ = angles::shortest_angular_distance(left_pos.theta, right_pos.theta);
  double T_theta_diff{ 0.0 }, feedforward_force{ 0.0 };
  if (controller->getBaseState() != 4 && detectd_flag)
  {
    left_cmd.force = pid_legs_[0]->computeCommand(desired_leg_length_ - left_pos.L0, period) + feedforward_force;
    right_cmd.force = pid_legs_[1]->computeCommand(desired_leg_length_ - right_pos.L0, period) + feedforward_force;

    if (controller->getRecoveryLegSpdTurnback())
    {
      controller->setRecoveryLegSpdTurnback(false);
      leg_recovery_velocity_ = -leg_recovery_velocity_;
    }
    if (chassis_state_.roll < -0.5)
    {
      left_cmd.torque = pid_thetas_[2]->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, period);
      right_cmd.torque = pid_thetas_[3]->computeCommand(0 - right_spd.dTheta, period);
      left_leg_recovery_feed_forward = 3 * leg_recovery_velocity_;
      right_leg_recovery_feed_forward = 0.0f;
      left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
      right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                    right_cmd.input);
    }
    else if (chassis_state_.roll > 0.5)
    {
      left_cmd.torque = pid_thetas_[2]->computeCommand(0 - left_spd.dTheta, period);
      right_cmd.torque = pid_thetas_[3]->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, period);
      left_leg_recovery_feed_forward = 0.0f;
      right_leg_recovery_feed_forward = 3 * leg_recovery_velocity_;
      left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
      right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                    right_cmd.input);
    }
    else
    {
      if (left_recovery_leg == NotReady && right_recovery_leg == Ready)
      {
        detectLegRecoveryState(left_recovery_leg, left_pos.theta);
        detectLegRecoveryState(right_recovery_leg, right_pos.theta);
        left_cmd.torque = pid_thetas_[2]->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, period);
        right_cmd.torque = pid_thetas_[3]->computeCommand(0 - right_spd.dTheta, period);
        left_leg_recovery_feed_forward = 3 * leg_recovery_velocity_;
        right_leg_recovery_feed_forward = 0.0f;
        left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
        right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                      right_cmd.input);
      }
      if (left_recovery_leg == Ready && right_recovery_leg == NotReady)
      {
        detectLegRecoveryState(left_recovery_leg, left_pos.theta);
        detectLegRecoveryState(right_recovery_leg, right_pos.theta);
        left_cmd.torque = pid_thetas_[2]->computeCommand(0 - left_spd.dTheta, period);
        right_cmd.torque = pid_thetas_[3]->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, period);
        left_leg_recovery_feed_forward = 0.0f;
        right_leg_recovery_feed_forward = 3 * leg_recovery_velocity_;
        left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
        right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                      right_cmd.input);
      }
      if (abs(leg_theta_diff_) < 0.4)
      {
        if ((left_recovery_leg == Ready && right_recovery_leg == Ready) ||
            (left_recovery_leg == NotReady && right_recovery_leg == NotReady))
        {
          T_theta_diff = pid_theta_diff_->computeCommand(leg_theta_diff_, period);
          detectLegRecoveryState(left_recovery_leg, left_pos.theta);
          detectLegRecoveryState(right_recovery_leg, right_pos.theta);
          left_cmd.torque = pid_thetas_[2]->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, period);
          right_cmd.torque = pid_thetas_[3]->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, period);
          left_leg_recovery_feed_forward = 3 * leg_recovery_velocity_;
          right_leg_recovery_feed_forward = left_leg_recovery_feed_forward;
          left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque + T_theta_diff,
                                       left_cmd.input);
          right_leg_state.vmc->leg_conv(
              right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque - T_theta_diff, right_cmd.input);
        }
      }
    }
  }
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (abs(chassis_state_.pitch) < 0.2 && chassis_state_.linear_acc.z > 5.0 && !controller->getOverturn())
  {
    controller->setMode(BalanceMode::SIT_DOWN);
    controller->setStateChange(false);
    controller->clearRecoveryFlag();
    ROS_INFO("[balance] Exit RECOVER");
  }
}

void Recover::detectChassisStateToRecover()
{
  // pitch_ is base_link pitch not model pitch
  if (chassis_state_.pitch > 0.45 && chassis_state_.pitch < M_PI)
  {
    ROS_INFO("forward");
    recovery_chassis_state_ = RecoveryChassisState::ForwardSlip;
  }
  else if (chassis_state_.pitch < -0.45 && chassis_state_.pitch > -M_PI)
  {
    ROS_INFO("back");
    recovery_chassis_state_ = RecoveryChassisState::BackwardSlip;
  }
}

inline void Recover::detectLegRecoveryState(LegRecoveryState& leg_recovery_state, const double& leg_pos)
{
  if (recovery_chassis_state_ == RecoveryChassisState::ForwardSlip)
  {
    if ((leg_pos < M_PI && leg_pos > M_PI - 0.3) || (leg_pos < (-M_PI_2 + 0.4) && leg_pos > -M_PI))
    {
      leg_recovery_state = Ready;
    }
    else
    {
      leg_recovery_state = NotReady;
    }
  }
  else
  {
    if ((leg_pos > 0.5 && leg_pos < M_PI) || (leg_pos < (-M_PI_2 + 1.0) && leg_pos > -M_PI))
    {
      leg_recovery_state = Ready;
    }
    else
    {
      leg_recovery_state = NotReady;
    }
  }
  ROS_DEBUG("%d", leg_recovery_state);
}
}  // namespace rm_chassis_controllers
