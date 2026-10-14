//
// Created by wk on 2025/11/1.
//

#include "bipedal_wheel_controller/controller_mode/upstairs.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"

namespace rm_chassis_controllers
{
Upstairs::Upstairs(BipedalControllerInterface* controller_,
                   const std::vector<hardware_interface::JointHandle*>& joint_handles,
                   const std::vector<control_toolbox::Pid*>& pid_legs,
                   const std::vector<control_toolbox::Pid*>& pid_thetas)
  : ModeBase(controller_), joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
}

void Upstairs::execute(const ros::Time& time, const ros::Duration& period)
{
  auto& left_leg_state = controller->getLegState(LEFT);
  auto& right_leg_state = controller->getLegState(RIGHT);
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter Upstairs");
    controller->setStateChange(true);
    controller->setCompleteStand(false);
    leg_state_threshold_ = controller->getLegThresholdParams();
    //    vmcPtr_ = controller->getVMCPtr();
    detectLegState(left_leg_state.x, left_leg_orientation);
    detectLegState(right_leg_state.x, right_leg_orientation);
  }

  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();

  double theta_des_l{ 1.57 }, theta_des_r{ 1.57 }, length_des_l{ 0.18 }, length_des_r{ 0.18 };
  auto model_params_ = controller->getModelParams();
  double left_spring_force = -controller->f_spring_force(left_pos.L0),
         right_spring_force = -controller->f_spring_force(right_pos.L0);

  length_des_l = length_des_r = leg_state_threshold_->upstair_des_length;
  theta_des_l = theta_des_r = leg_state_threshold_->upstair_des_theta;
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  left_cmd = computePidLegCommand(length_des_l, theta_des_l, left_leg_state.vmc, *pid_legs_[0], *pid_thetas_[0],
                                  *pid_thetas_[2], left_leg_orientation, period, left_spring_force);
  right_cmd = computePidLegCommand(length_des_r, theta_des_r, right_leg_state.vmc, *pid_legs_[1], *pid_thetas_[1],
                                   *pid_thetas_[3], right_leg_orientation, period, right_spring_force);
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (left_pos.theta > leg_state_threshold_->upstair_exit_theta_threshold &&
      right_pos.theta > leg_state_threshold_->upstair_exit_theta_threshold &&
      left_pos.L0 < leg_state_threshold_->upstair_exit_length_threshold &&
      right_pos.L0 < leg_state_threshold_->upstair_exit_length_threshold)
  {
    controller->pubLegLenStatus(true);
    controller->setMode(BalanceMode::STAND_UP);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit Upstairs");
  }
}

inline void Upstairs::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_state)
{
  if (!leg_state_threshold_)
  {
    ROS_ERROR_THROTTLE(1.0, "LegUtils threshold params not initialized!");
    return;
  }
  if (x[0] > leg_state_threshold_->under_lower && x[0] < leg_state_threshold_->under_upper)
    leg_state = LegOrientation::UNDER;
  else if ((x[0] < leg_state_threshold_->front_lower && x[0] > -M_PI) ||
           (x[0] < M_PI && x[0] > leg_state_threshold_->front_upper))
    leg_state = LegOrientation::FRONT;
  else if (x[0] > leg_state_threshold_->behind_lower && x[0] < leg_state_threshold_->behind_upper)
    leg_state = LegOrientation::BEHIND;
  switch (leg_state)
  {
    case LegOrientation::UNDER:
      ROS_INFO("[balance] x[0]: %.3f Leg state: UNDER", x[0]);
      break;
    case LegOrientation::FRONT:
      ROS_INFO("[balance] x[0]: %.3f Leg state: FRONT", x[0]);
      break;
    case LegOrientation::BEHIND:
      ROS_INFO("[balance] x[0]: %.3f Leg state: BEHIND", x[0]);
      break;
  }
}

inline LegCommand Upstairs::computePidLegCommand(double desired_length, double desired_angle, const VMCPtr& vmc_,
                                                 control_toolbox::Pid& length_pid, control_toolbox::Pid& angle_pid,
                                                 control_toolbox::Pid& angle_vel_pid,
                                                 const LegOrientation& leg_orientation, const ros::Duration& period,
                                                 double& feedforward_force)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };
  const auto& leg_pos = vmc_->getPos();
  const auto& leg_spd = vmc_->getSpd();

  cmd.force = length_pid.computeCommand(desired_length - leg_pos.L0, period) + feedforward_force;
  cmd.force = abs(cmd.force) > 250 ? std::copysign(1, cmd.force) * 250 : cmd.force;
  if (leg_orientation == LegOrientation::BEHIND || leg_orientation == LegOrientation::UNDER)
  {
    cmd.torque = angle_pid.computeCommand(-angles::shortest_angular_distance(desired_angle, leg_pos.theta), period);
  }
  else
  {
    cmd.torque = angle_vel_pid.computeCommand(-5 - leg_spd.dTheta, period);
  }
  vmc_->leg_conv(cmd.force, cmd.torque, cmd.input);
  return cmd;
}
}  // namespace rm_chassis_controllers
