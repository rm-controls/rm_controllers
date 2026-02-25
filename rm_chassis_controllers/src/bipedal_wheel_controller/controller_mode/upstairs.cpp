//
// Created by wk on 2025/11/1.
//

#include "bipedal_wheel_controller/controller_mode/upstairs.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"

namespace rm_chassis_controllers
{
Upstairs::Upstairs(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                   const std::vector<control_toolbox::Pid*>& pid_legs,
                   const std::vector<control_toolbox::Pid*>& pid_thetas)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
}

void Upstairs::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter Upstairs");
    controller->setStateChange(true);
    controller->setCompleteStand(false);
    leg_state_threshold_ = controller->getLegThresholdParams();
    detectLegState(x_left_, left_leg_state);
    detectLegState(x_right_, right_leg_state);
  }

  double theta_des_l{ M_PI_2 - 0.6 }, theta_des_r{ M_PI_2 - 0.6 }, length_des_l{ 0.18 }, length_des_r{ 0.18 };
  theta_des_l = theta_des_r = leg_state_threshold_->upstair_des_theta;
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  left_cmd = computePidLegCommand(length_des_l, theta_des_l, left_pos_, left_spd_, *pid_legs_[0], *pid_thetas_[0],
                                  *pid_thetas_[2], left_angle_, left_leg_state, period);
  right_cmd = computePidLegCommand(length_des_r, theta_des_r, right_pos_, right_spd_, *pid_legs_[1], *pid_thetas_[1],
                                   *pid_thetas_[3], right_angle_, right_leg_state, period);
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (left_pos_[0] < 0.2 && left_pos_[1] > leg_state_threshold_->upstair_exit_threshold && right_pos_[0] < 0.2 &&
      right_pos_[1] > leg_state_threshold_->upstair_exit_threshold)
  {
    controller->pubLegLenStatus(true);
    controller->setMode(BalanceMode::STAND_UP);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit Upstairs");
  }
}

inline void Upstairs::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, int& leg_state)
{
  if (!leg_state_threshold_)
  {
    ROS_ERROR_THROTTLE(1.0, "LegUtils threshold params not initialized!");
    return;
  }
  if (x[0] > leg_state_threshold_->under_lower && x[0] < leg_state_threshold_->under_upper)
    leg_state = LegState::UNDER;
  else if ((x[0] < leg_state_threshold_->front_lower && x[0] > -M_PI) ||
           (x[0] < M_PI && x[0] > leg_state_threshold_->front_upper))
    leg_state = LegState::FRONT;
  else if (x[0] > leg_state_threshold_->behind_lower && x[0] < leg_state_threshold_->behind_upper)
    leg_state = LegState::BEHIND;
  switch (leg_state)
  {
    case LegState::UNDER:
      ROS_INFO("[balance] x[0]: %.3f Leg state: UNDER", x[0]);
      break;
    case LegState::FRONT:
      ROS_INFO("[balance] x[0]: %.3f Leg state: FRONT", x[0]);
      break;
    case LegState::BEHIND:
      ROS_INFO("[balance] x[0]: %.3f Leg state: BEHIND", x[0]);
      break;
  }
}
}  // namespace rm_chassis_controllers
