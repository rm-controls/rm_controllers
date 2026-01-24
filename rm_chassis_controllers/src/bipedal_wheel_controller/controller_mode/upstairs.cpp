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
    detectLegState(x_left_, left_leg_state);
    detectLegState(x_right_, right_leg_state);
  }

  double theta_des_l{ M_PI_2 - 0.6 }, theta_des_r{ M_PI_2 - 0.6 }, length_des_l{ 0.18 }, length_des_r{ 0.18 };
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  left_cmd = computePidLegCommand(length_des_l, theta_des_l, left_pos_, left_spd_, *pid_legs_[0], *pid_thetas_[0],
                                  *pid_thetas_[2], left_angle_, left_leg_state, period);
  right_cmd = computePidLegCommand(length_des_r, theta_des_r, right_pos_, right_spd_, *pid_legs_[1], *pid_thetas_[1],
                                   *pid_thetas_[3], right_angle_, right_leg_state, period);
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (left_pos_[0] < 0.2 && left_pos_[1] > (M_PI_2 - 0.65) && right_pos_[0] < 0.2 && right_pos_[1] > (M_PI_2 - 0.65))
  {
    controller->pubLegLenStatus(false);
    controller->setMode(BalanceMode::STAND_UP);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit Upstairs");
  }
}
}  // namespace rm_chassis_controllers
