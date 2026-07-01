//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/sit_down.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
SitDown::SitDown(BipedalControllerInterface* controller_,
                 const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_wheels)
  : ModeBase(controller_), joint_handles_(joint_handles), pid_wheels_(pid_wheels)
{
}

void SitDown::execute(const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter SIT_DOWN");
    controller->setStateChange(true);
  }

  auto& chassis_state = controller->getChassisState();
  //  auto& left_leg_state = controller->getLegState(LEFT);
  //  auto& right_leg_state = controller->getLegState(RIGHT);
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  //  double left_wheel_cmd = pid_wheels_[0]->computeCommand(joint_handles_[0]->getVelocity(), period);
  //  double right_wheel_cmd = pid_wheels_[1]->computeCommand(joint_handles_[1]->getVelocity(), period);
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (abs(chassis_state.angular_vel.y) < 0.1 && controller->getBaseState() != rm_msgs::ChassisCmd::FALLEN)
  {
    controller->setStateChange(false);
    if (controller->getOverturn())
    {
      controller->setMode(BalanceMode::RECOVER);
    }
    else
    {
      controller->setMode(BalanceMode::STAND_UP);
    }
    ROS_INFO("[balance] Exit SIT_DOWN");
  }
}
}  // namespace rm_chassis_controllers
