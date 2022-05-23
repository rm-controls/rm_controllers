//
// Created by ljq on 2022/5/15.
//

#include "mimic_joint_controller/mimic_joint_controller.h"

namespace mimic_joint_controller
{
bool MimicJointController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("target_joint_name", target_joint_name_))
  {
    ROS_ERROR("mimic_joint is not set");
    return false;
  }
  target_state_handle_ = robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(target_joint_name_);
  mimic_joint_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), controller_nh);
  return true;
};

void MimicJointController::update(const ros::Time& time, const ros::Duration& period)
{
  mimic_joint_ctrl_.setCommand(target_state_handle_.getPosition());
  mimic_joint_ctrl_.update(time, period);
};

}  // namespace mimic_joint_controller
