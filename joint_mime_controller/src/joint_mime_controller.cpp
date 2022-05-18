//
// Created by ljq on 2022/5/15.
//

#include "joint_mime_controller/joint_mime_controller.h"

namespace joint_mime_controller
{
bool JointMimeController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("joint_mime", mimic_joint_name_))
  {
    ROS_ERROR("joint_mime is not set");
    return false;
  }
  target_state_handle_ = robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(mimic_joint_name_);
  mimic_joint_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), controller_nh);
  return true;
};

void JointMimeController::update(const ros::Time& time, const ros::Duration& period)
{
  mimic_joint_ctrl_.setCommand(target_state_handle_.getPosition());
  mimic_joint_ctrl_.update(time, period);
};

}  // namespace joint_mime_controller
