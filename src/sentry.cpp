//
// Created by flying on 2021/1/18.
//
#include "rm_chassis_controllers/sentry.h"
#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers {
bool SentryController::init(hardware_interface::RobotHW *robot_hw,
                            ros::NodeHandle &root_nh,
                            ros::NodeHandle &controller_nh) {
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  if (!ctrl_wheel_.init(effort_joint_interface_, controller_nh))
    return false;
  joint_handles_.push_back(effort_joint_interface_->getHandle(ctrl_wheel_.getJointName()));
  return true;
}

void SentryController::moveJoint(const ros::Duration &period) {
  ctrl_wheel_.setCommand(ramp_x->output());
}

geometry_msgs::Twist SentryController::forwardKinematics() {
  geometry_msgs::Twist vel_data;
  vel_data.linear.x = ctrl_wheel_.joint_.getVelocity() * wheel_radius_;
  return vel_data;
}

}// namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SentryController, controller_interface::ControllerBase)
