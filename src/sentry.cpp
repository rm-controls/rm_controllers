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
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_wheel_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_name", std::string("actuator_wheel")));
  joint_handles_.push_back(&joint_wheel_);

  if (!pid_wheel_.init(ros::NodeHandle(controller_nh, "pid_wheel")))
    return false;
  joint_pids_.push_back(&pid_wheel_);

  return true;
}

void SentryController::moveJoint(const ros::Duration &period) {
  ramp_x->input(vel_tfed_.vector.x);

  double error = ramp_x->output() / wheel_radius_ - joint_wheel_.getVelocity();
  pid_wheel_.computeCommand(error, period);
  joint_wheel_.setCommand(getEffortLimitScale() * pid_wheel_.getCurrentCmd());
}

geometry_msgs::Twist SentryController::forwardKinematics() {
  geometry_msgs::Twist vel_data;
  vel_data.linear.x = joint_wheel_.getVelocity() * wheel_radius_;

  return vel_data;
}

}// namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SentryController, controller_interface::ControllerBase)
