//
// Created by flying on 2021/1/18.
//
#include "rm_chassis_controllers/sentry.h"
#include <rm_common/ros_utilities.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_chassis_controllers {
bool SentryController::init(hardware_interface::RobotHW *robot_hw,
                            ros::NodeHandle &root_nh,
                            ros::NodeHandle &controller_nh) {
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_wheel_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_name", std::string("actuator_wheel")));
  joint_vector_.push_back(joint_wheel_);

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  if (!pid_wheel_.init(ros::NodeHandle(controller_nh, "pid_wheel")))
    return false;

  // init odom tf
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = "base_link";
  odom2base_.transform.rotation.w = 1;
  tf_broadcaster_.init(root_nh);
  tf_broadcaster_.sendTransform(odom2base_);

  return true;
}

void SentryController::update(const ros::Time &time, const ros::Duration &period) {
  ChassisBase::update(time, period);
  updateOdom(time, period);

  if (state_ == rm_chassis_base::PASSIVE)
    passive();
  else {
    if (state_ == rm_chassis_base::RAW)
      raw(period);
    moveJoint(period);
  }
}

void SentryController::moveJoint(const ros::Duration &period) {
  ramp_x->input(vel_tfed_.vector.x);

  double joint_wheel_des = ramp_x->output() / wheel_radius_;
  double joint_wheel_error = joint_wheel_des - joint_wheel_.getVelocity();
  pid_wheel_.computeCommand(joint_wheel_error, period);

  // Power limit
  double real_effort = std::abs(pid_wheel_.getCurrentCmd());
  double prop = real_effort > cmd_chassis_.effort_limit ? cmd_chassis_.effort_limit / real_effort : 1.;
  joint_wheel_.setCommand(prop * pid_wheel_.getCurrentCmd());
}

geometry_msgs::Twist SentryController::iKine(const ros::Duration &period) {
  geometry_msgs::Twist vel_data;
  double joint_rf_position = joint_wheel_.getVelocity();
  vel_data.linear.x = joint_rf_position * wheel_radius_;

  return vel_data;
}

void SentryController::updateOdom(const ros::Time &time, const ros::Duration &period) {
  geometry_msgs::Twist vel = iKine(period); //on base_link frame
  geometry_msgs::Vector3Stamped trans;
  trans.vector.x = vel.linear.x * period.toSec();

  try {
    odom2base_ = robot_state_handle_.lookupTransform("odom", "base_link", ros::Time(0));
    tf2::doTransform(trans, trans, odom2base_);
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
  ros::Time now = ros::Time::now();
  odom2base_.header.stamp = now;
  odom2base_.transform.translation.x += trans.vector.x;

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    tf_broadcaster_.sendTransform(odom2base_);
    last_publish_time_ = time;
  } else
    robot_state_handle_.setTransform(odom2base_, "rm_chassis_controllers");
}

}// namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SentryController, controller_interface::ControllerBase)
