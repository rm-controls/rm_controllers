//
// Created by flying on 2021/1/18.
//

#include <ros_utilities.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/ChassisCmd.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rm_chassis_controller/sentry.h"

namespace rm_chassis_controllers {
bool ChassisSentryController::init(hardware_interface::RobotHW *robot_hw,
                                   ros::NodeHandle &root_nh,
                                   ros::NodeHandle &controller_nh) {
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_wheel_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_name", std::string("actuator_wheel")));

  wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.0250);

  publish_rate_ = getParam(controller_nh, "publish_rate", 50);
  current_coeff_ = getParam(controller_nh, "current_coeff", 1.0);

  ramp_x = new RampFilter<double>(0, 0.001);

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  if (!pid_wheel_.init(ros::NodeHandle(controller_nh, "pid_wheel")))
    return false;

  // init odom tf
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = "base_link";
  odom2base_.transform.rotation.w = 1;
  tf_broadcaster_.init(root_nh);
  tf_broadcaster_.sendTransform(odom2base_);

  chassis_cmd_subscriber_ =
      root_nh.subscribe<rm_msgs::ChassisCmd>("cmd_chassis", 1, &ChassisSentryController::commandCB, this);
  vel_cmd_subscriber_ =
      root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisSentryController::velCmdCB, this);

  return true;
}

void ChassisSentryController::update(const ros::Time &time, const ros::Duration &period) {
  cmd_chassis_ = *chassis_rt_buffer_.readFromRT();
  ramp_x->setAcc(cmd_chassis_.accel.linear.x);

  geometry_msgs::Twist vel_cmd;
  vel_cmd = *vel_rt_buffer_.readFromRT();
  vel_cmd_.vector.x = vel_cmd.linear.x;

  updateOdom(time, period);

  if (state_ != cmd_chassis_.mode) {
    state_ = SentrytState(cmd_chassis_.mode);
    state_changed_ = true;
  }

  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == RAW)
      raw();
    moveJoint(period);
  }
}

void ChassisSentryController::moveJoint(const ros::Duration &period) {
  ramp_x->input(vel_tfed_.vector.x);

  double joint_wheel_des = ramp_x->output() / wheel_radius_;

  double joint_wheel_error = joint_wheel_des - joint_wheel_.getVelocity();

  pid_wheel_.computeCommand(joint_wheel_error, period);

  // Power limit
  double real_current = current_coeff_ * std::abs(pid_wheel_.getCurrentCmd());

  double prop = real_current > cmd_chassis_.current_limit ? cmd_chassis_.current_limit / real_current : 1.;
  joint_wheel_.setCommand(prop * pid_wheel_.getCurrentCmd());
}

geometry_msgs::Twist ChassisSentryController::iKine() {
  geometry_msgs::Twist vel_data;
  double joint_rf_position = joint_wheel_.getVelocity();

  vel_data.linear.x = joint_rf_position * wheel_radius_;

  return vel_data;
}

void ChassisSentryController::recovery() {
  geometry_msgs::Twist vel = iKine();

  ramp_x->clear(vel.linear.x);
}

void ChassisSentryController::passive() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter PASSIVE");

    joint_wheel_.setCommand(0);
  }
}

void ChassisSentryController::raw() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");

    recovery();
  }

  vel_tfed_ = vel_cmd_;
}

void ChassisSentryController::updateOdom(const ros::Time &time, const ros::Duration &period) {
  geometry_msgs::Twist vel = iKine(); //on base_link frame
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

void ChassisSentryController::commandCB(const rm_msgs::ChassisCmdConstPtr &msg) {
  chassis_rt_buffer_.writeFromNonRT(*msg);
}

void ChassisSentryController::velCmdCB(const geometry_msgs::Twist::ConstPtr &cmd) {
  vel_rt_buffer_.writeFromNonRT(*cmd);
}

}// namespace rm_chassis_controller

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ChassisSentryController, controller_interface::ControllerBase)
