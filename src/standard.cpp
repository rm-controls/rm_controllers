//
// Created by flying on 2021/1/18.
//
#include "rm_chassis_controllers/standard.h"
#include <rm_common/ros_utilities.h>
#include <string>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers {
bool StandardController::init(hardware_interface::RobotHW *robot_hw,
                              ros::NodeHandle &root_nh,
                              ros::NodeHandle &controller_nh) {
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_rf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rf_name", std::string("joint_rf")));
  joint_rb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rb_name", std::string("joint_rb")));
  joint_lb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lb_name", std::string("joint_lb")));
  joint_lf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lf_name", std::string("joint_lf")));
  joint_vector_.push_back(joint_rf_);
  joint_vector_.push_back(joint_rb_);
  joint_vector_.push_back(joint_lb_);
  joint_vector_.push_back(joint_lf_);

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  if (!pid_rf_.init(ros::NodeHandle(controller_nh, "pid_rf")) ||
      !pid_rb_.init(ros::NodeHandle(controller_nh, "pid_rb")) ||
      !pid_lf_.init(ros::NodeHandle(controller_nh, "pid_lf")) ||
      !pid_lb_.init(ros::NodeHandle(controller_nh, "pid_lb")) ||
      !pid_follow_.init(ros::NodeHandle(controller_nh, "pid_follow")))
    return false;

  // init odom tf
  if (enable_odom_tf_) {
    odom2base_.header.frame_id = "odom";
    odom2base_.header.stamp = ros::Time::now();
    odom2base_.child_frame_id = "base_link";
    odom2base_.transform.rotation.w = 1;
    tf_broadcaster_.init(root_nh);
    tf_broadcaster_.sendTransform(odom2base_);
  }

  return true;
}

void StandardController::moveJoint(const ros::Duration &period) {
  ramp_x->input(vel_tfed_.vector.x);
  ramp_y->input(vel_tfed_.vector.y);
  ramp_w->input(vel_tfed_.vector.z);

  double a = (wheel_base_ + wheel_track_) / 2.0;
  double joint_rf_des = (ramp_x->output() + ramp_y->output() + ramp_w->output() * a) / wheel_radius_;
  double joint_lf_des = (ramp_x->output() - ramp_y->output() - ramp_w->output() * a) / wheel_radius_;
  double joint_lb_des = (ramp_x->output() + ramp_y->output() - ramp_w->output() * a) / wheel_radius_;
  double joint_rb_des = (ramp_x->output() - ramp_y->output() + ramp_w->output() * a) / wheel_radius_;

  double joint_rf_error = joint_rf_des - joint_rf_.getVelocity();
  double joint_rb_error = joint_rb_des - joint_rb_.getVelocity();
  double joint_lf_error = joint_lf_des - joint_lf_.getVelocity();
  double joint_lb_error = joint_lb_des - joint_lb_.getVelocity();

  pid_rf_.computeCommand(joint_rf_error, period);
  pid_rb_.computeCommand(joint_rb_error, period);
  pid_lf_.computeCommand(joint_lf_error, period);
  pid_lb_.computeCommand(joint_lb_error, period);

  // Power limit
  double real_effort = (std::abs(pid_rf_.getCurrentCmd()) + std::abs(pid_rb_.getCurrentCmd()) +
      std::abs(pid_lf_.getCurrentCmd()) + std::abs(pid_lb_.getCurrentCmd()));

  double prop =
      real_effort > chassis_rt_buffer_.readFromRT()->effort_limit ? chassis_rt_buffer_.readFromRT()->effort_limit
          / real_effort : 1.;

  joint_rf_.setCommand(prop * pid_rf_.getCurrentCmd());
  joint_rb_.setCommand(prop * pid_rb_.getCurrentCmd());
  joint_lf_.setCommand(prop * pid_lf_.getCurrentCmd());
  joint_lb_.setCommand(prop * pid_lb_.getCurrentCmd());
}

geometry_msgs::Twist StandardController::forwardKinematics() {
  geometry_msgs::Twist vel_data;
  double k = wheel_radius_ / 4.0;
  double joint_rf_velocity = joint_rf_.getVelocity();
  double joint_rb_velocity = joint_rb_.getVelocity();
  double joint_lf_velocity = joint_lf_.getVelocity();
  double joint_lb_velocity = joint_lb_.getVelocity();
  vel_data.linear.x =
      (joint_rf_velocity + joint_lf_velocity + joint_lb_velocity + joint_rb_velocity) * k;
  vel_data.linear.y =
      (joint_rf_velocity - joint_lf_velocity + joint_lb_velocity - joint_rb_velocity) * k;
  vel_data.angular.z =
      2 * (joint_rf_velocity - joint_lf_velocity - joint_lb_velocity + joint_rb_velocity) * k
          / (wheel_base_ + wheel_track_);
  return vel_data;
}

} // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::StandardController, controller_interface::ControllerBase)
