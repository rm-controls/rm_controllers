//
// Created by flying on 2021/1/18.
//
#include "rm_chassis_controllers/mecanum.h"
#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers {
bool MecanumController::init(hardware_interface::RobotHW *robot_hw,
                             ros::NodeHandle &root_nh,
                             ros::NodeHandle &controller_nh) {
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_lf = ros::NodeHandle(controller_nh, "left_front");
  ros::NodeHandle nh_rf = ros::NodeHandle(controller_nh, "right_front");
  ros::NodeHandle nh_lb = ros::NodeHandle(controller_nh, "left_back");
  ros::NodeHandle nh_rb = ros::NodeHandle(controller_nh, "right_back");
  if (!ctrl_lf_.init(effort_joint_interface_, nh_lf) ||
      !ctrl_rf_.init(effort_joint_interface_, nh_rf) ||
      !ctrl_lb_.init(effort_joint_interface_, nh_lb) ||
      !ctrl_rb_.init(effort_joint_interface_, nh_rb))
    return false;
  joint_handles_.push_back(ctrl_lf_.joint_);
  joint_handles_.push_back(ctrl_rf_.joint_);
  joint_handles_.push_back(ctrl_lb_.joint_);
  joint_handles_.push_back(ctrl_rb_.joint_);
  return true;
}

void MecanumController::moveJoint(const ros::Duration &period) {

  double a = (wheel_base_ + wheel_track_) / 2.0;
  ctrl_lf_.setCommand((ramp_x->output() - ramp_y->output() - ramp_w->output() * a) / wheel_radius_);
  ctrl_rf_.setCommand((ramp_x->output() + ramp_y->output() + ramp_w->output() * a) / wheel_radius_);
  ctrl_lb_.setCommand((ramp_x->output() + ramp_y->output() - ramp_w->output() * a) / wheel_radius_);
  ctrl_rb_.setCommand((ramp_x->output() - ramp_y->output() + ramp_w->output() * a) / wheel_radius_);
}

geometry_msgs::Twist MecanumController::forwardKinematics() {
  geometry_msgs::Twist vel_data;
  double k = wheel_radius_ / 4.0;
  double lf_velocity = ctrl_lf_.joint_.getVelocity();
  double rf_velocity = ctrl_rf_.joint_.getVelocity();
  double lb_velocity = ctrl_lb_.joint_.getVelocity();
  double rb_velocity = ctrl_rb_.joint_.getVelocity();
  vel_data.linear.x = (rf_velocity + lf_velocity + lb_velocity + rb_velocity) * k;
  vel_data.linear.y = (rf_velocity - lf_velocity + lb_velocity - rb_velocity) * k;
  vel_data.angular.z = 2 * (rf_velocity - lf_velocity - lb_velocity + rb_velocity) * k / (wheel_base_ + wheel_track_);
  return vel_data;
}

} // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::MecanumController, controller_interface::ControllerBase)
