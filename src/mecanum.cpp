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

  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_rf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rf_name", std::string("joint_rf")));
  joint_rb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rb_name", std::string("joint_rb")));
  joint_lb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lb_name", std::string("joint_lb")));
  joint_lf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lf_name", std::string("joint_lf")));
  joint_handles_.push_back(&joint_rf_);
  joint_handles_.push_back(&joint_rb_);
  joint_handles_.push_back(&joint_lb_);
  joint_handles_.push_back(&joint_lf_);

  if (!pid_rf_.init(ros::NodeHandle(controller_nh, "pid_rf")) ||
      !pid_rb_.init(ros::NodeHandle(controller_nh, "pid_rb")) ||
      !pid_lf_.init(ros::NodeHandle(controller_nh, "pid_lf")) ||
      !pid_lb_.init(ros::NodeHandle(controller_nh, "pid_lb")))
    return false;
  wheel_pids_.push_back(&pid_rb_);
  wheel_pids_.push_back(&pid_rb_);
  wheel_pids_.push_back(&pid_lf_);
  wheel_pids_.push_back(&pid_lb_);

  return true;
}

void MecanumController::moveJoint(const ros::Duration &period) {

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

  double scale = getEffortLimitScale();
  joint_rf_.setCommand(scale * pid_rf_.getCurrentCmd());
  joint_rb_.setCommand(scale * pid_rb_.getCurrentCmd());
  joint_lf_.setCommand(scale * pid_lf_.getCurrentCmd());
  joint_lb_.setCommand(scale * pid_lb_.getCurrentCmd());
}

geometry_msgs::Twist MecanumController::forwardKinematics() {
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

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::MecanumController, controller_interface::ControllerBase)
