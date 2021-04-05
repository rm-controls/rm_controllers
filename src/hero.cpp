//
// Created by huakang on 2021/3/29.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controller/hero.h"

namespace rm_shooter_controllers {
bool HeroController::init(hardware_interface::RobotHW *robot_hw,
                          ros::NodeHandle &root_nh,
                          ros::NodeHandle &controller_nh) {
  ShooterBase::init(robot_hw, root_nh, controller_nh);
  // init joint
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_friction_lf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_friction_lf_name", std::string("joint_friction_lf")));
  joint_friction_rf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_friction_rf_name", std::string("joint_friction_rf")));
  joint_friction_lb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_friction_lb_name", std::string("joint_friction_lb")));
  joint_friction_rb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_friction_rb_name", std::string("joint_friction_rb")));
  joint_trigger_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_trigger_name", std::string("joint_trigger")));
  joint_friction_vector_.push_back(joint_friction_lf_);
  joint_friction_vector_.push_back(joint_friction_rf_);
  joint_friction_vector_.push_back(joint_friction_lb_);
  joint_friction_vector_.push_back(joint_friction_rb_);
  joint_trigger_vector_.push_back(joint_trigger_);

  if (!pid_friction_lf_.init(ros::NodeHandle(controller_nh, "pid_friction_lf"))
      || !pid_friction_rf_.init(ros::NodeHandle(controller_nh, "pid_friction_rf"))
      || !pid_friction_lb_.init(ros::NodeHandle(controller_nh, "pid_friction_lb"))
      || !pid_friction_rb_.init(ros::NodeHandle(controller_nh, "pid_friction_rb"))
      || !pid_trigger_.init(ros::NodeHandle(controller_nh, "pid_trigger")))
    return false;
  pid_friction_vector_.push_back(pid_friction_lf_);
  pid_friction_vector_.push_back(pid_friction_rf_);
  pid_friction_vector_.push_back(pid_friction_lb_);
  pid_friction_vector_.push_back(pid_friction_rb_);
  pid_trigger_vector_.push_back(pid_trigger_);

  return true;
}

void HeroController::push(const ros::Time &time, const ros::Duration &period) {
  ShooterBase::push(time, period);
  if (joint_friction_lf_.getVelocity() >= enter_push_qd_coef_ * friction_qd_des_
      && joint_friction_lf_.getVelocity() > 8.0 &&
      joint_friction_rf_.getVelocity() <= enter_push_qd_coef_ * (-friction_qd_des_)
      && joint_friction_rf_.getVelocity() < -8.0 &&
      joint_friction_lb_.getVelocity() >= enter_push_qd_coef_ * friction_qd_des_
      && joint_friction_lb_.getVelocity() > 8.0 &&
      joint_friction_rb_.getVelocity() <= enter_push_qd_coef_ * (-friction_qd_des_)
      && joint_friction_rb_.getVelocity() < -8.0 &&
      (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) { // Time to shoot!!!
    trigger_q_des_ = joint_trigger_.getPosition() - config_.push_angle;
    last_shoot_time_ = time;
  } else
    ROS_DEBUG("[Shooter] wait for friction wheel");
}

void HeroController::moveJoint(const ros::Duration &period) {
  double friction_lf_error = friction_qd_des_ - joint_friction_lf_.getVelocity();
  double friction_rf_error = -friction_qd_des_ - joint_friction_rf_.getVelocity();
  double friction_lb_error = friction_qd_des_ - joint_friction_lb_.getVelocity();
  double friction_rb_error = -friction_qd_des_ - joint_friction_rb_.getVelocity();
  double trigger_error = trigger_q_des_ - joint_trigger_.getPosition();

  pid_friction_lf_.computeCommand(friction_lf_error, period);
  pid_friction_rf_.computeCommand(friction_rf_error, period);
  pid_friction_lb_.computeCommand(friction_lb_error, period);
  pid_friction_rb_.computeCommand(friction_rb_error, period);
  pid_trigger_.computeCommand(trigger_error, period);
  joint_friction_lf_.setCommand(pid_friction_lf_.getCurrentCmd());
  joint_friction_rf_.setCommand(pid_friction_rf_.getCurrentCmd());
  joint_friction_lb_.setCommand(pid_friction_lb_.getCurrentCmd());
  joint_friction_rb_.setCommand(pid_friction_rb_.getCurrentCmd());
  joint_trigger_.setCommand(pid_trigger_.getCurrentCmd());
}

} // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::HeroController, controller_interface::ControllerBase)
