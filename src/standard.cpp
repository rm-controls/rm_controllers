//
// Created by huakang on 2021/1/18.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controller/standard.h"

namespace rm_shooter_controllers {
bool StandardController::init(hardware_interface::RobotHW *robot_hw,
                              ros::NodeHandle &root_nh,
                              ros::NodeHandle &controller_nh) {
  ShooterBase::init(robot_hw, root_nh, controller_nh);
  // init joint
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_friction_l_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_friction_left_name", std::string("joint_friction_left")));
  joint_friction_r_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_friction_right_name", std::string("joint_friction_right")));
  joint_trigger_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_trigger_name", std::string("joint_trigger")));
  joint_friction_vector_.push_back(joint_friction_l_);
  joint_friction_vector_.push_back(joint_friction_r_);
  joint_trigger_vector_.push_back(joint_trigger_);

  if (!pid_friction_l_.init(ros::NodeHandle(controller_nh, "pid_friction_l"))
      || !pid_friction_r_.init(ros::NodeHandle(controller_nh, "pid_friction_r"))
      || !pid_trigger_.init(ros::NodeHandle(controller_nh, "pid_trigger")))
    return false;
  pid_friction_vector_.push_back(pid_friction_l_);
  pid_friction_vector_.push_back(pid_friction_r_);
  pid_trigger_vector_.push_back(pid_trigger_);

  return true;
}

void StandardController::push(const ros::Time &time, const ros::Duration &period) {
  ShooterBase::push(time, period);
  if (joint_friction_l_.getVelocity() >= enter_push_qd_coef_ * friction_qd_des_
      && joint_friction_l_.getVelocity() > 8.0 &&
      joint_friction_r_.getVelocity() <= enter_push_qd_coef_ * (-friction_qd_des_)
      && joint_friction_r_.getVelocity() < -8.0 &&
      (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) { // Time to shoot!!!
    if (is_out_from_block_) {
      trigger_q_des_ = last_trigger_q_des_;
      if (fabs(trigger_q_des_ - joint_trigger_.getPosition()) <= push_angle_error_) {
        is_out_from_block_ = false;
      }
    } else {
      trigger_q_des_ = joint_trigger_.getPosition() - config_.push_angle;
      last_trigger_q_des_ = trigger_q_des_;
    }
    last_shoot_time_ = time;
  } else
    ROS_DEBUG("[Shooter] wait for friction wheel");
}

void StandardController::stop(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    pid_friction_l_.reset();
    pid_friction_r_.reset();
    pid_trigger_.reset();
  }
  friction_qd_des_ = 0;
  moveJointFriction(period);
  joint_trigger_.setCommand(0);
}

void StandardController::moveJointFriction(const ros::Duration &period) {
  double friction_l_error = friction_qd_des_ - joint_friction_l_.getVelocity();
  double friction_r_error = -friction_qd_des_ - joint_friction_r_.getVelocity();
  pid_friction_l_.computeCommand(friction_l_error, period);
  pid_friction_r_.computeCommand(friction_r_error, period);
  joint_friction_l_.setCommand(pid_friction_l_.getCurrentCmd());
  joint_friction_r_.setCommand(pid_friction_r_.getCurrentCmd());
}

void StandardController::moveJoint(const ros::Duration &period) {
  moveJointFriction(period);
  double trigger_error = trigger_q_des_ - joint_trigger_.getPosition();
  double magazine_error = magazine_q_des_ - joint_magazine_.getPosition();
  pid_trigger_.computeCommand(trigger_error, period);
  pid_magazine_.computeCommand(magazine_error, period);
  joint_trigger_.setCommand(pid_trigger_.getCurrentCmd());
  joint_magazine_.setCommand(pid_magazine_.getCurrentCmd());
}

} // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::StandardController, controller_interface::ControllerBase)
