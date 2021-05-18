//
// Created by huakang on 2021/1/18.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controller/standard.h"

namespace rm_shooter_controllers {
bool StandardController::init(hardware_interface::RobotHW *robot_hw,
                              ros::NodeHandle &root_nh,
                              ros::NodeHandle &controller_nh) {
  ShooterBase::init(robot_hw, root_nh, controller_nh);
  // init joint
  ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
  ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  ros::NodeHandle nh_magazine = ros::NodeHandle(controller_nh, "magazine");

  if (!ctrl_friction_l_.init(effort_joint_interface_, nh_friction_l) ||
      !ctrl_friction_r_.init(effort_joint_interface_, nh_friction_r) ||
      !ctrl_trigger_.init(effort_joint_interface_, nh_trigger) ||
      !ctrl_magazine_.init(effort_joint_interface_, nh_magazine))
    return false;
  joint_friction_handle_.push_back(ctrl_friction_l_.joint_);
  joint_friction_handle_.push_back(ctrl_friction_r_.joint_);
  joint_trigger_handle_.push_back(ctrl_trigger_.joint_);
  return true;
}

void StandardController::push(const ros::Time &time, const ros::Duration &period) {
  ShooterBase::push(time, period);
  if (ctrl_friction_l_.joint_.getVelocity() >= enter_push_qd_coef_ * friction_qd_des_
      && ctrl_friction_l_.joint_.getVelocity() > 8.0 &&
      ctrl_friction_r_.joint_.getVelocity() <= enter_push_qd_coef_ * (-friction_qd_des_)
      && ctrl_friction_r_.joint_.getVelocity() < -8.0 &&
      (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) { // Time to shoot!!!
    if (is_out_from_block_) {
      trigger_q_des_ = last_trigger_q_des_;
      if (fabs(trigger_q_des_ - ctrl_trigger_.joint_.getPosition()) <= push_angle_error_) {
        is_out_from_block_ = false;
      }
    } else {
      trigger_q_des_ = ctrl_trigger_.joint_.getPosition() - config_.push_angle;
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
  }
  friction_qd_des_ = 0;
  moveJointFriction(time, period);
  ctrl_trigger_.joint_.setCommand(0);
}

void StandardController::moveJointFriction(const ros::Time &time, const ros::Duration &period) {
  ctrl_friction_l_.setCommand(friction_qd_des_);
  ctrl_friction_r_.setCommand(-friction_qd_des_);

  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
}

void StandardController::moveJoint(const ros::Time &time, const ros::Duration &period) {
  moveJointFriction(time, period);

  ctrl_trigger_.setCommand(trigger_q_des_);

  ctrl_trigger_.update(time, period);
}

} // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::StandardController, controller_interface::ControllerBase)
