//
// Created by huakang on 2021/1/18.
//

#include <ros_utilities.h>
#include <string>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controller/standard.h"

namespace rm_shooter_controllers {
bool ShooterStandardBaseController::init(hardware_interface::RobotHW *robot_hw,
                                         ros::NodeHandle &root_nh,
                                         ros::NodeHandle &controller_nh) {
  auto *effort_jnt_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_fiction_l_ = effort_jnt_interface->getHandle(getParam(controller_nh,
                                                              "joint_fiction_left_name",
                                                              std::string(
                                                                  "joint_fiction_left")));
  joint_fiction_r_ = effort_jnt_interface->getHandle(getParam(controller_nh,
                                                              "joint_fiction_right_name",
                                                              std::string(
                                                                  "joint_fiction_right")));
  joint_trigger_ = effort_jnt_interface->getHandle(getParam(controller_nh,
                                                            "joint_trigger_name",
                                                            std::string(
                                                                "joint_trigger")));
  robot_state_handle_ =
      robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle(
          "robot_state");

  if (!pid_fiction_l_.init(ros::NodeHandle(controller_nh, "pid_fiction_l"))
      || !pid_fiction_r_.init(ros::NodeHandle(controller_nh, "pid_fiction_r"))
      || !pid_trigger_.init(ros::NodeHandle(controller_nh, "pid_trigger")))
    return false;

  cmd_subscriber_ = root_nh.subscribe<rm_msgs::ShootCmd>("cmd_shooter",
                                                         1,
                                                         &ShooterStandardBaseController::commandCB,
                                                         this);
  ros::NodeHandle private_nh("~/shooter");
  d_srv_ =
      new dynamic_reconfigure::Server<rm_shooter_controllers::ShooterStandardConfig>(
          private_nh);
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterStandardConfig>::CallbackType
      cb =
      boost::bind(&ShooterStandardBaseController::reconfigCB, this, _1, _2);
  d_srv_->setCallback(cb);

  return true;
}

void ShooterStandardBaseController::update(const ros::Time &time,
                                           const ros::Duration &period) {
  cmd_ = *cmd_rt_buffer_.readFromRT();

  if (state_ != cmd_.mode) {
    state_ = StandardState(cmd_.mode);
    state_changed_ = true;
  }
  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == READY)
      ready(period);
    else if (state_ == PUSH)
      push(period);
  }
}

void ShooterStandardBaseController::shoot(int num, double freq) {
  shoot_num_ = num;
  shoot_freq_ = freq;
}

void ShooterStandardBaseController::setSpeed(double speed) {
  bullet_speed_ = speed;
  fric_qd_des_ = bullet_speed_ / friction_radius_;
}

void ShooterStandardBaseController::passive() {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PASSIVE");

    joint_fiction_l_.setCommand(0);
    joint_fiction_r_.setCommand(0);
    joint_trigger_.setCommand(0);
    pid_fiction_l_.reset();
    pid_fiction_r_.reset();
    pid_trigger_.reset();
  }
}

void ShooterStandardBaseController::ready(const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");
  }

//  double wheel_l_error =
//      angles::shortest_angular_distance(joint_wheel_l_.getVelocity(),
//                                        fric_qd_des_);
//  double wheel_r_error =
//      angles::shortest_angular_distance(joint_wheel_r_.getVelocity(),
//                                        fric_qd_des_);

  double wheel_l_error = fric_qd_des_ - joint_fiction_l_.getVelocity();
  double wheel_r_error = fric_qd_des_ - joint_fiction_r_.getVelocity();
  pid_fiction_l_.computeCommand(wheel_l_error, period);
  pid_fiction_r_.computeCommand(wheel_r_error, period);
  joint_fiction_l_.setCommand(pid_fiction_l_.getCurrentCmd());
  joint_fiction_r_.setCommand(pid_fiction_r_.getCurrentCmd());
  joint_trigger_.setCommand(pid_trigger_.getCurrentCmd());

  if (shoot_num_ > 0 &&
      (ros::Time::now() - last_shoot_time_).toSec()
          >= 1. / shoot_freq_) {//Time to shoot!!!
    state_ = PUSH;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit READY");
  }
}

void ShooterStandardBaseController::push(const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");

    //Turn trigger
    double trigger_des = joint_trigger_.getPosition() + push_angle_;
    //Add impulse to bullet
    // double l_ff = ff_coff_ * joint_wheel_l_.getVelocity();
    // double r_ff = ff_coff_ * joint_wheel_r_.getVelocity();

//    double wheel_l_error =
//        angles::shortest_angular_distance(joint_wheel_l_.getVelocity(),
//                                          fric_qd_des_);
//    double wheel_r_error =
//        angles::shortest_angular_distance(joint_wheel_r_.getVelocity(),
//                                          fric_qd_des_);

    double wheel_l_error = fric_qd_des_ - joint_fiction_l_.getVelocity();
    double wheel_r_error = fric_qd_des_ - joint_fiction_r_.getVelocity();
    double trigger_error =
        angles::shortest_angular_distance(joint_trigger_.getPosition(),
                                          trigger_des);
    pid_fiction_l_.computeCommand(wheel_l_error, period);
    pid_fiction_r_.computeCommand(wheel_r_error, period);
    pid_trigger_.computeCommand(trigger_error, period);
    joint_fiction_l_.setCommand(pid_fiction_l_.getCurrentCmd());
    joint_fiction_r_.setCommand(pid_fiction_r_.getCurrentCmd());
    joint_trigger_.setCommand(pid_trigger_.getCurrentCmd());

    shoot_num_--;
    last_shoot_time_ = ros::Time::now();
    push_time_ = ros::Time::now();

    state_ = READY;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit PUSH");
  }
}

void ShooterStandardBaseController::commandCB(const rm_msgs::ShootCmdConstPtr &msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
  setSpeed(cmd_.speed);
  shoot(cmd_.num, cmd_.hz);
}

void ShooterStandardBaseController::reconfigCB(const rm_shooter_controllers::ShooterStandardConfig &config,
                                               uint32_t level) {
  ROS_INFO("[Shooter] Dynamic params change");
  (void) level;
  block_coff_ = config.block_coff;
  push_angle_ = config.push_angle;
  ff_coff_ = config.ff_coff;
  ff_duration_ = config.ff_duration;
  friction_radius_ = config.friction_radius;
  setSpeed(config.bullet_speed);
}
} // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::ShooterStandardBaseController,
                       controller_interface::ControllerBase)
