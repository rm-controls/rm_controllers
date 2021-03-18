//
// Created by huakang on 2021/1/18.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controller/standard.h"

namespace rm_shooter_controllers {
bool ShooterStandardController::init(hardware_interface::RobotHW *robot_hw,
                                     ros::NodeHandle &root_nh,
                                     ros::NodeHandle &controller_nh) {
  // init joint
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_fiction_l_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_fiction_left_name", std::string("joint_fiction_left")));
  joint_fiction_r_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_fiction_right_name", std::string("joint_fiction_right")));
  joint_trigger_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_trigger_name", std::string("joint_trigger")));

  // init config
  config_ = {.push_angle = getParam(controller_nh, "push_angle", 0.),
      .block_effort = getParam(controller_nh, "block_effort", 0.),
      .block_duration = getParam(controller_nh, "block_duration", 0.),
      .block_speed = getParam(controller_nh, "block_speed", 0.),
      .anti_block_angle = getParam(controller_nh, "anti_block_angle", 0.),
      .anti_block_error = getParam(controller_nh, "anti_block_error", 0.),
      .qd_10 = getParam(controller_nh, "qd_10", 0.),
      .qd_15 = getParam(controller_nh, "qd_15", 0.),
      .qd_16 = getParam(controller_nh, "qd_16", 0.),
      .qd_18 = getParam(controller_nh, "qd_18", 0.),
      .qd_30 = getParam(controller_nh, "qd_30", 0.)};
  config_rt_buffer.initRT(config_);

  // init dynamic reconfigure
  d_srv_ = new dynamic_reconfigure::Server<rm_shooter_controllers::ShooterStandardConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterStandardConfig>::CallbackType cb =
      [this](auto &&PH1, auto &&PH2) {
        reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
      };
  d_srv_->setCallback(cb);

  if (!pid_fiction_l_.init(ros::NodeHandle(controller_nh, "pid_fiction_l"))
      || !pid_fiction_r_.init(ros::NodeHandle(controller_nh, "pid_fiction_r"))
      || !pid_trigger_.init(ros::NodeHandle(controller_nh, "pid_trigger")))
    return false;

  cmd_subscriber_ = root_nh.subscribe<rm_msgs::ShootCmd>(
      "cmd_shoot", 1, &ShooterStandardController::commandCB, this);
  return true;
}

void ShooterStandardController::update(const ros::Time &time,
                                       const ros::Duration &period) {
  cmd_ = *cmd_rt_buffer_.readFromRT();
  config_ = *config_rt_buffer.readFromRT();

  if (state_ != cmd_.mode && state_ != BLOCK) {
    state_ = State(cmd_.mode);
    state_changed_ = true;
  }

  if (cmd_.speed == cmd_.SPEED_10M_PER_SECOND)
    friction_qd_des_ = config_.qd_10;
  else if (cmd_.speed == cmd_.SPEED_15M_PER_SECOND)
    friction_qd_des_ = config_.qd_15;
  else if (cmd_.speed == cmd_.SPEED_16M_PER_SECOND)
    friction_qd_des_ = config_.qd_16;
  else if (cmd_.speed == cmd_.SPEED_18M_PER_SECOND)
    friction_qd_des_ = config_.qd_18;
  else if (cmd_.speed == cmd_.SPEED_30M_PER_SECOND)
    friction_qd_des_ = config_.qd_30;
  else
    friction_qd_des_ = 0.;

  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == READY)
      ready(period);
    else if (state_ == PUSH)
      push(time, period);
    else if (state_ == BLOCK)
      block(time, period);
    moveJoint(period);
  }
}

void ShooterStandardController::passive() {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PASSIVE");

    joint_fiction_l_.setCommand(0);
    joint_fiction_r_.setCommand(0);
    joint_trigger_.setCommand(0);
  }
}

void ShooterStandardController::ready(const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");
    trigger_q_des_ = joint_trigger_.getPosition();
    pid_fiction_l_.reset();
    pid_fiction_r_.reset();
  }
}

void ShooterStandardController::push(const ros::Time &time,
                                     const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");

    trigger_q_des_ = joint_trigger_.getPosition();
    pid_trigger_.reset();
  }

  if (joint_fiction_l_.getVelocity() >= 0.95 * friction_qd_des_ && joint_fiction_l_.getVelocity() > 8.0 &&
      joint_fiction_r_.getVelocity() <= 0.95 * (-friction_qd_des_) && joint_fiction_r_.getVelocity() < -8.0 &&
      (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) { // Time to shoot!!!
    trigger_q_des_ = joint_trigger_.getPosition() - config_.push_angle;
    last_shoot_time_ = time;
  } else
    ROS_DEBUG("[Shooter] wait for friction wheel");

  bool is_block_now =
      fabs(joint_trigger_.getEffort()) > config_.block_effort && joint_trigger_.getVelocity() > (-config_.block_speed);
  if (is_block_now) {
    if (!is_start_block_time_) {
      block_time_ = time;
      is_start_block_time_ = true;
    }
  } else {
    block_time_ = time;
    is_start_block_time_ = false;
  }

  if ((time - block_time_).toSec() >= config_.block_duration) {
    state_ = BLOCK;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit PUSH");
  }
}

void ShooterStandardController::block(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter BLOCK");

    trigger_q_des_ = joint_trigger_.getPosition() + config_.anti_block_angle;
  }
  if (fabs(trigger_q_des_ - joint_trigger_.getPosition()) < config_.anti_block_error) {
    state_ = PUSH;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit BLOCK");
  }
}

void ShooterStandardController::commandCB(const rm_msgs::ShootCmdConstPtr &msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void ShooterStandardController::reconfigCB(rm_shooter_controllers::ShooterStandardConfig &config,
                                           uint32_t /*level*/) {

  ROS_INFO("[Shooter] Dynamic params change");
  if (!dynamic_reconfig_initialized_) {
    Config init_config = *config_rt_buffer.readFromNonRT(); // config init use yaml
    config.anti_block_angle = init_config.anti_block_angle;
    config.anti_block_error = init_config.anti_block_error;
    config.block_effort = init_config.block_effort;
    config.block_duration = init_config.block_duration;
    config.block_speed = init_config.block_speed;
    config.push_angle = init_config.push_angle;
    config.qd_10 = init_config.qd_10;
    config.qd_15 = init_config.qd_15;
    config.qd_16 = init_config.qd_16;
    config.qd_18 = init_config.qd_18;
    config.qd_30 = init_config.qd_30;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{
      .push_angle = config.push_angle,
      .block_effort = config.block_effort,
      .block_duration = config.block_duration,
      .block_speed = config.block_speed,
      .anti_block_angle = config.anti_block_angle,
      .anti_block_error = config.anti_block_error,
      .qd_10 = config.qd_10,
      .qd_15 = config.qd_15,
      .qd_16 = config.qd_16,
      .qd_18 = config.qd_18,
      .qd_30 = config.qd_30
  };
  config_rt_buffer.writeFromNonRT(config_non_rt);
}

void ShooterStandardController::moveJoint(const ros::Duration &period) {
  double friction_l_error = friction_qd_des_ - joint_fiction_l_.getVelocity();
  double friction_r_error = -friction_qd_des_ - joint_fiction_r_.getVelocity();
  double trigger_error = trigger_q_des_ - joint_trigger_.getPosition();

  pid_fiction_l_.computeCommand(friction_l_error, period);
  pid_fiction_r_.computeCommand(friction_r_error, period);
  pid_trigger_.computeCommand(trigger_error, period);
  joint_fiction_l_.setCommand(pid_fiction_l_.getCurrentCmd());
  joint_fiction_r_.setCommand(pid_fiction_r_.getCurrentCmd());
  joint_trigger_.setCommand(pid_trigger_.getCurrentCmd());
}

} // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::ShooterStandardController, controller_interface::ControllerBase)
