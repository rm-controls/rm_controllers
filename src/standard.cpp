//
// Created by huakang on 2021/1/18.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controller/standard.h"

namespace rm_shooter_controllers {

bool Block::isBlock(const ros::Time &time, const hardware_interface::JointHandle *joint_handle) {
  bool is_block_now = fabs(joint_handle->getEffort()) > block_config_.block_effort
      && fabs(joint_handle->getVelocity()) < block_config_.block_speed;
  if (is_block_now) {
    if (!is_start_block_time_) {
      block_time_ = time;
      is_start_block_time_ = true;
    }
  } else {
    block_time_ = time;
    is_start_block_time_ = false;
  }

  return (time - block_time_).toSec() >= block_config_.block_duration;
}

bool Controller::init(hardware_interface::RobotHW *robot_hw,
                      ros::NodeHandle &root_nh,
                      ros::NodeHandle &controller_nh) {
  // init config
  config_ = {.push_angle = getParam(controller_nh, "push_angle", 0.),
      .qd_10 = getParam(controller_nh, "qd_10", 0.),
      .qd_15 = getParam(controller_nh, "qd_15", 0.),
      .qd_16 = getParam(controller_nh, "qd_16", 0.),
      .qd_18 = getParam(controller_nh, "qd_18", 0.),
      .qd_30 = getParam(controller_nh, "qd_30", 0.)};
  config_rt_buffer.initRT(config_);
  block_ = new Block(controller_nh);

  enter_push_qd_coef_ = getParam(controller_nh, "enter_push_qd_coef", 0.);
  push_angle_error_ = getParam(controller_nh, "push_angle_error", 0.);
  cover_move_angle_ = getParam(controller_nh, "cover_move_angle", 0.);

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

  // init dynamic reconfigure
  d_srv_ = new dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>::CallbackType cb =
      [this](auto &&PH1, auto &&PH2) {
        reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
      };
  d_srv_->setCallback(cb);

  cmd_subscriber_ = root_nh.subscribe<rm_msgs::ShootCmd>(
      "cmd_shoot", 1, &Controller::commandCB, this);

  // init joint
  ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
  ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  ros::NodeHandle nh_cover = ros::NodeHandle(controller_nh, "cover");

  return !(!ctrl_friction_l_.init(effort_joint_interface_, nh_friction_l) ||
      !ctrl_friction_r_.init(effort_joint_interface_, nh_friction_r) ||
      !ctrl_trigger_.init(effort_joint_interface_, nh_trigger) ||
      !ctrl_cover_.init(effort_joint_interface_, nh_cover));
}

void Controller::update(const ros::Time &time, const ros::Duration &period) {
  cmd_ = *cmd_rt_buffer_.readFromRT();
  config_ = *config_rt_buffer.readFromRT();
  block_->block_config_ = *block_->block_config_rt_buffer_.readFromRT();

  if (state_ != cmd_.mode && state_ != BLOCK) {
    if (state_ == PASSIVE || state_ == STOP || state_ == PUSH)
      calibrate_trigger_pos_ = true;
    state_ = State(cmd_.mode);
    state_changed_ = true;
  }

  if (cover_state_ != cmd_.cover) {
    cover_state_ = cmd_.cover;
    cover_state_change_ = true;
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

  if (cover_state_change_) {
    cover_state_change_ = false;
    if (cover_state_) {
      cover_q_des_ = cover_move_angle_;
      ROS_INFO("[Cover] Enter OPEN");
    } else {
      cover_q_des_ = 0.0;
      ROS_INFO("[Cover] Enter CLOSE");
    }
  }

  if (state_ == PASSIVE) {
    passive();
  } else {
    if (state_ == READY)
      ready(period);
    else if (state_ == PUSH)
      push(time, period);
    else if (state_ == STOP)
      stop(time, period);
    else if (state_ == BLOCK)
      block(time, period);
    moveJoint(time, period);
  }
}

void Controller::calibrate() {
  if (calibrate_trigger_pos_) {
    double normalize_trigger_pos = fmod(ctrl_trigger_.joint_.getPosition(), 6.28);
    double calibrate_angle = fmod(normalize_trigger_pos, config_.push_angle);
    if (calibrate_angle > config_.push_angle / 2)
      calibrate_angle = calibrate_angle - config_.push_angle;
    else if (calibrate_angle <= -config_.push_angle / 2)
      calibrate_angle = calibrate_angle + config_.push_angle;
    trigger_q_des_ = ctrl_trigger_.joint_.getPosition() - calibrate_angle;
    calibrate_trigger_pos_ = false;
  }
}

void Controller::passive() {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PASSIVE");

    ctrl_friction_l_.joint_.setCommand(0);
    ctrl_friction_r_.joint_.setCommand(0);
    ctrl_trigger_.joint_.setCommand(0);
    ctrl_cover_.joint_.setCommand(0);
  }
}

void Controller::ready(const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");

    calibrate();
  }
}

void Controller::push(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");

    calibrate();
  }
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

  if (block_->isBlock(time, &ctrl_trigger_.joint_)) {
    state_ = BLOCK;
    state_changed_ = true;
    block_->is_start_block_time_ = false;
    ROS_INFO("[Shooter] Exit PUSH");
  }
}

void Controller::stop(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");
  }
  friction_qd_des_ = 0.0;
  ctrl_trigger_.joint_.setCommand(0);
}

void Controller::block(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter BLOCK");

    trigger_q_des_ = ctrl_trigger_.joint_.getPosition() + block_->block_config_.anti_block_angle;
  }
  if (fabs(trigger_q_des_ - ctrl_trigger_.joint_.getPosition()) < block_->block_config_.anti_block_error ||
      block_->isBlock(time, &ctrl_trigger_.joint_)) {
    state_ = PUSH;
    state_changed_ = true;
    block_->is_start_block_time_ = false;
    is_out_from_block_ = true;
    ROS_INFO("[Shooter] Exit BLOCK");
  }
}

void Controller::moveJoint(const ros::Time &time, const ros::Duration &period) {
  if (state_ != STOP) {
    ctrl_trigger_.setCommand(trigger_q_des_);
    ctrl_trigger_.update(time, period);
  }
  ctrl_friction_l_.setCommand(friction_qd_des_);
  ctrl_friction_r_.setCommand(-friction_qd_des_);
  ctrl_cover_.setCommand(cover_q_des_);

  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
  ctrl_cover_.update(time, period);
}

void Controller::commandCB(const rm_msgs::ShootCmdConstPtr &msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::reconfigCB(rm_shooter_controllers::ShooterConfig &config, uint32_t /*level*/) {
  ROS_INFO("[Shooter] Dynamic params change");
  if (!dynamic_reconfig_initialized_) {
    Config init_config = *config_rt_buffer.readFromNonRT(); // config init use yaml
    BlockConfig init_block_config = *block_->block_config_rt_buffer_.readFromNonRT();
    config.anti_block_angle = init_block_config.anti_block_angle;
    config.anti_block_error = init_block_config.anti_block_error;
    config.block_effort = init_block_config.block_effort;
    config.block_duration = init_block_config.block_duration;
    config.block_speed = init_block_config.block_speed;
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
      .qd_10 = config.qd_10,
      .qd_15 = config.qd_15,
      .qd_16 = config.qd_16,
      .qd_18 = config.qd_18,
      .qd_30 = config.qd_30
  };
  config_rt_buffer.writeFromNonRT(config_non_rt);

  BlockConfig block_config_non_rt{
      .block_effort = config.block_effort,
      .block_duration = config.block_duration,
      .block_speed = config.block_speed,
      .anti_block_angle = config.anti_block_angle,
      .anti_block_error = config.anti_block_error,
  };
  block_->block_config_rt_buffer_.writeFromNonRT(block_config_non_rt);
}

} // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::Controller, controller_interface::ControllerBase)
