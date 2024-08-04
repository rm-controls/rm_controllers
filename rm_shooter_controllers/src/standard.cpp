/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by huakang on 2021/1/18.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controllers/standard.h"

namespace rm_shooter_controllers
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  config_ = { .block_effort = getParam(controller_nh, "block_effort", 0.),
              .block_speed = getParam(controller_nh, "block_speed", 0.),
              .block_duration = getParam(controller_nh, "block_duration", 0.),
              .block_overtime = getParam(controller_nh, "block_overtime", 0.),
              .anti_block_angle = getParam(controller_nh, "anti_block_angle", 0.),
              .anti_block_threshold = getParam(controller_nh, "anti_block_threshold", 0.),
              .forward_push_threshold = getParam(controller_nh, "forward_push_threshold", 0.1),
              .exit_push_threshold = getParam(controller_nh, "exit_push_threshold", 0.1),
              .extra_wheel_speed = getParam(controller_nh, "extra_wheel_speed", 0.),
              .wheel_speed_drop_threshold = getParam(controller_nh, "wheel_speed_drop_threshold", 10.),
              .wheel_speed_raise_threshold = getParam(controller_nh, "wheel_speed_raise_threshold", 3.1) };
  config_rt_buffer.initRT(config_);
  push_per_rotation_ = getParam(controller_nh, "push_per_rotation", 0);
  push_wheel_speed_threshold_ = getParam(controller_nh, "push_wheel_speed_threshold", 0.);
  freq_threshold_ = getParam(controller_nh, "freq_threshold", 20.);
  anti_friction_block_duty_cycle_ = getParam(controller_nh, "anti_friction_block_duty_cycle", 0.5);
  anti_friction_block_vel_ = getParam(controller_nh, "anti_friction_block_vel", 810.0);
  friction_block_effort_ = getParam(controller_nh, "friction_block_effort", 0.2);
  friction_block_vel_ = getParam(controller_nh, "friction_block_vel", 1.0);

  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::ShootCmd>("command", 1, &Controller::commandCB, this);
  local_heat_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::LocalHeatState>(
      controller_nh, "/local_heat_state/shooter_state", 10));
  shoot_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::ShootState>(controller_nh, "state", 10));
  // Init dynamic reconfigure
  d_srv_ = new dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  d_srv_->setCallback(cb);

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  XmlRpc::XmlRpcValue friction_left, friction_right;
  controller_nh.getParam("friction_left", friction_left);
  controller_nh.getParam("friction_right", friction_right);
  double wheel_speed_offset;
  for (const auto& it : friction_left)
  {
    ros::NodeHandle nh = ros::NodeHandle(controller_nh, "friction_left/" + it.first);
    wheel_speed_offset_l_.push_back(nh.getParam("wheel_speed_offset", wheel_speed_offset) ? wheel_speed_offset : 0.);
    effort_controllers::JointVelocityController* ctrl_friction_l = new effort_controllers::JointVelocityController;
    if (ctrl_friction_l->init(effort_joint_interface_, nh))
      ctrls_friction_l_.push_back(ctrl_friction_l);
    else
      return false;
  }
  for (const auto& it : friction_right)
  {
    ros::NodeHandle nh = ros::NodeHandle(controller_nh, "friction_right/" + it.first);
    wheel_speed_offset_r_.push_back(nh.getParam("wheel_speed_offset", wheel_speed_offset) ? wheel_speed_offset : 0.);
    effort_controllers::JointVelocityController* ctrl_friction_r = new effort_controllers::JointVelocityController;
    if (ctrl_friction_r->init(effort_joint_interface_, nh))
      ctrls_friction_r_.push_back(ctrl_friction_r);
    else
      return false;
  }

  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  return ctrl_trigger_.init(effort_joint_interface_, nh_trigger);
}

void Controller::starting(const ros::Time& /*time*/)
{
  state_ = STOP;
  state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_ = *cmd_rt_buffer_.readFromRT();
  config_ = *config_rt_buffer.readFromRT();
  if (state_ != cmd_.mode)
  {
    if (state_ != BLOCK)
      if ((state_ != PUSH || cmd_.mode != READY) ||
          (cmd_.mode == READY &&
           (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
                config_.exit_push_threshold ||
            cmd_.hz >= freq_threshold_)))
      {
        state_ = cmd_.mode;
        state_changed_ = true;
      }
  }

  if (state_ != STOP)
    setSpeed(cmd_);
  switch (state_)
  {
    case READY:
      ready(period);
      break;
    case PUSH:
      push(time, period);
      break;
    case STOP:
      stop(time, period);
      break;
    case BLOCK:
      block(time, period);
      break;
  }
  judgeBulletShoot(time, period);
  if (shoot_state_pub_->trylock())
  {
    shoot_state_pub_->msg_.stamp = time;
    shoot_state_pub_->msg_.state = state_;
    shoot_state_pub_->unlockAndPublish();
  }
  for (auto& ctrl_friction_l : ctrls_friction_l_)
    ctrl_friction_l->update(time, period);
  for (auto& ctrl_friction_r : ctrls_friction_r_)
    ctrl_friction_r->update(time, period);
  ctrl_trigger_.update(time, period);
}

void Controller::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    for (auto& ctrl_friction_l : ctrls_friction_l_)
      ctrl_friction_l->setCommand(0.);
    for (auto& ctrl_friction_r : ctrls_friction_r_)
      ctrl_friction_r->setCommand(0.);
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
  }
}

void Controller::ready(const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");

    normalize();
  }
}

void Controller::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  bool wheel_speed_ready = true;
  for (auto& ctrl_friction_l : ctrls_friction_l_)
  {
    if (ctrl_friction_l->joint_.getVelocity() < push_wheel_speed_threshold_ * ctrl_friction_l->command_ ||
        ctrl_friction_l->joint_.getVelocity() <= M_PI)
      wheel_speed_ready = false;
  }
  for (auto& ctrl_friction_r : ctrls_friction_r_)
  {
    if (ctrl_friction_r->joint_.getVelocity() > push_wheel_speed_threshold_ * ctrl_friction_r->command_ ||
        ctrl_friction_r->joint_.getVelocity() >= -M_PI)
      wheel_speed_ready = false;
  }
  if ((cmd_.wheel_speed == 0. || wheel_speed_ready) && (time - last_shoot_time_).toSec() >= 1. / cmd_.hz)
  {  // Time to shoot!!!
    if (cmd_.hz >= freq_threshold_)
    {
      ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                                   2. * M_PI / static_cast<double>(push_per_rotation_),
                               -1 * cmd_.hz * 2. * M_PI / static_cast<double>(push_per_rotation_));
      last_shoot_time_ = time;
    }
    else if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
             config_.forward_push_threshold)
    {
      ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                               2. * M_PI / static_cast<double>(push_per_rotation_));
      last_shoot_time_ = time;
    }
    // Check block
    if ((ctrl_trigger_.joint_.getEffort() < -config_.block_effort &&
         std::abs(ctrl_trigger_.joint_.getVelocity()) < config_.block_speed) ||
        ((time - last_shoot_time_).toSec() > 1 / cmd_.hz &&
         std::abs(ctrl_trigger_.joint_.getVelocity()) < config_.block_speed))
    {
      if (!maybe_block_)
      {
        block_time_ = time;
        maybe_block_ = true;
      }
      if ((time - block_time_).toSec() >= config_.block_duration)
      {
        state_ = BLOCK;
        state_changed_ = true;
        ROS_INFO("[Shooter] Exit PUSH");
      }
    }
    else
      maybe_block_ = false;
  }
  else
    ROS_DEBUG("[Shooter] Wait for friction wheel");
}

void Controller::block(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter BLOCK");
    last_block_time_ = time;
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition() + config_.anti_block_angle);
  }
  if (std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.joint_.getPosition()) <
          config_.anti_block_threshold ||
      (time - last_block_time_).toSec() > config_.block_overtime)
  {
    normalize();
    state_ = PUSH;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit BLOCK");
  }
}

void Controller::setSpeed(const rm_msgs::ShootCmd& cmd)
{
  static int friction_block_count = 0;
  bool friction_wheel_block = false;
  for (auto& ctrl_friction_l : ctrls_friction_l_)
  {
    if (ctrl_friction_l->joint_.getVelocity() <= friction_block_vel_ &&
        abs(ctrl_friction_l->joint_.getEffort()) >= friction_block_effort_ && cmd.wheel_speed != 0)
      friction_wheel_block = true;
  }
  for (auto& ctrl_friction_r : ctrls_friction_r_)
  {
    if (ctrl_friction_r->joint_.getVelocity() >= -1.0 * friction_block_vel_ &&
        abs(ctrl_friction_r->joint_.getEffort()) >= friction_block_effort_ && cmd.wheel_speed != 0)
      friction_wheel_block = true;
  }
  if (!friction_wheel_block)
  {
    for (size_t i = 0; i < ctrls_friction_l_.size(); i++)
      ctrls_friction_l_[i]->setCommand(cmd_.wheel_speed + config_.extra_wheel_speed + wheel_speed_offset_l_[i]);
    for (size_t i = 0; i < ctrls_friction_r_.size(); i++)
      ctrls_friction_r_[i]->setCommand(-cmd_.wheel_speed - config_.extra_wheel_speed - wheel_speed_offset_r_[i]);
  }
  else
  {
    double command = (friction_block_count <= static_cast<int>(anti_friction_block_duty_cycle_ * 1000)) ?
                         anti_friction_block_vel_ :
                         0.;
    for (auto& ctrl_friction_l : ctrls_friction_l_)
    {
      ctrl_friction_l->setCommand(command);
    }
    for (auto& ctrl_friction_r : ctrls_friction_r_)
    {
      ctrl_friction_r->setCommand(command);
    }
    friction_block_count = (friction_block_count + 1) % 1000;
  }
}

void Controller::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(
      push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01 + config_.exit_push_threshold) / push_angle));
}

void Controller::judgeBulletShoot(const ros::Time& time, const ros::Duration& period)
{
  if (state_ != STOP)
  {
    if (abs(ctrls_friction_l_[0]->joint_.getVelocity()) - last_wheel_speed_ > config_.wheel_speed_raise_threshold &&
        wheel_speed_drop_)
    {
      wheel_speed_raise_ = true;
      wheel_speed_drop_ = false;
    }

    if (last_wheel_speed_ - abs(ctrls_friction_l_[0]->joint_.getVelocity()) > config_.wheel_speed_drop_threshold &&
        abs(ctrls_friction_l_[0]->joint_.getVelocity()) > 300. && wheel_speed_raise_)
    {
      wheel_speed_drop_ = true;
      wheel_speed_raise_ = false;
      has_shoot_ = true;
    }
  }
  double friction_change_vel = abs(ctrls_friction_l_[0]->joint_.getVelocity()) - last_wheel_speed_;
  last_wheel_speed_ = abs(ctrls_friction_l_[0]->joint_.getVelocity());
  count_++;
  if (has_shoot_last_)
  {
    has_shoot_ = true;
  }
  has_shoot_last_ = has_shoot_;
  if (count_ == 2)
  {
    if (local_heat_state_pub_->trylock())
    {
      local_heat_state_pub_->msg_.stamp = time;
      local_heat_state_pub_->msg_.has_shoot = has_shoot_;
      local_heat_state_pub_->msg_.friction_change_vel = friction_change_vel;
      local_heat_state_pub_->unlockAndPublish();
    }
    has_shoot_last_ = false;
    count_ = 0;
  }
  if (has_shoot_)
    has_shoot_ = false;
}
void Controller::reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/)
{
  ROS_INFO("[Shooter] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer.readFromNonRT();  // config init use yaml
    config.block_effort = init_config.block_effort;
    config.block_speed = init_config.block_speed;
    config.block_duration = init_config.block_duration;
    config.block_overtime = init_config.block_overtime;
    config.anti_block_angle = init_config.anti_block_angle;
    config.anti_block_threshold = init_config.anti_block_threshold;
    config.forward_push_threshold = init_config.forward_push_threshold;
    config.exit_push_threshold = init_config.exit_push_threshold;
    config.extra_wheel_speed = init_config.extra_wheel_speed;
    config.wheel_speed_drop_threshold = init_config.wheel_speed_drop_threshold;
    config.wheel_speed_raise_threshold = init_config.wheel_speed_raise_threshold;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .block_effort = config.block_effort,
                        .block_speed = config.block_speed,
                        .block_duration = config.block_duration,
                        .block_overtime = config.block_overtime,
                        .anti_block_angle = config.anti_block_angle,
                        .anti_block_threshold = config.anti_block_threshold,
                        .forward_push_threshold = config.forward_push_threshold,
                        .exit_push_threshold = config.exit_push_threshold,
                        .extra_wheel_speed = config.extra_wheel_speed,
                        .wheel_speed_drop_threshold = config.wheel_speed_drop_threshold,
                        .wheel_speed_raise_threshold = config.wheel_speed_raise_threshold };
  config_rt_buffer.writeFromNonRT(config_non_rt);
}

}  // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::Controller, controller_interface::ControllerBase)
