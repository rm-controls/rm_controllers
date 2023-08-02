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
              .extra_wheel_speed = getParam(controller_nh, "extra_wheel_speed", 0.) };
  config_rt_buffer.initRT(config_);
  push_per_rotation_ = getParam(controller_nh, "push_per_rotation", 0);
  push_wheel_speed_threshold_ = getParam(controller_nh, "push_wheel_speed_threshold", 0.);

  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::ShootCmd>("command", 1, &Controller::commandCB, this);
  shoot_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::ShootState>(controller_nh, "state", 10));
  // Init dynamic reconfigure
  d_srv_ = new dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  d_srv_->setCallback(cb);

  ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
  ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  return ctrl_friction_l_.init(effort_joint_interface_, nh_friction_l) &&
         ctrl_friction_r_.init(effort_joint_interface_, nh_friction_r) &&
         ctrl_trigger_.init(effort_joint_interface_, nh_trigger);
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
           std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
               config_.exit_push_threshold))
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
  if (shoot_state_pub_->trylock())
  {
    shoot_state_pub_->msg_.stamp = time;
    shoot_state_pub_->msg_.state = state_;
    shoot_state_pub_->unlockAndPublish();
  }
  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
  ctrl_trigger_.update(time, period);
}

void Controller::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_friction_l_.setCommand(0.);
    ctrl_friction_r_.setCommand(0.);
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
  if ((cmd_.wheel_speed == 0. ||
       (ctrl_friction_l_.joint_.getVelocity() >= push_wheel_speed_threshold_ * ctrl_friction_l_.command_ &&
        ctrl_friction_l_.joint_.getVelocity() > M_PI &&
        ctrl_friction_r_.joint_.getVelocity() <= push_wheel_speed_threshold_ * ctrl_friction_r_.command_ &&
        ctrl_friction_r_.joint_.getVelocity() < -M_PI)) &&
      (time - last_shoot_time_).toSec() >= 1. / cmd_.hz)
  {  // Time to shoot!!!
    if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
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
  ctrl_friction_l_.setCommand(cmd_.wheel_speed + config_.extra_wheel_speed);
  ctrl_friction_r_.setCommand(-cmd_.wheel_speed - config_.extra_wheel_speed);
}

void Controller::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
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
                        .extra_wheel_speed = config.extra_wheel_speed };
  config_rt_buffer.writeFromNonRT(config_non_rt);
}

}  // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::Controller, controller_interface::ControllerBase)
