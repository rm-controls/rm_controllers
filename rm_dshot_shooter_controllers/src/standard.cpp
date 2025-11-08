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
// Created by xie on 2025/11/1.
//

#include <rm_common/ros_utilities.h>
#include <memory>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include "ros/console.h"
#include "rm_dshot_shooter_controllers/standard.h"

namespace rm_dshot_shooter_controllers
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
  friction_block_vel_ = getParam(controller_nh, "friction_block_vel", 1.0);

  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::ShootCmd>("command", 1, &Controller::commandCB, this);
  local_heat_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::LocalHeatState>(
      controller_nh, "/local_heat_state/shooter_state", 10));
  shoot_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::ShootState>(controller_nh, "state", 10));
  // Init dynamic reconfigure
  d_srv_ = new dynamic_reconfigure::Server<rm_dshot_shooter_controllers::ShooterConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_dshot_shooter_controllers::ShooterConfig>::CallbackType cb = [this](auto&& PH1,
                                                                                                     auto&& PH2) {
    reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  d_srv_->setCallback(cb);

  XmlRpc::XmlRpcValue friction;
  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
  controller_nh.getParam("friction", friction);
  for (const auto& its : friction)
  {
    std::vector<double> wheel_speed_offset_temp;
    std::vector<velocity_controllers::JointVelocityController*> ctrl_frictions;
    std::vector<std::shared_ptr<control_toolbox::Pid>> pid_controllers_temp;
    std::vector<std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>>> state_pubs;
    for (const auto& it : its.second)
    {
      ros::NodeHandle nh = ros::NodeHandle(controller_nh, "friction/" + its.first + "/" + it.first);
      auto* ctrl_friction = new velocity_controllers::JointVelocityController;
      if (ctrl_friction->init(velocity_joint_interface_, nh))
      {
        ctrl_frictions.push_back(ctrl_friction);
      }
      else
      {
        return false;
      }
      auto pid = std::make_shared<control_toolbox::Pid>();
      ros::NodeHandle pid_nh(nh, "pid");
      if (!pid->init(pid_nh, false))
      {
        ROS_ERROR_STREAM("Failed to initialize PID for " << it.first);
        return false;
      }
      pid_controllers_temp.push_back(pid);
      std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> state_pub;
      state_pub =
          std::make_unique<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>>(nh, "state", 1);
      state_pubs.push_back(std::move(state_pub));
    }
    friction_state_publishers_.push_back(std::move(state_pubs));
    ctrls_friction_.push_back(ctrl_frictions);
    friction_pid_controllers_.push_back(pid_controllers_temp);
  }
  lp_filter_ = new LowPassFilter(controller_nh);
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
        if (state_ == STOP && cmd_.mode == READY)
          enter_ready_ = true;
        state_ = cmd_.mode;
        state_changed_ = true;
      }
  }

  if (state_ != STOP)
    setSpeed(cmd_, period);
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
  ctrl_trigger_.update(time, period);
}

void Controller::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");
    for (auto& ctrl_frictions : ctrls_friction_)
    {
      for (auto& ctrl_friction : ctrl_frictions)
      {
        ctrl_friction->joint_.setCommand(0.);
      }
    }
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
  for (size_t i = 0; i < ctrls_friction_.size(); i++)
  {
    for (size_t j = 0; j < ctrls_friction_[i].size(); j++)
    {
      if (ctrls_friction_[i][j]->joint_.getVelocity() <= M_PI)
        wheel_speed_ready = false;
    }
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
    ROS_INFO("[Shooter] Trigger Enter BLOCK");
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
    ROS_INFO("[Shooter] Trigger Exit BLOCK");
  }
}

void Controller::setSpeed(const rm_msgs::ShootCmd& cmd, const ros::Duration& period)
{
  for (size_t i = 0; i < ctrls_friction_.size(); i++)
  {
    for (size_t j = 0; j < ctrls_friction_[i].size(); j++)
    {
      double target_speed = 0.0;
      // Used to distinguish the front and rear friction wheels.
      if (j == 0)
      {
        target_speed = cmd_.wheel_speed + config_.extra_wheel_speed + cmd_.wheels_speed_offset_back;
      }
      else if (j == 1)
      {
        target_speed = cmd_.wheel_speed + config_.extra_wheel_speed + cmd_.wheels_speed_offset_front;
      }
      double current_speed = ctrls_friction_[i][j]->joint_.getVelocity();
      double error = target_speed - current_speed;
      double command = friction_pid_controllers_[i][j]->computeCommand(error, period);
      double final_command = target_speed + command;
      ctrls_friction_[i][j]->joint_.setCommand(final_command);
      if (loop_count_ % 10 == 0)
      {
        if (friction_state_publishers_[i][j] && friction_state_publishers_[i][j]->trylock())
        {
          friction_state_publishers_[i][j]->msg_.header.stamp = ros::Time::now();
          friction_state_publishers_[i][j]->msg_.set_point = target_speed;
          friction_state_publishers_[i][j]->msg_.process_value = current_speed;
          friction_state_publishers_[i][j]->msg_.error = error;
          friction_state_publishers_[i][j]->msg_.time_step = period.toSec();
          friction_state_publishers_[i][j]->msg_.command = final_command;

          double p, i_gain, d, i_clamp, dummy;
          bool antiwindup;
          friction_pid_controllers_[i][j]->getGains(p, i_gain, d, i_clamp, dummy, antiwindup);
          friction_state_publishers_[i][j]->msg_.p = p;
          friction_state_publishers_[i][j]->msg_.i = i_gain;
          friction_state_publishers_[i][j]->msg_.d = d;
          friction_state_publishers_[i][j]->msg_.i_clamp = i_clamp;
          friction_state_publishers_[i][j]->msg_.antiwindup = static_cast<char>(antiwindup);

          friction_state_publishers_[i][j]->unlockAndPublish();
        }
      }
    }
  }
  loop_count_++;
}

void Controller::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  if (cmd_.hz <= freq_threshold_)
  {
    ctrl_trigger_.setCommand(
        push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01 + config_.exit_push_threshold) / push_angle));
  }
  else if (enter_ready_)
  {
    ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
    enter_ready_ = false;
  }
  else
    ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() - 0.01) / push_angle));
}

void Controller::judgeBulletShoot(const ros::Time& time, const ros::Duration& period)
{
  lp_filter_->input(ctrls_friction_[0][0]->joint_.getVelocity());
  double friction_speed = lp_filter_->output();
  double friction_change_speed = abs(friction_speed) - last_fricition_speed_;
  double friction_change_speed_derivative = friction_change_speed - last_friction_change_speed_;
  if (state_ != STOP)
  {
    if (friction_change_speed_derivative > 0 && has_shoot_)
      has_shoot_ = false;
    if (friction_change_speed < -config_.wheel_speed_drop_threshold && !has_shoot_ &&
        friction_change_speed_derivative < 0)
      has_shoot_ = true;
  }
  last_fricition_speed_ = abs(friction_speed);
  last_friction_change_speed_ = friction_change_speed;

  if (local_heat_state_pub_->trylock())
  {
    local_heat_state_pub_->msg_.stamp = time;
    local_heat_state_pub_->msg_.has_shoot = has_shoot_;
    local_heat_state_pub_->msg_.friction_speed = friction_speed;
    local_heat_state_pub_->msg_.friction_change_speed = friction_change_speed;
    local_heat_state_pub_->msg_.friction_change_speed_derivative = friction_change_speed_derivative;
    local_heat_state_pub_->unlockAndPublish();
  }
}
void Controller::reconfigCB(rm_dshot_shooter_controllers::ShooterConfig& config, uint32_t /*level*/)
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

}  // namespace rm_dshot_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_dshot_shooter_controllers::Controller, controller_interface::ControllerBase)
