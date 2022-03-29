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

#pragma once

#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/ros_utilities.h>
#include <realtime_tools/realtime_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rm_shooter_controllers/ShooterConfig.h>
#include <rm_msgs/ShootCmd.h>

#include <utility>

namespace rm_shooter_controllers
{
struct Config
{
  double block_effort, block_speed, block_duration, block_overtime, anti_block_angle, anti_block_threshold;
  double qd_10, qd_15, qd_16, qd_18, qd_30, lf_extra_rotat_speed;
};

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                         rm_control::RobotStateInterface>
{
public:
  Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& /*time*/) override;

private:
  void stop(const ros::Time& time, const ros::Duration& period);
  void ready(const ros::Duration& period);
  void push(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  void setSpeed(const rm_msgs::ShootCmd& cmd);
  void normalize();
  void commandCB(const rm_msgs::ShootCmdConstPtr& msg)
  {
    cmd_rt_buffer_.writeFromNonRT(*msg);
  }
  void reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/);

  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
  effort_controllers::JointPositionController ctrl_trigger_;
  int push_per_rotation_{};
  double push_qd_threshold_{};
  bool dynamic_reconfig_initialized_ = false;
  bool state_changed_ = false;
  bool maybe_block_ = false;

  ros::Time last_shoot_time_, block_time_, last_block_time_;
  enum
  {
    STOP,
    READY,
    PUSH,
    BLOCK
  };
  int state_ = STOP;
  Config config_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer;
  realtime_tools::RealtimeBuffer<rm_msgs::ShootCmd> cmd_rt_buffer_;
  rm_msgs::ShootCmd cmd_;
  ros::Subscriber cmd_subscriber_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>* d_srv_{};
};

}  // namespace rm_shooter_controllers
