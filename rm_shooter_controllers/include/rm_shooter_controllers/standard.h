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
#include <rm_msgs/ShootState.h>
#include <rm_msgs/LocalHeatState.h>

#include <utility>

namespace rm_shooter_controllers
{
struct Config
{
  double block_effort, block_speed, block_duration, block_overtime, anti_block_angle, anti_block_threshold,
      forward_push_threshold, exit_push_threshold;
  double extra_wheel_speed, wheel_speed_drop_threshold, wheel_speed_raise_threshold;
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
  void judgeBulletShoot(const ros::Time& time, const ros::Duration& period);
  void commandCB(const rm_msgs::ShootCmdConstPtr& msg)
  {
    cmd_rt_buffer_.writeFromNonRT(*msg);
  }

  void reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/);

  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  std::vector<effort_controllers::JointVelocityController*> ctrls_friction_l_, ctrls_friction_r_;
  effort_controllers::JointPositionController ctrl_trigger_;
  std::vector<double> wheel_speed_offset_l_, wheel_speed_offset_r_;
  int push_per_rotation_{}, count_{};
  double push_wheel_speed_threshold_{};
  double freq_threshold_{};
  bool dynamic_reconfig_initialized_ = false;
  bool state_changed_ = false;
  bool maybe_block_ = false;
  double friction_block_effort_{}, friction_block_vel_{};
  double anti_friction_block_duty_cycle_{}, anti_friction_block_vel_{};
  bool has_shoot_ = false, has_shoot_last_ = false;
  bool wheel_speed_raise_ = true, wheel_speed_drop_ = true;
  double last_wheel_speed_{};

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
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LocalHeatState>> local_heat_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::ShootState>> shoot_state_pub_;
  ros::Subscriber cmd_subscriber_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>* d_srv_{};
};

}  // namespace rm_shooter_controllers
