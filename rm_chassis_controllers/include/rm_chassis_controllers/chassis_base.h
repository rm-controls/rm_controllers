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
// Created by huakang on 2021/3/21.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/filters/filters.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <rm_msgs/ChassisCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

namespace rm_chassis_controllers
{
struct Command
{
  geometry_msgs::Twist cmd_vel_;
  rm_msgs::ChassisCmd cmd_chassis_;
  ros::Time stamp_;
};

class ChassisBase : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                          rm_control::RobotStateInterface>
{
public:
  ChassisBase() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

protected:
  void raw();
  void follow(const ros::Time& time, const ros::Duration& period);
  void twist(const ros::Time& time, const ros::Duration& period);
  void gyro();
  virtual void moveJoint(const ros::Time& time, const ros::Duration& period) = 0;
  virtual geometry_msgs::Twist forwardKinematics() = 0;
  void updateOdom(const ros::Time& time, const ros::Duration& period);
  void recovery();
  void tfVelToBase(const std::string& from);
  void powerLimit();

  void cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr& msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};
  rm_control::RobotStateHandle robot_state_handle_{};

  double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, twist_angular_{}, power_coeff_{},
      power_min_vel_{}, timeout_{};
  bool enable_odom_tf_ = false;
  bool state_changed_ = true;
  enum
  {
    RAW,
    FOLLOW,
    GYRO,
    TWIST
  };
  int state_ = GYRO;
  RampFilter<double>*ramp_x_{}, *ramp_y_{}, *ramp_w_{};
  std::string follow_source_frame_{};

  ros::Time last_publish_time_;
  geometry_msgs::TransformStamped odom2base_{};
  geometry_msgs::Vector3 vel_cmd_{};   // x, y
  geometry_msgs::Vector3 vel_tfed_{};  // x, y
  control_toolbox::Pid pid_follow_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  rm_common::TfRtBroadcaster tf_broadcaster_{};
  ros::Subscriber cmd_chassis_sub_;
  ros::Subscriber cmd_vel_sub_;
  Command cmd_struct_;
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
};

}  // namespace rm_chassis_controllers
