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
// Created by qiayuan on 8/14/20.
//

#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <rm_gimbal_controllers/BulletSolverConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/eigen_types.h>
#include <rm_common/ros_utilities.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/ShootBeforehandCmd.h>

namespace rm_gimbal_controllers
{
struct Config
{
  double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
      resistance_coff_qd_30, g, delay, wait_next_armor_delay, wait_diagonal_armor_delay, dt, timeout,
      ban_shoot_duration, gimbal_switch_duration, ban_shoot_duration2, gimbal_switch_duration2, max_switch_angle,
      min_switch_angle, min_shoot_beforehand_vel, min_vel_track_diagonal_armor, max_chassis_angular_vel,
      track_rotate_target_delay, track_move_target_delay;
  int min_fit_switch_count;
};

class BulletSolver
{
public:
  explicit BulletSolver(ros::NodeHandle& controller_nh);

  bool solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw, double v_yaw,
             double r1, double r2, double dz, int armors_num, double chassis_angular_vel_z);
  double getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r1,
                        double r2, double dz, int armors_num, double yaw_real, double pitch_real, double bullet_speed);
  double getResistanceCoefficient(double bullet_speed) const;
  double getYaw() const
  {
    return output_yaw_;
  }
  double getPitch() const
  {
    return -output_pitch_;
  }
  void getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                 geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
                                 double r1, double r2, double dz, int armors_num);
  void judgeShootBeforehand(const ros::Time& time, double v_yaw);
  void bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time);
  void identifiedTargetChangeCB(const std_msgs::BoolConstPtr& msg);
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  ~BulletSolver() = default;

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_real_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::ShootBeforehandCmd>> shoot_beforehand_cmd_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> fly_time_pub_;
  ros::Subscriber identified_target_change_sub_;
  ros::Time switch_armor_time_{};
  ros::Time last_track_diagonal_armor_time_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  Config config_{};
  double max_track_target_vel_;
  double output_yaw_{}, output_pitch_{};
  double bullet_speed_{}, resistance_coff_{};
  double fly_time_;
  int shoot_beforehand_cmd_{};
  int selected_armor_;
  int count_;
  bool track_target_;
  bool identified_target_change_ = true;
  bool is_track_diagonal_armor_ = false;
  bool is_in_delay_before_switch_{};
  bool dynamic_reconfig_initialized_{};

  geometry_msgs::Point target_pos_{};
  visualization_msgs::Marker marker_desire_;
  visualization_msgs::Marker marker_real_;

  ros::NodeHandle nh;
  rm_msgs::TrackData msg;
  ros::Publisher pub = nh.advertise<rm_msgs::TrackData>("/test", 10);
};
}  // namespace rm_gimbal_controllers
