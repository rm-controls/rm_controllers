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
#include <rm_msgs/BulletSolverData.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/eigen_types.h>
#include <rm_common/ros_utilities.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/ShootBeforehandCmd.h>
#include <rm_msgs/LocalHeatState.h>

namespace rm_gimbal_controllers
{
struct Config
{
  double resistance_coff_qd_1, resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16,
      resistance_coff_qd_18, resistance_coff_qd_30, resistance_coff_qd_800, g, delay, outpost_delay, center_delay,
      max_switch_angle, switch_angle_offset, outpost_switch_angle_offset, min_shoot_beforehand_vel,
      track_rotate_target_delay, track_move_target_delay, yaw_max_acc, track_rotate_outpost_delay;
  int min_fit_switch_count, traject_start_fit_;
  double traject_ahead_;
  int clean_shoot_num_;
  double end_pos_offset;
  double traject_k_effort, traject_k_vel_;
};
struct TrajectoryFunctionCoefficients
{
  double a0, a1, a2, a3;
};
struct TrajectoryLimitParams
{
  double start_pos, end_pos, start_vel, end_vel;
};

class BulletSolver
{
public:
  explicit BulletSolver(ros::NodeHandle& controller_nh);

  bool solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw, double v_yaw,
             double r1, double r2, double dz, double armors_num, double start_vel, double track_id);
  double getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r1,
                        double r2, double dz, double armors_num, double yaw_real, double pitch_real,
                        double bullet_speed);
  double getResistanceCoefficient(double target_distance) const;
  double getYaw() const
  {
    return output_yaw_[0];
  }
  double getPitch() const
  {
    return -output_pitch_[0];
  }
  double getTrajectYaw() const
  {
    return traject_output_yaw_;
  }
  double getTrajectVel() const
  {
    return traject_vel_;
  }
  bool getUsingtraject() const
  {
    return using_traject_;
  }
  int getShootnum() const
  {
    return shoot_num_;
  }
  double getTrajectEffortff() const
  {
    return traject_effort_ff_;
  }
  bool getTrackTarget() const
  {
    return track_target_;
  }
  void CleanTrackCount()
  {
    track_count_ = 0;
  }
  double getFlyTime()
  {
    return fly_time_[0];
  }
  void getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                 geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
                                 double r1, double r2, double dz, double armors_num);
  uint8_t judgeShootBeforehand(double v_yaw, int id);
  void bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time);
  void identifiedTargetChangeCB(const std_msgs::BoolConstPtr& msg);
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  double planningPoint(ros::Time& time, ros::Time& start_trajectory_time_, double v_yaw);
  void heatCB(const rm_msgs::LocalHeatStateConstPtr& msg);
  ~BulletSolver() = default;

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_real_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> fly_time_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::BulletSolverData>> bullet_solver_pub;
  ros::Subscriber identified_target_change_sub_;
  ros::Subscriber shoot_state_sub_;
  ros::Time switch_armor_time_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  Config config_{};
  double yaw_[150], pos_x[150], pos_y[150];
  double max_track_target_vel_{};
  double output_yaw_[150], output_pitch_[150], traject_output_yaw_{};
  double bullet_speed_{}, resistance_coff_{};
  double fly_time_[150];
  double switch_hysteresis_{};
  double last_yaw_{}, filtered_yaw_{};
  double gimbal_switch_duration_{};
  double yaw_subtract_{};
  double switch_armor_angle{};
  double filtered_v_yaw_{};
  double switchtime{};
  double traject_effort_ff_{};
  double traject_switch_time_{};
  double switche_time_yaw_{};
  double traject_max_acc_{};
  double last_output_yaw_{};
  double traject_vel_{};
  double r_traject_{};
  double intital_yaw_{};

  int shoot_num_{};
  int shoot_beforehand_cmd_{};
  int count_[150]{};
  int next_count_[150]{};
  int ban_shoot_count_{};
  int selected_armor_[150] = {};
  int last_selected_armor_ = {};
  bool track_target_ = false;
  int track_count_{};
  bool identified_target_change_ = true;
  bool dynamic_reconfig_initialized_{};
  bool using_traject_{};
  bool last_shoot_state_{};
  bool is_aheading_two_[150]{};
  bool start_traject_{};
  double filtered_vel_des{};

  geometry_msgs::Point after_traject_output_yaw_{};
  geometry_msgs::Point target_pos_[150]{};
  visualization_msgs::Marker marker_desire_;
  visualization_msgs::Marker marker_real_;
  ros::Time start_using_traject_time{};
  ros::Time ban_shoot_time_{};
  ros::Time last_output_time_{};
  ros::Time ban_shoot_start_time_{};

  TrajectoryFunctionCoefficients trajectory_function_coefficients{};
  TrajectoryLimitParams stauts_limit_{};

  mutable std::mutex heat_mutex_{};
};
}  // namespace rm_gimbal_controllers
