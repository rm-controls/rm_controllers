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
#include "rm_gimbal_controllers/bullet_solver/target_kinematics_model.h"

namespace rm_gimbal_controllers
{
struct Config
{
  double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
      resistance_coff_qd_30, g, delay, dt, timeout;
};

enum class SelectedArmor
{
  FRONT = 0,
  LEFT = 1,
  BACK = 2,
  RIGHT = 3
};

class BulletSolver
{
public:
  explicit BulletSolver(ros::NodeHandle& controller_nh);
  // normal target(including robots and buildings)
  void input(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw, double v_yaw,
             double r1, double r2, double dz, int armors_num);
  // windmill
  void input(double theta, double theta_dot, double bullet_speed, geometry_msgs::TransformStamped windmill2odom);
  bool solve();
  double getGimbalError(double yaw_real, double pitch_real);
  double getResistanceCoefficient(double bullet_speed) const;
  double getYaw() const
  {
    return output_yaw_;
  }
  double getPitch() const
  {
    return -output_pitch_;
  }
  void getYawVelAndAccelDes(double& vel_des, double& accel_des);
  void getPitchVelAndAccelDes(double& vel_des, double& accel_des);
  void bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time);
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  ~BulletSolver() = default;

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_real_pub_;
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  Config config_{};
  double max_track_target_vel_;
  bool dynamic_reconfig_initialized_{};
  double output_yaw_{}, output_pitch_{};
  double last_pitch_vel_des_{};
  ros::Time last_pitch_vel_des_solve_time_{ 0 };
  double bullet_speed_{}, resistance_coff_{};
  double windmill_radius_;
  SelectedArmor selected_armor_;
  bool track_target_;

  std::shared_ptr<TargetKinematicsBase> target_kinematics_;
  geometry_msgs::Point target_pos_{};
  double fly_time_;
  visualization_msgs::Marker marker_desire_;
  visualization_msgs::Marker marker_real_;
};
}  // namespace rm_gimbal_controllers
