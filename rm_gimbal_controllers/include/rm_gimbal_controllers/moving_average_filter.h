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
// Created by chenzheng on 2021/5/8.
//

#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/filters/filters.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_msgs/MovingAverageData.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

namespace moving_average_filter
{
class MovingAverageFilterTrack
{
public:
  explicit MovingAverageFilterTrack(ros::NodeHandle& nh, int id,
                                    rm_control::RobotStateHandle robot_state_handle);
  void input(const geometry_msgs::TransformStamped& map2detection, const std::string pitch_frame);
  geometry_msgs::TransformStamped getTransform() const
  {
    return output_map2detection_;
  }
  geometry_msgs::Vector3 getVel() const
  {
    return output_vel_;
  }
  geometry_msgs::Point getPos() const
  {
    return output_pos_;
  }
  geometry_msgs::Point getPosObservation() const
  {
    return output_pos_observation_;
  }
  geometry_msgs::Point getCenter() const
  {
    return output_center_;
  }
  geometry_msgs::Point getCenterObservation() const
  {
    return output_center_observation_;
  }
  double getGyroVel() const
  {
    return output_gyro_vel_;
  }
  bool isGyro() const
  {
    return is_gyro_;
  }
  double getDelta() const
  {
    return delta_;
  }
  ~MovingAverageFilterTrack() = default;

private:
  ros::Time enter_gyro_time_;

  MovingAverageFilter<double>* ma_filter_pos_x_;
  MovingAverageFilter<double>* ma_filter_pos_y_;
  MovingAverageFilter<double>* ma_filter_pos_z_;
  MovingAverageFilter<double>* ma_filter_vel_x_;
  MovingAverageFilter<double>* ma_filter_vel_y_;
  MovingAverageFilter<double>* ma_filter_vel_z_;
  MovingAverageFilter<double>* ma_filter_center_x_;
  MovingAverageFilter<double>* ma_filter_center_y_;
  MovingAverageFilter<double>* ma_filter_center_z_;
  MovingAverageFilter<double>* ma_filter_gyro_vel_;

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::MovingAverageData>> realtime_pub_;
  rm_control::RobotStateHandle robot_state_handle_;

  bool is_debug_{};
  bool is_gyro_{};
  int switch_count_{};
  int pos_data_num_{}, vel_data_num_{}, center_data_num_{}, gyro_data_num_{};
  double delta_{};
  double output_gyro_vel_{};
  double center_offset_z_{};

  geometry_msgs::TransformStamped last_map2detection_{};
  geometry_msgs::TransformStamped output_map2detection_{};
  geometry_msgs::TransformStamped last_observation2detection_;
  geometry_msgs::Vector3 output_vel_{};
  geometry_msgs::Point output_pos_{};
  geometry_msgs::Point output_pos_observation_{};
  geometry_msgs::Point output_center_{};
  geometry_msgs::Point output_center_observation_{};
  geometry_msgs::Point last_output_pos_{};
};
}  // namespace moving_average_filter
