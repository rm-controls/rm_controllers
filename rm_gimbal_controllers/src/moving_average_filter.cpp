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

#include "rm_gimbal_controller/moving_average_filter.h"
#include <rm_common/ori_tool.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace moving_average_filter {
MovingAverageFilterTrack::MovingAverageFilterTrack(ros::NodeHandle &nh,
                                                   int id,
                                                   hardware_interface::RobotStateHandle robot_state_handle) {
  robot_state_handle_ = std::move(robot_state_handle);
  if (!nh.getParam("is_debug", is_debug_) ||
      !nh.getParam("pos_data_num", pos_data_num_) ||
      !nh.getParam("vel_data_num", vel_data_num_) ||
      !nh.getParam("center_data_num", center_data_num_) ||
      !nh.getParam("gyro_data_num", gyro_data_num_) ||
      !nh.getParam("center_offset_z", center_offset_z_)) {
    ROS_ERROR("Some gimbal params doesn't given (namespace: %s)", nh.getNamespace().c_str());
    return;
  }

  ma_filter_pos_x_ = new MovingAverageFilter<double>(pos_data_num_);
  ma_filter_pos_y_ = new MovingAverageFilter<double>(pos_data_num_);
  ma_filter_pos_z_ = new MovingAverageFilter<double>(pos_data_num_);
  ma_filter_vel_x_ = new MovingAverageFilter<double>(vel_data_num_);
  ma_filter_vel_y_ = new MovingAverageFilter<double>(vel_data_num_);
  ma_filter_vel_z_ = new MovingAverageFilter<double>(vel_data_num_);
  ma_filter_center_x_ = new MovingAverageFilter<double>(center_data_num_);
  ma_filter_center_y_ = new MovingAverageFilter<double>(center_data_num_);
  ma_filter_center_z_ = new MovingAverageFilter<double>(center_data_num_);
  ma_filter_gyro_vel_ = new MovingAverageFilter<double>(gyro_data_num_);

  if (is_debug_)
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::MovingAverageData>(nh,
                                                                                          "id" + std::to_string(id),
                                                                                          100));
}

void MovingAverageFilterTrack::input(const geometry_msgs::TransformStamped &map2detection,
                                     const std::string pitch_frame) {
  // Initialize the first time it enters the filter
  geometry_msgs::TransformStamped observation2map, observation2detection;
  try {
    geometry_msgs::TransformStamped map2pitch = robot_state_handle_.lookupTransform("map",
                                                                                    pitch_frame,
                                                                                    ros::Time(0));
    tf2::Quaternion quaternion;
    geometry_msgs::Transform map2observation;
    double map2pitch_yaw = yawFromQuat(map2pitch.transform.rotation);
    quaternion.setRPY(0, 0, map2pitch_yaw);
    map2observation.translation = map2pitch.transform.translation;
    map2observation.rotation = tf2::toMsg(quaternion);

    tf2::Transform map2observation_tf;
    tf2::fromMsg(map2observation, map2observation_tf);
    observation2map.transform = tf2::toMsg(map2observation_tf.inverse());
    observation2map.header.stamp = map2detection.header.stamp;
    tf2::doTransform(map2detection, observation2detection, observation2map);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  output_map2detection_ = map2detection;
  double dt = std::abs(map2detection.header.stamp.toSec() - last_map2detection_.header.stamp.toSec());
  if (dt > 0.5) {
    last_observation2detection_ = observation2detection;
    last_map2detection_ = map2detection;
    last_output_pos_.x = map2detection.transform.translation.x;
    last_output_pos_.y = map2detection.transform.translation.y;
    last_output_pos_.z = map2detection.transform.translation.z;
    for (int i = 0; i < pos_data_num_; ++i) {
      ma_filter_pos_x_->input(map2detection.transform.translation.x);
      ma_filter_pos_y_->input(map2detection.transform.translation.y);
      ma_filter_pos_z_->input(map2detection.transform.translation.z);
    }
    ma_filter_vel_x_->clear();
    ma_filter_vel_y_->clear();
    ma_filter_vel_z_->clear();
    for (int i = 0; i < center_data_num_; ++i) {
      ma_filter_center_x_->input(map2detection.transform.translation.x);
      ma_filter_center_y_->input(map2detection.transform.translation.y);
      ma_filter_center_z_->input(map2detection.transform.translation.z);
    }
    is_gyro_ = false;
    switch_count_ = 0;
    return;
  }

  // Abandon obvious error data
  geometry_msgs::TransformStamped now_map2detection{};
  now_map2detection = map2detection;
  double delta_x = map2detection.transform.translation.x - last_map2detection_.transform.translation.x;
  double delta_y = map2detection.transform.translation.y - last_map2detection_.transform.translation.y;
  double delta_z = map2detection.transform.translation.z - last_map2detection_.transform.translation.z;
  if (std::abs(delta_x) > 0.5 || std::abs(delta_y) > 0.5 || std::abs(delta_z) > 0.5)
    now_map2detection.transform = last_map2detection_.transform;

  // If true, the target armor is switching
  double delta = observation2detection.transform.translation.y - last_observation2detection_.transform.translation.y;
  if (std::abs(delta) > 0.1) {
    delta_ = delta;
    for (int i = 0; i < pos_data_num_; ++i) {
      ma_filter_pos_x_->input(now_map2detection.transform.translation.x);
      ma_filter_pos_y_->input(now_map2detection.transform.translation.y);
      ma_filter_pos_z_->input(now_map2detection.transform.translation.z);
    }
    for (int i = 0; i < vel_data_num_; ++i) {
      ma_filter_vel_x_->input(output_vel_.x);
      ma_filter_vel_y_->input(output_vel_.y);
      ma_filter_vel_z_->input(output_vel_.z);
    }

    // If switch number bigger than 3, so it is in gyro
    if (switch_count_ <= 3)
      switch_count_++;
    else {
      enter_gyro_time_ = now_map2detection.header.stamp;
      is_gyro_ = true;
    }
  }
  if (std::abs(now_map2detection.header.stamp.toSec() - enter_gyro_time_.toSec()) >= 1.0 && is_gyro_) {
    is_gyro_ = false;
    switch_count_ = 0;
  }

  // filter pos
  ma_filter_pos_x_->input(now_map2detection.transform.translation.x);
  ma_filter_pos_y_->input(now_map2detection.transform.translation.y);
  ma_filter_pos_z_->input(now_map2detection.transform.translation.z);
  output_pos_.x = ma_filter_pos_x_->output();
  output_pos_.y = ma_filter_pos_y_->output();
  output_pos_.z = ma_filter_pos_z_->output();
  try { tf2::doTransform(output_pos_, output_pos_observation_, observation2map); }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

  // filter vel
  double vel_x = std::abs((output_pos_.x - last_output_pos_.x) / dt - output_vel_.x) < 5.0 ?
                 (output_pos_.x - last_output_pos_.x) / dt : output_vel_.x;
  double vel_y = std::abs((output_pos_.y - last_output_pos_.y) / dt - output_vel_.y) < 5.0 ?
                 (output_pos_.y - last_output_pos_.y) / dt : output_vel_.y;
  double vel_z = std::abs((output_pos_.z - last_output_pos_.z) / dt - output_vel_.z) < 5.0 ?
                 (output_pos_.z - last_output_pos_.z) / dt : output_vel_.z;
  ma_filter_vel_x_->input(vel_x);
  ma_filter_vel_y_->input(vel_y);
  ma_filter_vel_z_->input(vel_z);
  output_vel_.x = ma_filter_vel_x_->output();
  output_vel_.y = ma_filter_vel_y_->output();
  output_vel_.z = ma_filter_vel_z_->output();

  // filter center
  ma_filter_center_x_->input(now_map2detection.transform.translation.x);
  ma_filter_center_y_->input(now_map2detection.transform.translation.y);
  ma_filter_center_z_->input(now_map2detection.transform.translation.z);
  output_center_.x = ma_filter_center_x_->output();
  output_center_.y = ma_filter_center_y_->output();
  output_center_.z = ma_filter_center_z_->output() - center_offset_z_;
  try { tf2::doTransform(output_center_, output_center_observation_, observation2map); }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

  // filter gyro vel
  double detection_gyro_vel{};
  if (delta < 0.1 && is_gyro_) {
    detection_gyro_vel =
        delta / (observation2detection.header.stamp.toSec() - last_observation2detection_.header.stamp.toSec());
    if (std::abs(detection_gyro_vel - output_gyro_vel_) < 3.0)
      ma_filter_gyro_vel_->input(detection_gyro_vel);
  }
  output_gyro_vel_ = ma_filter_gyro_vel_->output();

  // Update map2detection
  output_map2detection_.transform.translation.x = output_pos_.x;
  output_map2detection_.transform.translation.y = output_pos_.y;
  output_map2detection_.transform.translation.z = output_pos_.z;

  last_output_pos_ = output_pos_;
  last_map2detection_ = now_map2detection;
  last_observation2detection_ = observation2detection;

  if (is_debug_) {
    rm_msgs::MovingAverageData moving_average_data{};
    moving_average_data.header.stamp = now_map2detection.header.stamp;
    moving_average_data.real_pos_x = now_map2detection.transform.translation.x;
    moving_average_data.real_pos_y = now_map2detection.transform.translation.y;
    moving_average_data.real_pos_z = now_map2detection.transform.translation.z;
    moving_average_data.real_vel_x = vel_x;
    moving_average_data.real_vel_y = vel_y;
    moving_average_data.real_vel_z = vel_z;
    moving_average_data.filtered_pos_x = output_pos_.x;
    moving_average_data.filtered_pos_y = output_pos_.y;
    moving_average_data.filtered_pos_z = output_pos_.z;
    moving_average_data.filtered_vel_x = output_vel_.x;
    moving_average_data.filtered_vel_y = output_vel_.y;
    moving_average_data.filtered_vel_z = output_vel_.z;
    moving_average_data.filtered_center_x = output_center_.x;
    moving_average_data.filtered_center_y = output_center_.y;
    moving_average_data.filtered_center_z = output_center_.z;
    moving_average_data.real_gyro_vel = detection_gyro_vel;
    moving_average_data.filtered_gyro_vel = output_gyro_vel_;

    if (realtime_pub_->trylock()) {
      realtime_pub_->msg_ = moving_average_data;
      realtime_pub_->unlockAndPublish();
    }
  }
}
}
