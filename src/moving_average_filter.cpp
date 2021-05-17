//
// Created by chenzheng on 2021/5/8.
//

#include "rm_gimbal_controller/moving_average_filter.h"
#include <rm_common/ori_tool.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <utility>

namespace moving_average_filter {
MovingAverageFilterTrack::MovingAverageFilterTrack(ros::NodeHandle &nh,
                                                   int id,
                                                   hardware_interface::RobotStateHandle robot_state_handle) {
  robot_state_handle_ = std::move(robot_state_handle);
  is_debug_ = getParam(nh, "is_debug", false);
  pos_data_num_ = getParam(nh, "pos_data_num", 10);
  vel_data_num_ = getParam(nh, "vel_data_num", 20);
  center_data_num_ = getParam(nh, "center_data_num", 100);

  ma_filter_pos_x_ = new MovingAverageFilter<double>(pos_data_num_);
  ma_filter_pos_y_ = new MovingAverageFilter<double>(pos_data_num_);
  ma_filter_pos_z_ = new MovingAverageFilter<double>(pos_data_num_);
  ma_filter_vel_x_ = new MovingAverageFilter<double>(vel_data_num_);
  ma_filter_vel_y_ = new MovingAverageFilter<double>(vel_data_num_);
  ma_filter_vel_z_ = new MovingAverageFilter<double>(vel_data_num_);
  ma_filter_center_x_ = new MovingAverageFilter<double>(center_data_num_);
  ma_filter_center_y_ = new MovingAverageFilter<double>(center_data_num_);
  ma_filter_center_z_ = new MovingAverageFilter<double>(center_data_num_);

  if (is_debug_)
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::MovingAverageData>(nh,
                                                                                          "id" + std::to_string(id),
                                                                                          100));
}

void MovingAverageFilterTrack::input(const geometry_msgs::TransformStamped &map2detection) {
  // Initialize the first time it enters the filter
  geometry_msgs::TransformStamped yaw2detection;
  try {
    geometry_msgs::TransformStamped map2yaw = robot_state_handle_.lookupTransform("map",
                                                                                  "yaw",
                                                                                  ros::Time(0));
    tf2::Transform map2yaw_tf, map2detection_tf, yaw2detection_tf;
    tf2::fromMsg(map2yaw.transform, map2yaw_tf);
    tf2::fromMsg(map2detection.transform, map2detection_tf);
    yaw2detection_tf = map2yaw_tf.inverse() * map2detection_tf;
    yaw2detection.transform.translation.x = yaw2detection_tf.getOrigin().x();
    yaw2detection.transform.translation.y = yaw2detection_tf.getOrigin().y();
    yaw2detection.transform.translation.z = yaw2detection_tf.getOrigin().z();
    yaw2detection.transform.rotation.w = 1;
    yaw2detection.header.stamp = map2detection.header.stamp;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  double dt = std::abs(map2detection.header.stamp.toSec() - last_map2detection_.header.stamp.toSec());
  if (dt > 0.5) {
    last_yaw2detection_ = yaw2detection;
    last_map2detection_ = map2detection;
    output_map2detection_ = map2detection;
    last_output_pos_ = map2detection.transform.translation;
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
  delta_ = yaw2detection.transform.translation.y - last_yaw2detection_.transform.translation.y;
  if (std::abs(delta_) > 0.1) {
    last_appearance_yaw2detection_ = appearance_yaw2detection_;
    appearance_yaw2detection_ = yaw2detection;
    disappearance_yaw2detection_ = last_yaw2detection_;
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

  double pos_x = now_map2detection.transform.translation.x;
  double pos_y = now_map2detection.transform.translation.y;
  double pos_z = now_map2detection.transform.translation.z;

  // filter pos
  ma_filter_pos_x_->input(pos_x);
  ma_filter_pos_y_->input(pos_y);
  ma_filter_pos_z_->input(pos_z);
  output_map2detection_ = now_map2detection;
  output_map2detection_.transform.translation.x = ma_filter_pos_x_->output();
  output_map2detection_.transform.translation.y = ma_filter_pos_y_->output();
  output_map2detection_.transform.translation.z = ma_filter_pos_z_->output();

  // filter vel
  double vel_x =
      std::abs((output_map2detection_.transform.translation.x - last_output_pos_.x) / dt - output_vel_.x) < 5.0 ?
      (output_map2detection_.transform.translation.x - last_output_pos_.x) / dt : output_vel_.x;
  double vel_y =
      std::abs((output_map2detection_.transform.translation.y - last_output_pos_.y) / dt - output_vel_.y) < 5.0 ?
      (output_map2detection_.transform.translation.y - last_output_pos_.y) / dt : output_vel_.y;
  double vel_z =
      std::abs((output_map2detection_.transform.translation.z - last_output_pos_.z) / dt - output_vel_.z) < 5.0 ?
      (output_map2detection_.transform.translation.z - last_output_pos_.z) / dt : output_vel_.z;
  ma_filter_vel_x_->input(vel_x);
  ma_filter_vel_y_->input(vel_y);
  ma_filter_vel_z_->input(vel_z);
  output_vel_.x = ma_filter_vel_x_->output();
  output_vel_.y = ma_filter_vel_y_->output();
  output_vel_.z = ma_filter_vel_z_->output();
  last_output_pos_ = output_map2detection_.transform.translation;

  // filter center
  ma_filter_center_x_->input(pos_x);
  ma_filter_center_y_->input(pos_y);
  ma_filter_center_z_->input(pos_z);
  output_center_.x = ma_filter_center_x_->output();
  output_center_.y = ma_filter_center_y_->output();
  output_center_.z = ma_filter_center_z_->output();

  // filter gyro vel
  double gyro_vel =
      (disappearance_yaw2detection_.transform.translation.y - last_appearance_yaw2detection_.transform.translation.y)
          / (disappearance_yaw2detection_.header.stamp.toSec() - last_appearance_yaw2detection_.header.stamp.toSec());
  if (is_gyro_) {
    if (!std::isnan(gyro_vel) && last_gyro_vel_ != gyro_vel) {
      output_gyro_vel_ = last_gyro_vel_;
      last_gyro_vel_ = gyro_vel;
    }
  } else
    output_gyro_vel_ = 0;

  last_map2detection_ = now_map2detection;
  last_yaw2detection_ = yaw2detection;

  if (is_debug_) {
    rm_msgs::MovingAverageData moving_average_data{};
    moving_average_data.header.stamp = now_map2detection.header.stamp;

    moving_average_data.real_pos_x = pos_x;
    moving_average_data.real_pos_y = pos_y;
    moving_average_data.real_pos_z = pos_z;
    moving_average_data.real_vel_x = vel_x;
    moving_average_data.real_vel_y = vel_y;
    moving_average_data.real_vel_z = vel_z;

    moving_average_data.filtered_pos_x = output_map2detection_.transform.translation.x;
    moving_average_data.filtered_pos_y = output_map2detection_.transform.translation.y;
    moving_average_data.filtered_pos_z = output_map2detection_.transform.translation.z;
    moving_average_data.filtered_vel_x = output_vel_.x;
    moving_average_data.filtered_vel_y = output_vel_.y;
    moving_average_data.filtered_vel_z = output_vel_.z;
    moving_average_data.filtered_center_x = output_center_.x;
    moving_average_data.filtered_center_y = output_center_.y;
    moving_average_data.filtered_center_z = output_center_.z;
    moving_average_data.gyro_vel = gyro_vel;

    if (realtime_pub_->trylock()) {
      realtime_pub_->msg_ = moving_average_data;
      realtime_pub_->unlockAndPublish();
    }
  }
}

geometry_msgs::TransformStamped MovingAverageFilterTrack::getTransform() const {
  return output_map2detection_;
}

geometry_msgs::Vector3 MovingAverageFilterTrack::getVel() const {
  return output_vel_;
}

geometry_msgs::Point MovingAverageFilterTrack::getCenter() const {
  return output_center_;
}

double MovingAverageFilterTrack::getGyroVel() const {
  return output_gyro_vel_;
}

bool MovingAverageFilterTrack::isGyro() const {
  return is_gyro_;
}
}
