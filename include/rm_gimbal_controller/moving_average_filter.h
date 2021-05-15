//
// Created by chenzheng on 2021/5/8.
//

#ifndef SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H
#define SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/filters/filters.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_msgs/MovingAverageData.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

namespace moving_average_filter {

class MovingAverageFilterTrack {
 public:
  explicit MovingAverageFilterTrack(ros::NodeHandle &nh,
                                    int id,
                                    hardware_interface::RobotStateHandle robot_state_handle);
  void input(const geometry_msgs::TransformStamped &map2detection);
  geometry_msgs::TransformStamped getTransform() const;
  geometry_msgs::Vector3 getVel() const;
  geometry_msgs::Point getCenter() const;
  double getGyroVel() const;
  bool isGyro() const;
  double getDelta() const { return delta_; }
  ~MovingAverageFilterTrack() = default;
 private:
  ros::Time enter_gyro_time_;

  MovingAverageFilter<double> *ma_filter_pos_x_;
  MovingAverageFilter<double> *ma_filter_pos_y_;
  MovingAverageFilter<double> *ma_filter_pos_z_;
  MovingAverageFilter<double> *ma_filter_vel_x_;
  MovingAverageFilter<double> *ma_filter_vel_y_;
  MovingAverageFilter<double> *ma_filter_vel_z_;
  MovingAverageFilter<double> *ma_filter_center_x_;
  MovingAverageFilter<double> *ma_filter_center_y_;
  MovingAverageFilter<double> *ma_filter_center_z_;

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::MovingAverageData>> realtime_pub_;
  hardware_interface::RobotStateHandle robot_state_handle_;

  bool is_debug_{};
  bool is_gyro_{};
  int switch_count_{};
  int pos_data_num_{}, vel_data_num_{}, center_data_num_{};
  double delta_;
  double last_gyro_vel_{};
  double output_gyro_vel_{};

  geometry_msgs::TransformStamped last_map2detection_{};
  geometry_msgs::TransformStamped output_map2detection_{};
  geometry_msgs::TransformStamped last_yaw2detection_;
  geometry_msgs::TransformStamped appearance_yaw2detection_{};
  geometry_msgs::TransformStamped last_appearance_yaw2detection_{};
  geometry_msgs::TransformStamped disappearance_yaw2detection_{};
  geometry_msgs::Vector3 output_vel_{};
  geometry_msgs::Point output_center_{};
  geometry_msgs::Vector3 last_output_pos_{};
};
}

#endif
