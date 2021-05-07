//
// Created by chenzheng on 2021/4/17.
//

#ifndef SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H
#define SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/filters/kalman_filter.h>
#include <rm_common/filters/filters.h>
#include <rm_msgs/KalmanData.h>
#include <rm_gimbal_controllers/KalmanConfig.h>

namespace kalman_filter {
struct Config {
  double q_x_pos, q_y_pos, q_z_pos, q_yaw_pos, q_x_vel, q_y_vel, q_z_vel, q_yaw_vel,
      r_x_pos, r_y_pos, r_z_pos, r_yaw_pos, r_x_vel, r_y_vel, r_z_vel, r_yaw_vel;
};
class KalmanFilterTrack {
 public:
  explicit KalmanFilterTrack(ros::NodeHandle &nh, int id);
  void input(const geometry_msgs::TransformStamped &map2detection);
  geometry_msgs::TransformStamped getTransform();
  geometry_msgs::Twist getTwist();
  void perdict();
  void updateQR();
  geometry_msgs::Vector3 getCenter() const;
  bool isGyro() const;
  ~KalmanFilterTrack() = default;

 private:
  ros::Time last_detection_time_;
  ros::Time enter_gyro_time_;
  KalmanFilter<double> *kalman_filter_;
  MovingAverageFilter<double> *moving_average_filter_x_;
  MovingAverageFilter<double> *moving_average_filter_y_;
  MovingAverageFilter<double> *moving_average_filter_z_;

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::KalmanData>> realtime_pub_;
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::KalmanConfig> *d_srv_;
  void reconfigCB(rm_gimbal_controllers::KalmanConfig &config, uint32_t);

  Vec8<double> x_, u_, x_hat_;
  Mat8<double> a_, b_, h_, q_, r_;
  bool is_debug_{};
  bool dynamic_reconfig_initialized_{};
  bool is_filter_{};
  bool is_gyro_{};
  int switch_count_{};
  Config config_{};

  geometry_msgs::Vector3 last_pos_hat_{};
  geometry_msgs::Vector3 last_last_pos_hat_{};
  geometry_msgs::Vector3 center_{};
  geometry_msgs::TransformStamped map2detection_{};
  geometry_msgs::TransformStamped map2detection_now_{};
  geometry_msgs::TransformStamped map2detection_last_{};
  rm_msgs::KalmanData kalman_data_;
};
}

#endif //SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H
