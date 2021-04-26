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
#include <rm_msgs/KalmanData.h>
#include <rm_gimbal_controllers/KalmanConfig.h>

namespace kalman_filter {
struct Config {
  double q_x, q_dx, r_x, r_dx;
};
class KalmanFilterTrack {
 public:
  explicit KalmanFilterTrack(ros::NodeHandle &nh, int id);
  void input(const geometry_msgs::TransformStamped &map2detection);
  geometry_msgs::TransformStamped getTransform();
  geometry_msgs::Twist getTwist();
  void perdict();
  void updateQR();
  ~KalmanFilterTrack() = default;

 private:
  void reconfigCB(rm_gimbal_controllers::KalmanConfig &config, uint32_t);

  KalmanFilter<double> *kalman_filter_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::KalmanConfig> *d_srv_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::KalmanData>> realtime_pub_;
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  ros::Time last_detection_time_;
  Vec8<double> x_, x0_, u_, x_hat_;
  Mat8<double> a_, b_, h_, q_, r_;
  double last_pos_x_hat_{}, last_pos_y_hat_{}, last_pos_z_hat_{};
  double last_last_pos_x_hat_{}, last_last_pos_y_hat_{}, last_last_pos_z_hat_{};
  bool is_debug_{};
  bool dynamic_reconfig_initialized_ = false;
  bool is_filter_ = false;
  Config config_{};
  geometry_msgs::TransformStamped map2detection_;
  geometry_msgs::TransformStamped map2detection_new_;
  geometry_msgs::TransformStamped map2detection_last_;
  rm_msgs::KalmanData kalman_data_;
};
}

#endif //SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H
