//
// Created by liudec on 2021/3/29.
//

#ifndef SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H
#define SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/filters/kalman_filter.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/TrackDataArray.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_gimbal_controllers/KalmanConfig.h>

namespace rm_gimbal_controllers {
class TranTarget {
 public:
  void clear(const Vec8<double> &x) { kf_->clear(x); }
  void update(const Vec8<double> &z) { kf_->update(z); }
  void predict() { kf_->predict(u_); }
  const Vec8<double> &getState() {
    x_ = kf_->getState();
    return x_;
  }

  TranTarget(double dt,
             double q_x, double q_dx,
             double r_x, double r_dx);
  ~TranTarget() { delete kf_; }

 private:
  KalmanFilter<double> *kf_;
  Vec8<double> x_, u_;
  Mat8<double> a_, b_, h_, q_, r_;

};

class KalmanFilterTrack {
 public:
  KalmanFilterTrack(hardware_interface::RobotStateHandle &robot_state_handle, ros::NodeHandle &controller_nh);

  void getStateAndPub();

  ~KalmanFilterTrack() {
    for (auto item:id2detection_)
      delete item.second;
  }

  void
  update(realtime_tools::RealtimeBuffer<rm_msgs::TargetDetectionArray> &detection_rt_buffer, double time_compensation);

 private:

  void reconfigCB(const KalmanConfig &config, uint32_t level);

  double q_x_{}, q_dx_{}, r_x_{}, r_dx_{};
  double time_thresh_{};
  double distance_thresh_{};
  std::map<int, TranTarget *> id2detection_;
  std::map<int, geometry_msgs::TransformStamped> map2detections_last_;
  dynamic_reconfigure::Server<KalmanConfig> *d_srv_;
  bool begin_flag_ = false;
  hardware_interface::RobotStateHandle robot_state_handle_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>> track_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>> track_test_pub_;
  ros::Time last_detection_time_;
  tf2::Transform map2camera_tf_;
  robot_state_controller::TfRtBroadcaster tf_broadcaster_{};
};

}
#endif //SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H
