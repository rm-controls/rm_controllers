//
// Created by qiayuan on 1/16/21.
//

#ifndef RM_GIMBAL_CONTROLLER_STANDARD_H
#define RM_GIMBAL_CONTROLLER_STANDARD_H

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackDataArray.h>
#include <dynamic_reconfigure/server.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_gimbal_controllers/GimbalConfig.h>
#include <rm_gimbal_controller/bullet_solver.h>
#include <rm_gimbal_controller/kalman_filter.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/CameraInfo.h>
#include <rm_common/filters/lp_filter.h>

namespace rm_gimbal_controllers {
enum StandardState {
  PASSIVE,
  RATE,
  TRACK,
};

struct Config {
  double time_compensation;
};

class Controller :
    public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface, hardware_interface::RobotStateInterface> {
 public:
  Controller() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void setDes(const ros::Time &time, double yaw, double pitch);
 private:
  void passive();
  void rate(const ros::Time &time, const ros::Duration &period);
  void track(const ros::Time &time);
  void moveJoint(const ros::Time &time, const ros::Duration &period);
  void commandCB(const rm_msgs::GimbalCmdConstPtr &msg);
  void detectionCB(const rm_msgs::TargetDetectionArrayConstPtr &msg);
  void cameraCB(const sensor_msgs::CameraInfoConstPtr &msg);
  void updateTf();
  void updateTrack(int id);
  void reconfigCB(rm_gimbal_controllers::GimbalConfig &config, uint32_t);

  ros::Time last_publish_time_;
  ros::Time last_camera_time_{};
  ros::NodeHandle nh_kalman_;

  control_toolbox::Pid pid_yaw_, pid_pitch_;
  hardware_interface::JointHandle joint_yaw_, joint_pitch_;
  hardware_interface::RobotStateHandle robot_state_handle_;

  bullet_solver::Bullet3DSolver *bullet_solver_{};
  LowPassFilter *lp_filter_yaw_{};
  LowPassFilter *lp_filter_pitch_{};

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError> > error_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>> track_pub_;
  ros::Subscriber cmd_gimbal_sub_;
  ros::Subscriber data_detection_sub_;
  ros::Subscriber camera_sub_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig> *d_srv_{};
  rm_common::TfRtBroadcaster tf_broadcaster_{};

  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rm_msgs::TargetDetectionArray> detection_rt_buffer_;
  realtime_tools::RealtimeBuffer<sensor_msgs::CameraInfo> camera_rt_buffer_;
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;

  geometry_msgs::TransformStamped map2gimbal_des_, map2pitch_;
  geometry_msgs::Vector3 target_pos_{};
  rm_msgs::GimbalCmd cmd_;

  double upper_yaw_{}, lower_yaw_{}, upper_pitch_{}, lower_pitch_{};
  double publish_rate_{};
  bool dynamic_reconfig_initialized_ = false;
  bool state_changed_{};
  bool last_solve_success_{};
  Vec2<double> angle_init_{};

  Config config_{};
  StandardState state_ = PASSIVE;

  std::map<int, geometry_msgs::Twist> target_vel_;
  std::map<int, kalman_filter::KalmanFilterTrack *> kalman_filters_track_;
  std::map<int, ros::Time> last_detection_time_;
  std::map<int, geometry_msgs::Pose> last_detection_;
};
}

#endif  // RM_GIMBAL_CONTROLLER_STANDARD_H
