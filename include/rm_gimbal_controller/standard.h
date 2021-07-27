//
// Created by qiayuan on 1/16/21.
//

#ifndef RM_GIMBAL_CONTROLLER_STANDARD_H
#define RM_GIMBAL_CONTROLLER_STANDARD_H

#include <effort_controllers/joint_position_controller.h>
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
#include <rm_gimbal_controller/moving_average_filter.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/CameraInfo.h>

namespace rm_gimbal_controllers {

struct Config {
  double time_compensation;
};

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                         hardware_interface::RobotStateInterface> {
 public:
  Controller() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void setDes(const ros::Time &time, double yaw_des, double pitch_des);
 private:
  void rate(const ros::Time &time, const ros::Duration &period);
  void track(const ros::Time &time);
  void direct(const ros::Time &time);
  void moveJoint(const ros::Time &time, const ros::Duration &period);
  void commandCB(const rm_msgs::GimbalCmdConstPtr &msg);
  void detectionCB(const rm_msgs::TargetDetectionArrayConstPtr &msg);
  void cameraCB(const sensor_msgs::CameraInfoConstPtr &msg);
  void updateTrack(int id);
  void updateChassisVel();
  void reconfigCB(rm_gimbal_controllers::GimbalConfig &config, uint32_t);
  bool updateTf();

  ros::Time last_publish_time_{};
  ros::Time last_camera_time_{};
  ros::Time last_detection_time_{};
  ros::NodeHandle nh_moving_average_filter_;

  hardware_interface::EffortJointInterface *effort_joint_interface_{};
  hardware_interface::RobotStateHandle robot_state_handle_;
  effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

  bullet_solver::BulletSolver *bullet_solver_{};
  MovingAverageFilter<double> *ma_filter_chassis_angular_{};

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>> error_pub_;
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
  geometry_msgs::TransformStamped map2base_, last_map2base_;
  geometry_msgs::Twist chassis_vel_;

  rm_msgs::GimbalCmd cmd_gimbal_;

  double publish_rate_{};
  bool dynamic_reconfig_initialized_{};
  bool state_changed_{};
  std::string yaw_frame_id_{}, pitch_frame_id_{}, gimbal_des_frame_id_{}, base_frame_{}, detection_topic_{},
      camera_topic_{};

  Config config_{};
  enum { RATE, TRACK, DIRECT };
  int state_ = RATE;

  std::map<int, moving_average_filter::MovingAverageFilterTrack *> moving_average_filters_track_;
  std::map<int, geometry_msgs::Pose> last_detection_{};
  std::map<int, geometry_msgs::Vector3> detection_vel_{};
  std::map<int, geometry_msgs::Point> detection_pos_{};
  std::map<int, geometry_msgs::Point> detection_pos_observation_{};
  std::map<int, geometry_msgs::Point> center_pos_{};
  std::map<int, geometry_msgs::Point> center_pos_observation_{};
  std::map<int, double> gyro_vel_{};
};
}

#endif  // RM_GIMBAL_CONTROLLER_STANDARD_H
