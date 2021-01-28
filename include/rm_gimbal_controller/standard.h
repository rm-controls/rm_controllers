//
// Created by qiayuan on 1/16/21.
//

#ifndef RM_GIMBAL_CONTROLLER_STANDARD_H
#define RM_GIMBAL_CONTROLLER_STANDARD_H

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_base/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_gimbal_controllers/GimbalConfig.h>
#include <rm_gimbal_controller/bullet_solver.h>
#include <visualization_msgs/Marker.h>

namespace rm_gimbal_controllers {
enum StandardState {
  PASSIVE,
  RATE,
  TRACK,
};

class GimbalStandardController :
    public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface, hardware_interface::RobotStateInterface> {
 public:
  GimbalStandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void setDes(const ros::Time &time, double yaw, double pitch);
 private:
  void passive();
  void rate(const ros::Time &time, const ros::Duration &period);
  void track(const ros::Time &time);
  void moveJoint(const ros::Duration &period);
  void commandCB(const rm_msgs::GimbalCmdConstPtr &msg);
  void detectionCB(const rm_msgs::TargetDetectionArrayConstPtr &msg);

  control_toolbox::Pid pid_yaw_, pid_pitch_;
  hardware_interface::JointHandle joint_yaw_, joint_pitch_;
  hardware_interface::RobotStateHandle robot_state_handle_;
  geometry_msgs::TransformStamped map2gimbal_des_;

  Bullet3DSolver *bullet_solver_{};

  double *chassis_angular_z_{};
  bool state_changed_{};
  Vec2<double> angle_init_{};
  StandardState state_ = PASSIVE;
  ros::Subscriber cmd_subscriber_;
  ros::Subscriber cmd_sub_track_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError> > error_pub_;
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rm_msgs::TargetDetectionArray> detection_rt_buffer_;
  rm_msgs::GimbalCmd cmd_;
  double error_yaw_{};
  double error_pitch_{};
  double publish_rate_{};
  ros::Time last_publish_time_;

};
}

#endif  // RM_GIMBAL_CONTROLLER_STANDARD_H
