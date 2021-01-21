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
#include <rm_msgs/GimbalTrackCmd.h>
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
  void cmdTrackCB(const rm_msgs::GimbalTrackCmdConstPtr &msg);
  void reconfigCB(const rm_gimbal_controllers::GimbalConfig &config, uint32_t level);

  control_toolbox::Pid pid_yaw_, pid_pitch_;
  hardware_interface::JointHandle joint_yaw_, joint_pitch_;
  hardware_interface::RobotStateHandle robot_state_handle_;
  geometry_msgs::TransformStamped world2gimbal_des_;

  BulletSolver<double> *bullet_solver_{};
  double target_speed_x_{}, target_speed_y_{}, target_speed_z_{},
      target_position_x_{}, target_position_y_{}, target_position_z_{};
  double bullet_speed_{};
  double resistance_coff_{}, delay_{}, dt_{}, timeout_{};
  double g_ = 9.8;

  double *chassis_angular_z_{};
  bool state_changed_{};
  StandardState state_ = PASSIVE;
  ros::Subscriber cmd_subscriber_;
  ros::Subscriber cmd_sub_track_;
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer_;
  rm_msgs::GimbalCmd cmd_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig> *d_srv_{};
};
}

#endif  // RM_GIMBAL_CONTROLLER_STANDARD_H
