//
// Created by flying on 2021/1/18.
//

#ifndef RM_CHASSIS_CONTROLLER_SENTRY_H
#define RM_CHASSIS_CONTROLLER_SENTRY_H

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <robot_state_controller/robot_state_interface.h>
#include <robot_state_controller/tf_rt_broadcaster.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_common/filters/filters.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace rm_chassis_controllers {
enum SentrytState {
  PASSIVE,
  RAW,
};

class ChassisSentryController :
    public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface, hardware_interface::RobotStateInterface> {
 public:
  ChassisSentryController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
 private:
  void passive();
  void raw();
  void recovery();
  void moveJoint(const ros::Duration &period);
  void commandCB(const rm_msgs::ChassisCmdConstPtr &msg);
  void velCmdCB(const geometry_msgs::Twist::ConstPtr &cmd);
  void updateOdom(const ros::Time &time, const ros::Duration &period);
  geometry_msgs::Twist iKine();

  control_toolbox::Pid pid_wheel_;
  hardware_interface::JointHandle joint_wheel_;
  hardware_interface::RobotStateHandle robot_state_handle_{};

  double publish_rate_{}, wheel_radius_{}, current_coeff_{};
  ros::Time last_publish_time_;
  geometry_msgs::TransformStamped odom2base_{};
  geometry_msgs::Vector3Stamped vel_cmd_{};
  geometry_msgs::Vector3Stamped vel_tfed_{};

  RampFilter<double> *ramp_x{};

  bool state_changed_{};
  SentrytState state_ = PASSIVE;
  ros::Subscriber chassis_cmd_subscriber_;
  ros::Subscriber vel_cmd_subscriber_;
  realtime_tools::RealtimeBuffer<rm_msgs::ChassisCmd> chassis_rt_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> vel_rt_buffer_;
  rm_msgs::ChassisCmd cmd_chassis_;
  robot_state_controller::TfRtBroadcaster tf_broadcaster_{};
};
}
#endif //RM_CHASSIS_CONTROLLER_STANDARD_H
