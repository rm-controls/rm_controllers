//
// Created by huakang on 2021/3/21.
//

#ifndef RM_COMMON_INCLUDE_RM_COMMON_CHASSIS_BASE_H_
#define RM_COMMON_INCLUDE_RM_COMMON_CHASSIS_BASE_H_

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_common/filters/filters.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

namespace rm_chassis_base {
enum StandardState {
  PASSIVE,
  RAW,
  FOLLOW,
  TWIST,
  GYRO,
};

class ChassisBase : public controller_interface::MultiInterfaceController
    <hardware_interface::EffortJointInterface, hardware_interface::RobotStateInterface> {
 public:
  ChassisBase() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
 protected:
  void passive();
  void raw(const ros::Duration &period);
  virtual void follow(const ros::Time &time, const ros::Duration &period);
  virtual void twist(const ros::Time &time, const ros::Duration &period);
  virtual void gyro(const ros::Time &time, const ros::Duration &period);
  virtual void moveJoint(const ros::Duration &period) = 0;
  virtual geometry_msgs::Twist iKine(const ros::Duration &period) = 0;
  void updateOdom(const ros::Time &time, const ros::Duration &period);
  void recovery(const ros::Duration &period);
  void tfVelToBase(const std::string &from);
  void cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr &msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd);

  std::vector<hardware_interface::JointHandle> joint_vector_{};
  hardware_interface::RobotStateHandle robot_state_handle_{};

  double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, twist_angular_{};
  bool enable_odom_tf_{};
  RampFilter<double> *ramp_x{}, *ramp_y{}, *ramp_w{};

  ros::Time last_publish_time_;
  geometry_msgs::TransformStamped odom2base_{};
  geometry_msgs::Vector3Stamped vel_cmd_{};
  geometry_msgs::Vector3Stamped vel_tfed_{};
  geometry_msgs::Twist vel_base_{};
  control_toolbox::Pid pid_follow_;

  bool state_changed_{};
  StandardState state_ = PASSIVE;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  robot_state_controller::TfRtBroadcaster tf_broadcaster_{};
  ros::Subscriber cmd_chassis_sub_;
  ros::Subscriber cmd_vel_sub_;
  realtime_tools::RealtimeBuffer<rm_msgs::ChassisCmd> chassis_rt_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> vel_rt_buffer_;

};
}
#endif // RM_COMMON_INCLUDE_RM_COMMON_CHASSIS_BASE_H_
