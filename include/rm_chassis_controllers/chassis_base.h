//
// Created by huakang on 2021/3/21.
//

#ifndef RM_CHASSIS_CONTROLLER_CHASSIS_BASE_H_
#define RM_CHASSIS_CONTROLLER_CHASSIS_BASE_H_

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/filters/filters.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <rm_msgs/ChassisCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

namespace rm_chassis_controllers {
enum State {
  PASSIVE,
  RAW,
  FOLLOW,
  TWIST,
  GYRO,
};

struct Command {
  geometry_msgs::Twist cmd_vel_;
  rm_msgs::ChassisCmd cmd_chassis_;
  ros::Time stamp_;
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
  void raw();
  void follow(const ros::Time &time, const ros::Duration &period);
  void twist(const ros::Time &time, const ros::Duration &period);
  void gyro();
  virtual void moveJoint(const ros::Time &time, const ros::Duration &period) = 0;
  virtual geometry_msgs::Twist forwardKinematics() = 0;
  void updateOdom(const ros::Time &time, const ros::Duration &period);
  void recovery();
  void tfVelToBase(const std::string &from);
  void powerLimit();

  void cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr &msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  hardware_interface::EffortJointInterface *effort_joint_interface_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};
  hardware_interface::RobotStateHandle robot_state_handle_{};

  double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, twist_angular_{},
      power_coeff_{}, power_min_vel_{}, timeout_{};
  bool enable_odom_tf_ = false;
  bool state_changed_ = true;
  State state_ = PASSIVE;
  RampFilter<double> *ramp_x{}, *ramp_y{}, *ramp_w{};

  ros::Time last_publish_time_;
  geometry_msgs::TransformStamped odom2base_{};
  geometry_msgs::Vector3Stamped vel_cmd_{}; // x, y, w
  geometry_msgs::Vector3Stamped vel_tfed_{}; // x, y, w
  control_toolbox::Pid pid_follow_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  rm_common::TfRtBroadcaster tf_broadcaster_{};
  ros::Subscriber cmd_chassis_sub_;
  ros::Subscriber cmd_vel_sub_;
  Command cmd_struct_;
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
};

}

#endif // RM_CHASSIS_CONTROLLER_CHASSIS_BASE_H_
