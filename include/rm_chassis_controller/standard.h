//
// Created by flying on 2021/1/18.
//

#ifndef RM_CHASSIS_CONTROLLER_STANDARD_H
#define RM_CHASSIS_CONTROLLER_STANDARD_H

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_base/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/ChassisCmd.h>
#include <filters.h>
#include <geometry_msgs/TwistStamped.h>

namespace rm_chassis_controller {
enum StandardState {
  PASSIVE,
  RAW,
  FOLLOW,
  TWIST,
  GYRO,
//  FLY,
};

class ChassisStandardController :
    public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface, hardware_interface::RobotStateInterface> {
 public:
  ChassisStandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration & /*period*/) override;
 private:
  void passive();
  void raw();
  void follow(const ros::Duration &period);
  void twist(const ros::Duration &period);
  void gyro();
//  void fly();
  void transformGyroVel();
  void transformTwistVel(const ros::Duration &period);
  void transformFollowVel(const ros::Duration &period);
  void setVel();
  void moveJoint(const ros::Duration &period);
//  void setDes(const ros::Time &time, double x, double y, double z);  //????????
  void commandCB(const rm_msgs::ChassisCmdConstPtr &msg);
  void velCmdCB(const geometry_msgs::Twist::ConstPtr &cmd);
  void setTrans();
  geometry_msgs::Twist getVel();

  control_toolbox::Pid pid_rf_, pid_lf_, pid_rb_, pid_lb_;
  control_toolbox::Pid pid_follow_, pid_twist_;
  hardware_interface::JointHandle joint_rf_, joint_lf_, joint_rb_, joint_lb_;

  double wheel_base_{}, wheel_track_{}, wheel_radius_{};
  hardware_interface::RobotStateHandle robot_state_handle_;
  geometry_msgs::TransformStamped word2base_;
  geometry_msgs::Vector3Stamped vel_cmd_;
  geometry_msgs::Vector3Stamped vel_tfed_;

  RampFilter<double> *ramp_x, *ramp_y, *ramp_w;

  bool state_changed_{};
  StandardState state_ = PASSIVE;
  ros::Subscriber chassis_cmd_subscriber_;
  ros::Subscriber vel_cmd_subscriber_;
  realtime_tools::RealtimeBuffer<rm_msgs::ChassisCmd> chassis_rt_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> vel_rt_buffer_;
  rm_msgs::ChassisCmd cmd_;
};
}
#endif //RM_CHASSIS_CONTROLLER_STANDARD_H
