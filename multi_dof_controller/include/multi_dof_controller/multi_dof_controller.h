//
// Created by lsy on 23-3-15.
//

#pragma once

#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <rm_msgs/MultiDofCmd.h>

namespace multi_dof_controller
{
struct Joint
{
  std::string joint_name_;
  effort_controllers::JointPositionController* ctrl_position_;
  effort_controllers::JointVelocityController* ctrl_velocity_;
};
struct Motion
{
  std::string motion_name_;
  double position_per_step_;
  double velocity_max_speed_;
  std::vector<bool> is_need_reverse_;
  std::vector<double> position_config_;
  std::vector<double> velocity_config_;
};

class Controller : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,
                                                                         hardware_interface::EffortJointInterface>
{
public:
  enum
  {
    VELOCITY,
    POSITION
  };
  Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void judgeMotionGroup(rm_msgs::MultiDofCmd);
  void commandCB(const rm_msgs::MultiDofCmdPtr& msg);
  double judgeReverse(double value, bool is_need_reverse);
  void position(const ros::Time& time, const ros::Duration& period);
  void velocity(const ros::Time& time, const ros::Duration& period);

  int state_ = VELOCITY;
  bool state_changed_{};
  bool position_change_ = 1;
  double time_out_;
  double position_tolerance_;
  std::vector<Joint> joints_{};
  std::vector<double> targets_{};
  std::vector<Motion> motions_{};
  std::vector<std::string> motion_group_{};
  std::vector<double> motion_group_values_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};

  ros::Time start_time_;
  ros::Subscriber cmd_multi_dof_sub_;
  realtime_tools::RealtimeBuffer<rm_msgs::MultiDofCmd> cmd_rt_buffer_{};
  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

  rm_msgs::MultiDofCmd cmd_multi_dof_{};
  rm_control::RobotStateHandle robot_state_handle_;
};

}  // namespace multi_dof_controller
