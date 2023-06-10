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
  std::vector<bool> fixed_direction_;
  std::vector<double> position_;
  std::vector<double> velocity_;
};

class Controller : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,
                                                                         hardware_interface::EffortJointInterface>
{
public:
  Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void judgeMotionGroup();
  void commandCB(const rm_msgs::MultiDofCmdPtr& msg);
  double judgeInputDirection(double value, bool fixed_direction);
  void position(const ros::Time& time, const ros::Duration& period);
  void velocity(const ros::Time& time, const ros::Duration& period);

  int state_ = rm_msgs::MultiDofCmd::VELOCITY;
  bool state_changed_{}, position_change_{ true };
  double time_out_, position_tolerance_;
  std::vector<Joint> joints_{};
  std::vector<Motion> motions_{};
  std::vector<std::string> motion_group_{};
  std::vector<double> motion_group_values_{}, targets_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};

  ros::Time start_time_;
  ros::Subscriber cmd_multi_dof_sub_;
  realtime_tools::RealtimeBuffer<rm_msgs::MultiDofCmd> cmd_rt_buffer_{};
  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

  rm_msgs::MultiDofCmd cmd_multi_dof_{};
};

}  // namespace multi_dof_controller
