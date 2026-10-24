//
// Created by root on 2026/10/12.
//

#pragma once

#include "rm_chassis_controllers/chassis_base.h"
#include "rm_chassis_controllers/omni.h"
#include <effort_controllers/joint_position_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/ChassisAutoSuspension.h>
#include <std_msgs/Bool.h>
#include <rm_common/ros_utilities.h>
#include <string>
#include <Eigen/QR>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
enum class State
{
  DOWN = 0,
  MID = 1,
  UP = 2
};

enum class Prompt
{
  RESET = 0,
  CLIMB
};

class AutoSuspensionController : public OmniController
{
public:
  AutoSuspensionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void AutoSuspensionCallback(const rm_msgs::ChassisAutoSuspension& msg);
  void ChassisFollowCallBack(const rm_msgs::ChassisCmd& msg);

private:
  void reset(const ros::Duration& period);
  void climb(const ros::Duration& period);
  void drop(const ros::Duration& period);
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;

  std::vector<std::shared_ptr<effort_controllers::JointPositionController>> active_sus_ctrls_;
  std::vector<std::shared_ptr<effort_controllers::JointPositionController>> flipper_ctrls_;
  std::vector<hardware_interface::JointHandle> active_suspension_joints_{};
  std::vector<hardware_interface::JointHandle> flipper_joints_{};

  ros::Subscriber chassis_follow_sub_;
  ros::Subscriber cmd_sub_;

  hardware_interface::ImuSensorHandle imu_sensor_handle_;

  double sus_current_pos_{ 0. };
  double sus_target_pos_{ 0. };
  double sus_last_target_{ 0. };
  double sus_delta_target_{ 0. };
  double flipper_target_pos_{ 0. };
  double feedforward_offset{ 0. };
  double feedforward_effort{ 0. };
  double stretch_coff_A_{ 0. };
  double stretch_coff_k_{ 0. };
  double shrink_coff_A_{ 0. };
  double shrink_coff_k_{ 0. };
  double feedforward_effect_time_{ 0. };
  double static_effort{ 0. };
  double base_roll_{}, base_pitch_{}, base_yaw_{}, base_roll_rate_{}, base_pitch_rate_{}, base_yaw_rate_{};
  double pitch_threshold_ = 0.0;
  double pos_tolerance_ = 0.0;
  double flipper_cmd_vel_coff_ = 0.0, sus_cmd_vel_coff_ = 0.0;
  double sus_upper_pos_{}, sus_lower_pos_{}, flipper_upper_pos_{}, flipper_lower_pos_{};
  bool has_imu_{};
  bool use_manual_ = false;
  bool should_release_ = false;
  int reset_seq_ = 0;
  int last_reset_seq_ = 0;
  std::string imu_name_{};
  std::string chassis_follow_frame_;

  ros::Time feedforward_timer;
  tf2::Quaternion odom2base_q;
  tf2::Transform odom2base;
  State current_state_{ State::DOWN };
  State last_state_{ State::DOWN };
  Prompt prompt_{ Prompt::RESET };

  realtime_tools::RealtimeBuffer<rm_msgs::ChassisAutoSuspension> cmd_buffer_;
};
}  // namespace rm_chassis_controllers
