//
// Created by root on 2026/10/12.
//

#include "rm_chassis_controllers/auto_suspension.h"
#include <algorithm>

namespace rm_chassis_controllers
{
bool AutoSuspensionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                      ros::NodeHandle& controller_nh)
{
  if (!OmniController::init(robot_hw, root_nh, controller_nh)) return false;

  chassis_follow_sub_ =
      controller_nh.subscribe("/cmd_chassis", 10, &AutoSuspensionController::ChassisFollowCallBack, this);
  cmd_sub_ = controller_nh.subscribe("/cmd_auto_suspension", 1, &AutoSuspensionController::AutoSuspensionCallback, this);

  XmlRpc::XmlRpcValue suspension_legs;
  controller_nh.getParam("suspension_leg", suspension_legs);
  XmlRpc::XmlRpcValue flipper_joints;
  controller_nh.getParam("flipper_joints", flipper_joints);
  stretch_coff_A_ = getParam(controller_nh, "stretch_coff_A_", 0.);
  stretch_coff_k_ = getParam(controller_nh, "stretch_coff_k_", 0.);
  shrink_coff_A_ = getParam(controller_nh, "shrink_coff_A_", 0.);
  shrink_coff_k_ = getParam(controller_nh, "shrink_coff_k_", 0.);
  feedforward_offset = getParam(controller_nh, "feed_forward_offset", 0.);
  feedforward_effect_time_ = getParam(controller_nh, "feedforward_effect_time_", 0.);

  pitch_threshold_ = getParam(controller_nh, "pitch_threshold", 0.);
  pos_tolerance_ = getParam(controller_nh, "pos_tolerance", 0.0);
  flipper_cmd_vel_coff_ = getParam(controller_nh, "flipper_cmd_vel_coff", 0.);
  sus_cmd_vel_coff_ = getParam(controller_nh, "sus_cmd_vel_coff", 0.);
  sus_upper_pos_ = getParam(controller_nh, "sus_upper_pos", 0.);
  sus_lower_pos_ = getParam(controller_nh, "sus_lower_pos", 0.);
  flipper_upper_pos_ = getParam(controller_nh, "flipper_upper_pos", 0.);
  flipper_lower_pos_ = getParam(controller_nh, "flipper_lower_pos", 0.);

  for (const auto& suspension_leg : suspension_legs)
  {
    ros::NodeHandle nh_suspension = ros::NodeHandle(controller_nh, "suspension_leg/" + suspension_leg.first);
    active_sus_ctrls_.push_back(std::make_shared<effort_controllers::JointPositionController>());
    if (!active_sus_ctrls_.back()->init(effort_joint_interface_, nh_suspension))
    {
      ROS_ERROR("Failed to init active_suspension_joint");
      return false;
    }
  }
  
  for (const auto& flipper_joint : flipper_joints)
  {
    ros::NodeHandle nh_flipper = ros::NodeHandle(controller_nh, "flipper_joints/" + flipper_joint.first);
    flipper_ctrls_.push_back(std::make_shared<effort_controllers::JointPositionController>());
    if (!flipper_ctrls_.back()->init(effort_joint_interface_, nh_flipper))
    {
      ROS_ERROR("Failed to init flipper_joint");
      return false;
    }
  }
  
  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("chassis_imu")));
  return true;
}
void AutoSuspensionController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  OmniController::moveJoint(time, period);
  odom2base_q.setValue(imu_sensor_handle_.getOrientation()[0], imu_sensor_handle_.getOrientation()[1],
                       imu_sensor_handle_.getOrientation()[2], imu_sensor_handle_.getOrientation()[3]);
  odom2base.setRotation(odom2base_q);
  odom2base.setOrigin(tf2::Vector3(0, 0, 0));
  tf2::Matrix3x3(odom2base.getRotation()).getRPY(base_roll_, base_pitch_, base_yaw_);
  base_roll_rate_ = imu_sensor_handle_.getAngularVelocity()[0];
  base_pitch_rate_ = imu_sensor_handle_.getAngularVelocity()[1];
  base_yaw_rate_ = imu_sensor_handle_.getAngularVelocity()[2];
  // if (chassis_follow_frame_ == "yaw_reverse" && base_pitch_ < -0.35)
  // {
  //   protect_state_ = true;
  // }
  // else
  // {
  //   protect_state_ = false;
  // }
  const auto cmd = *cmd_buffer_.readFromRT();
  current_state_ = static_cast<State>(cmd.state);
  prompt_ = static_cast<Prompt>(cmd.prompt);
  use_manual_ = cmd.use_manual;
  reset_seq_ = cmd.reset_seq;

  if (use_manual_)
  {
    flipper_target_pos_ = 0.0;
    switch (current_state_)
    {
      case State::DOWN:
        sus_target_pos_ = 0.0;
        static_effort = -0.0;
        break;
      case State::MID:
        sus_target_pos_ = 0.40;
        static_effort = 8.0;
        break;
      case State::UP:
        sus_target_pos_ = 0.75;
        static_effort = 5.0;
        break;
    }
    if (current_state_ != last_state_)
    {
      last_state_ = current_state_;
      feedforward_timer = ros::Time::now();
    }
    if ((ros::Time::now() - feedforward_timer).toSec() < feedforward_effect_time_)
    {
      if (current_state_ == State::MID)
        feedforward_effort = 1.2 * (stretch_coff_A_ - 55.0 * sus_current_pos_);
      else if (current_state_ == State::UP)
        feedforward_effort = 1.5 * (stretch_coff_A_ + stretch_coff_k_ * sus_current_pos_);
      else if (current_state_ == State::DOWN)
        feedforward_effort = shrink_coff_A_ + shrink_coff_k_ * sus_current_pos_;
    }
    else
      feedforward_effort = 0.0;
  }
  else
  {
    static_effort = 0.0;
    const bool reset_requested = prompt_ != Prompt::CLIMB && reset_seq_ != last_reset_seq_;
    if (reset_requested)
    {
      reset(period);
      last_reset_seq_ = reset_seq_;
    }
    else
    {
      // Both flipper modes allow automatic leg leveling.
      climb(period);
    }

    if (prompt_ != Prompt::CLIMB)
    {
      prompt_ = Prompt::RESET;
      flipper_target_pos_ = flipper_lower_pos_;
    }

    const bool extending = !reset_requested && std::abs(pitch_) >= pitch_threshold_ &&
                           -sus_cmd_vel_coff_ * pitch_ > 0.0;
    should_release_ = prompt_ == Prompt::RESET && !extending &&
                      std::abs(sus_target_pos_ - sus_lower_pos_) <= pos_tolerance_ &&
                      std::abs(sus_current_pos_ - sus_lower_pos_) <= pos_tolerance_;

    const double error = sus_target_pos_ - sus_current_pos_;
    if (should_release_)
      feedforward_effort = 0.0;
    else if (error > pos_tolerance_)
      feedforward_effort = stretch_coff_A_ + stretch_coff_k_ * sus_current_pos_;
    else if (error < -pos_tolerance_)
      feedforward_effort = shrink_coff_A_ + shrink_coff_k_ * sus_current_pos_;
    else
      feedforward_effort = 0.0;
  }

  for (auto& ctrl : flipper_ctrls_)
  {
    ctrl->setCommand(flipper_target_pos_);
    ctrl->update(time, period);
  }
  for (auto& ctrl : active_sus_ctrls_)
  {
    if (should_release_ && !use_manual_ && prompt_ == Prompt::RESET)
    {
      ctrl->joint_.setCommand(0.0);
      sus_current_pos_ = ctrl->getPosition();
      continue;
    }
    ctrl->setCommand(sus_target_pos_);
    ctrl->update(time, period);
    sus_current_pos_ = ctrl->getPosition();
    auto cmd = ctrl->joint_.getCommand();
    cmd += feedforward_effort + feedforward_offset + static_effort;
    ctrl->joint_.setCommand(cmd);
  }
}

void AutoSuspensionController::reset(const ros::Duration& period)
{
  flipper_target_pos_ = flipper_lower_pos_;
  sus_target_pos_ = sus_lower_pos_;
}

void AutoSuspensionController::climb(const ros::Duration& period)
{
  flipper_target_pos_ = flipper_upper_pos_;
  if (std::abs(pitch_) >= pitch_threshold_)
  {
    sus_target_pos_ += sus_cmd_vel_coff_ * pitch_ * period.toSec();
  }
  sus_target_pos_ = std::max(sus_lower_pos_, std::min(sus_upper_pos_, sus_target_pos_));
}

void AutoSuspensionController::drop(const ros::Duration& period)
{
}

void AutoSuspensionController::AutoSuspensionCallback(const rm_msgs::ChassisAutoSuspension& msg)
{
  cmd_buffer_.writeFromNonRT(msg);
}

void AutoSuspensionController::ChassisFollowCallBack(const rm_msgs::ChassisCmd& msg)
{
  chassis_follow_frame_ = msg.follow_source_frame;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::AutoSuspensionController, controller_interface::ControllerBase)