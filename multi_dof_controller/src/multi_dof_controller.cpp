//
// Created by lsy on 23-3-15.
//

#include "multi_dof_controller/multi_dof_controller.h"

#include <string>
#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

namespace multi_dof_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue joints;
  controller_nh.getParam("joints", joints);
  ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  for (const auto& joint : joints)
  {
    Joint j{ .joint_name_ = joint.first,
             .ctrl_position_ = new effort_controllers::JointPositionController(),
             .ctrl_velocity_ = new effort_controllers::JointVelocityController() };
    ros::NodeHandle nh_position = ros::NodeHandle(controller_nh, "joints/" + joint.first + "/position");
    ros::NodeHandle nh_velocity = ros::NodeHandle(controller_nh, "joints/" + joint.first + "/velocity");
    if (!j.ctrl_position_->init(effort_joint_interface_, nh_position) ||
        !j.ctrl_velocity_->init(effort_joint_interface_, nh_velocity))
      return false;
    joint_handles_.push_back(j.ctrl_position_->joint_);
    joint_handles_.push_back(j.ctrl_velocity_->joint_);
    joints_.push_back(j);
  }
  XmlRpc::XmlRpcValue motions;
  controller_nh.getParam("motions", motions);
  for (const auto& motion : motions)
  {
    Motion m{ .motion_name_ = motion.first,
              .position_per_step_ = xmlRpcGetDouble(motion.second["position_per_step"]),
              .velocity_max_speed_ = xmlRpcGetDouble(motion.second["velocity_max_speed"]) };
    for (int i = 0; i < (int)motion.second["position_config"].size(); ++i)
    {
      ROS_ASSERT(motion.second["position_config"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(motion.second["velocity_config"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(motion.second["is_position_need_reverse"][i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(motion.second["is_velocity_need_reverse"][i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      m.position_config_.push_back(xmlRpcGetDouble(motion.second["position_config"][i]));
      m.velocity_config_.push_back(xmlRpcGetDouble(motion.second["velocity_config"][i]));
      m.is_position_need_reverse_.push_back(xmlRpcGetDouble(motion.second["is_position_need_reverse"][i]));
      m.is_velocity_need_reverse_.push_back(xmlRpcGetDouble(motion.second["is_velocity_need_reverse"][i]));
    }
    motions_.push_back(m);
  }
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  cmd_multi_dof_sub_ = controller_nh.subscribe("command", 1, &Controller::commandCB, this);
  return true;
}

void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = VELOCITY;
  state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_last_ = cmd_multi_dof_;
  motion_group_.clear();
  motion_group_values_.clear();
  cmd_multi_dof_ = *cmd_rt_buffer_.readFromRT();
  judgeMotionGroup(cmd_multi_dof_);
  if (state_ != cmd_multi_dof_.mode)
  {
    state_ = cmd_multi_dof_.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case VELOCITY:
      velocity(time, period);
      break;
    case POSITION:
      position(time, period);
      break;
  }
}

void Controller::velocity(const ros::Time& time, const ros::Duration& period)
{
  position_change_ = true;
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Multi_Dof] VELOCITY");
  }
  std::vector<double> results((double)joints_.size(), 0);
  for (int i = 0; i < (int)joints_.size(); ++i)
  {
    for (int j = 0; j < (int)motion_group_.size(); ++j)
    {
      for (int k = 0; k < (int)motions_.size(); ++k)
      {
        if (motions_[k].motion_name_ == motion_group_[j])
          results[i] += judgeReverse(motion_group_values_[j], motions_[k].is_velocity_need_reverse_[i]) *
                        motions_[k].velocity_max_speed_ * motions_[k].velocity_config_[i];
      }
    }
  }
  for (int i = 0; i < (int)joints_.size(); ++i)
  {
    joints_[i].ctrl_velocity_->setCommand(results[i]);
    joints_[i].ctrl_velocity_->update(time, period);
  }
}
void Controller::position(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Multi_Dof] POSITION");
  }
  if (position_change_)
  {
    start_time_ = ros::Time::now();
    std::vector<double> current_positions((double)joints_.size(), 0);
    std::vector<double> results((double)joints_.size(), 0);
    targets_ = results;
    for (int i = 0; i < (int)joints_.size(); ++i)
    {
      current_positions[i] = (joints_[i].ctrl_position_->getPosition());
      for (int j = 0; j < (int)motion_group_.size(); ++j)
      {
        for (int k = 0; k < (int)motions_.size(); ++k)
        {
          if (motions_[k].motion_name_ == motion_group_[j])
          {
            results[i] += judgeReverse(motion_group_values_[j], motions_[k].is_velocity_need_reverse_[i]) /
                          motions_[k].position_per_step_ * motions_[k].position_config_[i];
            ROS_INFO_STREAM(results[i]);
          }
        }
      }
      position_change_ = false;
      targets_[i] = results[i] + current_positions[i];
    }
  }
  double get = 0;
  for (int i = 0; i < (int)joints_.size(); ++i)
  {
    if (targets_[i] - tolerance_ <= joints_[i].ctrl_position_->getPosition() &&
        targets_[i] + tolerance_ >= joints_[i].ctrl_position_->getPosition())
    {
      get++;
      targets_[i] = joints_[i].ctrl_position_->getPosition();
    }
    joints_[i].ctrl_position_->setCommand(targets_[i]);
    joints_[i].ctrl_position_->update(time, period);
  }
  if (get == joints_.size() || (ros::Time::now() - start_time_).toSec() >= time_out_)
    position_change_ = true;
}

void Controller::moveJoint()
{
}
double Controller::judgeReverse(double value, bool is_need_reverse)
{
  if (!is_need_reverse)
    value = abs(value);
  return value;
}
void Controller::judgeMotionGroup(rm_msgs::MultiDofCmd msg)
{
  if (abs(msg.values.linear.x))
  {
    motion_group_.push_back("x");
    motion_group_values_.push_back(msg.values.linear.x);
  }
  if (abs(msg.values.linear.y))
  {
    motion_group_.push_back("y");
    motion_group_values_.push_back(msg.values.linear.y);
  }
  if (abs(msg.values.linear.z))
  {
    motion_group_.push_back("z");
    motion_group_values_.push_back(msg.values.linear.z);
  }
  if (abs(msg.values.angular.x))
  {
    motion_group_.push_back("roll");
    motion_group_values_.push_back(msg.values.angular.x);
  }
  if (abs(msg.values.angular.y))
  {
    motion_group_.push_back("pitch");
    motion_group_values_.push_back(msg.values.angular.y);
  }
  if (abs(msg.values.angular.z))
  {
    motion_group_.push_back("yaw");
    motion_group_values_.push_back(msg.values.angular.z);
  }
}

void Controller::commandCB(const rm_msgs::MultiDofCmdPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
}
}  // namespace multi_dof_controller
PLUGINLIB_EXPORT_CLASS(multi_dof_controller::Controller, controller_interface::ControllerBase)
