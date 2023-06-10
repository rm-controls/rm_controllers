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
  position_tolerance_ = getParam(controller_nh, "position_tolerance", 0.01);
  time_out_ = getParam(controller_nh, "time_out", 1);
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
    for (int i = 0; i < (int)motion.second["position"].size(); ++i)
    {
      ROS_ASSERT(motion.second["position"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(motion.second["velocity"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(motion.second["fixed_direction"][i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      m.position_.push_back(xmlRpcGetDouble(motion.second["position"][i]));
      m.velocity_.push_back(xmlRpcGetDouble(motion.second["velocity"][i]));
      m.fixed_direction_.push_back(xmlRpcGetDouble(motion.second["fixed_direction"][i]));
    }
    motions_.push_back(m);
  }
  cmd_multi_dof_sub_ = controller_nh.subscribe("command", 1, &Controller::commandCB, this);
  return true;
}

void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = rm_msgs::MultiDofCmd::VELOCITY;
  state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  motion_group_.clear();
  motion_group_values_.clear();
  cmd_multi_dof_ = *cmd_rt_buffer_.readFromRT();
  judgeMotionGroup();
  if (state_ != cmd_multi_dof_.mode)
  {
    state_ = cmd_multi_dof_.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case rm_msgs::MultiDofCmd::VELOCITY:
      velocity(time, period);
      break;
    case rm_msgs::MultiDofCmd::POSITION:
      position(time, period);
      break;
  }
}

void Controller::velocity(const ros::Time& time, const ros::Duration& period)
{
  position_change_ = true;
  if (state_changed_)
  {
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
          results[i] += judgeInputDirection(motion_group_values_[j], motions_[k].fixed_direction_[i]) *
                        motions_[k].velocity_max_speed_ * motions_[k].velocity_[i];
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
  {
    state_changed_ = false;
    ROS_INFO("[Multi_Dof] POSITION");
  }
  if (position_change_)
  {
    start_time_ = time;
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
            results[i] += judgeInputDirection(motion_group_values_[j], motions_[k].fixed_direction_[i]) /
                          motions_[k].position_per_step_ * motions_[k].position_[i];
          }
        }
      }
      position_change_ = false;
      targets_[i] = results[i] + current_positions[i];
    }
  }
  double arrived_joint_num = 0;
  for (int i = 0; i < (int)joints_.size(); ++i)
  {
    if (targets_[i] - position_tolerance_ <= joints_[i].ctrl_position_->getPosition() &&
        targets_[i] + position_tolerance_ >= joints_[i].ctrl_position_->getPosition())
    {
      arrived_joint_num++;
      targets_[i] = joints_[i].ctrl_position_->getPosition();
    }
    joints_[i].ctrl_position_->setCommand(targets_[i]);
    joints_[i].ctrl_position_->update(time, period);
  }
  if (arrived_joint_num == (int)joints_.size() || (time - start_time_).toSec() >= time_out_)
    position_change_ = true;
}

double Controller::judgeInputDirection(double value, bool fixed_direction)
{
  if (fixed_direction)
    value = abs(value);
  return value;
}
void Controller::judgeMotionGroup()
{
  std::vector<std::string> motion_names = { "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z" };
  std::vector<double> motion_values = { cmd_multi_dof_.linear.x,  cmd_multi_dof_.linear.y,  cmd_multi_dof_.linear.z,
                                        cmd_multi_dof_.angular.x, cmd_multi_dof_.angular.y, cmd_multi_dof_.angular.z };
  for (int i = 0; i < (int)motion_names.size(); i++)
  {
    if (abs(motion_values[i]))
    {
      motion_group_.push_back(motion_names[i]);
      motion_group_values_.push_back(motion_values[i]);
    }
  }
}

void Controller::commandCB(const rm_msgs::MultiDofCmdPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
}
}  // namespace multi_dof_controller
PLUGINLIB_EXPORT_CLASS(multi_dof_controller::Controller, controller_interface::ControllerBase)
