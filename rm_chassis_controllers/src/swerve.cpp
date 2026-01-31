/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 4/23/21.
//

#include "rm_chassis_controllers/swerve.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
bool SwerveController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                            ros::NodeHandle& controller_nh)
{
  if (!ChassisBase::init(robot_hw, root_nh, controller_nh))
    return false;
  XmlRpc::XmlRpcValue modules;
  controller_nh.getParam("modules", modules);
  ROS_ASSERT(modules.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (const auto& module : modules)
  {
    ROS_ASSERT(module.second.hasMember("position"));
    ROS_ASSERT(module.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(module.second["position"].size() == 2);
    ROS_ASSERT(module.second.hasMember("pivot"));
    ROS_ASSERT(module.second["pivot"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(module.second.hasMember("wheel"));
    ROS_ASSERT(module.second["wheel"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(module.second["wheel"].hasMember("radius"));

    Module m{ .position_ = Vec2<double>((double)module.second["position"][0], (double)module.second["position"][1]),
              .pivot_offset_ = module.second["pivot"]["offset"],
              .pivot_buffer_threshold_ = module.second["pivot"]["buffer_threshold"],
              .pivot_effort_threshold_ = module.second["pivot"]["effort_threshold"],
              .pivot_position_error_threshold_ = module.second["pivot"]["position_error_threshold"],
              .pivot_max_reduce_cnt_ = module.second["pivot"]["max_reduce_cnt"],
              .wheel_radius_ = module.second["wheel"]["radius"],
              .ctrl_pivot_ = new effort_controllers::JointPositionController(),
              .ctrl_wheel_ = new effort_controllers::JointVelocityController() };
    ros::NodeHandle nh_pivot = ros::NodeHandle(controller_nh, "modules/" + module.first + "/pivot");
    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "modules/" + module.first + "/wheel");
    if (!m.ctrl_pivot_->init(effort_joint_interface_, nh_pivot) ||
        !m.ctrl_wheel_->init(effort_joint_interface_, nh_wheel))
      return false;
    if (module.second["pivot"].hasMember("offset"))
      m.pivot_offset_ = module.second["pivot"]["offset"];
    joint_handles_.push_back(m.ctrl_pivot_->joint_);
    joint_handles_.push_back(m.ctrl_wheel_->joint_);
    modules_.push_back(m);
  }
  power_manager_sub_ = root_nh.subscribe<rm_msgs::PowerHeatData>("/rm_referee/power_manager", 10,
                                                                 &SwerveController::powerManagerCallback, this);
  return true;
}

// Ref: https://dominik.win/blog/programming-swerve-drive/

void SwerveController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  Vec2<double> vel_center(vel_cmd_.x, vel_cmd_.y);
  for (auto& module : modules_)
  {
    Vec2<double> vel = vel_center + vel_cmd_.z * Vec2<double>(-module.position_.y(), module.position_.x());
    double vel_angle = std::atan2(vel.y(), vel.x()) + module.pivot_offset_;
    // Direction flipping and Stray module mitigation
    double a = angles::shortest_angular_distance(module.ctrl_pivot_->joint_.getPosition(), vel_angle);
    double b = angles::shortest_angular_distance(module.ctrl_pivot_->joint_.getPosition(), vel_angle + M_PI);
    double target_pos = std::abs(a) < std::abs(b) ? vel_angle : vel_angle + M_PI;
    double pos_error = angles::shortest_angular_distance(module.ctrl_pivot_->joint_.getPosition(), target_pos);
    if (chassis_power_buffer_ > module.pivot_buffer_threshold_ &&
        !isPivotBlock(module.ctrl_pivot_->joint_.getEffort(), pos_error, module))
    {
      pivot_block_cnt_ = 0;
      module.ctrl_pivot_->setCommand(target_pos);
    }
    else
    {
      reduceTargetPosition(target_pos, pos_error, module);
      module.ctrl_pivot_->setCommand(target_pos);
    }

    module.ctrl_wheel_->setCommand(vel.norm() / module.wheel_radius_ * std::cos(a));
    module.ctrl_pivot_->update(time, period);
    module.ctrl_wheel_->update(time, period);
  }
}

bool SwerveController::isPivotBlock(double cur_effort, double position_error, Module module)
{
  return abs(cur_effort) > module.pivot_effort_threshold_ &&
         abs(position_error) > module.pivot_position_error_threshold_;
}

void SwerveController::reduceTargetPosition(double& target_pos, double& position_error, Module module)
{
  pivot_block_cnt_++;
  if (pivot_block_cnt_ > module.pivot_max_reduce_cnt_)
    pivot_block_cnt_ = module.pivot_max_reduce_cnt_;
  double reduce_step_size = position_error / module.pivot_max_reduce_cnt_;

  if (abs(position_error) > abs(pivot_block_cnt_ * reduce_step_size))
    target_pos -= pivot_block_cnt_ * reduce_step_size;
  else
    target_pos = module.ctrl_pivot_->joint_.getPosition();
}

geometry_msgs::Twist SwerveController::odometry()
{
  geometry_msgs::Twist vel_data{};
  geometry_msgs::Twist vel_modules{};
  for (auto& module : modules_)
  {
    geometry_msgs::Twist vel;
    vel.linear.x = module.ctrl_wheel_->joint_.getVelocity() * module.wheel_radius_ *
                   std::cos(module.ctrl_pivot_->joint_.getPosition());
    vel.linear.y = module.ctrl_wheel_->joint_.getVelocity() * module.wheel_radius_ *
                   std::sin(module.ctrl_pivot_->joint_.getPosition());
    vel.angular.z =
        module.ctrl_wheel_->joint_.getVelocity() * module.wheel_radius_ *
        std::cos(module.ctrl_pivot_->joint_.getPosition() - std::atan2(module.position_.x(), -module.position_.y()));
    vel_modules.linear.x += vel.linear.x;
    vel_modules.linear.y += vel.linear.y;
    vel_modules.angular.z += vel.angular.z;
  }
  vel_data.linear.x = vel_modules.linear.x / modules_.size();
  vel_data.linear.y = vel_modules.linear.y / modules_.size();
  vel_data.angular.z =
      vel_modules.angular.z / modules_.size() /
      std::sqrt(std::pow(modules_.begin()->position_.x(), 2) + std::pow(modules_.begin()->position_.y(), 2));
  return vel_data;
}

void SwerveController::powerManagerCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
{
  chassis_power_buffer_ = data->chassis_power_buffer;
}

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SwerveController, controller_interface::ControllerBase)
}  // namespace rm_chassis_controllers
