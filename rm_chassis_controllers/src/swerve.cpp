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
    module.ctrl_pivot_->setCommand(std::abs(a) < std::abs(b) ? vel_angle : vel_angle + M_PI);
    module.ctrl_wheel_->setCommand(vel.norm() / module.wheel_radius_ * std::cos(a));
    module.ctrl_pivot_->update(time, period);
    module.ctrl_wheel_->update(time, period);
  }
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

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SwerveController, controller_interface::ControllerBase)
}  // namespace rm_chassis_controllers
