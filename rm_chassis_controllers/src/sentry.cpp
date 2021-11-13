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
// Created by flying on 2021/1/18.
//
#include "rm_chassis_controllers/sentry.h"
#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
bool SentryController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                            ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "wheel");
  ros::NodeHandle nh_brake = ros::NodeHandle(controller_nh, "brake_joint");
  if( nh_brake.getParam("brake_angle", brake_angle_) && nh_brake.getParam("velocity_coefficient", vel_coff_) )
  {
      ROS_ERROR("Could not find parameters");
  }
  if (!ctrl_wheel_.init(effort_joint_interface_, nh_wheel))
    return false;
  if (!ctrl_brake_joint_.init(effort_joint_interface_, nh_brake))
    return false;
  run_state_ = NORMAL;
  joint_handles_.push_back(effort_joint_interface_->getHandle(ctrl_wheel_.getJointName()));
  joint_handles_.push_back(effort_joint_interface_->getHandle(ctrl_brake_joint_.getJointName()));
  return true;
}

void SentryController::update(const ros::Time& time, const ros::Duration& period)
{
    if ( last_vel_cmd_ * cmd_rt_buffer_.readFromRT()->cmd_vel_.linear.x < 0 )
    run_state_ = CATAPULT;
    last_vel_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_vel_.linear.x ;
  if (run_state_ == NORMAL)
  {
    ChassisBase::update(time, period);
    catapult_initial_velocity_ = ctrl_wheel_.joint_.getVelocity();
  }
  else
    catapult(time, period);
}

void SentryController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  ctrl_wheel_.setCommand(vel_cmd_.x / wheel_radius_);
  ctrl_brake_joint_.setCommand(0.);
  ctrl_wheel_.update(time, period);
  ctrl_brake_joint_.update(time, period);
}

void SentryController::catapult(const ros::Time& time, const ros::Duration& period)
{
  ctrl_brake_joint_.setCommand(catapult_initial_velocity_ > 0 ? brake_angle_ : -brake_angle_);
  if ((catapult_initial_velocity_ * ctrl_wheel_.joint_.getVelocity() < 0) &&
      (std::abs(ctrl_wheel_.joint_.getVelocity()) > std::abs(catapult_initial_velocity_ * vel_coff_)))
    run_state_ = NORMAL;
  if (run_state_ == CATAPULT)
  {
    ctrl_wheel_.joint_.setCommand(0.);
    ctrl_brake_joint_.update(time, period);
  }
}

geometry_msgs::Twist SentryController::forwardKinematics()
{
  geometry_msgs::Twist vel_data;
  vel_data.linear.x = ctrl_wheel_.joint_.getVelocity() * wheel_radius_;
  return vel_data;
}

}  // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SentryController, controller_interface::ControllerBase)
