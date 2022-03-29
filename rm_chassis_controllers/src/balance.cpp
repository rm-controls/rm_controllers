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
// Created by chenzheng on 2021/2/23.
//
#include "rm_chassis_controllers/balance.h"

#include <string>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/BalanceState.h>

namespace rm_chassis_controllers
{
bool BalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  com_pitch_offset_ = getParam(controller_nh, "com_pitch_offset", 0.);

  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));

  joint_left_ =
      effort_joint_interface_->getHandle(getParam(controller_nh, "joint_left_name", std::string("joint_left")));
  joint_right_ =
      effort_joint_interface_->getHandle(getParam(controller_nh, "joint_right_name", std::string("joint_right")));
  joint_handles_.push_back(joint_left_);
  joint_handles_.push_back(joint_right_);

  XmlRpc::XmlRpcValue a, b, q, r;
  controller_nh.getParam("a", a);
  controller_nh.getParam("b", b);
  controller_nh.getParam("q", q);
  controller_nh.getParam("r", r);
  getK(a, b, q, r);
  state_real_pub_ = root_nh.advertise<rm_msgs::BalanceState>("state_real", 100);

  return true;
}

void BalanceController::update(const ros::Time& time, const ros::Duration& period)
{
  ChassisBase::update(time, period);

  rm_msgs::BalanceState state_real_msg;
  state_real_msg.header.stamp = time;
  state_real_msg.alpha = x_(0);
  state_real_msg.alpha_dot = x_(1);
  state_real_msg.vel = x_(2);
  state_real_msg.theta_dot = x_(3);
  state_real_msg.control_1 = joint_left_.getEffort();
  state_real_msg.control_2 = joint_right_.getEffort();
  state_real_pub_.publish(state_real_msg);
}

geometry_msgs::Twist BalanceController::forwardKinematics()
{
  geometry_msgs::Twist vel_data;
  double joint_left_velocity = joint_left_.getVelocity();
  double joint_right_velocity = joint_right_.getVelocity();
  vel_data.linear.x =
      ((joint_left_velocity + joint_right_velocity) / 2 + imu_sensor_handle_.getAngularVelocity()[1]) * wheel_radius_;
  vel_data.angular.z = (-joint_left_velocity + joint_right_velocity) * wheel_radius_ / wheel_base_;
  return vel_data;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  x_ref_(2) = vel_cmd_.x;
  x_ref_(3) = vel_cmd_.z;
  geometry_msgs::Quaternion quat;
  quat.x = imu_sensor_handle_.getOrientation()[0];
  quat.y = imu_sensor_handle_.getOrientation()[1];
  quat.z = imu_sensor_handle_.getOrientation()[2];
  quat.w = imu_sensor_handle_.getOrientation()[3];
  double roll{}, pitch{}, yaw{};
  quatToRPY(quat, roll, pitch, yaw);
  x_ << pitch + com_pitch_offset_, imu_sensor_handle_.getAngularVelocity()[1],
      forwardKinematics().linear.x + imu_sensor_handle_.getAngularVelocity()[1] * 0.147,
      imu_sensor_handle_.getAngularVelocity()[2];

  u_ = k_ * (x_ref_ - x_);
  joint_left_.setCommand(u_(0));
  joint_right_.setCommand(u_(1));
}

void BalanceController::getK(XmlRpc::XmlRpcValue a, XmlRpc::XmlRpcValue b, XmlRpc::XmlRpcValue q, XmlRpc::XmlRpcValue r)
{
  // Check a and q
  ROS_ASSERT(a.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(q.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(a.size() == STATE_DIM);
  ROS_ASSERT(q.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i)
  {
    ROS_ASSERT(a[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(q[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(a[i].size() == STATE_DIM);
    ROS_ASSERT(q[i].size() == STATE_DIM);
    for (int j = 0; j < STATE_DIM; ++j)
    {
      ROS_ASSERT(a[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 a[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(q[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 q[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (a[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        a_(i, j) = static_cast<double>(a[i][j]);
      else if (a[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        a_(i, j) = static_cast<int>(a[i][j]);
      if (q[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        q_(i, j) = static_cast<double>(q[i][j]);
      else if (q[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        q_(i, j) = static_cast<int>(q[i][j]);
    }
  }
  // Check b
  ROS_ASSERT(b.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(b.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i)
  {
    ROS_ASSERT(b[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(b[i].size() == CONTROL_DIM);
    for (int j = 0; j < CONTROL_DIM; ++j)
    {
      ROS_ASSERT(b[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 b[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (b[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        b_(i, j) = static_cast<double>(b[i][j]);
      else if (b[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        b_(i, j) = static_cast<int>(b[i][j]);
    }
  }
  // Check r
  ROS_ASSERT(r.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(r.size() == CONTROL_DIM);
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    ROS_ASSERT(r[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(r[i].size() == CONTROL_DIM);
    for (int j = 0; j < CONTROL_DIM; ++j)
    {
      ROS_ASSERT(r[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 r[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (r[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        r_(i, j) = static_cast<double>(r[i][j]);
      else if (r[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        r_(i, j) = static_cast<int>(r[i][j]);
    }
  }

  Lqr<double> lqr(a_, b_, q_, r_);
  lqr.computeK();
  k_ = lqr.getK();
}

}  // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
