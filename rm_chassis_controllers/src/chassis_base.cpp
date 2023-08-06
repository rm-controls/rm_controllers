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
// Created by huakang on 2021/3/21.
//
#include "rm_chassis_controllers/chassis_base.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/math_utilities.h>
#include <rm_common/ori_tool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <hardware_interface/imu_sensor_interface.h>

namespace rm_chassis_controllers
{
template class ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>;
template class ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                           hardware_interface::EffortJointInterface>;

template <typename... T>
bool ChassisBase<T...>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_) ||
      !controller_nh.getParam("power/vel_coeff", velocity_coeff_) ||
      !controller_nh.getParam("power/effort_coeff", effort_coeff_) ||
      !controller_nh.getParam("power/power_offset", power_offset_))
  {
    ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.02);
  twist_angular_ = getParam(controller_nh, "twist_angular", M_PI / 6);
  max_odom_vel_ = getParam(controller_nh, "max_odom_vel", 0);
  enable_odom_tf_ = getParam(controller_nh, "enable_odom_tf", true);
  publish_odom_tf_ = getParam(controller_nh, "publish_odom_tf", false);

  // Get and check params for covariances
  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";
  odom_pub_->msg_.twist.covariance = { static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0.,
                                       static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
                                       static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0.,
                                       static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
                                       static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0.,
                                       static_cast<double>(twist_cov_list[5]) };

  ramp_x_ = new RampFilter<double>(0, 0.001);
  ramp_y_ = new RampFilter<double>(0, 0.001);
  ramp_w_ = new RampFilter<double>(0, 0.001);

  // init odom tf
  if (enable_odom_tf_)
  {
    odom2base_.header.frame_id = "odom";
    odom2base_.header.stamp = ros::Time::now();
    odom2base_.child_frame_id = "base_link";
    odom2base_.transform.rotation.w = 1;
    tf_broadcaster_.init(root_nh);
    tf_broadcaster_.sendTransform(odom2base_);
  }
  world2odom_.setRotation(tf2::Quaternion::getIdentity());

  outside_odom_sub_ =
      controller_nh.subscribe<nav_msgs::Odometry>("/odometry", 10, &ChassisBase::outsideOdomCallback, this);
  cmd_chassis_sub_ =
      controller_nh.subscribe<rm_msgs::ChassisCmd>("/cmd_chassis", 1, &ChassisBase::cmdChassisCallback, this);
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

  if (controller_nh.hasParam("pid_follow"))
    if (!pid_follow_.init(ros::NodeHandle(controller_nh, "pid_follow")))
      return false;

  return true;
}

template <typename... T>
void ChassisBase<T...>::update(const ros::Time& time, const ros::Duration& period)
{
  rm_msgs::ChassisCmd cmd_chassis = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
  geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;

  if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_)
  {
    vel_cmd_.x = 0.;
    vel_cmd_.y = 0.;
    vel_cmd_.z = 0.;
  }
  else
  {
    ramp_x_->setAcc(cmd_chassis.accel.linear.x);
    ramp_y_->setAcc(cmd_chassis.accel.linear.y);
    ramp_x_->input(cmd_vel.linear.x);
    ramp_y_->input(cmd_vel.linear.y);
    vel_cmd_.x = ramp_x_->output();
    vel_cmd_.y = ramp_y_->output();
    vel_cmd_.z = cmd_vel.angular.z;
  }

  if (cmd_rt_buffer_.readFromRT()->cmd_chassis_.follow_source_frame.empty())
    follow_source_frame_ = "yaw";
  else
    follow_source_frame_ = cmd_rt_buffer_.readFromRT()->cmd_chassis_.follow_source_frame;
  if (cmd_rt_buffer_.readFromRT()->cmd_chassis_.command_source_frame.empty())
    command_source_frame_ = "yaw";
  else
    command_source_frame_ = cmd_rt_buffer_.readFromRT()->cmd_chassis_.command_source_frame;

  if (state_ != cmd_chassis.mode)
  {
    state_ = cmd_chassis.mode;
    state_changed_ = true;
  }

  updateOdom(time, period);

  switch (state_)
  {
    case RAW:
      raw();
      break;
    case FOLLOW:
      follow(time, period);
      break;
    case TWIST:
      twist(time, period);
      break;
  }

  ramp_w_->setAcc(cmd_chassis.accel.angular.z);
  ramp_w_->input(vel_cmd_.z);
  vel_cmd_.z = ramp_w_->output();

  moveJoint(time, period);
  powerLimit();
}

template <typename... T>
void ChassisBase<T...>::follow(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");

    recovery();
    pid_follow_.reset();
  }

  tfVelToBase(command_source_frame_);
  try
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform("base_link", follow_source_frame_, ros::Time(0)).transform.rotation,
              roll, pitch, yaw);
    double follow_error = angles::shortest_angular_distance(yaw, 0);
    pid_follow_.computeCommand(-follow_error, period);
    vel_cmd_.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

template <typename... T>
void ChassisBase<T...>::twist(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter TWIST");

    recovery();
    pid_follow_.reset();
  }
  tfVelToBase(command_source_frame_);
  try
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform("base_link", command_source_frame_, ros::Time(0)).transform.rotation,
              roll, pitch, yaw);

    double angle[4] = { -0.785, 0.785, 2.355, -2.355 };
    double off_set = 0.0;
    for (double i : angle)
    {
      if (std::abs(angles::shortest_angular_distance(yaw, i)) < 0.79)
      {
        off_set = i;
        break;
      }
    }
    double follow_error =
        angles::shortest_angular_distance(yaw, twist_angular_ * sin(2 * M_PI * time.toSec()) + off_set);

    pid_follow_.computeCommand(-follow_error, period);  // The actual output is opposite to the calculated value
    vel_cmd_.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

template <typename... T>
void ChassisBase<T...>::raw()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");

    recovery();
  }
  tfVelToBase(command_source_frame_);
}

template <typename... T>
void ChassisBase<T...>::updateOdom(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Twist vel_base = odometry();  // on base_link frame
  if (enable_odom_tf_)
  {
    geometry_msgs::Vector3 linear_vel_odom, angular_vel_odom;
    try
    {
      odom2base_ = robot_state_handle_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      tf_broadcaster_.sendTransform(odom2base_);  // TODO: For some reason, the sendTransform in init sometime not work?
      ROS_WARN("%s", ex.what());
      return;
    }
    odom2base_.header.stamp = time;
    // integral vel to pos and angle
    tf2::doTransform(vel_base.linear, linear_vel_odom, odom2base_);
    tf2::doTransform(vel_base.angular, angular_vel_odom, odom2base_);
    double length =
        std::sqrt(std::pow(linear_vel_odom.x, 2) + std::pow(linear_vel_odom.y, 2) + std::pow(linear_vel_odom.z, 2));
    if (length < max_odom_vel_)
    {
      // avoid nan vel
      odom2base_.transform.translation.x += linear_vel_odom.x * period.toSec();
      odom2base_.transform.translation.y += linear_vel_odom.y * period.toSec();
      odom2base_.transform.translation.z += linear_vel_odom.z * period.toSec();
    }
    length =
        std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) + std::pow(angular_vel_odom.z, 2));
    if (length > 0.001)
    {  // avoid nan quat
      tf2::Quaternion odom2base_quat, trans_quat;
      tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
      trans_quat.setRotation(tf2::Vector3(angular_vel_odom.x / length, angular_vel_odom.y / length,
                                          angular_vel_odom.z / length),
                             length * period.toSec());
      odom2base_quat = trans_quat * odom2base_quat;
      odom2base_quat.normalize();
      odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
    }
  }

  if (topic_update_)
  {
    auto* odom_msg = odom_buffer_.readFromRT();

    tf2::Transform world2sensor;
    world2sensor.setOrigin(
        tf2::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));
    world2sensor.setRotation(tf2::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                                             odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w));

    if (world2odom_.getRotation() == tf2::Quaternion::getIdentity())  // First received
    {
      tf2::Transform odom2sensor;
      try
      {
        geometry_msgs::TransformStamped tf_msg =
            robot_state_handle_.lookupTransform("odom", "livox_frame", odom_msg->header.stamp);
        tf2::fromMsg(tf_msg.transform, odom2sensor);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }
      world2odom_ = world2sensor * odom2sensor.inverse();
    }
    tf2::Transform base2sensor;
    try
    {
      geometry_msgs::TransformStamped tf_msg =
          robot_state_handle_.lookupTransform("base_link", "livox_frame", odom_msg->header.stamp);
      tf2::fromMsg(tf_msg.transform, base2sensor);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();
    odom2base_.transform.translation.x = odom2base.getOrigin().x();
    odom2base_.transform.translation.y = odom2base.getOrigin().y();
    odom2base_.transform.translation.z = odom2base.getOrigin().z();
    topic_update_ = false;
  }

  robot_state_handle_.setTransform(odom2base_, "rm_chassis_controllers");

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.twist.twist.linear.x = vel_base.linear.x;
      odom_pub_->msg_.twist.twist.linear.y = vel_base.linear.y;
      odom_pub_->msg_.twist.twist.angular.z = vel_base.angular.z;
      odom_pub_->unlockAndPublish();
    }
    if (enable_odom_tf_ && publish_odom_tf_)
      tf_broadcaster_.sendTransform(odom2base_);
    last_publish_time_ = time;
  }
}

template <typename... T>
void ChassisBase<T...>::recovery()
{
  ramp_x_->clear(vel_cmd_.x);
  ramp_y_->clear(vel_cmd_.y);
  ramp_w_->clear(vel_cmd_.z);
}

template <typename... T>
void ChassisBase<T...>::powerLimit()
{
  double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis_.power_limit;
  // Three coefficients of a quadratic equation in one variable
  double a = 0., b = 0., c = 0.;
  for (const auto& joint : joint_handles_)
  {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    if (joint.getName().find("wheel") != std::string::npos)  // The pivot joint of swerve drive doesn't need power limit
    {
      a += square(cmd_effort);
      b += std::abs(cmd_effort * real_vel);
      c += square(real_vel);
    }
  }
  a *= effort_coeff_;
  c = c * velocity_coeff_ - power_offset_ - power_limit;
  // Root formula for quadratic equation in one variable
  double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
  for (auto joint : joint_handles_)
    if (joint.getName().find("wheel") != std::string::npos)
    {
      joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
    }
}

template <typename... T>
void ChassisBase<T...>::tfVelToBase(const std::string& from)
{
  try
  {
    tf2::doTransform(vel_cmd_, vel_cmd_, robot_state_handle_.lookupTransform("base_link", from, ros::Time(0)));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

template <typename... T>
void ChassisBase<T...>::cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr& msg)
{
  cmd_struct_.cmd_chassis_ = *msg;
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

template <typename... T>
void ChassisBase<T...>::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_struct_.cmd_vel_ = *msg;
  cmd_struct_.stamp_ = ros::Time::now();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

template <typename... T>
void ChassisBase<T...>::outsideOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_buffer_.writeFromNonRT(*msg);
  topic_update_ = true;
}

}  // namespace rm_chassis_controllers
