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

namespace rm_chassis_controllers
{
template class ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>;
template class ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                           hardware_interface::EffortJointInterface>;

template <typename... T>
void ChassisBase<T...>::initialize_parameters(ros::NodeHandle& controller_nh)
{
  try
  {
    controller_nh.getParam("publish_rate", publish_rate_);
    controller_nh.getParam("publish_map_tf", publish_map_tf_);
    controller_nh.getParam("publish_odom_tf", publish_odom_tf_);
    controller_nh.getParam("gravity_estimation_offset", gravity_estimation_offset_);
    controller_nh.getParam("slam_topic", slam_topic_);
    controller_nh.getParam("localization_topic", localization_topic_);

    controller_nh.getParam("wheel_radius", wheel_radius_);
    controller_nh.getParam("twist_angular", twist_angular_);
    controller_nh.getParam("max_odom_vel", max_odom_vel_);
    controller_nh.getParam("timeout", timeout_);
    raw_yaw_feedforward_k_ = getParam(controller_nh, "raw_yaw_feedforward_k", 0.0);

    if (controller_nh.hasParam("pid_follow"))
      pid_follow_.init(ros::NodeHandle(controller_nh, "pid_follow"));
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Chassis parameter initialization failed: %s", e.what());
  }
}
template <typename... T>
bool ChassisBase<T...>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  initialize_parameters(controller_nh);
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);
  cmd_chassis_sub_ =
      controller_nh.subscribe<rm_msgs::ChassisCmd>("/cmd_chassis", 1, &ChassisBase::cmdChassisCallback, this);
  slam_sub_ = controller_nh.subscribe<nav_msgs::Odometry>(slam_topic_, 10, &ChassisBase::slamCallback, this);
  localization_sub_ = controller_nh.subscribe<geometry_msgs::TransformStamped>(
      localization_topic_, 10, &ChassisBase::localizationCallback, this);
  capacity_sub_ = root_nh.subscribe<rm_msgs::PowerManagementSampleAndStatusData>(capacity_topic_, 10,
                                                                                 &ChassisBase::capacityCallback, this);

  // Setup real_power from capacity publishers.
  auto chassis_power_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64>>(controller_nh, "power/chassis_power", 100);
  this->chassis_power_pub_ = std::move(chassis_power_publisher);

  // Setup odometry realtime publisher + odom message constant fields
  auto odometry_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(root_nh, "odom", 100);
  this->odometry_rt_pub_ = std::move(odometry_publisher);
  odometry_rt_pub_->msg_.header.frame_id = robot_odom_frame_id_;
  odometry_rt_pub_->msg_.child_frame_id = robot_base_frame_id_;
  odometry_rt_pub_->msg_.twist.covariance = { 0.001, 0., 0.,    0., 0.,    0., 0., 0.001, 0., 0.,    0., 0.,
                                              0.,    0., 0.001, 0., 0.,    0., 0., 0.,    0., 0.001, 0., 0.,
                                              0.,    0., 0.,    0., 0.001, 0., 0., 0.,    0., 0.,    0., 0.001 };

  ramp_x_ = std::make_unique<RampFilter<double>>(0, 0.001);
  ramp_y_ = std::make_unique<RampFilter<double>>(0, 0.001);
  ramp_w_ = std::make_unique<RampFilter<double>>(0, 0.001);

  if (publish_map_tf_)
  {
    global_map2robot_odom_.header.stamp = ros::Time::now();
    global_map2robot_odom_.header.frame_id = global_map_frame_id_;
    global_map2robot_odom_.child_frame_id = robot_odom_frame_id_;
    global_map2robot_odom_.transform.rotation.w = 1;
    brcst4global_map2robot_odom_.init(root_nh);
    brcst4global_map2robot_odom_.sendTransform(global_map2robot_odom_);

    global_map2camera_init_.header.stamp = ros::Time::now();
    global_map2camera_init_.header.frame_id = global_map_frame_id_;
    global_map2camera_init_.child_frame_id = "camera_init";
    global_map2camera_init_.transform.rotation.w = 1;
    brcst4global_map2camera_init_.init(root_nh);
    brcst4global_map2camera_init_.sendTransform(global_map2camera_init_);
  }

  if (publish_odom_tf_)
  {
    robot_odom2robot_base_.header.stamp = ros::Time::now();
    robot_odom2robot_base_.header.frame_id = robot_odom_frame_id_;
    robot_odom2robot_base_.child_frame_id = robot_base_frame_id_;
    global_map2robot_odom_.transform.rotation.w = 1;
    brcst4robot_odom2robot_base_.init(root_nh);
    brcst4robot_odom2robot_base_.sendTransform(robot_odom2robot_base_);
  }

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
    quatToRPY(
        robot_state_handle_.lookupTransform(robot_base_frame_id_, follow_source_frame_, ros::Time(0)).transform.rotation,
        roll, pitch, yaw);
    double follow_error = angles::shortest_angular_distance(yaw, 0);
    pid_follow_.computeCommand(-follow_error, period);
    vel_cmd_.z = pid_follow_.getCurrentCmd() + cmd_rt_buffer_.readFromRT()->cmd_chassis_.follow_vel_des;
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
    quatToRPY(robot_state_handle_.lookupTransform(robot_base_frame_id_, command_source_frame_, ros::Time(0))
                  .transform.rotation,
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
  double yaw_offset;
  if (command_source_frame_ == "yaw")
    yaw_offset = raw_yaw_feedforward_k_ * vel_cmd_.z;
  else
    yaw_offset = 0.;
  tfVelToBase(command_source_frame_, yaw_offset);
}

template <typename... T>
void ChassisBase<T...>::updateOdom(const ros::Time& time, const ros::Duration& period)
{
  if (publish_map_tf_)
  {
    if (!odom_initialized_)
    {
      try
      {
        geometry_msgs::TransformStamped global_map2lidar_odom =
            robot_state_handle_.lookupTransform(robot_base_frame_id_, lidar_base_frame_id_, ros::Time(0));
        T_global_map2lidar_odom_.setOrigin(tf2::Vector3(global_map2lidar_odom.transform.translation.x,
                                                        global_map2lidar_odom.transform.translation.y,
                                                        global_map2lidar_odom.transform.translation.z));
        if (gravity_estimation_offset_)
        {
          T_global_map2lidar_odom_.setRotation(tf2::Quaternion(0, 0, 0, 1));
          global_map2camera_init_.transform.translation = global_map2lidar_odom.transform.translation;
        }
        else
        {
          T_global_map2lidar_odom_.setRotation(
              tf2::Quaternion(global_map2lidar_odom.transform.rotation.x, global_map2lidar_odom.transform.rotation.y,
                              global_map2lidar_odom.transform.rotation.z, global_map2lidar_odom.transform.rotation.w));
          global_map2camera_init_.transform = global_map2lidar_odom.transform;
        }
        odom_initialized_ = true;
      }
      catch (...)
      {
        ROS_WARN("Failed to init robot_odom2lidar_odom.");
      }
    }

    if (localization_updated_)
    {
      try
      {
        localization_updated_ = false;
        const auto& localization = localization_rt_buffer_.readFromRT();
        T_global_map2lidar_odom_.setOrigin(tf2::Vector3(localization->transform.translation.x,
                                                        localization->transform.translation.y,
                                                        localization->transform.translation.z));
        T_global_map2lidar_odom_.setRotation(
            tf2::Quaternion(localization->transform.rotation.x, localization->transform.rotation.y,
                            localization->transform.rotation.z, localization->transform.rotation.w));
        global_map2camera_init_.transform = tf2::toMsg(T_global_map2lidar_odom_);
      }
      catch (...)
      {
        ROS_WARN("Failed to update localization offset.");
      }
    }
    ros::Time tmp_time = ros::Time::now();

    if (slam_updated_)
    {
      try
      {
        slam_updated_ = false;
        const auto& slam = slam_rt_buffer_.readFromRT();
        tmp_time = slam->header.stamp;
        T_lidar_odom2lidar_base_.setOrigin(
            tf2::Vector3(slam->pose.pose.position.x, slam->pose.pose.position.y, slam->pose.pose.position.z));
        T_lidar_odom2lidar_base_.setRotation(
            tf2::Quaternion(slam->pose.pose.orientation.x, slam->pose.pose.orientation.y, slam->pose.pose.orientation.z,
                            slam->pose.pose.orientation.w));

        robot_base2lidar_base_ =
            robot_state_handle_.lookupTransform(robot_base_frame_id_, lidar_base_frame_id_, slam->header.stamp);
        T_robot_base2lidar_base_.setOrigin(tf2::Vector3(robot_base2lidar_base_.transform.translation.x,
                                                        robot_base2lidar_base_.transform.translation.y,
                                                        robot_base2lidar_base_.transform.translation.z));
        T_robot_base2lidar_base_.setRotation(
            tf2::Quaternion(robot_base2lidar_base_.transform.rotation.x, robot_base2lidar_base_.transform.rotation.y,
                            robot_base2lidar_base_.transform.rotation.z, robot_base2lidar_base_.transform.rotation.w));

        auto tmp_robot_odom2robot_base =
            robot_state_handle_.lookupTransform(robot_odom_frame_id_, robot_base_frame_id_, slam->header.stamp);

        T_robot_odom_2robot_base_.setOrigin(tf2::Vector3(tmp_robot_odom2robot_base.transform.translation.x,
                                                         tmp_robot_odom2robot_base.transform.translation.y,
                                                         tmp_robot_odom2robot_base.transform.translation.z));
        T_robot_odom_2robot_base_.setRotation(tf2::Quaternion(
            tmp_robot_odom2robot_base.transform.rotation.x, tmp_robot_odom2robot_base.transform.rotation.y,
            tmp_robot_odom2robot_base.transform.rotation.z, tmp_robot_odom2robot_base.transform.rotation.w));

        T_global_map2robot_odom_ = T_global_map2lidar_odom_ * T_lidar_odom2lidar_base_ *
                                   T_robot_base2lidar_base_.inverse() * T_robot_odom_2robot_base_.inverse();

        global_map2robot_odom_.transform = tf2::toMsg(T_global_map2robot_odom_);
      }
      catch (...)
      {
        ROS_WARN("Failed to update global_map2robot_odom.");
      }
    }
    global_map2robot_odom_.header.stamp = tmp_time;
    global_map2camera_init_.header.stamp = tmp_time;
  }

  if (publish_odom_tf_)
  {
    try
    {
      robot_odom2robot_base_ =
          robot_state_handle_.lookupTransform(robot_odom_frame_id_, robot_base_frame_id_, ros::Time(0));
      robot_odom2robot_base_.header.stamp = time;
      geometry_msgs::Twist vel_base = odometry();  // on base_link frame
      geometry_msgs::Vector3 linear_vel_odom, angular_vel_odom;
      tf2::doTransform(vel_base.linear, linear_vel_odom, robot_odom2robot_base_);
      tf2::doTransform(vel_base.angular, angular_vel_odom, robot_odom2robot_base_);

      double length =
          std::sqrt(std::pow(linear_vel_odom.x, 2) + std::pow(linear_vel_odom.y, 2) + std::pow(linear_vel_odom.z, 2));
      if (length < max_odom_vel_)
      {  // avoid nan vel
        robot_odom2robot_base_.transform.translation.x += linear_vel_odom.x * period.toSec();
        robot_odom2robot_base_.transform.translation.y += linear_vel_odom.y * period.toSec();
        robot_odom2robot_base_.transform.translation.z += linear_vel_odom.z * period.toSec();
      }
      length = std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) +
                         std::pow(angular_vel_odom.z, 2));
      if (length > 0.001)
      {  // avoid nan quat
        tf2::Quaternion odom2base_quat, trans_quat;
        tf2::fromMsg(robot_odom2robot_base_.transform.rotation, odom2base_quat);
        trans_quat.setRotation(tf2::Vector3(angular_vel_odom.x / length, angular_vel_odom.y / length,
                                            angular_vel_odom.z / length),
                               length * period.toSec());
        odom2base_quat = trans_quat * odom2base_quat;
        odom2base_quat.normalize();
        robot_odom2robot_base_.transform.rotation = tf2::toMsg(odom2base_quat);
      }

      quatToRPY(robot_odom2robot_base_.transform.rotation, roll_, pitch_, yaw_);

      robot_state_handle_.setTransform(robot_odom2robot_base_, "rm_chassis_controllers");

      odometry_rt_pub_->msg_.header.stamp = time;
      odometry_rt_pub_->msg_.pose.pose.position.x = robot_odom2robot_base_.transform.translation.x;
      odometry_rt_pub_->msg_.pose.pose.position.y = robot_odom2robot_base_.transform.translation.y;
      odometry_rt_pub_->msg_.pose.pose.orientation.x = robot_odom2robot_base_.transform.rotation.x;
      odometry_rt_pub_->msg_.pose.pose.orientation.y = robot_odom2robot_base_.transform.rotation.y;
      odometry_rt_pub_->msg_.pose.pose.orientation.z = robot_odom2robot_base_.transform.rotation.z;
      odometry_rt_pub_->msg_.pose.pose.orientation.w = robot_odom2robot_base_.transform.rotation.w;
      odometry_rt_pub_->msg_.twist.twist.linear.x = linear_vel_odom.x;
      odometry_rt_pub_->msg_.twist.twist.linear.y = linear_vel_odom.y;
      odometry_rt_pub_->msg_.twist.twist.angular.z = angular_vel_odom.z;
    }
    catch (...)
    {
      ROS_WARN("Failed to update robot_odom2robot_base.");
    }
  }

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (publish_map_tf_)
    {
      brcst4global_map2robot_odom_.sendTransform(global_map2robot_odom_);
      brcst4global_map2camera_init_.sendTransform(global_map2camera_init_);
    }

    if (publish_odom_tf_)
    {
      brcst4robot_odom2robot_base_.sendTransform(robot_odom2robot_base_);
      if (odometry_rt_pub_->trylock())
        odometry_rt_pub_->unlockAndPublish();
    }

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
}

template <typename... T>
void ChassisBase<T...>::updatePowerStatus()
{
}

template <typename... T>
void ChassisBase<T...>::tfVelToBase(const std::string& from, double yaw_offset)
{
  try
  {
    geometry_msgs::TransformStamped transform = robot_state_handle_.lookupTransform("base_link", from, ros::Time(0));
    if (std::abs(yaw_offset) > 1e-9)
    {
      tf2::Quaternion rotation;
      tf2::fromMsg(transform.transform.rotation, rotation);
      tf2::Quaternion yaw_feedforward;
      yaw_feedforward.setRPY(0., 0., yaw_offset);
      rotation *= yaw_feedforward;
      rotation.normalize();
      transform.transform.rotation = tf2::toMsg(rotation);
    }
    tf2::doTransform(vel_cmd_, vel_cmd_, transform);
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
void ChassisBase<T...>::slamCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  slam_rt_buffer_.writeFromNonRT(*msg);
  slam_updated_ = true;
}

template <typename... T>
void ChassisBase<T...>::localizationCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  localization_rt_buffer_.writeFromNonRT(*msg);
  localization_updated_ = true;
}

template <typename... T>
void ChassisBase<T...>::capacityCallback(const rm_msgs::PowerManagementSampleAndStatusData::ConstPtr& msg)
{
  chassis_power_ = msg->chassis_power + msg->capacity_discharge_power;
  capacity_update_flag_ = true;
  if (chassis_power_pub_->trylock())
  {
    chassis_power_pub_->msg_.data = chassis_power_;
    chassis_power_pub_->unlockAndPublish();
  }
}

}  // namespace rm_chassis_controllers
