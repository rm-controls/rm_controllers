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
// Created by qiayuan on 1/16/21.
//
#include "rm_gimbal_controllers/gimbal_base.h"

#include <string>
#include <angles/angles.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_gimbal_controllers
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  ros::NodeHandle nh_bullet_solver = ros::NodeHandle(controller_nh, "bullet_solver");
  bullet_solver_ = new BulletSolver(nh_bullet_solver);

  ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch))
    return false;
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu"));
  hardware_interface::ImuSensorInterface* imu_sensor_interface =
      robot_hw->get<hardware_interface::ImuSensorInterface>();
  imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_);

  gimbal_des_frame_id_ = ctrl_pitch_.joint_urdf_->child_link_name + "_des";
  map2gimbal_des_.header.frame_id = "map";
  map2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  map2gimbal_des_.transform.rotation.w = 1.;
  map2pitch_.header.frame_id = "map";
  map2pitch_.child_frame_id = ctrl_pitch_.joint_urdf_->child_link_name;
  map2pitch_.transform.rotation.w = 1.;
  map2base_.header.frame_id = "map";
  map2base_.child_frame_id = ctrl_yaw_.joint_urdf_->parent_link_name;
  map2base_.transform.rotation.w = 1.;

  cmd_gimbal_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this);
  publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error", 100));

  return true;
}

void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = RATE;
  state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  try
  {
    map2pitch_ = robot_state_handle_.lookupTransform("map", ctrl_pitch_.joint_urdf_->child_link_name, time);
    map2base_ = robot_state_handle_.lookupTransform("map", ctrl_yaw_.joint_urdf_->parent_link_name, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  if (state_ != cmd_gimbal_.mode)
  {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case RATE:
      rate(time, period);
      break;
    case TRACK:
      track(time);
      break;
    case DIRECT:
      direct(time);
      break;
  }
  moveJoint(time, period);
}

void Controller::rate(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    map2gimbal_des_.transform.rotation = map2pitch_.transform.rotation;
    map2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controllers");
  }
  else
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(map2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
  }
}

void Controller::track(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  double roll_real, pitch_real, yaw_real;
  quatToRPY(map2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  double yaw_compute = yaw_real;
  double pitch_compute = -pitch_real;
  geometry_msgs::Point target_pos = cmd_gimbal_.target_pos.point;
  geometry_msgs::Vector3 target_vel = cmd_gimbal_.target_vel.vector;
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty())
      tf2::doTransform(target_pos, target_pos,
                       robot_state_handle_.lookupTransform("map", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
    if (!cmd_gimbal_.target_vel.header.frame_id.empty())
      tf2::doTransform(target_vel, target_vel,
                       robot_state_handle_.lookupTransform("map", cmd_gimbal_.target_vel.header.frame_id,
                                                           cmd_gimbal_.target_vel.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  bool solve_success = bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed);

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (error_pub_->trylock())
    {
      double error =
          bullet_solver_->getGimbalError(target_pos, target_vel, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
      error_pub_->msg_.stamp = time;
      error_pub_->msg_.error = solve_success ? error : 1.0;
      error_pub_->unlockAndPublish();
    }
    bullet_solver_->bulletModelPub(map2pitch_, time);
    last_publish_time_ = time;
  }

  if (solve_success)
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
  else
  {
    map2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controllers");
  }
}

void Controller::direct(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter DIRECT");
  }
  geometry_msgs::Point aim_point_map = cmd_gimbal_.target_pos.point;
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty())
      tf2::doTransform(aim_point_map, aim_point_map,
                       robot_state_handle_.lookupTransform("map", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  double yaw = std::atan2(aim_point_map.y - map2pitch_.transform.translation.y,
                          aim_point_map.x - map2pitch_.transform.translation.x);
  double pitch = -std::atan2(aim_point_map.z - map2pitch_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_map.x - map2pitch_.transform.translation.x, 2) +
                                       std::pow(aim_point_map.y - map2pitch_.transform.translation.y, 2)));
  setDes(time, yaw, pitch);
}

void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des)
{
  double roll_base{}, pitch_base{}, yaw_base{};
  quatToRPY(map2base_.transform.rotation, roll_base, pitch_base, yaw_base);
  double roll_now{}, pitch_now{}, yaw_now{};
  quatToRPY(map2gimbal_des_.transform.rotation, roll_now, pitch_now, yaw_now);
  double pitch_delta = angles::shortest_angular_distance(pitch_base, pitch_des);
  double yaw_delta = angles::shortest_angular_distance(yaw_base, yaw_des);
  map2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(
      0.,
      (pitch_delta <= ctrl_pitch_.joint_urdf_->limits->upper && pitch_delta >= ctrl_pitch_.joint_urdf_->limits->lower) ||
              (angles::two_pi_complement(pitch_delta) <= ctrl_pitch_.joint_urdf_->limits->upper &&
               angles::two_pi_complement(pitch_delta) >= ctrl_pitch_.joint_urdf_->limits->lower) ?
          pitch_des :
          pitch_now,
      (yaw_delta <= ctrl_yaw_.joint_urdf_->limits->upper && yaw_delta >= ctrl_yaw_.joint_urdf_->limits->lower) ||
              (angles::two_pi_complement(yaw_delta) <= ctrl_yaw_.joint_urdf_->limits->upper &&
               angles::two_pi_complement(yaw_delta) >= ctrl_yaw_.joint_urdf_->limits->lower) ?
          yaw_des :
          yaw_now);
  map2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controllers");
}

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
  gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
  gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
  gyro.z = imu_sensor_handle_.getAngularVelocity()[2];

  geometry_msgs::TransformStamped base_frame2des;
  try
  {
    base_frame2des =
        robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name, gimbal_des_frame_id_, time);
    tf2::doTransform(gyro, angular_vel_pitch,
                     robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
                                                         imu_sensor_handle_.getFrameId(), time));
    tf2::doTransform(gyro, angular_vel_yaw,
                     robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->child_link_name,
                                                         imu_sensor_handle_.getFrameId(), time));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(base_frame2des.transform.rotation, roll_des, pitch_des, yaw_des);

  ctrl_yaw_.setCommand(yaw_des <= ctrl_yaw_.joint_urdf_->limits->upper &&
                               yaw_des >= ctrl_yaw_.joint_urdf_->limits->lower ?
                           yaw_des :
                           angles::two_pi_complement(yaw_des),
                       ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z);
  ctrl_pitch_.setCommand(pitch_des <= ctrl_pitch_.joint_urdf_->limits->upper &&
                                 pitch_des >= ctrl_pitch_.joint_urdf_->limits->lower ?
                             pitch_des :
                             angles::two_pi_complement(pitch_des),
                         ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);
  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

}  // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
