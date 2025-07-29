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
#include <tf/transform_datatypes.h>

namespace rm_gimbal_controllers
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  bool enable_feedforward;
  enable_feedforward = controller_nh.getParam("feedforward", xml_rpc_value);
  if (enable_feedforward)
  {
    ROS_ASSERT(xml_rpc_value.hasMember("mass_origin"));
    ROS_ASSERT(xml_rpc_value.hasMember("gravity"));
    ROS_ASSERT(xml_rpc_value.hasMember("enable_gravity_compensation"));
  }
  mass_origin_.x = enable_feedforward ? (double)xml_rpc_value["mass_origin"][0] : 0.;
  mass_origin_.z = enable_feedforward ? (double)xml_rpc_value["mass_origin"][2] : 0.;
  gravity_ = enable_feedforward ? (double)xml_rpc_value["gravity"] : 0.;
  enable_gravity_compensation_ = enable_feedforward && (bool)xml_rpc_value["enable_gravity_compensation"];

  ros::NodeHandle chassis_vel_nh(controller_nh, "chassis_vel");
  chassis_vel_ = std::make_shared<ChassisVel>(chassis_vel_nh);
  ros::NodeHandle nh_bullet_solver = ros::NodeHandle(controller_nh, "bullet_solver");
  bullet_solver_ = std::make_shared<BulletSolver>(nh_bullet_solver);

  config_ = { .yaw_k_v_ = getParam(controller_nh, "controllers/yaw/k_v", 0.),
              .pitch_k_v_ = getParam(controller_nh, "controllers/pitch/k_v", 0.),
              .chassis_comp_a_ = getParam(controller_nh, "controllers/yaw/chassis_comp_a", 0.),
              .chassis_comp_b_ = getParam(controller_nh, "controllers/yaw/chassis_comp_b", 0.),
              .chassis_comp_c_ = getParam(controller_nh, "controllers/yaw/chassis_comp_c", 0.),
              .chassis_comp_d_ = getParam(controller_nh, "controllers/yaw/chassis_comp_d", 0.),
              .accel_pitch_ = getParam(controller_nh, "controllers/pitch/accel", 99.),
              .accel_yaw_ = getParam(controller_nh, "controllers/yaw/accel", 99.) };
  config_rt_buffer_.initRT(config_);
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (controller_nh.getParam("controllers", xml_rpc_value))
  {
    for (const auto& it : xml_rpc_value)
    {
      ros::NodeHandle nh = ros::NodeHandle(controller_nh, "controllers/" + it.first);
      ros::NodeHandle nh_pid_pos = ros::NodeHandle(controller_nh, "controllers/" + it.first + "/pid_pos");

      // Get URDF info about joint
      urdf::Model urdf;
      int axis;
      if (!urdf.initParamWithNodeHandle("robot_description", controller_nh))
      {
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }
      auto joint_urdf = urdf.getJoint(getParam(nh, "joint", std::string()));
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint in urdf");
        return false;
      }
      else
      {
        axis = (joint_urdf->axis.x == 1) * 0 + (joint_urdf->axis.y == 1) * 1 + (joint_urdf->axis.z == 1) * 2;
        joint_urdfs_.insert(std::make_pair(axis, joint_urdf));
      }

      ctrls_.insert(std::make_pair(axis, std::make_unique<effort_controllers::JointVelocityController>()));
      pid_pos_.insert(std::make_pair(axis, std::make_unique<control_toolbox::Pid>()));
      pos_des_in_limit_.insert(std::make_pair(axis, true));
      pos_state_pub_.insert(std::make_pair(
          axis, std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>>(nh, "pos_state", 1)));

      if (!ctrls_.at(axis)->init(effort_joint_interface, nh) || !pid_pos_.at(axis)->init(nh_pid_pos))
        return false;
    }
  }

  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  if (!controller_nh.hasParam("imu_name"))
    has_imu_ = false;
  if (has_imu_)
  {
    imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu"));
    hardware_interface::ImuSensorInterface* imu_sensor_interface =
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_);
  }
  else
  {
    ROS_INFO("Param imu_name has not set, use motors' data instead of imu.");
  }

  gimbal_des_frame_id_ = getGimbalFrameID(joint_urdfs_) + "_des";
  odom2gimbal_des_.header.frame_id = "odom";
  odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.;
  odom2gimbal_.header.frame_id = "odom";
  odom2gimbal_.child_frame_id = getGimbalFrameID(joint_urdfs_);
  odom2gimbal_.transform.rotation.w = 1.;
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = getBaseFrameID(joint_urdfs_);
  odom2base_.transform.rotation.w = 1.;

  cmd_gimbal_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this);
  data_track_sub_ = controller_nh.subscribe<rm_msgs::TrackData>("/track", 1, &Controller::trackCB, this);
  publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error", 100));

  return true;
}

void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = RATE;
  state_changed_ = true;
  start_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
  config_ = *config_rt_buffer_.readFromRT();
  try
  {
    odom2gimbal_ = robot_state_handle_.lookupTransform("odom", odom2gimbal_.child_frame_id, time);
    odom2base_ = robot_state_handle_.lookupTransform("odom", odom2base_.child_frame_id, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5, "%s\n", ex.what());
    return;
  }
  updateChassisVel();
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
    case TRAJ:
      traj(time);
      break;
  }
  moveJoint(time, period);
}

void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des)
{
  tf2::Quaternion odom2base, odom2gimbal_des;
  tf2::fromMsg(odom2base_.transform.rotation, odom2base);
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
  tf2::Quaternion base2gimbal_des = odom2base.inverse() * odom2gimbal_des;
  for (const auto& it : joint_urdfs_)
    pos_des_in_limit_[it.first] = setDesIntoLimit(base2gimbal_des, it.second, base2gimbal_des);
  odom2gimbal_des_.transform.rotation = tf2::toMsg(odom2base * base2gimbal_des);
  odom2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
}

void Controller::rate(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    if (start_)
    {
      odom2gimbal_des_.transform.rotation = odom2gimbal_.transform.rotation;
      odom2gimbal_des_.header.stamp = time;
      robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
      start_ = false;
    }
  }
  else
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
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
  quatToRPY(odom2gimbal_.transform.rotation, roll_real, pitch_real, yaw_real);
  double yaw_compute = yaw_real;
  double pitch_compute = -pitch_real;
  geometry_msgs::Point target_pos = data_track_.position;
  geometry_msgs::Vector3 target_vel{};
  if (data_track_.id != 12)
    target_vel = data_track_.velocity;
  try
  {
    if (!data_track_.header.frame_id.empty())
    {
      geometry_msgs::TransformStamped transform =
          robot_state_handle_.lookupTransform("odom", data_track_.header.frame_id, data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
    }
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  double yaw = data_track_.yaw + data_track_.v_yaw * ((time - data_track_.header.stamp).toSec());
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  target_pos.x += target_vel.x * (time - data_track_.header.stamp).toSec() - odom2gimbal_.transform.translation.x;
  target_pos.y += target_vel.y * (time - data_track_.header.stamp).toSec() - odom2gimbal_.transform.translation.y;
  target_pos.z += target_vel.z * (time - data_track_.header.stamp).toSec() - odom2gimbal_.transform.translation.z;
  target_vel.x -= chassis_vel_->linear_->x();
  target_vel.y -= chassis_vel_->linear_->y();
  target_vel.z -= chassis_vel_->linear_->z();
  bool solve_success = bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed, yaw, data_track_.v_yaw,
                                             data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                             data_track_.armors_num, chassis_vel_->angular_->z());
  bullet_solver_->judgeShootBeforehand(time, data_track_.v_yaw);

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (error_pub_->trylock())
    {
      double error =
          bullet_solver_->getGimbalError(target_pos, target_vel, data_track_.yaw, data_track_.v_yaw,
                                         data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                         data_track_.armors_num, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
      error_pub_->msg_.stamp = time;
      error_pub_->msg_.error = solve_success ? error : 1.0;
      error_pub_->unlockAndPublish();
    }
    bullet_solver_->bulletModelPub(odom2gimbal_, time);
    last_publish_time_ = time;
  }

  if (solve_success)
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
  else
  {
    odom2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
  }
}

void Controller::direct(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter DIRECT");
  }
  geometry_msgs::Point aim_point_odom = cmd_gimbal_.target_pos.point;
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty())
      tf2::doTransform(aim_point_odom, aim_point_odom,
                       robot_state_handle_.lookupTransform("odom", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  double yaw = std::atan2(aim_point_odom.y - odom2gimbal_.transform.translation.y,
                          aim_point_odom.x - odom2gimbal_.transform.translation.x);
  double pitch = -std::atan2(aim_point_odom.z - odom2gimbal_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_odom.x - odom2gimbal_.transform.translation.x, 2) +
                                       std::pow(aim_point_odom.y - odom2gimbal_.transform.translation.y, 2)));
  setDes(time, yaw, pitch);
}

void Controller::traj(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRAJ");
  }
  double traj[3]{ 0. };
  try
  {
    if (!cmd_gimbal_.traj_frame_id.empty())
    {
      geometry_msgs::TransformStamped odom2traj =
          robot_state_handle_.lookupTransform("odom", cmd_gimbal_.traj_frame_id, time);
      quatToRPY(odom2traj.transform.rotation, traj[0], traj[1], traj[2]);
      traj[1] += cmd_gimbal_.traj_pitch;
      traj[2] += cmd_gimbal_.traj_yaw;
    }
    else
    {
      traj[1] = cmd_gimbal_.traj_pitch;
      traj[2] = cmd_gimbal_.traj_yaw;
    }
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  setDes(time, traj[2], traj[1]);
}

bool Controller::setDesIntoLimit(const tf2::Quaternion& base2gimbal_des, const urdf::JointConstSharedPtr& joint_urdf,
                                 tf2::Quaternion& base2new_des)
{
  double base2gimbal_current_des[3];
  quatToRPY(toMsg(base2gimbal_des), base2gimbal_current_des[0], base2gimbal_current_des[1], base2gimbal_current_des[2]);
  double upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
  double lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;
  int index = (joint_urdf->axis.x == 1) * 0 + (joint_urdf->axis.y == 1) * 1 + (joint_urdf->axis.z == 1) * 2;
  if ((base2gimbal_current_des[index] <= upper_limit && base2gimbal_current_des[index] >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des[index]) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des[index]) >= lower_limit))
  {
    base2new_des = base2gimbal_des;
    return true;
  }
  else
  {
    base2gimbal_current_des[index] =
        std::abs(angles::shortest_angular_distance(base2gimbal_current_des[index], upper_limit)) <
                std::abs(angles::shortest_angular_distance(base2gimbal_current_des[index], lower_limit)) ?
            upper_limit :
            lower_limit;
    base2new_des.setRPY(base2gimbal_current_des[0], base2gimbal_current_des[1], base2gimbal_current_des[2]);
    return false;
  }
}

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel;
  if (has_imu_)
  {
    gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
    gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
    gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
    try
    {
      tf2::doTransform(gyro, angular_vel,
                       robot_state_handle_.lookupTransform(odom2gimbal_.child_frame_id, imu_sensor_handle_.getFrameId(),
                                                           time));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
  else
  {
    if (ctrls_.find(0) != ctrls_.end())
      angular_vel.x = ctrls_.at(0)->joint_.getVelocity();
    if (ctrls_.find(1) != ctrls_.end())
      angular_vel.y = ctrls_.at(1)->joint_.getVelocity();
    if (ctrls_.find(2) != ctrls_.end())
      angular_vel.z = ctrls_.at(2)->joint_.getVelocity();
  }
  double pos_real[3]{ 0. }, pos_des[3]{ 0. }, vel_des[3]{ 0. }, angle_error[3]{ 0. };
  quatToRPY(odom2gimbal_des_.transform.rotation, pos_des[0], pos_des[1], pos_des[2]);
  quatToRPY(odom2gimbal_.transform.rotation, pos_real[0], pos_real[1], pos_real[2]);
  for (int i = 0; i < 3; i++)
    angle_error[i] = angles::shortest_angular_distance(pos_real[i], pos_des[i]);
  if (state_ == RATE)
  {
    vel_des[2] = cmd_gimbal_.rate_yaw;
    vel_des[1] = cmd_gimbal_.rate_pitch;
  }
  else if (state_ == TRACK)
  {
    geometry_msgs::Point target_pos;
    geometry_msgs::Vector3 target_vel;
    if (data_track_.id != 12)
    {
      geometry_msgs::Point pos = data_track_.position;
      double yaw = data_track_.yaw + data_track_.v_yaw * ((time - data_track_.header.stamp).toSec());
      pos.x += data_track_.velocity.x * (time - data_track_.header.stamp).toSec();
      pos.y += data_track_.velocity.y * (time - data_track_.header.stamp).toSec();
      pos.z += data_track_.velocity.z * (time - data_track_.header.stamp).toSec();
      bullet_solver_->getSelectedArmorPosAndVel(target_pos, target_vel, pos, data_track_.velocity, yaw,
                                                data_track_.v_yaw, data_track_.radius_1, data_track_.radius_2,
                                                data_track_.dz, data_track_.armors_num);
    }
    else
    {
      target_pos = data_track_.position;
      target_vel = data_track_.velocity;
    }
    target_vel.x -= chassis_vel_->linear_->x();
    target_vel.y -= chassis_vel_->linear_->y();
    target_vel.z -= chassis_vel_->linear_->z();
    tf2::Vector3 target_pos_tf, target_vel_tf;
    try
    {
      geometry_msgs::TransformStamped transform;
      if (joint_urdfs_.find(2) != joint_urdfs_.end())
      {
        transform = robot_state_handle_.lookupTransform(odom2base_.child_frame_id, data_track_.header.frame_id,
                                                        data_track_.header.stamp);
        tf2::doTransform(target_pos, target_pos, transform);
        tf2::doTransform(target_vel, target_vel, transform);
        tf2::fromMsg(target_pos, target_pos_tf);
        tf2::fromMsg(target_vel, target_vel_tf);
        vel_des[2] = target_pos_tf.cross(target_vel_tf).z() / std::pow((target_pos_tf.length()), 2);
      }
      if (joint_urdfs_.find(1) != joint_urdfs_.end())
      {
        transform = robot_state_handle_.lookupTransform(joint_urdfs_.at(1)->parent_link_name,
                                                        data_track_.header.frame_id, data_track_.header.stamp);
        tf2::doTransform(target_pos, target_pos, transform);
        tf2::doTransform(target_vel, target_vel, transform);
        tf2::fromMsg(target_pos, target_pos_tf);
        tf2::fromMsg(target_vel, target_vel_tf);
        vel_des[1] = target_pos_tf.cross(target_vel_tf).y() / std::pow((target_pos_tf.length()), 2);
      }
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }
  for (const auto& in_limit : pos_des_in_limit_)
    if (!in_limit.second)
      vel_des[in_limit.first] = 0.;

  if (pid_pos_.find(1) != pid_pos_.end() && ctrls_.find(1) != ctrls_.end())
  {
    pid_pos_.at(1)->computeCommand(angle_error[1], period);
    ctrls_.at(1)->setCommand(pid_pos_.at(1)->getCurrentCmd() + config_.pitch_k_v_ * vel_des[1] +
                             ctrls_.at(1)->joint_.getVelocity() - angular_vel.y);
    ctrls_.at(1)->update(time, period);
    ctrls_.at(1)->joint_.setCommand(ctrls_.at(1)->joint_.getCommand() + feedForward(time));
  }
  if (pid_pos_.find(2) != pid_pos_.end() && ctrls_.find(2) != ctrls_.end())
  {
    pid_pos_.at(2)->computeCommand(angle_error[2], period);
    ctrls_.at(2)->setCommand(pid_pos_.at(2)->getCurrentCmd() -
                             updateCompensation(chassis_vel_->angular_->z()) * chassis_vel_->angular_->z() +
                             config_.yaw_k_v_ * vel_des[2] + ctrls_.at(2)->joint_.getVelocity() - angular_vel.z);
    ctrls_.at(2)->update(time, period);
  }

  // publish state
  if (loop_count_ % 10 == 0)
  {
    for (const auto& pub : pos_state_pub_)
    {
      if (pub.second && pub.second->trylock())
      {
        pub.second->msg_.header.stamp = time;
        pub.second->msg_.set_point = pos_des[pub.first];
        pub.second->msg_.set_point_dot = vel_des[pub.first];
        pub.second->msg_.process_value = pos_real[pub.first];
        pub.second->msg_.error = angle_error[pub.first];
        pub.second->msg_.command = pid_pos_[pub.first]->getCurrentCmd();
        pub.second->unlockAndPublish();
      }
    }
  }
  loop_count_++;
}

double Controller::feedForward(const ros::Time& time)
{
  if (joint_urdfs_.find(1) != joint_urdfs_.end())
  {
    Eigen::Vector3d gravity(0, 0, -gravity_);
    tf2::doTransform(gravity, gravity,
                     robot_state_handle_.lookupTransform(joint_urdfs_.at(1)->child_link_name, "base_link", time));
    Eigen::Vector3d mass_origin(mass_origin_.x, 0, mass_origin_.z);
    double feedforward = -mass_origin.cross(gravity).y();
    if (enable_gravity_compensation_)
    {
      Eigen::Vector3d gravity_compensation(0, 0, gravity_);
      tf2::doTransform(gravity_compensation, gravity_compensation,
                       robot_state_handle_.lookupTransform(joint_urdfs_.at(1)->child_link_name,
                                                           joint_urdfs_.at(1)->parent_link_name, time));
      feedforward -= mass_origin.cross(gravity_compensation).y();
    }
    return feedforward;
  }
  return 0.;
}

void Controller::updateChassisVel()
{
  double tf_period = odom2base_.header.stamp.toSec() - last_odom2base_.header.stamp.toSec();
  double linear_x = (odom2base_.transform.translation.x - last_odom2base_.transform.translation.x) / tf_period;
  double linear_y = (odom2base_.transform.translation.y - last_odom2base_.transform.translation.y) / tf_period;
  double linear_z = (odom2base_.transform.translation.z - last_odom2base_.transform.translation.z) / tf_period;
  double last_angular_position_x, last_angular_position_y, last_angular_position_z, angular_position_x,
      angular_position_y, angular_position_z;
  quatToRPY(odom2base_.transform.rotation, angular_position_x, angular_position_y, angular_position_z);
  quatToRPY(last_odom2base_.transform.rotation, last_angular_position_x, last_angular_position_y,
            last_angular_position_z);
  double angular_x = angles::shortest_angular_distance(last_angular_position_x, angular_position_x) / tf_period;
  double angular_y = angles::shortest_angular_distance(last_angular_position_y, angular_position_y) / tf_period;
  double angular_z = angles::shortest_angular_distance(last_angular_position_z, angular_position_z) / tf_period;
  double linear_vel[3]{ linear_x, linear_y, linear_z };
  double angular_vel[3]{ angular_x, angular_y, angular_z };
  chassis_vel_->update(linear_vel, angular_vel, tf_period);
  last_odom2base_ = odom2base_;
}

std::string Controller::getGimbalFrameID(std::unordered_map<int, urdf::JointConstSharedPtr> joint_urdfs)
{
  if (joint_urdfs.find(1) != joint_urdfs.end())
    return joint_urdfs.at(1)->child_link_name.c_str();
  if (joint_urdfs.find(0) != joint_urdfs.end())
    return joint_urdfs.at(0)->child_link_name.c_str();
  if (joint_urdfs.find(2) != joint_urdfs.end())
    return joint_urdfs.at(2)->child_link_name.c_str();
  return std::string();
}

std::string Controller::getBaseFrameID(std::unordered_map<int, urdf::JointConstSharedPtr> joint_urdfs)
{
  if (joint_urdfs.find(2) != joint_urdfs.end())
    return joint_urdfs.at(2)->parent_link_name.c_str();
  if (joint_urdfs.find(0) != joint_urdfs.end())
    return joint_urdfs.at(0)->parent_link_name.c_str();
  if (joint_urdfs.find(1) != joint_urdfs.end())
    return joint_urdfs.at(1)->parent_link_name.c_str();
  return std::string();
}

double Controller::updateCompensation(double chassis_vel_angular_z)
{
  chassis_compensation_ =
      config_.chassis_comp_a_ * sin(config_.chassis_comp_b_ * chassis_vel_angular_z + config_.chassis_comp_c_) +
      config_.chassis_comp_d_;
  return chassis_compensation_;
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::trackCB(const rm_msgs::TrackDataConstPtr& msg)
{
  if (msg->id == 0)
    return;
  track_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::reconfigCB(rm_gimbal_controllers::GimbalBaseConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Gimbal Base] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    GimbalConfig init_config = *config_rt_buffer_.readFromNonRT();  // config init use yaml
    config.yaw_k_v_ = init_config.yaw_k_v_;
    config.pitch_k_v_ = init_config.pitch_k_v_;
    config.chassis_comp_a_ = init_config.chassis_comp_a_;
    config.chassis_comp_b_ = init_config.chassis_comp_b_;
    config.chassis_comp_c_ = init_config.chassis_comp_c_;
    config.chassis_comp_d_ = init_config.chassis_comp_d_;
    config.accel_pitch_ = init_config.accel_pitch_;
    config.accel_yaw_ = init_config.accel_yaw_;
    dynamic_reconfig_initialized_ = true;
  }
  GimbalConfig config_non_rt{ .yaw_k_v_ = config.yaw_k_v_,
                              .pitch_k_v_ = config.pitch_k_v_,
                              .chassis_comp_a_ = config.chassis_comp_a_,
                              .chassis_comp_b_ = config.chassis_comp_b_,
                              .chassis_comp_c_ = config.chassis_comp_c_,
                              .chassis_comp_d_ = config.chassis_comp_d_,
                              .accel_pitch_ = config.accel_pitch_,
                              .accel_yaw_ = config.accel_yaw_ };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}

}  // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
