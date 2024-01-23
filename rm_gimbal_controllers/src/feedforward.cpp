//
// Created by yezi on 24-1-11.
//

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include "rm_gimbal_controllers/feedforward.h"

void FrictionCompensation::init(XmlRpc::XmlRpcValue config)
{
  ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  resistance_ = config.hasMember("resistance") ? static_cast<double>(config["resistance"]) : 0.;
  velocity_saturation_point_ =
      config.hasMember("velocity_saturation_point") ? static_cast<double>(config["velocity_saturation_point"]) : 0.;
  effort_saturation_point_ =
      config.hasMember("effort_saturation_point") ? static_cast<double>(config["effort_saturation_point"]) : 0.;
}

double FrictionCompensation::output(double vel_act, double effort_cmd) const
{
  double resistance_compensation = 0.;
  if (std::abs(vel_act) > velocity_saturation_point_)
    resistance_compensation = (vel_act > 0 ? 1 : -1) * resistance_;
  else if (std::abs(effort_cmd) > effort_saturation_point_)
    resistance_compensation = (effort_cmd > 0 ? 1 : -1) * resistance_;
  else
    resistance_compensation = effort_cmd * resistance_ / effort_saturation_point_;
  return resistance_compensation;
}

void InputFeedforward::init(XmlRpc::XmlRpcValue config)
{
  ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  k_v_ = config.hasMember("k_v") ? static_cast<double>(config["k_v"]) : 0.;
  k_a_ = config.hasMember("k_a") ? static_cast<double>(config["k_a"]) : 0.;
}

double InputFeedforward::output(double vel_desire, double accel_desire) const
{
  return k_v_ * vel_desire + k_a_ * accel_desire;
}

void GravityCompensation::init(XmlRpc::XmlRpcValue config)
{
  bool enable_feedforward;
  enable_feedforward = config.hasMember("gravity_compensation");
  if (enable_feedforward)
  {
    ROS_ASSERT(config["gravity_compensation"].hasMember("mass_origin"));
    ROS_ASSERT(config["gravity_compensation"].hasMember("gravity"));
    ROS_ASSERT(config["gravity_compensation"].hasMember("enable_gravity_compensation"));
  }
  mass_origin_.x = enable_feedforward ? static_cast<double>(config["gravity_compensation"]["mass_origin"][0]) : 0.;
  mass_origin_.z = enable_feedforward ? static_cast<double>(config["gravity_compensation"]["mass_origin"][2]) : 0.;
  gravity_ = enable_feedforward ? static_cast<double>(config["gravity_compensation"]["gravity"]) : 0.;
  enable_gravity_compensation_ =
      enable_feedforward && static_cast<bool>(config["gravity_compensation"]["enable_gravity_compensation"]);
}

double GravityCompensation::output(rm_control::RobotStateHandle* robot_state_handle,
                                   const urdf::JointConstSharedPtr& joint_urdf, ros::Time time) const
{
  Eigen::Vector3d gravity(0, 0, -gravity_);
  tf2::doTransform(gravity, gravity, robot_state_handle->lookupTransform(joint_urdf->child_link_name, "odom", time));
  Eigen::Vector3d mass_origin(mass_origin_.x, 0, mass_origin_.z);
  double feedforward = -mass_origin.cross(gravity).y();
  if (enable_gravity_compensation_)
  {
    Eigen::Vector3d gravity_compensation(0, 0, gravity_);
    tf2::doTransform(gravity_compensation, gravity_compensation,
                     robot_state_handle->lookupTransform(joint_urdf->child_link_name, joint_urdf->parent_link_name,
                                                         time));
    feedforward -= mass_origin.cross(gravity_compensation).y();
  }
  return feedforward;
}

void BaseVelCompensation::init(XmlRpc::XmlRpcValue config)
{
  k_chassis_vel_ = config.hasMember("k_chassis_vel") ? static_cast<double>(config["k_chassis_vel"]) : 0.;
}

double BaseVelCompensation::output(double base_angular_vel_z) const
{
  return -k_chassis_vel_ * base_angular_vel_z;
}
