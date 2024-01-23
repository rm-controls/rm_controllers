//
// Created by yezi on 24-1-11.
//

#pragma once

#include <XmlRpc.h>
#include <geometry_msgs/Vector3.h>
#include <urdf/model.h>
#include <rm_common/hardware_interface/robot_state_interface.h>

class FrictionCompensation
{
public:
  void init(XmlRpc::XmlRpcValue config);
  double output(double vel_act, double effort_cmd) const;

private:
  double resistance_{ 0. };
  double velocity_saturation_point_{ 0. }, effort_saturation_point_{ 0. };
};

class InputFeedforward
{
public:
  void init(XmlRpc::XmlRpcValue config);
  double output(double vel_desire, double accel_desire) const;

private:
  double k_v_{ 0. }, k_a_{ 0. };
};

class GravityCompensation
{
public:
  void init(XmlRpc::XmlRpcValue config);
  double output(rm_control::RobotStateHandle* robot_state_handle, const urdf::JointConstSharedPtr& joint_urdf,
                ros::Time time) const;

private:
  geometry_msgs::Vector3 mass_origin_{};
  double gravity_{};
  bool enable_gravity_compensation_ = false;
};

class BaseVelCompensation
{
public:
  void init(XmlRpc::XmlRpcValue config);
  double output(double base_angular_vel_z) const;

private:
  double k_chassis_vel_{ 0. };
};
