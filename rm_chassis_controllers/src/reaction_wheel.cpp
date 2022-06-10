//
// Created by qiayuan on 2022/4/17.
//
#include "rm_chassis_controllers/reaction_wheel.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <geometry_msgs/Quaternion.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
bool ReactionWheelController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  if (!MultiInterfaceController::init(robot_hw, root_nh, controller_nh))
    return false;

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));
  joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(
      getParam(controller_nh, "joint", std::string("reaction_wheel_joint")));

  left_wheel_handle_ = robot_hw->get<hardware_interface::JointStateInterface>()->getHandle("left_wheel_joint");
  right_wheel_handle_ = robot_hw->get<hardware_interface::JointStateInterface>()->getHandle("right_wheel_joint");

  // i_b is moment of inertia of the pendulum body around the pivot point,
  // i_w is the moment of inertia of the wheel around the rotational axis of the motor
  // l is the distance between the motor axis and the pivot point
  // l_b is the distance between the center of mass of the pendulum body and the pivot point
  double m_b, m_w, i_b, i_w, l, l_b, g;

  if (!controller_nh.getParam("m_b", m_b))
  {
    ROS_ERROR("Params m_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m_w", m_w))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_b", i_b))
  {
    ROS_ERROR("Params i_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_w", i_w))
  {
    ROS_ERROR("Params i_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l", l))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l_b", l_b))
  {
    ROS_ERROR("Params l_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("g", g))
  {
    ROS_ERROR("Params g doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  if (!controller_nh.getParam("alpha", alpha_))
  {
    ROS_ERROR("Params alpha doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  q_.setZero();
  r_.setZero();
  XmlRpc::XmlRpcValue q, r;
  controller_nh.getParam("q", q);
  controller_nh.getParam("r", r);
  // Check and get Q
  ROS_ASSERT(q.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(q.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i)
  {
    ROS_ASSERT(q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || q[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      q_(i, i) = static_cast<double>(q[i]);
    else if (q[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      q_(i, i) = static_cast<int>(q[i]);
  }
  // Check and get R
  ROS_ASSERT(r.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(r.size() == CONTROL_DIM);
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    ROS_ASSERT(r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || r[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      r_(i, i) = static_cast<double>(r[i]);
    else if (r[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      r_(i, i) = static_cast<int>(r[i]);
  }

  // Continuous model \dot{x} = A x + B u
  torque_g_ = (m_b * l_b + m_w * l) * g;
  double temp = i_b + m_w * l * l;
  double a_1_0 = torque_g_ / temp;
  double a_1_1 = 0.;  // TODO:  dynamic friction coefficient
  double a_1_2 = 0.;  // TODO:  dynamic friction coefficient
  double a_2_0 = -a_1_0;
  double a_2_1 = 0.;
  double a_2_2 = 0.;
  double b_1 = -1. / temp;
  double b_2 = (temp + i_w) / (i_w * temp);
  a_ << 0., 1., 0., a_1_0, a_1_1, a_1_2, a_2_0, a_2_1, a_2_2;
  b_ << 0., b_1, b_2;

  Lqr<double> lqr(a_, b_, q_, r_);
  if (!lqr.computeK())
  {
    ROS_ERROR("Failed to compute K of LQR.");
    return false;
  }

  k_ = lqr.getK();
  ROS_INFO_STREAM("K of LQR:" << k_);
  return true;
}

void ReactionWheelController::starting(const ros::Time& time)
{
  pitch_offset_ = 0.;
}

void ReactionWheelController::update(const ros::Time& time, const ros::Duration& period)
{
  // TODO: simplify, add new quatToRPY
  geometry_msgs::Quaternion quat;
  quat.x = imu_handle_.getOrientation()[0];
  quat.y = imu_handle_.getOrientation()[1];
  quat.z = imu_handle_.getOrientation()[2];
  quat.w = imu_handle_.getOrientation()[3];
  double roll{}, pitch{}, yaw{};
  quatToRPY(quat, roll, pitch, yaw);
  Eigen::Matrix<double, STATE_DIM, 1> x;
  x(0) = pitch;
  x(1) = imu_handle_.getAngularVelocity()[1];
  x(2) = joint_handle_.getVelocity();

  pitch_offset_ = (1. - alpha_) * pitch_offset_ + alpha_ * pitch;

  if (std::abs(pitch_offset_) > 0.2)  // TODO pitch_max rosparam
  {
    if (std::abs(x(2)) < 1e-2)  // Damped
      pitch_offset_ = 0.;
    else  // Damping
    {
      joint_handle_.setCommand(-0.4 * x(2));  // TODO damping_factor rosparam
      return;
    }
  }

  //  double pitch_des = std::asin((left_wheel_handle_.getEffort() + right_wheel_handle_.getEffort()) / torque_g_);
  double pitch_des = pitch_offset_;
  //  double pitch_des = 0.;

  x(0) = x(0) - pitch_des;
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  u = k_ * (-x);  // regulate to zero: K*(0 - x)
  joint_handle_.setCommand(u(0));
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ReactionWheelController, controller_interface::ControllerBase)
