//
// Created by qiayuan on 2022/11/15.
//
#include "rm_chassis_controllers/balance.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/BalanceState.h>
#include <angles/angles.h>

namespace rm_chassis_controllers
{
bool BalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));
  std::string left_wheel_joint, right_wheel_joint, left_momentum_block_joint, right_momentum_block_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("left/block_joint", left_momentum_block_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint) ||
      !controller_nh.getParam("right/block_joint", right_momentum_block_joint))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);
  left_momentum_block_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_momentum_block_joint);
  right_momentum_block_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_momentum_block_joint);

  // m_w is mass of single wheel
  // m is mass of the robot except wheels and momentum_blocks
  // m_b is mass of single momentum_block
  // i_w is the moment of inertia of the wheel around the rotational axis of the motor
  // l is the vertical component of the distance between the wheel center and the center of mass of robot
  // y_b is the y-axis component of the coordinates of the momentum block in the base_link coordinate system
  // z_b is the vertical component of the distance between the momentum block and the center of mass of robot
  // i_m is the moment of inertia of the robot around the y-axis of base_link coordinate.
  double m_w, m, m_b, i_w, l, y_b, z_b, g, i_m;

  if (!controller_nh.getParam("m_w", m_w))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m", m))
  {
    ROS_ERROR("Params m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m_b", m_b))
  {
    ROS_ERROR("Params m_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
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
  if (!controller_nh.getParam("y_b", y_b))
  {
    ROS_ERROR("Params y_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("z_b", z_b))
  {
    ROS_ERROR("Params z_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("g", g))
  {
    ROS_ERROR("Params g doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_m", i_m))
  {
    ROS_ERROR("Params i_m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("wheel_radius", wheel_radius_))
  {
    ROS_ERROR("Params wheel_radius doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("wheel_base", wheel_base_))
  {
    ROS_ERROR("Params wheel_base_ doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("block_duration", block_duration_))
  {
    ROS_ERROR("Params block_duration doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("block_angle", block_angle_))
  {
    ROS_ERROR("Params block_angle doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("block_effort", block_effort_))
  {
    ROS_ERROR("Params block_speed doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("block_velocity", block_velocity_))
  {
    ROS_ERROR("Params block_velocity doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("anti_block_effort", anti_block_effort_))
  {
    ROS_ERROR("Params anti_block_effort doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("block_overtime", block_overtime_))
  {
    ROS_ERROR("Params block_overtime doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  controller_nh.getParam("position_offset", position_offset_);
  controller_nh.getParam("position_clear_threshold", position_clear_threshold_);

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
    {
      q_(i, i) = static_cast<double>(q[i]);
    }
    else if (q[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      q_(i, i) = static_cast<int>(q[i]);
    }
  }
  // Check and get R
  ROS_ASSERT(r.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(r.size() == CONTROL_DIM);
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    ROS_ASSERT(r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || r[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      r_(i, i) = static_cast<double>(r[i]);
    }
    else if (r[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      r_(i, i) = static_cast<int>(r[i]);
    }
  }

  // Continuous model \dot{x} = A x + B u
  double a_5_2 = -(pow(wheel_radius_, 2) * g * (pow(l, 2) * pow(m, 2) + 2 * m_b * pow(l, 2) * m + 2 * i_m * m_b)) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double a_5_3 = -(pow(wheel_radius_, 2) * g * l * m * m_b) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double a_5_4 = a_5_3;
  double a_7_2 =
      (g * l * m *
       (2 * i_w + pow(wheel_radius_, 2) * m + 2 * pow(wheel_radius_, 2) * m_b + 2 * pow(wheel_radius_, 2) * m_w)) /
      (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
       2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double a_7_3 = (g * m_b * (2 * i_w + pow(wheel_radius_, 2) * m + 2 * pow(wheel_radius_, 2) * m_w)) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double a_7_4 = a_7_3;
  double a_8_2 =
      (g * (i_m - l * m * z_b) *
       (2 * i_w + pow(wheel_radius_, 2) * m + 2 * pow(wheel_radius_, 2) * m_b + 2 * pow(wheel_radius_, 2) * m_w)) /
      (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
       2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double a_8_3 = -(g * m_b *
                   (2 * i_w * l + 2 * i_w * z_b + 2 * pow(wheel_radius_, 2) * l * m_w +
                    pow(wheel_radius_, 2) * m * z_b + 2 * pow(wheel_radius_, 2) * m_w * z_b)) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double a_8_4 = a_8_3;
  double a_9_2 = a_8_2;
  double a_9_3 = a_8_3;
  double a_9_4 = a_8_4;

  double b_5_0 = (wheel_radius_ * (m * pow(l, 2) + wheel_radius_ * m * l + i_m)) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double b_5_1 = b_5_0;
  double b_5_2 = (pow(wheel_radius_, 2) * l * m * z_b) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double b_5_3 = b_5_2;
  double b_6_0 = -wheel_radius_ / (wheel_base_ * (4 * m_w * pow(wheel_radius_, 2) + i_w));
  double b_6_1 = -b_6_0;
  double b_6_2 = (2 * pow(wheel_radius_, 2) * y_b) / (pow(wheel_base_, 2) * (4 * m_w * pow(wheel_radius_, 2) + i_w));
  double b_6_3 = -b_6_2;
  double b_7_0 = -(2 * i_w + pow(wheel_radius_, 2) * m + 2 * pow(wheel_radius_, 2) * m_w + wheel_radius_ * l * m) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double b_7_1 = b_7_0;
  double b_7_2 = -(z_b * (2 * i_w + pow(wheel_radius_, 2) * m + 2 * pow(wheel_radius_, 2) * m_w)) /
                 (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
                  2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w);
  double b_7_3 = b_7_2;
  double b_9_0 =
      (2 * pow(i_w, 2) * l * wheel_base_ + 2 * pow(i_w, 2) * wheel_base_ * z_b -
       4 * pow(wheel_radius_, 3) * i_m * m_w * wheel_base_ + pow(wheel_radius_, 3) * i_m * m * y_b +
       2 * pow(wheel_radius_, 3) * i_m * m_w * y_b + 8 * pow(wheel_radius_, 4) * l * pow(m_w, 2) * wheel_base_ +
       8 * pow(wheel_radius_, 4) * pow(m_w, 2) * wheel_base_ * z_b - wheel_radius_ * i_m * i_w * wheel_base_ +
       2 * wheel_radius_ * i_m * i_w * y_b + 2 * pow(wheel_radius_, 3) * pow(l, 2) * m * m_w * y_b +
       10 * pow(wheel_radius_, 2) * i_w * l * m_w * wheel_base_ + 2 * wheel_radius_ * i_w * pow(l, 2) * m * y_b +
       pow(wheel_radius_, 2) * i_w * m * wheel_base_ * z_b + 10 * pow(wheel_radius_, 2) * i_w * m_w * wheel_base_ * z_b +
       4 * pow(wheel_radius_, 4) * m * m_w * wheel_base_ * z_b +
       4 * pow(wheel_radius_, 3) * l * m * m_w * wheel_base_ * z_b + wheel_radius_ * i_w * l * m * wheel_base_ * z_b) /
      (wheel_base_ * (4 * m_w * pow(wheel_radius_, 2) + i_w) *
       (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
        2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w));
  double b_9_1 =
      (2 * pow(i_w, 2) * l * wheel_base_ + 2 * pow(i_w, 2) * wheel_base_ * z_b -
       4 * pow(wheel_radius_, 3) * i_m * m_w * wheel_base_ - pow(wheel_radius_, 3) * i_m * m * y_b -
       2 * pow(wheel_radius_, 3) * i_m * m_w * y_b + 8 * pow(wheel_radius_, 4) * l * pow(m_w, 2) * wheel_base_ +
       8 * pow(wheel_radius_, 4) * pow(m_w, 2) * wheel_base_ * z_b - wheel_radius_ * i_m * i_w * wheel_base_ -
       2 * wheel_radius_ * i_m * i_w * y_b - 2 * pow(wheel_radius_, 3) * pow(l, 2) * m * m_w * y_b +
       10 * pow(wheel_radius_, 2) * i_w * l * m_w * wheel_base_ - 2 * wheel_radius_ * i_w * pow(l, 2) * m * y_b +
       pow(wheel_radius_, 2) * i_w * m * wheel_base_ * z_b + 10 * pow(wheel_radius_, 2) * i_w * m_w * wheel_base_ * z_b +
       4 * pow(wheel_radius_, 4) * m * m_w * wheel_base_ * z_b +
       4 * pow(wheel_radius_, 3) * l * m * m_w * wheel_base_ * z_b + wheel_radius_ * i_w * l * m * wheel_base_ * z_b) /
      (wheel_base_ * (4 * m_w * pow(wheel_radius_, 2) + i_w) *
       (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
        2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w));
  double b_9_2 =
      (-4 * m * pow(wheel_radius_, 4) * pow(l, 2) * m_w * pow(y_b, 2) +
       8 * pow(wheel_radius_, 4) * l * pow(m_w, 2) * pow(wheel_base_, 2) * z_b +
       8 * pow(wheel_radius_, 4) * pow(m_w, 2) * pow(wheel_base_, 2) * pow(z_b, 2) +
       4 * m * pow(wheel_radius_, 4) * m_w * pow(wheel_base_, 2) * pow(z_b, 2) -
       4 * i_m * pow(wheel_radius_, 4) * m_w * pow(y_b, 2) - 2 * i_m * m * pow(wheel_radius_, 4) * pow(y_b, 2) -
       4 * m * pow(wheel_radius_, 2) * i_w * pow(l, 2) * pow(y_b, 2) +
       10 * pow(wheel_radius_, 2) * i_w * l * m_w * pow(wheel_base_, 2) * z_b +
       10 * pow(wheel_radius_, 2) * i_w * m_w * pow(wheel_base_, 2) * pow(z_b, 2) +
       m * pow(wheel_radius_, 2) * i_w * pow(wheel_base_, 2) * pow(z_b, 2) -
       4 * i_m * pow(wheel_radius_, 2) * i_w * pow(y_b, 2) + 2 * pow(i_w, 2) * l * pow(wheel_base_, 2) * z_b +
       2 * pow(i_w, 2) * pow(wheel_base_, 2) * pow(z_b, 2)) /
      (pow(wheel_base_, 2) * (4 * m_w * pow(wheel_radius_, 2) + i_w) *
       (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
        2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w));
  double b_9_3 =
      (8 * m * pow(wheel_radius_, 4) * pow(l, 2) * pow(m_w, 2) * pow(wheel_base_, 2) +
       4 * m * m_b * pow(wheel_radius_, 4) * pow(l, 2) * m_w * pow(y_b, 2) +
       8 * m_b * pow(wheel_radius_, 4) * l * pow(m_w, 2) * pow(wheel_base_, 2) * z_b +
       8 * m_b * pow(wheel_radius_, 4) * pow(m_w, 2) * pow(wheel_base_, 2) * pow(z_b, 2) +
       8 * i_m * pow(wheel_radius_, 4) * pow(m_w, 2) * pow(wheel_base_, 2) +
       4 * m * m_b * pow(wheel_radius_, 4) * m_w * pow(wheel_base_, 2) * pow(z_b, 2) +
       4 * i_m * m * pow(wheel_radius_, 4) * m_w * pow(wheel_base_, 2) +
       4 * i_m * m_b * pow(wheel_radius_, 4) * m_w * pow(y_b, 2) +
       2 * i_m * m * m_b * pow(wheel_radius_, 4) * pow(y_b, 2) +
       10 * m * pow(wheel_radius_, 2) * i_w * pow(l, 2) * m_w * pow(wheel_base_, 2) +
       4 * m * m_b * pow(wheel_radius_, 2) * i_w * pow(l, 2) * pow(y_b, 2) +
       10 * m_b * pow(wheel_radius_, 2) * i_w * l * m_w * pow(wheel_base_, 2) * z_b +
       10 * m_b * pow(wheel_radius_, 2) * i_w * m_w * pow(wheel_base_, 2) * pow(z_b, 2) +
       10 * i_m * pow(wheel_radius_, 2) * i_w * m_w * pow(wheel_base_, 2) +
       m * m_b * pow(wheel_radius_, 2) * i_w * pow(wheel_base_, 2) * pow(z_b, 2) +
       i_m * m * pow(wheel_radius_, 2) * i_w * pow(wheel_base_, 2) +
       4 * i_m * m_b * pow(wheel_radius_, 2) * i_w * pow(y_b, 2) +
       2 * m * pow(i_w, 2) * pow(l, 2) * pow(wheel_base_, 2) + 2 * m_b * pow(i_w, 2) * l * pow(wheel_base_, 2) * z_b +
       2 * m_b * pow(i_w, 2) * pow(wheel_base_, 2) * pow(z_b, 2) + 2 * i_m * pow(i_w, 2) * pow(wheel_base_, 2)) /
      (m_b * pow(wheel_base_, 2) * (4 * m_w * pow(wheel_radius_, 2) + i_w) *
       (2 * i_m * i_w + 2 * i_w * pow(l, 2) * m + pow(wheel_radius_, 2) * i_m * m +
        2 * pow(wheel_radius_, 2) * i_m * m_w + 2 * pow(wheel_radius_, 2) * pow(l, 2) * m * m_w));
  double b_8_0 = b_9_1;
  double b_8_1 = b_9_0;
  double b_8_2 = b_9_3;
  double b_8_3 = b_9_2;

  a_ << 0., 0., 0., 0., 0., 1.0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
      1.0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.0, 0., 0., a_5_2,
      a_5_3, a_5_4, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., a_7_2, a_7_3, a_7_4, 0., 0., 0.,
      0., 0., 0., 0., a_8_2, a_8_3, a_8_4, 0., 0., 0., 0., 0., 0., 0., a_9_2, a_9_3, a_9_4, 0., 0., 0., 0., 0.;
  b_ << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., b_5_0, b_5_1, b_5_2, b_5_3,
      b_6_0, b_6_1, b_6_2, b_6_3, b_7_0, b_7_1, b_7_2, b_7_3, b_8_0, b_8_1, b_8_2, b_8_3, b_9_0, b_9_1, b_9_2, b_9_3;

  ROS_INFO_STREAM("A:" << a_);
  ROS_INFO_STREAM("B:" << b_);
  Lqr<double> lqr(a_, b_, q_, r_);
  if (!lqr.computeK())
  {
    ROS_ERROR("Failed to compute K of LQR.");
    return false;
  }

  k_ = lqr.getK();
  ROS_INFO_STREAM("K of LQR:" << k_);

  state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::BalanceState>(root_nh, "/state", 100));
  balance_mode_ = BalanceMode::NORMAL;

  return true;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  try
  {
    tf2::doTransform(gyro, angular_vel_base_,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2imu, imu2base, odom2base;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    left_wheel_joint_handle_.setCommand(0.);
    right_wheel_joint_handle_.setCommand(0.);
    left_momentum_block_joint_handle_.setCommand(0.);
    right_momentum_block_joint_handle_.setCommand(0.);
    return;
  }
  tf2::Quaternion odom2imu_quaternion;
  tf2::Vector3 odom2imu_origin;
  odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                               imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
  odom2imu_origin.setValue(0, 0, 0);
  odom2imu.setOrigin(odom2imu_origin);
  odom2imu.setRotation(odom2imu_quaternion);
  odom2base = odom2imu * imu2base;

  quatToRPY(toMsg(odom2base).rotation, roll_, pitch_, yaw_);

  // Check block
  if (balance_mode_ != BalanceMode::BLOCK)
  {
    if (std::abs(pitch_) > block_angle_ &&
        (std::abs(left_wheel_joint_handle_.getEffort()) + std::abs(right_wheel_joint_handle_.getEffort())) / 2. >
            block_effort_ &&
        (left_wheel_joint_handle_.getVelocity() < block_velocity_ ||
         right_wheel_joint_handle_.getVelocity() < block_velocity_))
    {
      if (!maybe_block_)
      {
        block_time_ = time;
        maybe_block_ = true;
      }
      if ((time - block_time_).toSec() >= block_duration_)
      {
        balance_mode_ = BalanceMode::BLOCK;
        balance_state_changed_ = true;
        ROS_INFO("[balance] Exit NOMAl");
      }
    }
    else
    {
      maybe_block_ = false;
    }
  }

  switch (balance_mode_)
  {
    case BalanceMode::NORMAL:
    {
      normal(time, period);
      break;
    }
    case BalanceMode::BLOCK:
    {
      block(time, period);
      break;
    }
  }
}

void BalanceController::normal(const ros::Time& time, const ros::Duration& period)
{
  if (balance_state_changed_)
  {
    ROS_INFO("[balance] Enter NOMAl");
    balance_state_changed_ = false;
  }

  x_[5] = ((left_wheel_joint_handle_.getVelocity() + right_wheel_joint_handle_.getVelocity()) / 2 -
           imu_handle_.getAngularVelocity()[1]) *
          wheel_radius_;
  x_[0] += x_[5] * period.toSec();
  x_[1] = yaw_;
  x_[2] = pitch_;
  x_[3] = left_momentum_block_joint_handle_.getPosition();
  x_[4] = right_momentum_block_joint_handle_.getPosition();
  x_[6] = angular_vel_base_.z;
  x_[7] = angular_vel_base_.y;
  x_[8] = left_momentum_block_joint_handle_.getVelocity();
  x_[9] = right_momentum_block_joint_handle_.getVelocity();
  yaw_des_ += vel_cmd_.z * period.toSec();
  position_des_ += vel_cmd_.x * period.toSec();
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  auto x = x_;
  x(0) -= position_des_;
  x(1) = angles::shortest_angular_distance(yaw_des_, x_(1));
  if (state_ != RAW)
    x(5) -= vel_cmd_.x;
  x(6) -= vel_cmd_.z;
  if (std::abs(x(0) + position_offset_) > position_clear_threshold_)
  {
    x_[0] = 0.;
    position_des_ = position_offset_;
  }
  u = k_ * (-x);
  if (state_pub_->trylock())
  {
    state_pub_->msg_.header.stamp = time;
    state_pub_->msg_.x = x(0);
    state_pub_->msg_.phi = x(1);
    state_pub_->msg_.theta = x(2);
    state_pub_->msg_.x_b_l = x(3);
    state_pub_->msg_.x_b_r = x(4);
    state_pub_->msg_.x_dot = x(5);
    state_pub_->msg_.phi_dot = x(6);
    state_pub_->msg_.theta_dot = x(7);
    state_pub_->msg_.x_b_l_dot = x(8);
    state_pub_->msg_.x_b_r_dot = x(9);
    state_pub_->msg_.T_l = u(0);
    state_pub_->msg_.T_r = u(1);
    state_pub_->msg_.f_b_l = u(2);
    state_pub_->msg_.f_b_r = u(3);
    state_pub_->unlockAndPublish();
  }

  left_wheel_joint_handle_.setCommand(u(0));
  right_wheel_joint_handle_.setCommand(u(1));
  left_momentum_block_joint_handle_.setCommand(u(2));
  right_momentum_block_joint_handle_.setCommand(u(3));
}

void BalanceController::block(const ros::Time& time, const ros::Duration& period)
{
  if (balance_state_changed_)
  {
    ROS_INFO("[balance] Enter BLOCK");
    balance_state_changed_ = false;

    last_block_time_ = ros::Time::now();
  }
  if ((ros::Time::now() - last_block_time_).toSec() > block_overtime_)
  {
    balance_mode_ = BalanceMode::NORMAL;
    balance_state_changed_ = true;
    ROS_INFO("[balance] Exit BLOCK");
  }
  else
  {
    left_momentum_block_joint_handle_.setCommand(pitch_ > 0 ? -80 : 80);
    right_momentum_block_joint_handle_.setCommand(pitch_ > 0 ? -80 : 80);
    left_wheel_joint_handle_.setCommand(pitch_ > 0 ? -anti_block_effort_ : anti_block_effort_);
    right_wheel_joint_handle_.setCommand(pitch_ > 0 ? -anti_block_effort_ : anti_block_effort_);
  }
}

geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_[5];
  twist.angular.z = x_[6];
  return twist;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
