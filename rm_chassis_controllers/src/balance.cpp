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
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

namespace rm_chassis_controllers
{
bool BalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle("left_wheel_joint");
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle("right_wheel_joint");
  left_momentum_block_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle("left_momentum_block_joint");
  right_momentum_block_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle("right_momentum_block_joint");

  if (!pid_controller_.init(ros::NodeHandle(controller_nh, "pid_block_error")))
    return false;
  // i_b is moment of inertia of the pendulum body around the pivot point,
  // i_w is the moment of inertia of the wheel around the rotational axis of the motor
  // l is the distance between the motor axis and the pivot point
  // l_b is the distance between the center of mass of the pendulum body and the pivot point
  double m_b, m_p, m_c, l_p, l_b, g;

  if (!controller_nh.getParam("m_b", m_b))
  {
    ROS_ERROR("Params m_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m_p", m_p))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m_c", m_c))
  {
    ROS_ERROR("Params i_b doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l_p", l_p))
  {
    ROS_ERROR("Params i_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
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
  if (!controller_nh.getParam("wheel_radius", wheel_radius_))
  {
    ROS_ERROR("Params wheel_radius doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
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
  //  double m = m_b + m_c + m_p;
  //  double a_2_0 = -g * (m_b + m_p) / m_c;
  //  double a_2_1 = -g * m_b / (l_p * m_c);
  //  double a_3_0 = m * g / (l_p * m_c);
  //  double a_3_1 = m_b * g * (m_c + m_p) / (l_p * l_p * m_c * m_p);
  //  double a_4_0 = m * g * (l_p - l_b) / (m_c * l_p);
  //  double a_4_1 = g * m_b * (-l_b * m_c - l_b * m_p + l_p * m_p) / (l_p * l_p * m_c * m_p);
  //
  //  double b_2_0 = 1 / m_c;
  //  double b_2_1 = (l_b - l_p) / (l_p * m_c);
  //  double b_3_0 = -1 / (l_p * m_c);
  //  double b_3_1 = (-l_b * m_c - l_b * m_p + l_p * m_p) / (l_p * l_p * m_c * m_p);
  //  double b_4_0 = (l_b - l_p) / (l_p * m_c);
  //  double b_4_1 =
  //      l_b * l_b / (l_p * l_p * m_p) + l_b * l_b / (l_p * l_p * m_c) - 2.0 * l_b / (l_p * m_c) + 1 / m_c + 1 / m_b;
  a_ << 0., 0., 0., 0., 1.0, 0., 0., 0., 0., 0., 0., 0., 0., 1.0, 0., 0., 0., 0., 0., 0., 0., 0., 1.0, 0., 0., 0., 0.,
      0., 0., 0., 0., 1.0, 0., 0., -1.60656831231196, -2.40650017923402, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 25.0595846927374, 106.644165861824, 0., 0., 0., 0., 0., 0., 7.92579199849074, -12.4063744589733, 0., 0.,
      0., 0.;
  b_ << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.614753821325744, 0.614753821325744, 0.0132316980332364,
      -1.65961661528494, 1.65961661528494, 0., -6.929097995272, -6.929097995272, -0.586363305461782, 0.347697890217537,
      0.347697890217537, 0.607588490877498;

  Lqr<double> lqr(a_, b_, q_, r_);
  if (!lqr.computeK())
  {
    ROS_ERROR("Failed to compute K of LQR.");
    return false;
  }

  k_ = lqr.getK();
  ROS_INFO_STREAM("K of LQR:" << k_);

  state_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("/state", 10);
  return true;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_base;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  try
  {
    tf2::doTransform(gyro, angular_vel_base,
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
  odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                               imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
  odom2imu.setRotation(odom2imu_quaternion);
  odom2base = odom2imu * imu2base;
  double roll, pitch, yaw;
  quatToRPY(toMsg(odom2base).rotation, roll, pitch, yaw);
  x_[4] = ((left_wheel_joint_handle_.getVelocity() + right_wheel_joint_handle_.getVelocity()) / 2 -
           imu_handle_.getAngularVelocity()[1]) *
          0.125;
  x_[0] += x_[4] * period.toSec();
  x_[1] = yaw;
  x_[2] = pitch;
  x_[3] = left_momentum_block_joint_handle_.getPosition();
  x_[5] = angular_vel_base.z;
  x_[6] = angular_vel_base.y;
  x_[7] = left_momentum_block_joint_handle_.getVelocity();
  if (vel_cmd_.z != 0.)
    yaw_des_ = x_[1] + vel_cmd_.z * period.toSec();
  if (vel_cmd_.x != 0.)
    position_des_ = x_[0] + vel_cmd_.x * period.toSec();
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  auto x = x_;
  x(0) -= position_des_;
  x(1) = angles::shortest_angular_distance(yaw_des_, x_(1));
  x(4) -= vel_cmd_.x;
  x(5) -= vel_cmd_.z;
  u = k_ * (-x);
  std_msgs::Float64MultiArray state;
  for (int i = 0; i < 8; i++)
  {
    state.data.push_back(x(i));
  }
  for (int i = 0; i < 3; i++)
  {
    state.data.push_back(u(i));
  }
  state_pub_.publish(state);

  left_wheel_joint_handle_.setCommand(u(0));
  right_wheel_joint_handle_.setCommand(u(1));
  left_momentum_block_joint_handle_.setCommand(u(2) / 2);
  double error = left_momentum_block_joint_handle_.getPosition() - right_momentum_block_joint_handle_.getPosition();
  double commanded_effort = pid_controller_.computeCommand(error, period);
  right_momentum_block_joint_handle_.setCommand(u(2) / 2 + commanded_effort);
}

geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  return twist;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
