//
// Created by qiayuan on 2022/11/15.
//
#include "rm_chassis_controllers/balance.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <geometry_msgs/Quaternion.h>
#include <pluginlib/class_list_macros.hpp>

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
  momentum_block_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle("momentum_block_joint");

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
  a_ << 0., 0., 0., 1., 0., 0., 0., 0., 0., 1., -1.6066, -2.4065, 0., 0., 0., 25.0596, 106.6442, 0., 0., 0., 7.9258,
      -12.4064, 0., 0., 0.;
  b_ << 0., 0., 0., 0., 0.6148, -0.0419, -6.9291, -0.6828, 0.3477, 0.6761;

  Lqr<double> lqr(a_, b_, q_, r_);
  if (!lqr.computeK())
  {
    ROS_ERROR("Failed to compute K of LQR.");
    return false;
  }

  k_ = lqr.getK();
  ROS_INFO_STREAM("K of LQR:" << k_);

  model_states_sub_ = root_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1,
                                                                  &BalanceController::modelStatesCallBack, this);
  return true;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  x_[1] = momentum_block_joint_handle_.getPosition();
  x_[2] -= vel_cmd_.x;
  x_[4] = momentum_block_joint_handle_.getVelocity();
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  u = k_ * (-x_);

  right_wheel_joint_handle_.setCommand(u(0) / 2);
  left_wheel_joint_handle_.setCommand(u(0) / 2);
  momentum_block_joint_handle_.setCommand(u(1));
}

void BalanceController::modelStatesCallBack(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  double roll, pitch, yaw;
  quatToRPY(msg->pose[1].orientation, roll, pitch, yaw);
  x_[0] = pitch;
  x_[2] = ((left_wheel_joint_handle_.getVelocity() + right_wheel_joint_handle_.getVelocity()) / 2 > 0 ? 1 : -1) *
          sqrt(pow(msg->twist[1].linear.x, 2) + pow(msg->twist[1].linear.y, 2));
  x_[3] = msg->twist[1].angular.y;
}
geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  return twist;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
