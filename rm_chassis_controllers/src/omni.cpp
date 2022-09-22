//
// Created by qiayuan on 2022/7/29.
//

#include <string>
#include <Eigen/QR>

#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

#include "rm_chassis_controllers/omni.h"

namespace rm_chassis_controllers
{
bool OmniController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  XmlRpc::XmlRpcValue wheels;
  controller_nh.getParam("wheels", wheels);
  chassis2joints_.resize(wheels.size(), 3);

  size_t i = 0;
  for (const auto& wheel : wheels)
  {
    ROS_ASSERT(wheel.second.hasMember("pose"));
    ROS_ASSERT(wheel.second["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(wheel.second["pose"].size() == 3);
    ROS_ASSERT(wheel.second.hasMember("roller_angle"));
    ROS_ASSERT(wheel.second.hasMember("radius"));

    // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
    Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
    double beta = (double)wheel.second["pose"][2];
    double roller_angle = (double)wheel.second["roller_angle"];
    direction << 1, tan(roller_angle);
    in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
    in_chassis << -(double)wheel.second["pose"][1], 1., 0., (double)wheel.second["pose"][0], 0., 1.;
    Eigen::MatrixXd chassis2joint = 1. / (double)wheel.second["radius"] * direction * in_wheel * in_chassis;
    chassis2joints_.block<1, 3>(i, 0) = chassis2joint;

    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "wheels/" + wheel.first);
    joints_.push_back(std::make_shared<effort_controllers::JointVelocityController>());
    if (!joints_.back()->init(effort_joint_interface_, nh_wheel))
      return false;
    joint_handles_.push_back(joints_[i]->joint_);

    i++;
  }
  return true;
}

void OmniController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  Eigen::Vector3d vel_chassis;
  vel_chassis << vel_cmd_.z, vel_cmd_.x, vel_cmd_.y;
  Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;
  for (size_t i = 0; i < joints_.size(); i++)
  {
    joints_[i]->setCommand(vel_joints(i));
    joints_[i]->update(time, period);
  }
}

geometry_msgs::Twist OmniController::odometry()
{
  Eigen::VectorXd vel_joints(joints_.size());
  for (size_t i = 0; i < joints_.size(); i++)
    vel_joints[i] = joints_[i]->joint_.getVelocity();
  Eigen::Vector3d vel_chassis =
      (chassis2joints_.transpose() * chassis2joints_).inverse() * chassis2joints_.transpose() * vel_joints;
  geometry_msgs::Twist twist;
  twist.angular.z = vel_chassis(0);
  twist.linear.x = vel_chassis(1);
  twist.linear.y = vel_chassis(2);
  return twist;
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::OmniController, controller_interface::ControllerBase)
