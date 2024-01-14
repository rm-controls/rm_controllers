//
// Created by bruce on 2021/5/19.
//

#include "rm_orientation_controller/orientation_controller.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm_orientation_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  std::string name;
  if (!controller_nh.getParam("name", name) || !controller_nh.getParam("frame_source", frame_source_) ||
      !controller_nh.getParam("frame_target", frame_target_))
  {
    ROS_ERROR("Some params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  imu_sensor_ = robot_hw->get<rm_control::RmImuSensorInterface>()->getHandle(name);
  robot_state_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");

  tf_broadcaster_.init(root_nh);
  imu_data_sub_ = root_nh.subscribe<sensor_msgs::Imu>("data", 1, &Controller::imuDataCallback, this);
  source2target_msg_.header.frame_id = frame_source_;
  source2target_msg_.child_frame_id = frame_target_;
  source2target_msg_.transform.rotation.w = 1.0;
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  if (imu_sensor_.getTimeStamp() > last_imu_update_time_)
  {
    last_imu_update_time_ = imu_sensor_.getTimeStamp();
    geometry_msgs::TransformStamped source2target;
    source2target.header.stamp = time;
    source2target.header.stamp.nsec += 1;  // Avoid redundant timestamp
    source2target_msg_.header.stamp = time;
    source2target_msg_.header.stamp.nsec += 1;
    source2target_msg_ =
        getTransform(ros::Time(0), source2target, imu_sensor_.getOrientation()[0], imu_sensor_.getOrientation()[1],
                     imu_sensor_.getOrientation()[2], imu_sensor_.getOrientation()[3]) ?
            source2target :
            source2target_msg_;
    robot_state_.setTransform(source2target_msg_, "rm_orientation_controller");
    if (!receive_imu_msg_)
      tf_broadcaster_.sendTransform(source2target_msg_);
  }
}

bool Controller::getTransform(const ros::Time& time, geometry_msgs::TransformStamped& source2target, const double x,
                              const double y, const double z, const double w)
{
  source2target.header.frame_id = frame_source_;
  source2target.child_frame_id = frame_target_;
  source2target.transform.rotation.w = 1.0;
  tf2::Transform source2odom, odom2fixed, fixed2target;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_.lookupTransform(frame_source_, "odom", time);
    tf2::fromMsg(tf_msg.transform, source2odom);
    tf_msg = robot_state_.lookupTransform("odom", imu_sensor_.getFrameId(), time);
    tf2::fromMsg(tf_msg.transform, odom2fixed);
    tf_msg = robot_state_.lookupTransform(imu_sensor_.getFrameId(), frame_target_, time);
    tf2::fromMsg(tf_msg.transform, fixed2target);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  tf2::Quaternion odom2fixed_quat;
  odom2fixed_quat.setValue(x, y, z, w);
  odom2fixed.setRotation(odom2fixed_quat);
  source2target.transform = tf2::toMsg(source2odom * odom2fixed * fixed2target);
  return true;
}

void Controller::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (!receive_imu_msg_)
    receive_imu_msg_ = true;
  geometry_msgs::TransformStamped source2target;
  source2target.header.stamp = msg->header.stamp;
  getTransform(ros::Time(0), source2target, msg->orientation.x, msg->orientation.y, msg->orientation.z,
               msg->orientation.w);
  tf_broadcaster_.sendTransform(source2target);
}

}  // namespace rm_orientation_controller

PLUGINLIB_EXPORT_CLASS(rm_orientation_controller::Controller, controller_interface::ControllerBase)
