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
      !controller_nh.getParam("frame_target", frame_target_) || !controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Some params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  imu_sensor_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(name);
  robot_state_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");

  imu_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(root_nh, "imu_data", 100));
  tf_broadcaster_.init(root_nh);
  source2target_msg_.header.frame_id = frame_source_;
  source2target_msg_.child_frame_id = frame_target_;
  source2target_msg_.transform.rotation.w = 1.0;
  last_orientation_x = 0.0;
  last_orientation_y = 0.0;
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  if (imu_sensor_.getOrientation()[0] == last_orientation_x && imu_sensor_.getOrientation()[1] == last_orientation_y)
  {
    return;
  }
  last_orientation_x = imu_sensor_.getOrientation()[0];
  last_orientation_y = imu_sensor_.getOrientation()[1];
  source2target_msg_.header.stamp = time;
  source2target_msg_.header.stamp.nsec += 1;  // Avoid redundant timestamp
  tf2::Transform source2odom, odom2fixed, fixed2target;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_.lookupTransform(frame_source_, "odom", ros::Time(0));
    tf2::fromMsg(tf_msg.transform, source2odom);
    tf_msg = robot_state_.lookupTransform("odom", imu_sensor_.getFrameId(), ros::Time(0));
    tf2::fromMsg(tf_msg.transform, odom2fixed);
    tf_msg = robot_state_.lookupTransform(imu_sensor_.getFrameId(), frame_target_, ros::Time(0));
    tf2::fromMsg(tf_msg.transform, fixed2target);
  }
  catch (tf2::TransformException& ex)
  {
    robot_state_.setTransform(source2target_msg_, "rm_orientation_controller");
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Quaternion odom2fixed_quat;
  odom2fixed_quat.setValue(imu_sensor_.getOrientation()[0], imu_sensor_.getOrientation()[1],
                           imu_sensor_.getOrientation()[2], imu_sensor_.getOrientation()[3]);
  odom2fixed.setRotation(odom2fixed_quat);
  source2target_msg_.transform = tf2::toMsg(source2odom * odom2fixed * fixed2target);
  if ((time.toSec() - last_br_.toSec()) > 0.01)
  {
    tf_broadcaster_.sendTransform(source2target_msg_);
    last_br_ = time;
  }
  robot_state_.setTransform(source2target_msg_, "rm_orientation_controller");
}

}  // namespace rm_orientation_controller

PLUGINLIB_EXPORT_CLASS(rm_orientation_controller::Controller, controller_interface::ControllerBase)
