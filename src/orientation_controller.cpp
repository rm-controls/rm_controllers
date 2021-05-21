//
// Created by bruce on 2021/5/19.
//

#include "rm_orientation_controllers/orientation_controller.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>

namespace rm_orientation_controller {
bool Controller::init(hardware_interface::RobotHW *robot_hw,
                      ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  std::string imu_name;
  if (controller_nh.getParam("imu_name", imu_name)) {

  }
  imu_sensor_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("imu_name");
  robot_state_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");

  if (!controller_nh.getParam("frame_source", frame_source_) ||
      !controller_nh.getParam("frame_source", frame_source_) ||
      !controller_nh.getParam("frame_source", frame_source_)
      ) {
    ROS_ERROR("Some imu frame name params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  return true;
}

void Controller::update(const ros::Time &time, const ros::Duration &period) {
  data_.orientation.x = imu_sensor_.getOrientation()[0];
  data_.orientation.y = imu_sensor_.getOrientation()[1];
  data_.orientation.z = imu_sensor_.getOrientation()[2];
  data_.orientation.w = imu_sensor_.getOrientation()[3];
  data_.angular_velocity.x = imu_sensor_.getAngularVelocity()[0];
  data_.angular_velocity.y = imu_sensor_.getAngularVelocity()[1];
  data_.angular_velocity.z = imu_sensor_.getAngularVelocity()[2];
  data_.linear_acceleration.x = imu_sensor_.getLinearAcceleration()[0];
  data_.linear_acceleration.y = imu_sensor_.getLinearAcceleration()[1];
  data_.linear_acceleration.z = imu_sensor_.getLinearAcceleration()[2];

  fixtf();
}

void Controller::fixtf() {
  ros::Time now = ros::Time::now();
  double roll{}, pitch{}, yaw{};
  tf2::Transform source2odom, odom2fixed, fixed2target;
  tf2::Quaternion odom2fixed_quat;
  geometry_msgs::TransformStamped tf_msg;

  quatToRPY(data_.orientation, roll, pitch, yaw);

  try {
    tf_msg = tf_->lookupTransform(frame_source_,
                                  "odom", ros::Time(0));
    tf2::fromMsg(tf_msg.transform, source2odom);
    tf_msg = tf_->lookupTransform("odom",
                                  frame_fixed_,
                                  ros::Time(0));
    tf2::fromMsg(tf_msg.transform, odom2fixed);

    tf_msg = tf_->lookupTransform(frame_fixed_,
                                  frame_target_,
                                  ros::Time(0));
    tf2::fromMsg(tf_msg.transform, fixed2target);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  source2target_msg_.header.stamp = now;
  tf2::fromMsg(data_.orientation, odom2fixed_quat);//convert imu data to tf2 type
  odom2fixed.setRotation(odom2fixed_quat);
  source2target_msg_.transform =
      tf2::toMsg(source2odom * odom2fixed * fixed2target);
  tf_->setTransform(source2target_msg_, "rm_base");
  if ((now.toSec() - last_br_.toSec()) > 0.01) { //tf
    br_->sendTransform(source2target_msg_);
    last_br_ = now;
  }
}

