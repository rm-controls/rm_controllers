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
  assembly_error_pub_.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::AssemblyErrorData>(root_nh, "imu_assembly_error", 10));
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
  AssemblyErrorPub(time);
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

void Controller::AssemblyErrorPub(const ros::Time& time)
{
  double roll_error, pitch_error, yaw_error, roll_imu, pitch_imu, yaw_imu;
  geometry_msgs::TransformStamped tf_source2target, tf_target2imu;
  tf2::Transform source2target, target2imu;
  Eigen::Matrix3d roll_eigen, pitch_eigen, yaw_eigen, R_eigen;
  Eigen::Vector3d error_eigen, assembly_error_eigen;
  try
  {
    tf_source2target = robot_state_.lookupTransform(frame_source_, frame_target_, time);
    tf_target2imu = robot_state_.lookupTransform(frame_target_, imu_sensor_.getFrameId(), time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  tf2::fromMsg(tf_source2target.transform, source2target);
  tf2::Matrix3x3(source2target.getRotation()).getRPY(roll_error, pitch_error, yaw_error);
  tf2::fromMsg(tf_target2imu.transform, target2imu);
  tf2::Matrix3x3(target2imu.getRotation()).getRPY(roll_imu, pitch_imu, yaw_imu);
  roll_imu = static_cast<int>(roll_imu * 2 / M_PI);
  pitch_imu = static_cast<int>(pitch_imu * 2 / M_PI);
  yaw_imu = static_cast<int>(yaw_imu * 2 / M_PI);
  roll_eigen << 1, 0, 0, 0, cos(roll_imu * M_PI / 2), -sin(roll_imu * M_PI / 2), 0, sin(roll_imu * M_PI / 2),
      cos(roll_imu * M_PI / 2);
  pitch_eigen << cos(pitch_imu * M_PI / 2), 0, sin(pitch_imu * M_PI / 2), 0, 1, 0, -sin(pitch_imu * M_PI / 2), 0,
      cos(pitch_imu * M_PI / 2);
  yaw_eigen << cos(yaw_imu * M_PI / 2), -sin(yaw_imu * M_PI / 2), 0, sin(yaw_imu * M_PI / 2), cos(yaw_imu * M_PI / 2),
      0, 0, 0, 1;
  R_eigen = roll_eigen * pitch_eigen * yaw_eigen;
  error_eigen << roll_error, pitch_error, 0.0;
  assembly_error_eigen = R_eigen * error_eigen;
  if (loop_count_ % 100 == 0)
  {
    if (assembly_error_pub_->trylock())
    {
      assembly_error_pub_->msg_.header.stamp = time;
      assembly_error_pub_->msg_.roll_error = assembly_error_eigen[0];
      assembly_error_pub_->msg_.pitch_error = assembly_error_eigen[1];
      assembly_error_pub_->msg_.yaw_error = assembly_error_eigen[2];
      assembly_error_pub_->unlockAndPublish();
    }
    else
      ROS_WARN_THROTTLE(1, "Can't publish assembly error data");
  }
  loop_count_++;
}
}  // namespace rm_orientation_controller

PLUGINLIB_EXPORT_CLASS(rm_orientation_controller::Controller, controller_interface::ControllerBase)
