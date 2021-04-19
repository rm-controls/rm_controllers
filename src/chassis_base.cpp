//
// Created by huakang on 2021/3/21.
//
#include "rm_chassis_controllers/chassis_base.h"
#include <rm_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_chassis_base {
bool ChassisBase::init(hardware_interface::RobotHW *robot_hw,
                       ros::NodeHandle &root_nh,
                       ros::NodeHandle &controller_nh) {
  wheel_base_ = getParam(controller_nh, "wheel_base", 0.320);
  wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.07625);
  publish_rate_ = getParam(controller_nh, "publish_rate", 100);
  timeout_ = getParam(controller_nh, "timeout", 1.0);
  enable_timeout_ = getParam(controller_nh, "enable_timeout", true);

  // Get and check params for covariances
  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";
  odom_pub_->msg_.twist.covariance = {
      static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
      0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
      0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
      0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
      0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
      0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};

  ramp_x = new RampFilter<double>(0, 0.001);
  ramp_y = new RampFilter<double>(0, 0.001);
  ramp_w = new RampFilter<double>(0, 0.001);

  cmd_chassis_sub_ =
      root_nh.subscribe<rm_msgs::ChassisCmd>("cmd_chassis", 1, &ChassisBase::cmdChassisCallback, this);
  cmd_vel_sub_ =
      root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

  return true;
}

void ChassisBase::update(const ros::Time &time, const ros::Duration &period) {
  if (enable_timeout_) {
    timeOut(time);
  }

  cmd_chassis_ = *chassis_rt_buffer_.readFromRT();
  ramp_x->setAcc(cmd_chassis_.accel.linear.x);
  ramp_y->setAcc(cmd_chassis_.accel.linear.y);
  ramp_w->setAcc(cmd_chassis_.accel.angular.z);

  geometry_msgs::Twist vel_cmd;
  vel_cmd = *vel_rt_buffer_.readFromRT();
  vel_cmd_.vector.x = vel_cmd.linear.x;
  vel_cmd_.vector.y = vel_cmd.linear.y;
  vel_cmd_.vector.z = vel_cmd.angular.z;

  if (state_ != cmd_chassis_.mode) {
    state_ = StandardState(cmd_chassis_.mode);
    state_changed_ = true;
  }
}

void ChassisBase::recovery(const ros::Duration &period) {
  geometry_msgs::Twist vel = iKine(period);

  ramp_x->clear(vel.linear.x);
  ramp_y->clear(vel.linear.y);
  ramp_w->clear(vel.angular.z);
}

void ChassisBase::passive() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter PASSIVE");

    for (unsigned int i = 0; i < joint_vector_.size() - 1; ++i)
      joint_vector_[i].setCommand(0);
  }
}

void ChassisBase::raw(const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");

    recovery(period);
  }
  vel_tfed_ = vel_cmd_;
}

void ChassisBase::updateOdom(const ros::Time &time, const ros::Duration &period) {
  last_publish_time_ = time;
  // publish odom message
  if (odom_pub_->trylock()) {
    odom_pub_->msg_.header.stamp = time;
    odom_pub_->msg_.twist.twist.linear.x = linear_vel_.x;
    odom_pub_->msg_.twist.twist.linear.y = linear_vel_.y;
    odom_pub_->msg_.twist.twist.angular.z = angular_vel_.z;
    odom_pub_->unlockAndPublish();
  }
}

void ChassisBase::cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr &msg) {
  chassis_rt_buffer_.writeFromNonRT(*msg);
}

void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd) {
  vel_rt_buffer_.writeFromNonRT(*cmd);
}

void ChassisBase::tfVelFromYawToBase(const ros::Time &time) {
  try {
    tf2::doTransform(
        vel_cmd_, vel_tfed_,
        robot_state_handle_.lookupTransform("base_link", "yaw", ros::Time(0)));
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
}

void ChassisBase::timeOut(const ros::Time &time) {
  if ((time - cmd_chassis_callback_time_).toSec() > timeout_ || (time - cmd_vel_callback_time_).toSec() > timeout_) {
    cmd_chassis_.effort_limit = 0;
    chassis_rt_buffer_.writeFromNonRT(cmd_chassis_);
    //ROS_INFO_THROTTLE(2.0, "Message cmd_vel and cmd_chassis timeout!");
  }
  //else
  //ROS_INFO_THROTTLE(2.0, "Message cmd_vel and cmd_chassis come in time!");
}

} // namespace rm_chassis_base
