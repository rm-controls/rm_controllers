//
// Created by huakang on 2021/3/21.
//
#include "rm_chassis_controllers/chassis_base.h"
#include <rm_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <rm_common/ori_tool.h>

namespace rm_chassis_base {
bool ChassisBase::init(hardware_interface::RobotHW *robot_hw,
                       ros::NodeHandle &root_nh,
                       ros::NodeHandle &controller_nh) {
  wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.07625);
  wheel_base_ = getParam(controller_nh, "wheel_base", 0.320);
  wheel_track_ = getParam(controller_nh, "wheel_track", 0.410);
  publish_rate_ = getParam(controller_nh, "publish_rate", 100);
  twist_angular_ = getParam(controller_nh, "twist_angular", M_PI / 6);
  enable_odom_tf_ = getParam(controller_nh, "enable_odom_tf", true);
  timeout_ = getParam(controller_nh, "timeout", 1.0);

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

  // init odom tf
  if (enable_odom_tf_) {
    odom2base_.header.frame_id = "odom";
    odom2base_.header.stamp = ros::Time::now();
    odom2base_.child_frame_id = "base_link";
    odom2base_.transform.rotation.w = 1;
    tf_broadcaster_.init(root_nh);
    tf_broadcaster_.sendTransform(odom2base_);
  }

  cmd_chassis_sub_ =
      root_nh.subscribe<rm_msgs::ChassisCmd>("cmd_chassis", 1, &ChassisBase::cmdChassisCallback, this);
  cmd_vel_sub_ =
      root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

  return true;
}

void ChassisBase::update(const ros::Time &time, const ros::Duration &period) {
  rm_msgs::ChassisCmd cmd_chassis_ = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
  ramp_x->setAcc(cmd_chassis_.accel.linear.x);
  ramp_y->setAcc(cmd_chassis_.accel.linear.y);
  ramp_w->setAcc(cmd_chassis_.accel.angular.z);

  if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_) {
    vel_cmd_.vector.x = 0.;
    vel_cmd_.vector.y = 0.;
    vel_cmd_.vector.z = 0.;
  } else {
    geometry_msgs::Twist vel_cmd = cmd_rt_buffer_.readFromRT()->cmd_vel_;
    vel_cmd_.vector.x = vel_cmd.linear.x;
    vel_cmd_.vector.y = vel_cmd.linear.y;
    vel_cmd_.vector.z = vel_cmd.angular.z;
  }

  if (state_ != cmd_chassis_.mode) {
    state_ = StandardState(cmd_chassis_.mode);
    state_changed_ = true;
  }

  updateOdom(time, period);

  if (state_ == rm_chassis_base::PASSIVE)
    passive();
  else {
    if (state_ == rm_chassis_base::RAW)
      raw();
    else if (state_ == rm_chassis_base::GYRO)
      gyro();
    else if (state_ == rm_chassis_base::FOLLOW)
      follow(time, period);
    else if (state_ == rm_chassis_base::TWIST)
      twist(time, period);
    moveJoint(period);
  }
}

void ChassisBase::passive() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter PASSIVE");

    for (unsigned int i = 0; i < joint_vector_.size() - 1; ++i)
      joint_vector_[i].setCommand(0);
  }
}

void ChassisBase::follow(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");

    recovery();
    pid_follow_.reset();
  }
  tfVelToBase("yaw");
  try {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation,
              roll, pitch, yaw);
    double follow_error = angles::shortest_angular_distance(yaw, 0);
    pid_follow_.computeCommand(-follow_error, period);
    vel_tfed_.vector.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
}

void ChassisBase::twist(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");

    recovery();
    pid_follow_.reset();
  }
  tfVelToBase("yaw");
  try {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation,
              roll, pitch, yaw);

    double angle[4] = {-0.785, 0.785, 2.355, -2.355};
    double off_set;
    for (double i : angle) {
      if (std::abs(angles::shortest_angular_distance(yaw, i)) < 0.79) {
        off_set = i;
        break;
      }
    }
    double follow_error =
        angles::shortest_angular_distance(yaw, twist_angular_ * sin(2 * M_PI * time.toSec()) + off_set);

    pid_follow_.computeCommand(-follow_error, period);  //The actual output is opposite to the caculated value
    vel_tfed_.vector.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
}

void ChassisBase::gyro() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter GYRO");

    recovery();
  }
  tfVelToBase("yaw");
}

void ChassisBase::raw() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");

    recovery();
  }
  vel_tfed_ = vel_cmd_;
}

void ChassisBase::updateOdom(const ros::Time &time, const ros::Duration &period) {
  geometry_msgs::Twist vel_base = forwardKinematics(); // on base_link frame
  if (enable_odom_tf_) {
    geometry_msgs::Vector3 linear_vel_odom, angular_vel_odom;
    try {
      const geometry_msgs::TransformStamped
          odom2base = robot_state_handle_.lookupTransform("odom", "base_link", ros::Time(0));
      tf2::doTransform(vel_base.linear, linear_vel_odom, odom2base);
      tf2::doTransform(vel_base.angular, angular_vel_odom, odom2base);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    odom2base_.header.stamp = time;
    // integral vel to pos and angle
    odom2base_.transform.translation.x += linear_vel_odom.x * period.toSec();
    odom2base_.transform.translation.y += linear_vel_odom.y * period.toSec();
    odom2base_.transform.translation.z += linear_vel_odom.z * period.toSec();
    double length =
        std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) + std::pow(angular_vel_odom.z, 2));
    if (length > 0.001) { // avoid nan quat
      tf2::Quaternion odom2base_quat, trans_quat;
      tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
      trans_quat.setRotation(tf2::Vector3(angular_vel_odom.x / length,
                                          angular_vel_odom.y / length,
                                          angular_vel_odom.z / length), length * period.toSec());
      odom2base_quat = trans_quat * odom2base_quat;
      odom2base_quat.normalize();
      odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
    }
  }

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    if (odom_pub_->trylock()) {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.twist.twist.linear.x = vel_base.linear.x;
      odom_pub_->msg_.twist.twist.linear.y = vel_base.linear.y;
      odom_pub_->msg_.twist.twist.angular.z = vel_base.angular.z;
      odom_pub_->unlockAndPublish();
    }
    if (enable_odom_tf_)
      tf_broadcaster_.sendTransform(odom2base_);
    last_publish_time_ = time;
  } else if (enable_odom_tf_)
    robot_state_handle_.setTransform(odom2base_, "rm_chassis_controllers");
}

void ChassisBase::recovery() {
  geometry_msgs::Twist vel = forwardKinematics();

  ramp_x->clear(vel.linear.x);
  ramp_y->clear(vel.linear.y);
  ramp_w->clear(vel.angular.z);
}

void ChassisBase::cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr &msg) {
  cmd_struct_.cmd_chassis_ = *msg;
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  cmd_struct_.cmd_vel_ = *msg;
  cmd_struct_.stamp_ = ros::Time::now();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

void ChassisBase::tfVelToBase(const std::string &from) {
  try {
    tf2::doTransform(
        vel_cmd_, vel_tfed_,
        robot_state_handle_.lookupTransform(from, "yaw", ros::Time(0)));
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
}

} // namespace rm_chassis_base
