//
// Created by flying on 2021/1/18.
//

#include <ros_utilities.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/ChassisCmd.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rm_chassis_controller/standard.h"

namespace rm_chassis_controllers {
bool ChassisStandardController::init(hardware_interface::RobotHW *robot_hw,
                                     ros::NodeHandle &root_nh,
                                     ros::NodeHandle &controller_nh) {

  wheel_base_ = getParam(controller_nh, "wheel_base", 0.320);
  wheel_track_ = getParam(controller_nh, "wheel_track", 0.410);
  wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.07625);

  publish_rate_ = getParam(controller_nh, "publish_rate_", 50);

  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";
  odom_pub_->msg_.pose.covariance = {
      static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
      0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
      0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
      0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
      0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
      0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])};
  odom_pub_->msg_.twist.covariance = {
      static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
      0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
      0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
      0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
      0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
      0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};

  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_rf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rf_name", std::string("joint_rf")));
  joint_rb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rb_name", std::string("joint_rb")));
  joint_lb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lb_name", std::string("joint_lb")));
  joint_lf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lf_name", std::string("joint_lf")));

  ramp_x = new RampFilter<double>(0, 0.001);
  ramp_y = new RampFilter<double>(0, 0.001);
  ramp_w = new RampFilter<double>(0, 0.001);

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  if (!pid_rf_.init(ros::NodeHandle(controller_nh, "pid_rf")) ||
      !pid_rb_.init(ros::NodeHandle(controller_nh, "pid_rb")) ||
      !pid_lf_.init(ros::NodeHandle(controller_nh, "pid_lf")) ||
      !pid_lb_.init(ros::NodeHandle(controller_nh, "pid_lb")))
    return false;

  // init odom tf
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = "base_link";
  odom2base_.transform.rotation.w = 1;
  tf_broadcaster_.init(root_nh);
  tf_broadcaster_.sendTransform(odom2base_);

  chassis_cmd_subscriber_ =
      root_nh.subscribe<rm_msgs::ChassisCmd>("cmd_chassis", 1, &ChassisStandardController::commandCB, this);
  vel_cmd_subscriber_ =
      root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisStandardController::velCmdCB, this);

  return true;
}

void ChassisStandardController::update(const ros::Time &time, const ros::Duration &period) {
  cmd_chassis_ = *chassis_rt_buffer_.readFromRT();
  ramp_x->setAcc(cmd_chassis_.accel.linear.x);
  ramp_y->setAcc(cmd_chassis_.accel.linear.y);
  ramp_w->setAcc(cmd_chassis_.accel.angular.z);

  geometry_msgs::Twist vel_cmd;
  vel_cmd = *vel_rt_buffer_.readFromRT();
  vel_cmd_.vector.x = vel_cmd.linear.x;
  vel_cmd_.vector.y = vel_cmd.linear.y;
  vel_cmd_.vector.z = vel_cmd.angular.z;

  updateOdom(time, period);

  if (state_ != cmd_chassis_.mode) {
    state_ = StandardState(cmd_chassis_.mode);
    state_changed_ = true;
  }

  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == RAW)
      raw();
    else if (state_ == GYRO)
      gyro(time);
    else if (state_ == FOLLOW)
      follow(time, period);
    else if (state_ == TWIST)
      twist(time, period);
    moveJoint(period);
  }
}

void ChassisStandardController::moveJoint(const ros::Duration &period) {
  ramp_x->input(vel_tfed_.vector.x);
  ramp_y->input(vel_tfed_.vector.y);
  ramp_w->input(vel_tfed_.vector.z);

  double a = wheel_base_ + wheel_track_;
  double joint_rf_des = (ramp_x->output() + ramp_y->output() + ramp_w->output() * a) / wheel_radius_;
  double joint_lf_des = (ramp_x->output() - ramp_y->output() - ramp_w->output() * a) / wheel_radius_;
  double joint_lb_des = (ramp_x->output() + ramp_y->output() - ramp_w->output() * a) / wheel_radius_;
  double joint_rb_des = (ramp_x->output() - ramp_y->output() + ramp_w->output() * a) / wheel_radius_;

  double joint_rf_error = joint_rf_des - joint_rf_.getVelocity();
  double joint_rb_error = joint_rb_des - joint_rb_.getVelocity();
  double joint_lf_error = joint_lf_des - joint_lf_.getVelocity();
  double joint_lb_error = joint_lb_des - joint_lb_.getVelocity();

  pid_rf_.computeCommand(joint_rf_error, period);
  pid_rb_.computeCommand(joint_rb_error, period);
  pid_lf_.computeCommand(joint_lf_error, period);
  pid_lb_.computeCommand(joint_lb_error, period);

  // Power limit
  double real_effort = (std::abs(pid_rf_.getCurrentCmd()) + std::abs(pid_rb_.getCurrentCmd()) +
      std::abs(pid_lf_.getCurrentCmd()) + std::abs(pid_lb_.getCurrentCmd()));

  double prop = real_effort > cmd_chassis_.effort_limit ? cmd_chassis_.effort_limit / real_effort : 1.;

  joint_rf_.setCommand(prop * pid_rf_.getCurrentCmd());
  joint_rb_.setCommand(prop * pid_rb_.getCurrentCmd());
  joint_lf_.setCommand(prop * pid_lf_.getCurrentCmd());
  joint_lb_.setCommand(prop * pid_lb_.getCurrentCmd());
}

geometry_msgs::Twist ChassisStandardController::iKine() {
  geometry_msgs::Twist vel_data;
  double k = wheel_radius_ / 4.0;
  double joint_rf_velocity = joint_rf_.getVelocity();
  double joint_rb_velocity = joint_rb_.getVelocity();
  double joint_lf_velocity = joint_lf_.getVelocity();
  double joint_lb_velocity = joint_lb_.getVelocity();
  vel_data.linear.x =
      (joint_rf_velocity + joint_lf_velocity + joint_lb_velocity + joint_rb_velocity) * k;
  vel_data.linear.y =
      (joint_rf_velocity - joint_lf_velocity + joint_lb_velocity - joint_rb_velocity) * k;
  vel_data.angular.z =
      2 * (joint_rf_velocity - joint_lf_velocity - joint_lb_velocity + joint_rb_velocity) * k
          / (wheel_base_ + wheel_track_);
  return vel_data;
}

void ChassisStandardController::recovery() {
  geometry_msgs::Twist vel = iKine();

  ramp_x->clear(vel.linear.x);
  ramp_y->clear(vel.linear.y);
  ramp_w->clear(vel.angular.z);
}

void ChassisStandardController::passive() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter PASSIVE");

    joint_rf_.setCommand(0);
    joint_rb_.setCommand(0);
    joint_lf_.setCommand(0);
    joint_lb_.setCommand(0);
  }
}

void ChassisStandardController::raw() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");

    recovery();
  }

  vel_tfed_ = vel_cmd_;
}

void ChassisStandardController::follow(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");

    recovery();
    pid_follow_.reset();
  }

  tfVelFromYawToBase(time);
  try {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform("base_link", "link_yaw", time).transform.rotation, roll, pitch, yaw);
    double follow_error = 0 - yaw;
    pid_follow_.computeCommand(follow_error, period);
    vel_tfed_.vector.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
}

void ChassisStandardController::twist(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter TWIST");

    recovery();
    pid_twist_.reset();
  }

  tfVelFromYawToBase(time);
}

void ChassisStandardController::gyro(const ros::Time &time) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter GYRO");

    recovery();
  }
  tfVelFromYawToBase(time);
}

void ChassisStandardController::updateOdom(const ros::Time &time, const ros::Duration &period) {
  geometry_msgs::Twist vel_base = iKine(); //on base_link frame
  geometry_msgs::Vector3 linear_vel{}, angular_vel{};
  linear_vel.x = vel_base.linear.x;
  linear_vel.y = vel_base.linear.y;
  angular_vel.z = vel_base.angular.z;
  try {
    odom2base_ = robot_state_handle_.lookupTransform("odom", "base_link", ros::Time(0));
    tf2::doTransform(linear_vel, linear_vel, odom2base_);
    tf2::doTransform(angular_vel, angular_vel, odom2base_);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  odom2base_.header.stamp = time;
  // integral vel to pos and angle
  odom2base_.transform.translation.x += linear_vel.x * period.toSec();
  odom2base_.transform.translation.y += linear_vel.y * period.toSec();
  odom2base_.transform.translation.z += linear_vel.z * period.toSec();
  double length =
      std::sqrt(std::pow(angular_vel.x, 2) + std::pow(angular_vel.y, 2) + std::pow(angular_vel.z, 2));
  if (length > 0.001) { // avoid nan quat
    tf2::Quaternion odom2base_quat, trans_quat;
    tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
    trans_quat.setRotation(tf2::Vector3(angular_vel.x / length,
                                        angular_vel.y / length,
                                        angular_vel.z / length), length * period.toSec());
    odom2base_quat = trans_quat * odom2base_quat;
    odom2base_quat.normalize();
    odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
  }

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    last_publish_time_ = time;
    tf_broadcaster_.sendTransform(odom2base_);
    // publish odom message
    if (odom_pub_->trylock()) {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = odom2base_.transform.translation.x;
      odom_pub_->msg_.pose.pose.position.y = odom2base_.transform.translation.y;
      odom_pub_->msg_.pose.pose.position.z = odom2base_.transform.translation.z;
      odom_pub_->msg_.pose.pose.orientation = odom2base_.transform.rotation;
      odom_pub_->unlockAndPublish();
    }
  } else
    robot_state_handle_.setTransform(odom2base_, "rm_chassis_controllers");
}

void ChassisStandardController::commandCB(const rm_msgs::ChassisCmdConstPtr &msg) {
  chassis_rt_buffer_.writeFromNonRT(*msg);
}

void ChassisStandardController::velCmdCB(const geometry_msgs::Twist::ConstPtr &cmd) {
  vel_rt_buffer_.writeFromNonRT(*cmd);
}

void ChassisStandardController::tfVelFromYawToBase(const ros::Time &time) {
  try {
    tf2::doTransform(
        vel_cmd_, vel_tfed_,
        robot_state_handle_.lookupTransform("base_link", "link_yaw", ros::Time(0)));
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
}

} // namespace rm_chassis_controller

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ChassisStandardController, controller_interface::ControllerBase)
