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
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_rf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rf_name", std::string("joint_rf")));
  joint_rb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rb_name", std::string("joint_rb")));
  joint_lb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lb_name", std::string("joint_lb")));
  joint_lf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lf_name", std::string("joint_lf")));

  wheel_base_ = getParam(controller_nh, "wheel_base", 0.320);
  wheel_track_ = getParam(controller_nh, "wheel_track", 0.410);
  wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.07625);

  publish_rate_ = getParam(controller_nh, "publish_rate_", 50);
  current_coeff_ = getParam(controller_nh, "current_coeff_", 1.0);

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
  double coeff = 1.0;
  double real_current =
      coeff * (pid_rf_.getCurrentCmd() + pid_rb_.getCurrentCmd() + pid_lf_.getCurrentCmd() + pid_lb_.getCurrentCmd());

  double prop = real_current > cmd_chassis_.current_limit ? cmd_chassis_.current_limit / real_current : 1.;

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
  geometry_msgs::Twist vel = iKine(); //on base_link frame
  geometry_msgs::Vector3Stamped trans;
  trans.vector.x = vel.linear.x * period.toSec();
  trans.vector.y = vel.linear.y * period.toSec();

  try {
    odom2base_ = robot_state_handle_.lookupTransform("odom", "base_link", ros::Time(0));
    tf2::doTransform(trans, trans, odom2base_);
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
  ros::Time now = ros::Time::now();
  odom2base_.header.stamp = now;
  odom2base_.transform.translation.x += trans.vector.x;
  odom2base_.transform.translation.y += trans.vector.y;
  odom2base_.transform.translation.z += trans.vector.z;

  double roll, pitch, yaw;
  quatToRPY(odom2base_.transform.rotation, roll, pitch, yaw);
  odom2base_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(
      roll, pitch, yaw + period.toSec() * vel.angular.z);

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    tf_broadcaster_.sendTransform(odom2base_);
    last_publish_time_ = time;
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

}// namespace rm_chassis_controller

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ChassisStandardController, controller_interface::ControllerBase)
