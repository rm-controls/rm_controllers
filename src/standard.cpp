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

namespace rm_chassis_controller {
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

  ramp_x = new RampFilter<double>(0, 0.001);
  ramp_y = new RampFilter<double>(0, 0.001);
  ramp_w = new RampFilter<double>(0, 0.001);

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  if (!pid_rf_.init(ros::NodeHandle(controller_nh, "pid_rf")) ||
      !pid_rb_.init(ros::NodeHandle(controller_nh, "pid_rb")) ||
      !pid_lf_.init(ros::NodeHandle(controller_nh, "pid_lf")) ||
      !pid_lb_.init(ros::NodeHandle(controller_nh, "pid_lb")))
    return false;

  chassis_cmd_subscriber_ =
      root_nh.subscribe<rm_msgs::ChassisCmd>("cmd_chassis", 1, &ChassisStandardController::commandCB, this);
  vel_cmd_subscriber_ =
      root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisStandardController::velCmdCB, this);

  return true;
}

void ChassisStandardController::update(const ros::Time &time, const ros::Duration &period) {
  cmd_ = *chassis_rt_buffer_.readFromRT();

  if (state_ != cmd_.mode) {
    state_ = StandardState(cmd_.mode);
    state_changed_ = true;
  }

  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == RAW)
      raw();
    else if (state_ == GYRO)
      gyro();
    else if (state_ == FOLLOW)
      follow(period);
    else if (state_ == TWIST)
      twist(period);
    moveJoint(period);
    setTrans();
  }

//  switch (state_) {
//    case PASSIVE: passive();
//      break;
//    case RAW: raw(time);
//      break;
//    case FOLLOW: follow(time);
//      break;
//    case TWIST: twist(time);
//      break;
//    case GYRO: gyro(time);
//      break;
// //    case FLY: fly(time);
// //      break;
//    default: break;
//  }
// //  if (state_ != FLY)
//    moveJoint(period);
}

void ChassisStandardController::moveJoint(const ros::Duration &period) {
  ramp_x->input(vel_tfed_.vector.x);
  ramp_y->input(vel_tfed_.vector.y);
  ramp_w->input(vel_tfed_.vector.z);

  double a = wheel_base_ + wheel_track_;
  double joint_rf_des{}, joint_lf_des{}, joint_lb_des{}, joint_rb_des{};
  joint_rf_des =
      (ramp_x->output() + ramp_y->output() + ramp_w->output() * a)
          / wheel_radius_;
  joint_lf_des =
      (ramp_x->output() - ramp_y->output() - ramp_w->output() * a)
          / wheel_radius_;
  joint_lb_des =
      (ramp_x->output() + ramp_y->output() - ramp_w->output() * a)
          / wheel_radius_;
  joint_rb_des =
      (ramp_x->output() - ramp_y->output() + ramp_w->output() * a)
          / wheel_radius_;

  double joint_rf_error = joint_rf_des - joint_rf_.getPosition();
  double joint_rb_error = joint_rb_des - joint_rb_.getPosition();
  double joint_lf_error = joint_lf_des - joint_lf_.getPosition();
  double joint_lb_error = joint_lb_des - joint_lb_.getPosition();

  pid_rf_.computeCommand(joint_rf_error, period);
  pid_rb_.computeCommand(joint_rb_error, period);
  pid_lf_.computeCommand(joint_lf_error, period);
  pid_lb_.computeCommand(joint_lb_error, period);

  joint_rf_.setCommand(pid_rf_.getCurrentCmd());
  joint_rb_.setCommand(pid_rb_.getCurrentCmd());
  joint_lf_.setCommand(pid_lf_.getCurrentCmd());
  joint_lb_.setCommand(pid_lb_.getCurrentCmd());
}

geometry_msgs::Twist ChassisStandardController::getVel() {
  geometry_msgs::Twist vel_data;
  double k = wheel_radius_ / 4.0;
  double joint_rf_position{}, joint_rb_position{}, joint_lf_position{}, joint_lb_position{};
  joint_rf_position = joint_rf_.getPosition();
  joint_rb_position = joint_rb_.getPosition();
  joint_lf_position = joint_lf_.getPosition();
  joint_lb_position = joint_lb_.getPosition();
  vel_data.linear.x =
      (joint_rf_position + joint_lf_position + joint_lb_position + joint_rb_position) * k;
  vel_data.linear.y =
      (joint_rf_position - joint_lf_position + joint_lb_position - joint_rb_position) * k;
  vel_data.angular.z =
      2 * (joint_rf_position - joint_lf_position - joint_lb_position + joint_rb_position) * k
          / (wheel_base_ + wheel_track_);
  return vel_data;
}

void ChassisStandardController::setVel() {
  geometry_msgs::Twist vel = getVel();

  ramp_x->clear(vel.linear.x);
  ramp_y->clear(vel.linear.y);
  ramp_w->clear(vel.angular.z);
}

void ChassisStandardController::passive() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enther PASSIVE");

    joint_rf_.setCommand(0);
    joint_rb_.setCommand(0);
    joint_lf_.setCommand(0);
    joint_lb_.setCommand(0);
  }
}

void ChassisStandardController::raw() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enther RAW");

    setVel();
  }

  vel_tfed_ = vel_cmd_;
}

void ChassisStandardController::follow(const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enther FOLLOW");

    setVel();
  }

  transformFollowVel(period);
}

void ChassisStandardController::twist(const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enther TWIST");

    setVel();
  }

  transformTwistVel(period);
}

void ChassisStandardController::gyro() {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enther GYRO");

    setVel();
  }

  transformGyroVel();
}

//void ChassisStandardController::fly() {
//  if(state_changed_) {
//    state_changed_ = false;
//    ROS_INFO("[Chassis] Enther FLY");
//  }
//}

void ChassisStandardController::transformGyroVel() {
  try {
    tf2::doTransform(vel_cmd_, vel_tfed_,
                     robot_state_handle_.lookupTransform("base_link", "yaw_link", ros::Time(0)));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

void ChassisStandardController::transformFollowVel(const ros::Duration &period) {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = robot_state_handle_.lookupTransform("base_link", "yaw_link", ros::Time(0));
    tf2::doTransform(vel_cmd_, vel_tfed_, transformStamped);
    double roll{}, pitch{}, yaw{};

    quatToRPY(transformStamped.transform.rotation, roll, pitch, yaw);

    double follow_error = 0 - yaw;
    pid_follow_.computeCommand(follow_error, period);
    vel_tfed_.vector.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

void ChassisStandardController::transformTwistVel(const ros::Duration &period) {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = robot_state_handle_.lookupTransform("base_link", "yaw_link", ros::Time(0));
    tf2::doTransform(vel_cmd_, vel_tfed_, transformStamped);
    double roll{}, pitch{}, yaw{};

    quatToRPY(transformStamped.transform.rotation, roll, pitch, yaw);

    ros::Time now = ros::Time::now();
    double t = now.toSec();
    //    yaw = -yaw;
    double twist_error = angles::shortest_angular_distance(yaw, (0.3 * sin(t) - yaw));

    pid_twist_.computeCommand(twist_error, period);
    vel_tfed_.vector.z = pid_twist_.getCurrentCmd();
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

void ChassisStandardController::setTrans() {
  geometry_msgs::Twist vel = getVel(); //on base_link frame
  geometry_msgs::Vector3Stamped trans;
  trans.vector.x = vel.linear.x * 0.001;
  trans.vector.y = vel.linear.y * 0.001;

  try {
    word2base_ = robot_state_handle_.lookupTransform("word", "base_link", ros::Time(0));
    tf2::doTransform(trans, trans, word2base_);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  ros::Time now = ros::Time::now();
  word2base_.header.stamp = now;
  word2base_.transform.translation.x += trans.vector.x;
  word2base_.transform.translation.y += trans.vector.y;
  word2base_.transform.translation.z += trans.vector.z;

  double roll, pitch, yaw;
  quatToRPY(word2base_.transform.rotation, roll, pitch, yaw);

  word2base_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(
      roll, pitch, yaw + 0.001 * vel.angular.z);
  robot_state_handle_.setTransform(word2base_, "rm_base");
}

//void ChassisStandardController::setDes(const ros::Time &time, double x, double y, double z) {
//
//}

void ChassisStandardController::commandCB(const rm_msgs::ChassisCmdConstPtr &msg) {
  chassis_rt_buffer_.writeFromNonRT(*msg);
}

void ChassisStandardController::velCmdCB(const geometry_msgs::Twist::ConstPtr &cmd) {
  vel_rt_buffer_.writeFromNonRT(*cmd);
  geometry_msgs::Twist vel_cmd;
  vel_cmd = *vel_rt_buffer_.readFromRT();
  vel_cmd_.vector.x = vel_cmd.linear.x;
  vel_cmd_.vector.y = vel_cmd.linear.y;
  vel_cmd_.vector.z = vel_cmd.angular.z;
}

}// namespace rm_chassis_controller

PLUGINLIB_EXPORT_CLASS(rm_chassis_controller::ChassisStandardController, controller_interface::ControllerBase)
