//
// Created by flying on 2021/1/18.
//
#include "rm_chassis_controllers/standard.h"
#include <rm_common/ros_utilities.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_chassis_controllers {
bool StandardController::init(hardware_interface::RobotHW *robot_hw,
                              ros::NodeHandle &root_nh,
                              ros::NodeHandle &controller_nh) {
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_rf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rf_name", std::string("joint_rf")));
  joint_rb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_rb_name", std::string("joint_rb")));
  joint_lb_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lb_name", std::string("joint_lb")));
  joint_lf_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_lf_name", std::string("joint_lf")));
  joint_vector_.push_back(joint_rf_);
  joint_vector_.push_back(joint_rb_);
  joint_vector_.push_back(joint_lb_);
  joint_vector_.push_back(joint_lf_);

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  if (!pid_rf_.init(ros::NodeHandle(controller_nh, "pid_rf")) ||
      !pid_rb_.init(ros::NodeHandle(controller_nh, "pid_rb")) ||
      !pid_lf_.init(ros::NodeHandle(controller_nh, "pid_lf")) ||
      !pid_lb_.init(ros::NodeHandle(controller_nh, "pid_lb")) ||
      !pid_follow_.init(ros::NodeHandle(controller_nh, "pid_follow")))
    return false;

  // init odom tf
  if (enable_odom_tf_) {
    odom2base_.header.frame_id = "odom";
    odom2base_.header.stamp = ros::Time::now();
    odom2base_.child_frame_id = "base_link";
    odom2base_.transform.rotation.w = 1;
    tf_broadcaster_.init(root_nh);
    tf_broadcaster_.sendTransform(odom2base_);
  }

  return true;
}

void StandardController::moveJoint(const ros::Duration &period) {
  ramp_x->input(vel_tfed_.vector.x);
  ramp_y->input(vel_tfed_.vector.y);
  ramp_w->input(vel_tfed_.vector.z);

  double a = (wheel_base_ + wheel_track_) / 2.0;
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

geometry_msgs::Twist StandardController::iKine(const ros::Duration &period) {
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

void StandardController::follow(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");

    recovery(period);
    pid_follow_.reset();
  }

  tfVelFromYawToBase(time);
  try {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation,
              roll, pitch, yaw);
    double follow_error =
        angles::shortest_angular_distance(yaw, 0);   //60^.

    pid_follow_.computeCommand(-follow_error, period);
    vel_tfed_.vector.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
}

void StandardController::twist(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter TWIST");

    recovery(period);
    pid_follow_.reset();
  }

  tfVelFromYawToBase(time);
  try {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation,
              roll, pitch, yaw);
    ros::Time now = ros::Time::now();
    double t = now.toSec();
    double angle[4] = {-0.785, 0.785, 2.355, -2.355};
    double off_set;
    for (double i : angle) {
      if (std::abs(angles::shortest_angular_distance(yaw, i)) < 0.79) {
        off_set = i;
        break;
      }
    }
    double follow_error =
        angles::shortest_angular_distance(yaw, twist_angular_ * sin(2 * M_PI * t) + off_set);

    pid_follow_.computeCommand(-follow_error, period);  //The actual output is opposite to the caculated value
    vel_tfed_.vector.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

}

void StandardController::gyro(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter GYRO");

    recovery(period);
  }
  tfVelFromYawToBase(time);
}

void StandardController::updateOdom(const ros::Time &time, const ros::Duration &period) {
  vel_base_ = iKine(period); // on base_link frame
  bool need_publish = publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time;

  linear_vel_.x = vel_base_.linear.x;
  linear_vel_.y = vel_base_.linear.y;
  angular_vel_.z = vel_base_.angular.z;
  // If enable_odom_tf, the linear and angular vel should be translated to odom from base_link then add to old odom->base_link
  // If not only publish the linear and angular vel under base_link
  if (enable_odom_tf_) {
    try {
      odom2base_ = robot_state_handle_.lookupTransform("odom", "base_link", ros::Time(0));
      tf2::doTransform(linear_vel_, linear_vel_, odom2base_);
      tf2::doTransform(angular_vel_, angular_vel_, odom2base_);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    odom2base_.header.stamp = time;
    // integral vel to pos and angle
    odom2base_.transform.translation.x += linear_vel_.x * period.toSec();
    odom2base_.transform.translation.y += linear_vel_.y * period.toSec();
    odom2base_.transform.translation.z += linear_vel_.z * period.toSec();
    double length =
        std::sqrt(std::pow(angular_vel_.x, 2) + std::pow(angular_vel_.y, 2) + std::pow(angular_vel_.z, 2));
    if (length > 0.001) { // avoid nan quat
      tf2::Quaternion odom2base_quat, trans_quat;
      tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
      trans_quat.setRotation(tf2::Vector3(angular_vel_.x / length,
                                          angular_vel_.y / length,
                                          angular_vel_.z / length), length * period.toSec());
      odom2base_quat = trans_quat * odom2base_quat;
      odom2base_quat.normalize();
      odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
    }
  }
  if (need_publish) {
    ChassisBase::updateOdom(time, period);
    if (enable_odom_tf_)
      tf_broadcaster_.sendTransform(odom2base_);
  } else if (enable_odom_tf_)
    robot_state_handle_.setTransform(odom2base_, "rm_chassis_controllers");
}

} // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::StandardController, controller_interface::ControllerBase)
