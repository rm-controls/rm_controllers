//
// Created by qiayuan on 2/15/21.
//

#ifndef RM_CHASSIS_CONTROLLER_TEST_COMMON_H
#define RM_CHASSIS_CONTROLLER_TEST_COMMON_H

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <rm_msgs/ChassisCmd.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>

const double POSITION_TOLERANCE = 0.08; // 1 mm-s precision
const double VELOCITY_TOLERANCE = 0.05; // 1 mm-s-1 precision
const double EPS = 0.01;

class StandardChassisTest : public ::testing::Test {
 public:
  StandardChassisTest() :
      cmd_chassis_pub_(nh_.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 10)),
      cmd_vel_pub_(nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
      vel_cmd_pub_(nh_.advertise<geometry_msgs::Twist>("/vel_cmd", 10)),
      chassis_cmd_pub_(nh_.advertise<rm_msgs::ChassisCmd>("/chassis_cmd", 10)),
      odom_sub_(nh_.subscribe("/odom", 10, &StandardChassisTest::odomCallback, this)),
      link_states_sub_(nh_.subscribe("/gazebo/link_states", 10, &StandardChassisTest::linkStatesCallback, this)),
      joint_states_sub_(nh_.subscribe("/joint_states", 10, &StandardChassisTest::jointStatesCallback, this)),
      received_first_odom_(false) {};

  ~StandardChassisTest() override {
    odom_sub_.shutdown();
    link_states_sub_.shutdown();
  }

  void waitForController() const {
    while (!isControllerAlive() && ros::ok()) {
      ROS_INFO_STREAM_THROTTLE(.5, "Waiting for controller.");
      ros::Duration(0.1).sleep();
    }
    if (!ros::ok())
      FAIL() << "Something went wrong while executing test.";
  }

  void waitForOdomMsgs() const {
    while (!received_first_odom_ && ros::ok()) {
      ROS_INFO_STREAM_THROTTLE(.5, "Waiting for odom messages to be published.");
      ros::Duration(0.01).sleep();
    }
    if (!ros::ok())
      FAIL() << "Something went wrong while executing test.";
  }

  const geometry_msgs::Pose &getPose() { return base_link_pose_; }
  const geometry_msgs::Twist &getTwist() { return base_link_twist_; }
  const nav_msgs::Odometry getLastOdom() { return last_odom_; }
  const sensor_msgs::JointState getJointStates() { return joint_state_; }

  bool isControllerAlive() const { return (odom_sub_.getNumPublishers() > 0); }

  geometry_msgs::Twist cmd_vel_{};
  rm_msgs::ChassisCmd cmd_chassis_{};

  void publish(const rm_msgs::ChassisCmd &chassis_cmd, const geometry_msgs::Twist &twist) {
    cmd_chassis_pub_.publish(chassis_cmd);
    cmd_vel_pub_.publish(twist);
  }
  void publish() {
    cmd_chassis_pub_.publish(cmd_chassis_);
    cmd_vel_pub_.publish(cmd_vel_);
  }

  void publishFromWrongTopic(const rm_msgs::ChassisCmd &chassis_cmd, const geometry_msgs::Twist &twist) {
    chassis_cmd_pub_.publish(chassis_cmd);
    vel_cmd_pub_.publish(twist);
  }
  void publishFromWrongTopic() {
    chassis_cmd_pub_.publish(cmd_chassis_);
    vel_cmd_pub_.publish(cmd_vel_);
  }

  void zeroCmdVel() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }

  void zeroCmdChassis() {
    cmd_chassis_.mode = cmd_chassis_.RAW;
    cmd_chassis_.accel.linear.x = 0.0;
    cmd_chassis_.accel.linear.y = 0.0;
    cmd_chassis_.accel.angular.z = 0.0;
    cmd_chassis_.power_limit = 99;
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_chassis_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher vel_cmd_pub_;
  ros::Publisher chassis_cmd_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber link_states_sub_;
  ros::Subscriber joint_states_sub_;
  nav_msgs::Odometry last_odom_;
  geometry_msgs::Pose base_link_pose_;  //  from Gazebo
  geometry_msgs::Twist base_link_twist_; //  from Gazebo
  sensor_msgs::JointState joint_state_;
  bool received_first_odom_;

  void odomCallback(const nav_msgs::Odometry &odom) {
    last_odom_ = odom;
    received_first_odom_ = true;
  }

  void jointStatesCallback(const sensor_msgs::JointState &data) {
    joint_state_ = data;
  }

  void linkStatesCallback(const gazebo_msgs::LinkStates &link_states) {
    for (size_t i = 0; i < link_states.name.size(); ++i) {
      if (link_states.name[i] == static_cast<std::string>("standard::base_link")) {
        // TODO: Add ROS_DEBUG for debugging
        base_link_pose_ = link_states.pose[i];
        base_link_twist_ = link_states.twist[i];
      }
    }
  }
};

#endif //RM_CHASSIS_CONTROLLER_TEST_COMMON_H
