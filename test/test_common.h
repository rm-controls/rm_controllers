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

class StandardChassisTest : public ::testing::Test {
 public:
  StandardChassisTest() :
      cmd_chassis_pub_(nh_.advertise<rm_msgs::ChassisCmd>("cmd_chassis", 10)),
      cmd_vel_pub_(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
      received_first_odom_(false) {};

 private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_chassis_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry last_odom_;

  bool received_first_odom_;

  void odomCallback(const nav_msgs::Odometry &odom) {
    ROS_INFO_STREAM("Callback received: pos.x: " << odom.pose.pose.position.x
                                                 << ", orient.z: " << odom.pose.pose.orientation.z
                                                 << ", lin_est: " << odom.twist.twist.linear.x
                                                 << ", ang_est: " << odom.twist.twist.angular.z);
    last_odom_ = odom;
    received_first_odom_ = true;
  }
};

#endif //RM_CHASSIS_CONTROLLER_TEST_COMMON_H
