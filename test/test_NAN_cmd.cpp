#include "test_common.h"
#include <limits>

// TEST CASES
TEST_F(StandardChassisTest, testCmdVelNaN) {
  // wait for ROS
  while (!isControllerAlive()) {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel{};
  rm_msgs::ChassisCmd cmd_chassis{};
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 8;
  cmd_chassis.accel.linear.y = 8;
  cmd_chassis.accel.angular.z = 8;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();

  cmd_vel.linear.x = 1.0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.2).sleep();


  // send NaN
  cmd_vel.linear.x = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.5).sleep();
  nav_msgs::Odometry odom = getLastOdom();

  EXPECT_NEAR(odom.twist.twist.linear.x, 0.0, VELOCITY_TOLERANCE);

  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 1.0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.2).sleep();

  cmd_vel.linear.y = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();
  odom = getLastOdom();

  EXPECT_NEAR(odom.twist.twist.linear.y, 0.0, VELOCITY_TOLERANCE);

  cmd_vel.linear.y = 0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.2).sleep();

  cmd_vel.angular.z = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();
  odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.0, VELOCITY_TOLERANCE);
}


//Test case2
TEST_F(StandardChassisTest, testCmdChassisNaNEffortLimit) {
  // wait for ROS
  while (!isControllerAlive()) {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel{};
  rm_msgs::ChassisCmd cmd_chassis{};
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 8;
  cmd_chassis.accel.linear.y = 8;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(2).sleep();

  cmd_vel.linear.x = 1;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.2).sleep();

  cmd_chassis.effort_limit = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.5).sleep();

  nav_msgs::Odometry odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.0, VELOCITY_TOLERANCE);

}


//Test case3
TEST_F(StandardChassisTest, testCmdChassisNaNAccel) {
  // wait for ROS
  while (!isControllerAlive()) {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel{};
  rm_msgs::ChassisCmd cmd_chassis{};
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 8;
  cmd_chassis.accel.linear.y = 8;
  cmd_chassis.accel.angular.z = 8;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();

  cmd_vel.linear.x = 1.0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.2).sleep();

  cmd_chassis.accel.linear.x = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.5).sleep();

  nav_msgs::Odometry odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.0, VELOCITY_TOLERANCE);

  cmd_chassis.accel.linear.x = 0;
  cmd_vel.linear.y = 1.0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.2).sleep();

  cmd_chassis.accel.linear.y = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();

  odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.linear.y, 0.0, VELOCITY_TOLERANCE);

  cmd_chassis.accel.linear.y = 0;
  cmd_vel.angular.z = 1.0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.2).sleep();

  cmd_chassis.accel.angular.z = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();

  odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.0, VELOCITY_TOLERANCE);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "nan_cmd_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}