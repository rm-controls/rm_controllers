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
  cmd_chassis.accel.linear.x = 0;
  cmd_chassis.accel.linear.y = 0;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.1).sleep();

  // send NaN
  for (int i = 0; i < 10; ++i) {
    cmd_vel.linear.x = NAN;
    cmd_vel.linear.y = NAN;
    cmd_vel.angular.z = NAN;
    publish(cmd_chassis, cmd_vel);
    geometry_msgs::Twist odom_msg = getTwist();
    EXPECT_FALSE(std::isnan(odom_msg.linear.x));
    EXPECT_FALSE(std::isnan(odom_msg.linear.y));
    EXPECT_FALSE(std::isnan(odom_msg.angular.z));
    ros::Duration(0.1).sleep();
  }

  nav_msgs::Odometry odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.0, VELOCITY_TOLERANCE);
  EXPECT_NEAR(odom.twist.twist.linear.y, 0.0, VELOCITY_TOLERANCE);
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.0, VELOCITY_TOLERANCE);
  EXPECT_NEAR(odom.pose.pose.position.x, 0.0, POSITION_TOLERANCE);
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, POSITION_TOLERANCE);

  geometry_msgs::Twist odom_msg = getTwist();
  EXPECT_NEAR(odom_msg.linear.x, 0.0, VELOCITY_TOLERANCE);
  EXPECT_NEAR(odom_msg.linear.y, 0.0, VELOCITY_TOLERANCE);
  EXPECT_NEAR(odom_msg.angular.z, 0.0, VELOCITY_TOLERANCE);

  ROS_INFO("Test send NAN cmd_vel success!");
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
  cmd_chassis.accel.linear.x = 0;
  cmd_chassis.accel.linear.y = 0;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.1).sleep();

  cmd_chassis.effort_limit = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(1).sleep();
  cmd_vel.linear.x = 0.5;
  publish(cmd_chassis, cmd_vel);

  ros::Duration(10).sleep();

  nav_msgs::Odometry odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.0, VELOCITY_TOLERANCE);
  EXPECT_NEAR(odom.pose.pose.position.x, 0.0, POSITION_TOLERANCE);
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, POSITION_TOLERANCE);

  geometry_msgs::Twist odom_msg = getTwist();
  EXPECT_NEAR(odom_msg.linear.x, 0.0, VELOCITY_TOLERANCE);

  ROS_INFO("Test send NAN effort limit success!");
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
  cmd_chassis.accel.linear.x = 0;
  cmd_chassis.accel.linear.y = 0;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(0.1).sleep();

  cmd_vel.linear.x = 0.5;
  cmd_chassis.accel.linear.x = NAN;
  //cmd_chassis.accel.angular.z = NAN;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(5).sleep();

  nav_msgs::Odometry odom = getLastOdom();
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.0, VELOCITY_TOLERANCE);
  EXPECT_NEAR(odom.pose.pose.position.x, 0.0, POSITION_TOLERANCE);
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, POSITION_TOLERANCE);

  geometry_msgs::Twist odom_msg = getTwist();
  EXPECT_NEAR(odom_msg.linear.x, 0.0, VELOCITY_TOLERANCE);

  ROS_INFO("Test send NAN effort limit success!");
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