
#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testLinearYDirectionJerkLimits) {
// wait for ROS
  waitForController();

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
  ros::Duration(5.0).sleep();
// get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  geometry_msgs::Pose old_base_link_pose_ = getPose();  //  from Gazebo
  geometry_msgs::Twist old_base_link_twist_ = getTwist();//  from Gazebo
// send a big command
  cmd_vel.linear.y = 10.0;
  cmd_chassis.accel.linear.y = 10.0;
  publish(cmd_chassis, cmd_vel);
// wait for a while
  ros::Duration(1.0).sleep();

  geometry_msgs::Pose new_base_link_pose_ = getPose();  //  from Gazebo
  geometry_msgs::Twist new_base_link_twist_ = getTwist();//  from Gazebo
  nav_msgs::Odometry new_odom = getLastOdom();

// check if the robot speed is now 4.0m.s-1
  EXPECT_NEAR(new_odom.twist.twist.linear.y, 3.8, 0.20);
  EXPECT_LT(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), POSITION_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), 0.2);

  EXPECT_NEAR(new_base_link_twist_.linear.y, 3.8, 0.50);
  EXPECT_LT(fabs(new_base_link_pose_.position.x - old_base_link_pose_.position.x), 0.2);
  EXPECT_LT(fabs(new_base_link_twist_.angular.z - old_base_link_twist_.angular.z), 0.2);

  cmd_vel.linear.y = 0.0;
  cmd_chassis.accel.linear.y = 0.0;
  publish(cmd_chassis, cmd_vel);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "limits_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}