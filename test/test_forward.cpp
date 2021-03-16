

#include "test_common.h"


// TEST CASES
TEST_F(StandardChassisTest, testForward) {
  // wait for ROS
  waitForController();

  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  rm_msgs::ChassisCmd cmd_chassis{};

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 0;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(5.0).sleep();


  // send a velocity command of 1.0 m/s
  cmd_vel.linear.x = 1;
  cmd_chassis.accel.linear.x = 10;
  publish(cmd_chassis, cmd_vel);
  // wait for 1s,waiting for the car uniform motion
  ros::Duration(2.0).sleep();

  // get initial odom,then wait 10s
  geometry_msgs::Pose old_base_link_pose_ = getPose();
  nav_msgs::Odometry old_odom = getLastOdom();
  ros::Duration(3.0).sleep();
  geometry_msgs::Pose new_base_link_pose_ = getPose();
  geometry_msgs::Twist new_base_link_twist_ = getTwist();
  nav_msgs::Odometry new_odom = getLastOdom();

  const ros::Duration actual_elapsed_time = new_odom.header.stamp - old_odom.header.stamp;

  const double expected_distance = cmd_vel.linear.x * actual_elapsed_time.toSec();

  // check if the robot traveled 1 meter in XY plane, changes in z should be ~~0
  const double dx = new_base_link_pose_.position.x - old_base_link_pose_.position.x;
  const double dy = new_base_link_pose_.position.y - old_base_link_pose_.position.y;
  EXPECT_NEAR(sqrt(dx * dx + dy * dy), expected_distance, 0.3);

  EXPECT_NEAR(fabs(new_base_link_twist_.linear.x), cmd_vel.linear.x, 0.08);
  EXPECT_LT(fabs(new_base_link_twist_.linear.y), VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_base_link_twist_.angular.z), VELOCITY_TOLERANCE);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "forward_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}