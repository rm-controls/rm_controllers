#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testTimeout) {
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
  cmd_chassis.accel.linear.y = 0;

  publish(cmd_chassis, cmd_vel);
  // give some time to the controller to react to the command
  ros::Duration(5.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();

  // send a velocity command of 1 m/s
  cmd_vel.linear.x = 1.0;
  cmd_chassis.accel.linear.x = 1.0;
  publish(cmd_chassis, cmd_vel);
  // wait a bit
  ros::Duration(0.5).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();


  // check if the robot has stopped after 0.5s, thus covering less than 0.5s*1.0m.s-1 + some (big) tolerance
  EXPECT_LT(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), 0.8);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), 0.8);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "timeout_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
