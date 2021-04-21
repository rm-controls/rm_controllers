#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testScopperil) {
  // wait for ROS
  waitForController();

  // zero everything before test
  rm_msgs::ChassisCmd chassis_cmd{};
  geometry_msgs::Twist cmd_vel{};
  publish(chassis_cmd, cmd_vel);
  ros::Duration(0.5).sleep();

  chassis_cmd.mode = chassis_cmd.GYRO;
  chassis_cmd.effort_limit = 99;
  chassis_cmd.accel.linear.x = 10;
  chassis_cmd.accel.linear.y = 10;
  chassis_cmd.accel.angular.z = 10;

  cmd_vel.linear.x = 0.5;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0.5;

  publish(chassis_cmd, cmd_vel);
  ros::Duration(10).sleep();

  EXPECT_NEAR(cmd_vel.linear.x, getTwist().linear.x, VELOCITY_TOLERANCE);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_scopperil");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}