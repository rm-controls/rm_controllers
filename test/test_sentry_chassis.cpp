

#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testForward) {
  // wait for ROS
  waitForController();

  // zero everything before test
  rm_msgs::ChassisCmd chassis_cmd{};
  geometry_msgs::Twist cmd_vel{};
  publish(chassis_cmd, cmd_vel);
  ros::Duration(5.0).sleep();

  // send a velocity command of 0.25 m/s
  chassis_cmd.mode = chassis_cmd.RAW;
  chassis_cmd.effort_limit = 99;
  chassis_cmd.accel.linear.x = 20;

  cmd_vel.linear.x = 0.20;
  publish(chassis_cmd, cmd_vel);
  ros::Duration(10.0).sleep();
  EXPECT_NEAR(cmd_vel.linear.x, getLastOdom().twist.twist.linear.x, VELOCITY_TOLERANCE);

  cmd_vel.linear.x = 0.;
  publish(chassis_cmd, cmd_vel);
  ros::Duration(5.0).sleep();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_sentry_chassis");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
