//
// Created by qiayuan on 2/14/21.
//

#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testForward) {
  // wait for ROS
  waitForController();

  // zero everything before test
  rm_msgs::ChassisCmd chassis_cmd{};
  geometry_msgs::Twist cmd_vel{};
  publish(chassis_cmd, cmd_vel);
  ros::Duration(0.5).sleep();

  // send a velocity command of 0.5 m/s
  chassis_cmd.mode = chassis_cmd.RAW;
  chassis_cmd.effort_limit = 99;
  chassis_cmd.accel.linear.x = 8;
  chassis_cmd.accel.linear.y = 8;

  cmd_vel.linear.x = 0.5;
  publish(chassis_cmd, cmd_vel);
  ros::Duration(2).sleep();
  EXPECT_NEAR(cmd_vel.linear.x, getTwist().linear.x, VELOCITY_TOLERANCE);

  cmd_vel.linear.x = 0.;
  cmd_vel.linear.y = 0.5;
  publish(chassis_cmd, cmd_vel);
  ros::Duration(2).sleep();
  EXPECT_NEAR(cmd_vel.linear.y, getTwist().linear.y, VELOCITY_TOLERANCE);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dummy");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
