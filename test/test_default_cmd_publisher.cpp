#include "test_common.h"

//TEST CASES
TEST_F(StandardChassisTest, testDefaultVelAndChassisCmd) {
  // wait for ROS
  waitForController();
  waitForOdomMsgs();


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
  ros::Duration(2.0).sleep();

  cmd_vel.linear.x = 10;
  cmd_chassis.accel.linear.x = 8.0;
  publishFromWrongTopic(cmd_chassis, cmd_vel);
  ros::Duration(1.0).sleep();

  EXPECT_NEAR(0.0, getTwist().linear.x, VELOCITY_TOLERANCE);

  publish(cmd_chassis, cmd_vel);
  ros::Duration(1.0).sleep();
  EXPECT_GE(getTwist().linear.x, 0);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "default_cmd_vel_out_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}