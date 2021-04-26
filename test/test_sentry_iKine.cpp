#include "test_common.h"

//TEST CASES
TEST_F(StandardChassisTest, testIKine) {
  // wait for ROS
  waitForController();
  waitForOdomMsgs();

  double r = 0.025;

  // zero everything before test
  geometry_msgs::Twist cmd_vel{};
  rm_msgs::ChassisCmd cmd_chassis{};
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 20;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(5.0).sleep();

  cmd_vel.linear.x = 0.25;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(10.0).sleep();
  EXPECT_NEAR(cmd_vel.linear.x, getJointStates().velocity[0] * r, 0.06);

  cmd_vel.linear.x = 0.;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(5.0).sleep();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "iKine_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}