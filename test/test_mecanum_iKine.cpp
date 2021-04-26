#include "test_common.h"

//TEST CASES
TEST_F(StandardChassisTest, testIKine) {
  // wait for ROS
  waitForController();
  waitForOdomMsgs();

  double r = 0.07625;
  double wheel_base_and_wheel_track = 0.395 + 0.40;
  double a = wheel_base_and_wheel_track / 2;


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

  cmd_vel.linear.x = 0.5;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[0], 0.3);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[1], 0.3);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[2], 0.3);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[3], 0.3);

  cmd_vel.linear.x = 0.;
  cmd_vel.linear.y = 0.5;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[0], 0.8);
  EXPECT_NEAR(-0.5 / r, getJointStates().velocity[1], 0.5);
  EXPECT_NEAR(-0.5 / r, getJointStates().velocity[2], 0.5);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[3], 0.5);

  cmd_vel.linear.y = 0.;
  cmd_vel.angular.z = 0.5;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();
  EXPECT_NEAR(-a * cmd_vel.angular.z / r, getJointStates().velocity[0], 0.7);
  EXPECT_NEAR(-a * cmd_vel.angular.z / r, getJointStates().velocity[1], 0.7);
  EXPECT_NEAR(a * cmd_vel.angular.z / r, getJointStates().velocity[2], 0.7);
  EXPECT_NEAR(a * cmd_vel.angular.z / r, getJointStates().velocity[3], 0.7);

  cmd_vel.angular.z = 0.0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();

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