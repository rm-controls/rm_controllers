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
  this->zeroCmdVel();
  this->zeroCmdChassis();
  this->publish();
  ros::Duration(2.0).sleep();

  this->cmd_vel_.linear.x = 0.5;
  this->cmd_chassis_.accel.linear.x = 8.0;
  publish();
  ros::Duration(2.0).sleep();
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[0], 0.3);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[1], 0.3);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[2], 0.3);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[3], 0.3);

  this->cmd_vel_.linear.x = 0.0;
  this->cmd_vel_.linear.y = 0.5;
  this->cmd_chassis_.accel.linear.y = 8.0;
  publish();
  ros::Duration(2.0).sleep();
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[0], 0.8);
  EXPECT_NEAR(-0.5 / r, getJointStates().velocity[1], 0.5);
  EXPECT_NEAR(-0.5 / r, getJointStates().velocity[2], 0.5);
  EXPECT_NEAR(0.5 / r, getJointStates().velocity[3], 0.5);

  this->cmd_vel_.linear.y = 0.;
  this->cmd_vel_.angular.z = 0.5;
  this->cmd_chassis_.accel.angular.z = 8.0;
  publish();
  ros::Duration(2.0).sleep();
  EXPECT_NEAR(-a * this->cmd_vel_.angular.z / r, getJointStates().velocity[0], 0.7);
  EXPECT_NEAR(-a * this->cmd_vel_.angular.z / r, getJointStates().velocity[1], 0.7);
  EXPECT_NEAR(a * this->cmd_vel_.angular.z / r, getJointStates().velocity[2], 0.7);
  EXPECT_NEAR(a * this->cmd_vel_.angular.z / r, getJointStates().velocity[3], 0.7);

  this->cmd_vel_.angular.z = 0.0;
  this->cmd_chassis_.accel.angular.z = 8.0;
  publish();
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