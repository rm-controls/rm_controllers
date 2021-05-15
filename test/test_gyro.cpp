#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testScopperil) {
  // wait for ROS
  waitForController();

  // zero everything before test
  this->zeroCmdVel();
  this->zeroCmdChassis();
  this->cmd_chassis_.mode = cmd_chassis_.PASSIVE;
  this->publish();
  ros::Duration(0.5).sleep();

  this->cmd_chassis_.mode = cmd_chassis_.GYRO;
  this->cmd_chassis_.accel.linear.x = 10;
  this->cmd_chassis_.accel.linear.y = 10;
  this->cmd_chassis_.accel.angular.z = 10;

  this->cmd_vel_.linear.x = 0.5;
  //cmd_vel.linear.y = 0;
  this->cmd_vel_.angular.z = 0.5;

  this->publish();
  ros::Duration(10).sleep();

  EXPECT_NEAR(this->cmd_vel_.linear.x, getTwist().linear.x, VELOCITY_TOLERANCE);

  this->zeroCmdVel();
  this->zeroCmdChassis();
  this->publish();
  ros::Duration(2.0).sleep();

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