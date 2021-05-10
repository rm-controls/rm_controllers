

#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testForward) {
  // wait for ROS
  waitForController();

  // zero everything before test
  this->zeroCmdVel();
  this->zeroCmdChassis();
  this->publish();
  ros::Duration(5.0).sleep();

  // send a velocity command of 0.25 m/s
  this->cmd_chassis_.accel.linear.x = 20;
  this->cmd_vel_.linear.x = 0.016;
  publish();
  ros::Duration(10.0).sleep();
  EXPECT_NEAR(this->cmd_vel_.linear.x, getLastOdom().twist.twist.linear.x, VELOCITY_TOLERANCE);

  this->cmd_vel_.linear.x = 0.;
  publish();
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
