//
// Created by qiayuan on 2/14/21.
//

#include "test_common.h"

// TEST CASES
TEST_F(StandardChassisTest, testForward) {
  // wait for ROS
  waitForController();

  // zero everything before test
  this->zeroCmdVel();
  this->zeroCmdChassis();
  publish();
  ros::Duration(0.5).sleep();

  // send a velocity command of 0.5 m/s
  this->cmd_chassis_.accel.linear.x = 8;
  this->cmd_vel_.linear.x = 0.5;
  publish();
  ros::Duration(2).sleep();
  EXPECT_NEAR(this->cmd_vel_.linear.x, getTwist().linear.x, VELOCITY_TOLERANCE);
  EXPECT_NEAR(this->cmd_vel_.linear.x, getLastOdom().twist.twist.linear.x, VELOCITY_TOLERANCE);

  this->cmd_vel_.linear.x = 0.;
  this->cmd_vel_.linear.y = 0.5;
  this->cmd_chassis_.accel.linear.y = 8;
  publish();
  ros::Duration(2).sleep();
  EXPECT_NEAR(this->cmd_vel_.linear.y, getTwist().linear.y, VELOCITY_TOLERANCE);
  EXPECT_NEAR(this->cmd_vel_.linear.y, getLastOdom().twist.twist.linear.y, VELOCITY_TOLERANCE);

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
