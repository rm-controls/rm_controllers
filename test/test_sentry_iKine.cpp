#include "test_common.h"

//TEST CASES
TEST_F(StandardChassisTest, testIKine) {
  // wait for ROS
  waitForController();
  waitForOdomMsgs();

  double r = 0.025;

  // zero everything before test
  this->zeroCmdVel();
  this->zeroCmdChassis();
  this->publish();
  ros::Duration(10.0).sleep();

  this->cmd_chassis_.accel.linear.x = 20;
  this->cmd_vel_.linear.x = 0.016;
  publish();
  ros::Duration(10.0).sleep();
  EXPECT_NEAR(this->cmd_vel_.linear.x, getJointStates().velocity[0] * r, 0.06);

  this->cmd_vel_.linear.x = 0.;
  publish();
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