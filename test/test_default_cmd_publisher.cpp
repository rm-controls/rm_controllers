#include "test_common.h"

//TEST CASES
TEST_F(StandardChassisTest, testDefaultVelAndChassisCmd) {
  // wait for ROS
  waitForController();
  waitForOdomMsgs();


  // zero everything before test
  this->zeroCmdVel();
  this->zeroCmdChassis();
  this->publish();
  ros::Duration(2.0).sleep();

  this->cmd_vel_.linear.x = 10;
  this->cmd_chassis_.accel.linear.x = 8.0;
  this->publishFromWrongTopic();
  ros::Duration(1.0).sleep();

  EXPECT_LT(abs(abs(0.1) - abs(getTwist().linear.x)), VELOCITY_TOLERANCE);

  this->publish();
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