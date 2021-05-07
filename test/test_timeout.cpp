#include "test_common.h"

// TEST CASES
//This send a x direction vel and accel,then stop quickly,after that,see whether the controller can stop in time.
TEST_F(StandardChassisTest, testTimeout) {
  // wait for ROS
  waitForController();

  // zero everything before test
  this->zeroCmdVel();
  this->zeroCmdChassis();
  publish();

  // give some time to the controller to react to the command
  ros::Duration(1.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();

  // send a velocity command of 1 m/s
  this->cmd_vel_.linear.x = 1.0;
  this->cmd_chassis_.accel.linear.x = 8.0;
  publish();
  // wait a bit
  ros::Duration(5).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  EXPECT_LT(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), POSITION_TOLERANCE);
  EXPECT_LT(new_odom.twist.twist.linear.x, VELOCITY_TOLERANCE);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "timeout_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
