
#include "test_common.h"

TEST_F(StandardChassisTest, testAngularZDirectionAccelerationLimits) {
// wait for ROS
  while (!isControllerAlive()) {
    ros::Duration(0.1).sleep();
  }
// zero everything before test
  this->zeroCmdVel();
  this->zeroCmdChassis();
  this->publish();
  ros::Duration(10.0).sleep();

// get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
// send a big command
  this->cmd_vel_.linear.x = 0.016;
  this->cmd_chassis_.accel.linear.x = 20.0;
  publish();
// wait for a while
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

// check if the robot speed is now 1.0 m.s-1, the limit
  EXPECT_NEAR(this->cmd_vel_.linear.x,
              fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x),
              VELOCITY_TOLERANCE + 0.03);

  this->cmd_vel_.linear.x = 0.0;
  this->cmd_chassis_.accel.linear.x = 20.0;
  publish();
  ros::Duration(15.0).sleep();

  old_odom = getLastOdom();

  this->cmd_vel_.linear.x = 0.2;
  this->cmd_chassis_.accel.linear.x = 0.5;
  publish();
  ros::Duration(10.0).sleep();

  new_odom = getLastOdom();

  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), 1.0);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "sentry_linear_x__accel_limit_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}