
#include "test_common.h"

TEST_F(StandardChassisTest, testAngularZDirectionAccelerationLimits) {
// wait for ROS
  while (!isControllerAlive()) {
    ros::Duration(0.1).sleep();
  }
// zero everything before test
  geometry_msgs::Twist cmd_vel{};
  rm_msgs::ChassisCmd cmd_chassis{};
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 0;
  cmd_chassis.accel.linear.y = 0;
  cmd_chassis.accel.angular.z = 0;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(10.0).sleep();
// get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
// send a big command
  cmd_vel.linear.x = 0.2;
  cmd_chassis.accel.linear.x = 20.0;
  publish(cmd_chassis, cmd_vel);
// wait for a while
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

// check if the robot speed is now 1.0 m.s-1, the limit
  EXPECT_NEAR(cmd_vel.linear.x,
              fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x),
              VELOCITY_TOLERANCE + 0.03);

  cmd_vel.linear.x = 0.0;
  cmd_chassis.accel.linear.x = 20.0;
  publish(cmd_chassis, cmd_vel);
  ros::Duration(15.0).sleep();

  old_odom = getLastOdom();

  cmd_vel.linear.x = 0.2;
  cmd_chassis.accel.linear.x = 0.5;
  publish(cmd_chassis, cmd_vel);
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