
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

  cmd_chassis.mode = cmd_chassis.GYRO;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 0;
  cmd_chassis.accel.linear.y = 0;
  cmd_chassis.accel.angular.z = 0;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(2.0).sleep();
// get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
// send a big command
  cmd_vel.linear.x = 1.0;
  cmd_chassis.accel.linear.x = 10.0;
  publish(cmd_chassis, cmd_vel);
// wait for a while
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

// check if the robot speed is now 1.0rad.s-1, which is 2.0rad.s-2 * 0.5s
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), 2.0 + VELOCITY_TOLERANCE);

  EXPECT_LT(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), POSITION_TOLERANCE);
  EXPECT_LT(fabs(new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), POSITION_TOLERANCE);

  cmd_vel.angular.z = 0.0;
  cmd_chassis.accel.angular.z = 0.0;
  publish(cmd_chassis, cmd_vel);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "angular_z_direction_accel_limit_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}