
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
  ros::Duration(8.0).sleep();
// get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  geometry_msgs::Pose old_base_link_pose_ = getPose();  //  from Gazebo
  geometry_msgs::Twist old_base_link_twist_ = getTwist(); //  from Gazebo
// send a big command
  cmd_vel.linear.y = 1.0;
  cmd_chassis.accel.linear.y = 10.0;
  publish(cmd_chassis, cmd_vel);
// wait for a while
  ros::Duration(3.0).sleep();

  geometry_msgs::Pose new_base_link_pose_ = getPose();  //  from Gazebo
  geometry_msgs::Twist new_base_link_twist_ = getTwist(); //  from Gazebo
  nav_msgs::Odometry new_odom = getLastOdom();


// check if the robot speed is now 1.0 m.s-1, the limit
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y - old_odom.twist.twist.linear.y), 1.0 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), 0.03);
  EXPECT_LT(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), POSITION_TOLERANCE);

  EXPECT_LT(fabs(new_base_link_twist_.linear.y - old_base_link_twist_.linear.y), 1.0 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_base_link_twist_.angular.z - old_base_link_twist_.angular.z), 0.03);
  EXPECT_LT(fabs(new_base_link_pose_.position.x - old_base_link_pose_.position.x), 0.04);

  cmd_vel.angular.z = 0.0;
  cmd_chassis.accel.angular.z = 0.0;
  publish(cmd_chassis, cmd_vel);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "linear_y_direction_accel_limit_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}