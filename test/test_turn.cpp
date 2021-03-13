
#include "test_common.h"

TEST_F(StandardChassisTest, testTurn180) {
// wait for ROS
  waitForController();

// zero everything before test
  geometry_msgs::Twist cmd_vel;
  rm_msgs::ChassisCmd cmd_chassis{};

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 0;
  cmd_chassis.accel.angular.z = 0;

  publish(cmd_chassis, cmd_vel);
  ros::Duration(10.0).sleep();

// send a velocity command
  cmd_vel.angular.z = M_PI / 10.0;
  cmd_chassis.accel.angular.z = 10;
  publish(cmd_chassis, cmd_vel);
// wait for 2s,waiting for the car uniform motion
  ros::Duration(5.0).sleep();

// get initial odom,then wait 3s
  geometry_msgs::Pose old_base_link_pose_ = getPose();
  nav_msgs::Odometry old_odom = getLastOdom();
  ros::Duration(3.0).sleep();
  geometry_msgs::Pose new_base_link_pose_ = getPose();
  geometry_msgs::Twist new_base_link_twist_ = getTwist();
  nav_msgs::Odometry new_odom = getLastOdom();



// check if the robot rotated PI around z, changes in the other fields should be ~~0
  EXPECT_LT(fabs(new_base_link_pose_.position.x - old_base_link_pose_.position.x), EPS);
  EXPECT_LT(fabs(new_base_link_pose_.position.y - old_base_link_pose_.position.y), EPS);

  const ros::Duration actual_elapsed_time = new_odom.header.stamp - old_odom.header.stamp;
  const double expected_rotation = cmd_vel.angular.z * actual_elapsed_time.toSec();

  EXPECT_LT(fabs(new_base_link_twist_.linear.x), EPS);
  EXPECT_LT(fabs(new_base_link_twist_.linear.y), EPS);
  EXPECT_NEAR(fabs(new_base_link_twist_.angular.z), expected_rotation / actual_elapsed_time.toSec(), EPS);

  cmd_vel.angular.z = 0;
  cmd_chassis.accel.angular.z = 0;
  publish(cmd_chassis, cmd_vel);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "turn_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}