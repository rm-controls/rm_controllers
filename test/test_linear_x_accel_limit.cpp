
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
  ros::Duration(2.5).sleep();

// get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  geometry_msgs::Pose old_base_link_pose = getPose();    //  from Gazebo
  geometry_msgs::Twist old_base_link_twist = getTwist(); //  from Gazebo
// send a big command
  this->cmd_vel_.linear.x = 1.0;
  this->cmd_chassis_.accel.linear.x = 10.0;
  this->publish();
  ros::Duration(1.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();
  geometry_msgs::Pose new_base_link_pose = getPose();    //  from Gazebo
  geometry_msgs::Twist new_base_link_twist = getTwist(); //  from Gazebo

// check if the robot speed is now 1.0 m.s-1, the limit
  EXPECT_NEAR(this->cmd_vel_.linear.x,
              fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x),
              VELOCITY_TOLERANCE + 0.03);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), POSITION_TOLERANCE);

  EXPECT_NEAR(this->cmd_vel_.linear.x,
              fabs(new_base_link_twist.linear.x - old_base_link_twist.linear.x),
              VELOCITY_TOLERANCE + 0.03);
  EXPECT_LT(fabs(new_base_link_twist.angular.z - old_base_link_twist.angular.z), VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_base_link_pose.position.y - old_base_link_pose.position.y), POSITION_TOLERANCE);

  this->cmd_vel_.linear.x = 0.0;
  this->cmd_chassis_.accel.linear.x = 10.0;
  this->publish();
  ros::Duration(5.0).sleep();

  old_odom = getLastOdom();
  old_base_link_pose = getPose();    //  from Gazebo
  old_base_link_twist = getTwist(); //  from Gazebo

  this->cmd_vel_.linear.x = 1.0;
  this->cmd_chassis_.accel.linear.x = 0.5;
  this->publish();
  ros::Duration(1.0).sleep();

  new_odom = getLastOdom();
  new_base_link_pose = getPose();    //  from Gazebo
  new_base_link_twist = getTwist(); //  from Gazebo

  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), 1.0);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), POSITION_TOLERANCE);

  EXPECT_LT(fabs(new_base_link_twist.linear.x - old_base_link_twist.linear.x), 1.0);
  EXPECT_LT(fabs(new_base_link_twist.angular.z - old_base_link_twist.angular.z), VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_base_link_pose.position.y - old_base_link_pose.position.y), POSITION_TOLERANCE);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "angular_x_direction_accel_limit_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}