

#include "test_common.h"
#include <tf2_ros/transform_listener.h>


// TEST CASES
TEST_F(StandardChassisTest, testOdomFrame) {
  // wait for ROS
  waitForController();

  // set up tf listener
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  ros::Duration(2.0).sleep();
  // check the original odom frame doesn't exist
  EXPECT_TRUE(buffer._frameExists("odom"));
}

TEST_F(StandardChassisTest, testOdomTopic) {
  // wait for ROS
  waitForController();
  waitForOdomMsgs();

  // get an odom message
  nav_msgs::Odometry odom_msg = getLastOdom();
  // check its frame_id
  ASSERT_STREQ(odom_msg.header.frame_id.c_str(), "odom");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "default_odom_frame_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}