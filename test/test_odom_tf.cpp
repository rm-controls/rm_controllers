#include "test_common.h"
#include <tf2_ros/transform_listener.h>


// TEST CASES
TEST_F(StandardChassisTest, testOdomTF) {
  // wait for ROS
  waitForController();

  // set up tf listener
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  ros::Duration(2.0).sleep();
  // check the original odom frame doesn't exist
  EXPECT_FALSE(buffer._frameExists("odom"));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "odom_tf_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}