#include "test_common.h"

TEST_F(StandardChassisTest, testWrongJointName) {
  // the controller should never be alive
  int secs = 0;
  while (!isControllerAlive() && ros::ok() && secs < 5) {
    ros::Duration(1.0).sleep();
    secs++;
  }
  if (!ros::ok())
    FAIL() << "Something went wrong while executing test.";

  // give up and assume controller load failure after 5 seconds
  EXPECT_LE(secs, 5);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "wrong_joint_name_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}