//
// Created by qiayuan on 1/16/21.
//

#include "rm_gimbal_controller/standard.h"
#include <ros_utilities.h>

namespace rm_gimbal_controller {
bool GimbalStandardController::init(hardware_interface::RobotHW *robot_hw,
                                    ros::NodeHandle &root_nh,
                                    ros::NodeHandle &controller_nh) {
  std::string joint_yaw_name = getParam(controller_nh, "joint_yaw_name", std::string("joint_yaw"));
  std::string joint_pitch_name = getParam(controller_nh, "joint_pitch_name", std::string("joint_pitch"));

  if (!pid_yaw_.init(ros::NodeHandle(controller_nh, "pid_yaw"))
      || pid_pitch_.init(ros::NodeHandle(controller_nh, "pid_pitch")))
    return false;

  sub_command_ = root_nh.subscribe<rm_msgs::GimbalCmd>("cmd_gimbal", 1, &GimbalStandardController::commandCB, this);
  return true;
}

void GimbalStandardController::update(const ros::Time &time, const ros::Duration &) {

}

void GimbalStandardController::commandCB(const rm_msgs::GimbalCmdConstPtr &msg) {

}

}