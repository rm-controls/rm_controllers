//
// Created by ljq on 2022/5/15.
//

#include "joint_mime_controller/joint_mime_controller.h"

#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>
namespace joint_mime_controller
{
bool JointMimeController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh){

};

void JointMimeController::update(const ros::Time& time){};

}  // namespace joint_mime_controller
