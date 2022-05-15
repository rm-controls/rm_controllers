//
// Created by ljq on 2022/5/15.
//

#pragma once

#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/tf_rt_broadcaster.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace joint_mime_controller
{
class JointMimeController
{
public:
  JointMimeController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh);
  void upddate(const ros::Time& time);

private:
};

}  // namespace joint_mime_controller

PLUGINLIB_EXPORT_CLASS(joint_mime_controller::JointMimeController, controller_interface::ControllerBase)
