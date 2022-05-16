//
// Created by ljq on 2022/5/15.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <effort_controllers/joint_position_controller.h>

#include <pluginlib/class_list_macros.hpp>

namespace joint_mime_controller
{
class JointMimeController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                          hardware_interface::JointStateInterface>
{
public:
  JointMimeController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);

private:
  effort_controllers::JointPositionController joint_mime_ctrl_;
  hardware_interface::JointStateHandle joint_state_handle_;
  std::string joint_mime;
};

}  // namespace joint_mime_controller

PLUGINLIB_EXPORT_CLASS(joint_mime_controller::JointMimeController, controller_interface::ControllerBase)
