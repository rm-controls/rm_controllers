//
// Created by ljq on 2022/5/15.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <effort_controllers/joint_position_controller.h>

#include <pluginlib/class_list_macros.hpp>

namespace mimic_joint_controller
{
class MimicJointController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                          hardware_interface::JointStateInterface>
{
public:
  MimicJointController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  effort_controllers::JointPositionController mimic_joint_ctrl_;
  hardware_interface::JointStateHandle target_state_handle_;
  std::string target_joint_name_;
};

}  // namespace mimic_joint_controller

PLUGINLIB_EXPORT_CLASS(mimic_joint_controller::MimicJointController, controller_interface::ControllerBase)
