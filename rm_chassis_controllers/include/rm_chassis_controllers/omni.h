//
// Created by yezi on 2021/12/3.
//

#pragma once

#include <rm_chassis_controllers/chassis_base.h>

namespace rm_chassis_controllers
{
class OmniController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>
{
public:
  OmniController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  geometry_msgs::Twist forwardKinematics() override;

  double chassis_radius_;
  effort_controllers::JointVelocityController ctrl_lf_, ctrl_rf_, ctrl_lb_, ctrl_rb_;
};
}  // namespace rm_chassis_controllers
