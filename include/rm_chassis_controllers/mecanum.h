//
// Created by flying on 2021/1/18.
//

#ifndef RM_CHASSIS_CONTROLLER_MECANUM_CONTROLLER_H
#define RM_CHASSIS_CONTROLLER_MECANUM_CONTROLLER_H

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers {

class MecanumController : public ChassisBase {
 public:
  MecanumController() = default;
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

 private:
  void moveJoint(const ros::Duration &period) override;
  geometry_msgs::Twist forwardKinematics() override;

  effort_controllers::JointVelocityController ctrl_lf_, ctrl_rf_, ctrl_lb_, ctrl_rb_;
};

} // namespace rm_chassis_controllers

#endif // RM_CHASSIS_CONTROLLER_MECANUM_CONTROLLER_H
