//
// Created by flying on 2021/1/18.
//

#ifndef RM_CHASSIS_CONTROLLER_STANDARD_H
#define RM_CHASSIS_CONTROLLER_STANDARD_H

#include <rm_chassis_controllers/chassis_base.h>

namespace rm_chassis_controllers {

class StandardController : public ChassisBase {
 public:
  StandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

 private:
  void moveJoint(const ros::Duration &period) override;
  geometry_msgs::Twist forwardKinematics() override;

  control_toolbox::Pid pid_rf_, pid_lf_, pid_rb_, pid_lb_;
  control_toolbox::Pid pid_follow_;
  hardware_interface::JointHandle joint_rf_, joint_lf_, joint_rb_, joint_lb_;
};

} // namespace rm_chassis_controllers

#endif // RM_CHASSIS_CONTROLLER_STANDARD_H
