//
// Created by qiayuan on 4/23/21.
//

#ifndef RM_CHASSIS_CONTROLLERS_SWERVE_H_
#define RM_CHASSIS_CONTROLLERS_SWERVE_H_

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers {
struct Module {
  hardware_interface::JointHandle pivot_, wheel_;
};

class SwerveKinematics {
 public:
  SwerveKinematics() = default;
  void addModule();
  void inverseKinematics(const geometry_msgs::Twist &twist);
  geometry_msgs::Twist forwardKinematics();
 private:
//  std::vector<>
};

class SwerveController : public ChassisBase {
 public:
  SwerveController() = default;
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

 private:
  void moveJoint(const ros::Duration &period) override;
  geometry_msgs::Twist forwardKinematics() override;

  control_toolbox::Pid pid_rf_, pid_lf_, pid_rb_, pid_lb_;
  hardware_interface::JointHandle joint_rf_, joint_lf_, joint_rb_, joint_lb_;
};

} // namespace rm_chassis_controllers

#endif //RM_CHASSIS_CONTROLLERS_INCLUDE_RM_CHASSIS_CONTROLLERS_SWERVE_H_
