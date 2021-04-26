//
// Created by qiayuan on 4/23/21.
//

#ifndef RM_CHASSIS_CONTROLLERS_SWERVE_H_
#define RM_CHASSIS_CONTROLLERS_SWERVE_H_

#include "rm_chassis_controllers/chassis_base.h"

#include <rm_common/eigen_types.h>

namespace rm_chassis_controllers {

struct Module {
  Vec2<double> position_;
  double pivot_offset_, wheel_radius_;
  hardware_interface::JointHandle joint_pivot_, joint_wheel_;
  control_toolbox::Pid *pid_pivot_{}, *pid_wheel_{};
};

class SwerveController : public ChassisBase {
 public:
  SwerveController() = default;
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

 private:
  void moveJoint(const ros::Duration &period) override;
  geometry_msgs::Twist forwardKinematics() override;
  std::vector<Module> modules_;
};

} // namespace rm_chassis_controllers

#endif //RM_CHASSIS_CONTROLLERS_INCLUDE_RM_CHASSIS_CONTROLLERS_SWERVE_H_
