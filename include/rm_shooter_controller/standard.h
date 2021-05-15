//
// Created by huakang on 2021/1/18.
//

#ifndef SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
#define SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_

#include "rm_shooter_controller/shooter_base.h"

namespace rm_shooter_controllers {

class StandardController : public ShooterBase {
 public:
  StandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
 protected:
  void push(const ros::Time &time, const ros::Duration &period) override;
  void stop(const ros::Time &time, const ros::Duration &period) override;
  void moveJointFriction(const ros::Time &time, const ros::Duration &period);
  void moveJoint(const ros::Time &time, const ros::Duration &period) override;
  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
  effort_controllers::JointPositionController ctrl_trigger_;
};

} // namespace rm_shooter_controllers
#endif //SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
