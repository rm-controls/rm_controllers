//
// Created by huakang on 2021/1/18.
//

#ifndef SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
#define SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_

#include "rm_shooter_controller/shooter_base.h"

namespace rm_shooter_controllers {

class StandardController : public rm_shooter_base::ShooterBase {
 public:
  StandardController() = default;
  using rm_shooter_base::ShooterBase::update;
  using rm_shooter_base::ShooterBase::passive;
  using rm_shooter_base::ShooterBase::ready;
  using rm_shooter_base::ShooterBase::block;
  using rm_shooter_base::ShooterBase::commandCB;
  using rm_shooter_base::ShooterBase::reconfigCB;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
 protected:
  void push(const ros::Time &time, const ros::Duration &period) override;
  void stop(const ros::Time &time, const ros::Duration &period) override;
  void moveJointFriction(const ros::Duration &period);
  void moveJoint(const ros::Duration &period) override;
  hardware_interface::JointHandle joint_friction_l_{}, joint_friction_r_{}, joint_trigger_{};
  control_toolbox::Pid pid_friction_l_{}, pid_friction_r_{}, pid_trigger_{};
};

} // namespace rm_shooter_controllers
#endif //SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
