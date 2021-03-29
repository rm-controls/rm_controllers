//
// Created by huakang on 2021/3/29.
//

#ifndef RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_HERO_H_
#define RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_HERO_H_

#include "rm_shooter_controller/shooter_base.h"

namespace rm_shooter_controllers {

class HeroController : public rm_shooter_base::ShooterBase {
 public:
  HeroController() = default;
  using rm_shooter_base::ShooterBase::update;
  using rm_shooter_base::ShooterBase::passive;
  using rm_shooter_base::ShooterBase::ready;
  using rm_shooter_base::ShooterBase::block;
  using rm_shooter_base::ShooterBase::stop;
  using rm_shooter_base::ShooterBase::commandCB;
  using rm_shooter_base::ShooterBase::reconfigCB;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
 protected:
  void push(const ros::Time &time, const ros::Duration &period) override;
  void moveJoint(const ros::Duration &period) override;
  hardware_interface::JointHandle joint_friction_lf_{}, joint_friction_rf_{}, joint_friction_lb_{},
      joint_friction_rb_{}, joint_trigger_{};
  control_toolbox::Pid pid_friction_lf_{}, pid_friction_rf_{}, pid_friction_lb_{}, pid_friction_rb_{}, pid_trigger_{};
};

} // namespace rm_shooter_controllers
#endif //RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_HERO_H_
