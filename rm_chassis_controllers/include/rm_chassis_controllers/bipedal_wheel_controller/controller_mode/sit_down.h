//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
class SitDown : public ModeBase
{
public:
  explicit SitDown(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                   const std::vector<control_toolbox::Pid*>& pid_wheels);
  void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "SIT_DOWN";
  }

private:
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_wheels_;
};
}  // namespace rm_chassis_controllers
