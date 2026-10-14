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
class Recover : public ModeBase
{
  typedef enum
  {
    ForwardSlip,
    BackwardSlip,
  } RecoveryChassisState;

  typedef enum
  {
    INITIALIZED,
    START,
    MOVING,
    MOVING_TOGETHER,
    STOP,
  } LegRecoveryCalibratedState;

  typedef enum
  {
    NotReady,
    Ready
  } LegRecoveryState;

public:
  Recover(BipedalControllerInterface* controller_, const std::vector<hardware_interface::JointHandle*>& joint_handles,
          const std::vector<control_toolbox::Pid*>& pid_legs, const std::vector<control_toolbox::Pid*>& pid_thetas,
          control_toolbox::Pid* pid_theta_diff);
  void execute(const ros::Time& time, const ros::Duration& period) override;
  inline void detectChassisStateToRecover();
  inline void detectLegRecoveryState(LegRecoveryState& recovery_state, const double& leg_pos);
  const char* name() const override
  {
    return "RECOVER";
  }

private:
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_, pid_thetas_;
  control_toolbox::Pid* pid_theta_diff_;
  double leg_recovery_velocity_{ 5.0 }, threshold_{ 0.05 }, leg_theta_diff_{ 0.0 }, desired_leg_length_{ 0.36 };
  double left_leg_recovery_feed_forward{ 10.0 }, right_leg_recovery_feed_forward{ 10.0 };
  const double leg_recovery_velocity_const_{ 5.0 };
  LegRecoveryState left_recovery_leg, right_recovery_leg;
  RecoveryChassisState recovery_chassis_state_{ ForwardSlip };
  bool detectd_flag{ false };
  ChassisState chassis_state_;
};
}  // namespace rm_chassis_controllers
