//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include "rm_common/filters/filters.h"

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
class Normal : public ModeBase
{
public:
  Normal(const std::vector<hardware_interface::JointHandle*>& joint_handles,
         const std::vector<control_toolbox::Pid*>& pid_legs, control_toolbox::Pid* pid_yaw_vel,
         control_toolbox::Pid* pid_theta_diff, control_toolbox::Pid* pid_roll);
  void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "NORMAL";
  }

private:
  double calculateSupportForce(double F, double Tp, double leg_length, const double& leg_len_spd, double acc_z,
                               Eigen::Matrix<double, STATE_DIM, 1> x, const std::shared_ptr<ModelParams>& model_params,
                               const ros::Duration& period);
  bool unstickDetection(const double& F_leg, const double& Tp, const double& leg_len_spd, const double& leg_length,
                        const double& acc_z, const std::shared_ptr<ModelParams>& model_params,
                        Eigen::Matrix<double, STATE_DIM, 1> x,
                        std::shared_ptr<MovingAverageFilter<double>> supportForceAveragePtr,
                        const ros::Duration& period);
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_;
  control_toolbox::Pid *pid_yaw_vel_, *pid_theta_diff_, *pid_roll_;

  ros::Time lastJumpTime_{};

  double pos_des_{ 0.0 }, leg_length_des{ 0.2 };
  int jump_phase_ = JumpPhase::IDLE, jumpTime_{ 0 };
  std::shared_ptr<MovingAverageFilter<double>> leftSupportForceAveragePtr_, rightSupportForceAveragePtr_;
};
}  // namespace rm_chassis_controllers
