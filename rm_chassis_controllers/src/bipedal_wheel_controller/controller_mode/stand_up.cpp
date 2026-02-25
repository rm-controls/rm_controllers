//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"

namespace rm_chassis_controllers
{
StandUp::StandUp(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
}

void StandUp::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter STAND_UP");
    controller->setStateChange(true);
    controller->setCompleteStand(false);
    leg_state_threshold_ = controller->getLegThresholdParams();
    StandUp::detectLegState(x_left_, left_leg_state);
    StandUp::detectLegState(x_right_, right_leg_state);
  }

  auto model_params_ = controller->getModelParams();
  spring_force_ = -model_params_->f_spring;
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  setUpLegMotion(x_left_, right_leg_state, left_pos_[0], left_pos_[1], left_leg_state, theta_des_l, length_des_l);
  setUpLegMotion(x_right_, left_leg_state, right_pos_[0], right_pos_[1], right_leg_state, theta_des_r, length_des_r);
  left_cmd = computePidLegCommand(length_des_l, theta_des_l, left_pos_, left_spd_, *pid_legs_[0], *pid_thetas_[0],
                                  *pid_thetas_[2], left_angle_, left_leg_state, period, spring_force_);
  right_cmd = computePidLegCommand(length_des_r, theta_des_r, right_pos_, right_spd_, *pid_legs_[1], *pid_thetas_[1],
                                   *pid_thetas_[3], right_angle_, right_leg_state, period, spring_force_);

  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (((left_pos_[1] < 0.3 && left_leg_state == LegState::BEHIND) ||
       (left_pos_[1] > -0.4 && left_leg_state == LegState::UNDER)) &&
      ((right_pos_[1] < 0.3 && right_leg_state == LegState::BEHIND) ||
       (right_pos_[1] > -0.4 && right_leg_state == LegState::UNDER)))
  {
    controller->setMode(BalanceMode::NORMAL);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit STAND_UP");
  }
}

void StandUp::setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const int& other_leg_state,
                             const double& leg_length, const double& leg_theta, int& leg_state, double& theta_des,
                             double& length_des)
{
  switch (leg_state)
  {
    case LegState::UNDER:
      if (spring_force_ > 0)
      {
        theta_des = M_PI / 2 - 0.35;
        length_des = 0.36;
        if (leg_length > 0.35)
        {
          leg_state = LegState::FRONT;
        }
      }
      else
      {
        theta_des = 0.0;
        length_des = 0.18;
      }
      break;
    case LegState::FRONT:
      theta_des = M_PI_2 - 0.35;
      length_des = 0.36;
      if ((abs(x[0] - theta_des) < 0.3 && abs(x[4]) < 0.3) || (abs(x[1]) < 0.1 && x[0] > M_PI_2))
        leg_state = LegState::BEHIND;
      break;
    case LegState::BEHIND:
      theta_des = leg_theta;
      length_des = leg_length;
      if (other_leg_state != LegState::FRONT)
      {
        length_des = 0.18;
        if (leg_length < 0.3)
          theta_des = 0.0;
      }
      break;
  }
}

inline void StandUp::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, int& leg_state)
{
  if (!leg_state_threshold_)
  {
    ROS_ERROR_THROTTLE(1.0, "LegUtils threshold params not initialized!");
    return;
  }
  if (x[0] > leg_state_threshold_->under_lower && x[0] < leg_state_threshold_->under_upper)
    leg_state = LegState::UNDER;
  else if ((x[0] < leg_state_threshold_->front_lower && x[0] > -M_PI) ||
           (x[0] < M_PI && x[0] > leg_state_threshold_->front_upper))
    leg_state = LegState::FRONT;
  else if (x[0] > leg_state_threshold_->behind_lower && x[0] < leg_state_threshold_->behind_upper)
    leg_state = LegState::BEHIND;
  switch (leg_state)
  {
    case LegState::UNDER:
      ROS_INFO("[balance] x[0]: %.3f Leg state: UNDER", x[0]);
      break;
    case LegState::FRONT:
      ROS_INFO("[balance] x[0]: %.3f Leg state: FRONT", x[0]);
      break;
    case LegState::BEHIND:
      ROS_INFO("[balance] x[0]: %.3f Leg state: BEHIND", x[0]);
      break;
  }
}
}  // namespace rm_chassis_controllers
