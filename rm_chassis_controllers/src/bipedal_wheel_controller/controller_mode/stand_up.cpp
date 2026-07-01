//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"

namespace rm_chassis_controllers
{
StandUp::StandUp(BipedalControllerInterface* controller_,
                 const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas)
  : ModeBase(controller_), joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
  double leg_len_acc = 50, leg_theta_acc = 7.5;
  ramp_length_des_l_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_length_des_r_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_angle_des_l_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
  ramp_angle_des_r_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
}

void StandUp::execute(const ros::Time& time, const ros::Duration& period)
{
  auto& left_leg_state = controller->getLegState(LEFT);
  auto& right_leg_state = controller->getLegState(RIGHT);
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter STAND_UP");
    controller->setStateChange(true);
    controller->setCompleteStand(false);
    leg_state_threshold_ = controller->getLegThresholdParams();
    //    vmcPtr_ = controller->getVMCPtr();
    left_arrive_flag_ = right_arrive_flag_ = false;
    StandUp::detectLegState(left_leg_state.x, left_leg_orientation);
    StandUp::detectLegState(right_leg_state.x, right_leg_orientation);
  }

  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();

  auto model_params_ = controller->getModelParams();
  double left_spring_force = -controller->f_spring_force(left_pos.L0),
         right_spring_force = -controller->f_spring_force(right_pos.L0);
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  setUpLegMotion(left_leg_state.x, right_leg_orientation, left_pos.L0, left_pos.theta, left_leg_orientation,
                 left_leg_command_, left_stop_, left_arrive_flag_, left_arrive_time_);
  setUpLegMotion(right_leg_state.x, left_leg_orientation, right_pos.L0, right_pos.theta, right_leg_orientation,
                 right_leg_command_, right_stop_, right_arrive_flag_, right_arrive_time_);

  ramp_length_des_l_->input(left_leg_command_.desired_length);
  ramp_angle_des_l_->input(left_leg_command_.desired_angle);
  ramp_length_des_r_->input(right_leg_command_.desired_length);
  ramp_angle_des_r_->input(right_leg_command_.desired_angle);
  left_leg_command_.desired_length = ramp_length_des_l_->output();
  left_leg_command_.desired_angle = ramp_angle_des_l_->output();
  right_leg_command_.desired_length = ramp_length_des_r_->output();
  right_leg_command_.desired_angle = ramp_angle_des_r_->output();

  if (!left_stop_)
  {
    left_cmd = computePidLegCommand(left_leg_command_, left_leg_state.vmc, *pid_legs_[0], *pid_thetas_[0],
                                    *pid_thetas_[2], left_leg_orientation, period, left_spring_force);
  }
  if (!right_stop_)
  {
    right_cmd = computePidLegCommand(right_leg_command_, right_leg_state.vmc, *pid_legs_[1], *pid_thetas_[1],
                                     *pid_thetas_[3], right_leg_orientation, period, right_spring_force);
  }

  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if ((((abs(left_pos.theta) < 0.3f && left_leg_orientation == LegOrientation::BEHIND)) &&
       ((abs(right_pos.theta) < 0.3f && right_leg_orientation == LegOrientation::BEHIND))) ||
      ((abs(left_pos.theta) < 0.3f && left_leg_orientation == LegOrientation::UNDER) &&
       (abs(right_pos.theta) < 0.3f && right_leg_orientation == LegOrientation::UNDER)))
  {
    controller->setMode(BalanceMode::NORMAL);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit STAND_UP");
  }
  if (controller->getOverturn())
  {
    controller->setMode(BalanceMode::RECOVER);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit STAND_UP");
  }
}

void StandUp::setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const LegOrientation& other_leg_orientation,
                             const double& leg_length, const double& leg_theta, LegOrientation& leg_orientation,
                             StandUpLegCommand& legCommand, bool& stop_flag, bool& arrive_flag, ros::Time arrive_time)
{
  switch (leg_orientation)
  {
    case LegOrientation::UNDER:
      stop_flag = false;
      legCommand.desired_angle = leg_theta;
      legCommand.desired_length = 0.34;
      if (leg_length > 0.33)
      {
        leg_orientation = LegOrientation::FRONT;
      }
      break;
    case LegOrientation::FRONT:
      stop_flag = false;
      if (!arrive_flag)
        arrive_flag = false;
      legCommand.desired_angle = M_PI_2 - 0.35;
      legCommand.desired_length = 0.34;
      legCommand.desired_angle_vel = 0.0;
      if (leg_length > 0.30)
        legCommand.desired_angle_vel = -5.0;
      if (abs(legCommand.desired_angle - leg_theta) < 0.5)
      {
        legCommand.desired_angle_vel = -1.5f;
      }
      if (abs(x[1]) < 0.1)
      {
        if (x[0] > 0 && x[0] < M_PI_2 + 0.4f)
        {
          legCommand.desired_angle_vel = -0.5f;
          if (!arrive_flag)
          {
            arrive_flag = true;
            arrive_time = ros::Time::now();
          }
          if ((ros::Time::now() - arrive_time).toSec() > leg_state_threshold_->arrive_time_threshold)
            leg_orientation = LegOrientation::BEHIND;
        }
      }
      break;
    case LegOrientation::BEHIND:
      stop_flag = true;
      legCommand.desired_angle = leg_theta;
      legCommand.desired_length = leg_length;
      if (other_leg_orientation == LegOrientation::BEHIND)
      {
        stop_flag = false;

        legCommand.desired_length = 0.12f;
        legCommand.desired_angle = 0.0f;
        //        legCommand.desired_angle = leg_theta;
        //        double h = controller->getChassisGeometryParams()->chassis_height;
        //        legCommand.desired_angle = acos(h / leg_length);
      }
      break;
  }
}

inline void StandUp::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_orientation)
{
  if (!leg_state_threshold_)
  {
    ROS_ERROR_THROTTLE(1.0, "LegUtils threshold params not initialized!");
    return;
  }
  if (x[0] > leg_state_threshold_->under_lower && x[0] < leg_state_threshold_->under_upper)
    leg_orientation = LegOrientation::UNDER;
  else if ((x[0] < leg_state_threshold_->front_lower && x[0] > -M_PI) ||
           (x[0] < M_PI && x[0] > leg_state_threshold_->front_upper))
    leg_orientation = LegOrientation::FRONT;
  else if (x[0] > leg_state_threshold_->behind_lower && x[0] < leg_state_threshold_->behind_upper)
    leg_orientation = LegOrientation::BEHIND;
  switch (leg_orientation)
  {
    case LegOrientation::UNDER:
      ROS_INFO("[balance] x[0]: %.3f Leg state: UNDER", x[0]);
      break;
    case LegOrientation::FRONT:
      ROS_INFO("[balance] x[0]: %.3f Leg state: FRONT", x[0]);
      break;
    case LegOrientation::BEHIND:
      ROS_INFO("[balance] x[0]: %.3f Leg state: BEHIND", x[0]);
      break;
  }
}

inline LegCommand StandUp::computePidLegCommand(const StandUpLegCommand& leg_command, const VMCPtr& vmc_,
                                                control_toolbox::Pid& length_pid, control_toolbox::Pid& angle_pid,
                                                control_toolbox::Pid& angle_vel_pid,
                                                const LegOrientation& leg_orientation, const ros::Duration& period,
                                                double feedforward_force)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };

  const auto& leg_pos = vmc_->getPos();
  const auto& leg_spd = vmc_->getSpd();

  double Tp_leg_comp{}, F_leg_comp{}, beta{};
  double leg_mass = 2.0f;
  double G_leg = leg_mass * 9.81f;
  double l_leg = get_LM(leg_pos.L0);
  double theta_leg_offset = get_theta_leg_offset(leg_pos.L0);
  if (leg_pos.theta > -M_PI_2 && leg_pos.theta < M_PI_2)
  {
    beta = leg_pos.theta + theta_leg_offset;
    F_leg_comp = -G_leg * l_leg * cos(beta);
  }
  else
  {
    if (leg_pos.theta > -M_PI && leg_pos.theta < -M_PI_2)
    {
      beta = -leg_pos.theta - M_PI - theta_leg_offset;
    }
    else if (leg_pos.theta > M_PI_2 && leg_pos.theta < M_PI)
    {
      beta = M_PI - leg_pos.theta - theta_leg_offset;
    }
    F_leg_comp = G_leg * l_leg * cos(beta);
  }
  Tp_leg_comp = G_leg * l_leg * sin(beta);

  double F_pid_force = length_pid.computeCommand(leg_command.desired_length - leg_pos.L0, period);
  F_pid_force = abs(F_pid_force) > 200 ? std::copysign(1, F_pid_force) * 200 : F_pid_force;
  cmd.force = F_pid_force + feedforward_force;
  if (leg_orientation == LegOrientation::BEHIND || leg_orientation == LegOrientation::UNDER)
  {
    cmd.torque =
        angle_pid.computeCommand(-angles::shortest_angular_distance(leg_command.desired_angle, leg_pos.theta), period);
  }
  else
  {
    cmd.torque = angle_vel_pid.computeCommand(leg_command.desired_angle_vel - leg_spd.dTheta, period);
  }
  vmc_->leg_conv(cmd.force + F_leg_comp, cmd.torque + Tp_leg_comp, cmd.input);
  return cmd;
}
}  // namespace rm_chassis_controllers
