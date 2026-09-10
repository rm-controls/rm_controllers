//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
Normal::Normal(BipedalControllerInterface* controller_,
               const std::vector<hardware_interface::JointHandle*>& joint_handles,
               const std::vector<control_toolbox::Pid*>& pid_legs, control_toolbox::Pid* pid_yaw_vel,
               control_toolbox::Pid* pid_theta_diff, control_toolbox::Pid* pid_roll,
               control_toolbox::Pid* pid_wheel_vel_diff)
  : ModeBase(controller_)
  , joint_handles_(joint_handles)
  , pid_legs_(pid_legs)
  , pid_yaw_vel_(pid_yaw_vel)
  , pid_theta_diff_(pid_theta_diff)
  , pid_roll_(pid_roll)
  , pid_wheel_vel_diff_(pid_wheel_vel_diff)
{
  leftSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
  rightSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
  if (controller_->getLegThresholdParams() != nullptr)
    unstick_threshold = controller_->getLegThresholdParams()->unstick_threshold;
}

void Normal::execute(const ros::Time& time, const ros::Duration& period)
{
  const auto& bias_params_ = controller->getBiasParams();
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter NORMAL");
    controller->clearStatus();
    jump_phase_ = JumpPhase::IDLE;
    pos_des_ = 0.0f;
    controller->setStateChange(true);
  }
  const auto& chassis_state = controller->getChassisState();
  auto& left_leg_state = controller->getLegState(LEFT);
  auto& right_leg_state = controller->getLegState(RIGHT);
  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& left_spd = left_leg_state.vmc->getSpd();
  const auto& right_spd = right_leg_state.vmc->getSpd();

  if (abs(left_leg_state.x[PITCH]) < 0.2 && (abs(left_leg_state.x[THETA] + right_leg_state.x[THETA]) / 2.0f) < 0.2)
  {
    protect_flag_ = false;
    if (!controller->getCompleteStand())
    {
      controller->setCompleteStand(true);
    }
  }

  auto vel_cmd_ = controller->getVelCmd();
  double current_leg_length = (left_pos.L0 + right_pos.L0) / 2.0f;
  if (abs(chassis_state.x_vel) < 0.1f && abs(vel_cmd_.x) < 0.01f)
  {
    controller->setMoveFlag(false);
    if (x_offset_flag_)
    {
      x_offset_flag_ = false;
      pos_des_ =
          current_leg_length * sin(-(left_leg_state.x(THETA) + right_leg_state.x(THETA)) / 2.0f) + bias_params_->x;
    }
  }

  double friction_circle = chassis_state.x_vel * chassis_state.angular_vel.z;
  double friction_circle_alpha = abs(friction_circle) > 10.0f ? (10.0f / abs(friction_circle)) : 1.0f;
  // PID
  double T_yaw = pid_yaw_vel_->computeCommand(friction_circle_alpha * vel_cmd_.z - chassis_state.angular_vel.z, period);
  double theta_diff = right_pos.theta - left_pos.theta;
  double T_theta_diff = pid_theta_diff_->computeCommand(theta_diff, period);
  double F_roll = pid_roll_->computeCommand(0. - chassis_state.roll, period);
  // LQR
  Matrix<double, 4, 12> coeffs_ = controller->getCoeffs();
  Matrix<double, 2, 6> k_left, k_right;
  k_left.setZero();
  k_right.setZero();

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      k_left(i, j) = coeffs_(0, i + 2 * j) * pow(left_pos.L0, 3) + coeffs_(1, i + 2 * j) * pow(left_pos.L0, 2) +
                     coeffs_(2, i + 2 * j) * left_pos.L0 + coeffs_(3, i + 2 * j);
      k_right(i, j) = coeffs_(0, i + 2 * j) * pow(right_pos.L0, 3) + coeffs_(1, i + 2 * j) * pow(right_pos.L0, 2) +
                      coeffs_(2, i + 2 * j) * right_pos.L0 + coeffs_(3, i + 2 * j);
    }
  }

  Eigen::Matrix<double, CONTROL_DIM, 1> u_left, u_right;
  u_left.setZero();
  u_right.setZero();
  auto x_left = left_leg_state.x;
  auto x_right = right_leg_state.x;
  Matrix<double, 6, 1> x_left_ref, x_right_ref;
  x_left_ref.setZero();
  x_right_ref.setZero();

  if (controller->getCompleteStand())
  {
    x_left_ref(POS) = x_right_ref(POS) = pos_des_;
    if (controller->getBaseState() != rm_msgs::ChassisCmd::RAW)
    {
      x_left_ref(VEL) = x_right_ref(VEL) = friction_circle_alpha * vel_cmd_.x;
    }
    else
    {
      // raw move but  bug
      //      x_left_ref(VEL) = x_right_ref(VEL) = vel_cmd_.x;
      //      x_left_ref(POS) = x_right_ref(POS) = 0.0f;
      x_left_ref(VEL) = x_right_ref(VEL) = 0.0f;
    }
    if (protect_flag_)
    {
      x_left_ref(VEL) = x_right_ref(VEL) = 0.0f;
      leg_length_des = controller->getDefaultLegLength();
    }
    else
    {
      leg_length_des = controller->getLegCmd();
    }
  }
  else
  {
    leg_length_des = controller->getDefaultLegLength();
  }
  if (controller->getBaseState() != rm_msgs::ChassisCmd::RAW)
  {
    if (!controller->getMoveFlag())
    {
      x_offset_flag_ = true;
      x_left(THETA) -= bias_params_->theta;
      x_right(THETA) -= bias_params_->theta;
    }
  }
  else
  {
    x_left(THETA) -= bias_params_->raw_theta;
    x_right(THETA) -= bias_params_->raw_theta;
    x_left(PITCH) -= bias_params_->raw_pitch;
    x_right(PITCH) -= bias_params_->raw_pitch;
  }

  x_left -= x_left_ref;
  x_right -= x_right_ref;

  clamp(x_left(VEL), -1.2f, 1.2f);
  clamp(x_right(VEL), -1.2f, 1.2f);

  const double k_pitch = -0.1f, b = 0.35f;
  double pitch_error_clamp = k_pitch * chassis_state.x_vel + b;
  clamp(x_left(PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(x_right(PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(x_left(THETA), -0.6f, 0.6f);
  clamp(x_right(THETA), -0.6f, 0.6f);

  u_left = k_left * (-x_left);
  u_right = k_right * (-x_right);

  // Compute leg thrust
  auto model_params_ = controller->getModelParams();
  auto control_params_ = controller->getControlParams();
  double wheel_vel_diff = abs(joint_handles_[4]->getVelocity()) - abs(joint_handles_[5]->getVelocity());
  double gravity = model_params_->f_gravity, left_spring_force = controller->f_spring_force(left_pos.L0),
         right_spring_force = controller->f_spring_force(right_pos.L0);
  double F_inertia_left =
      model_params_->M * friction_circle * left_pos.L0 / controller->getChassisGeometryParams()->wheel_track;
  double F_inertia_right =
      model_params_->M * friction_circle * right_pos.L0 / controller->getChassisGeometryParams()->wheel_track;
  double F_pid_left{}, F_pid_right{}, T_wheel_diff{};
  Eigen::Matrix<double, 2, 1> F_leg;
  F_leg.setZero();
  // check jump
  if (jump_phase_ == JumpPhase::IDLE &&
      ros::Time::now() - lastJumpTime_ > ros::Duration(control_params_->jumpOverTime_) && controller->getJumpCmd())
  {
    jump_phase_ = JumpPhase::LEG_RETRACTION;
    ROS_INFO("[balance] Jump start");
  }
  if (jump_phase_ == JumpPhase::IDLE)
  {
    static double last_left_length_des = leg_length_des, last_right_length_des = leg_length_des;
    double left_length_des = controller->getCompleteStand() ? (0.8 * leg_length_des + 0.2 * last_left_length_des) :
                                                              controller->getDefaultLegLength();
    double right_length_des = controller->getCompleteStand() ? (0.8 * leg_length_des + 0.2 * last_right_length_des) :
                                                               controller->getDefaultLegLength();
    last_left_length_des = left_length_des;
    last_right_length_des = right_length_des;
    F_pid_left = pid_legs_[LEFT]->computeCommand(left_length_des - current_leg_length, period);
    F_pid_right = pid_legs_[RIGHT]->computeCommand(right_length_des - current_leg_length, period);
    F_pid_left = abs(F_pid_left) > 150 ? std::copysign(1, F_pid_left) * 150 : F_pid_left;
    F_pid_right = abs(F_pid_right) > 150 ? std::copysign(1, F_pid_right) * 150 : F_pid_right;
    F_leg[LEFT] = F_pid_left - F_inertia_left + gravity / cos(left_pos.theta) + F_roll - left_spring_force;
    F_leg[RIGHT] = F_pid_right + F_inertia_right + gravity / cos(right_pos.theta) - F_roll - right_spring_force;
    T_wheel_diff = controller->getBaseState() == rm_msgs::ChassisCmd::RAW ?
                       pid_wheel_vel_diff_->computeCommand(wheel_vel_diff, period) :
                       0.0f;
  }
  else
  {
    leg_length_des = jumpLengthDes[jump_phase_].second;
    double s_left = (left_pos.L0 - 0.12) / (0.35 - 0.11);
    double s_right = (right_pos.L0 - 0.12) / (0.35 - 0.11);
    switch (jump_phase_)
    {
      case JumpPhase::LEG_RETRACTION:
      {
        ROS_INFO("[balance] ENTER LEG_RETRACTION");
        F_leg(LEFT) = pid_legs_[LEFT]->computeCommand(leg_length_des - current_leg_length, period) +
                      gravity / cos(left_pos.theta) + F_roll - left_spring_force;
        F_leg(RIGHT) = pid_legs_[RIGHT]->computeCommand(leg_length_des - current_leg_length, period) +
                       gravity / cos(right_pos.theta) - F_roll - right_spring_force;
        if (current_leg_length < leg_length_des + 0.02f)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 10)
        {
          jumpTime_ = 0;
          jump_phase_ = JumpPhase::JUMP_UP;
        }
        break;
      }
      case JumpPhase::JUMP_UP:
        ROS_INFO("[balance] ENTER JUMP_UP");
        F_leg(0) = 300 * (1 - 3 * pow(s_left, 2) + 2 * pow(s_left, 3)) + gravity;
        F_leg(1) = 300 * (1 - 3 * pow(s_right, 2) + 2 * pow(s_right, 3)) + gravity;
        if (current_leg_length > leg_length_des)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 2)
        {
          jumpTime_ = 0;
          jump_phase_ = JumpPhase::OFF_GROUND;
        }
        break;
      case JumpPhase::OFF_GROUND:
        ROS_INFO("[balance] ENTER OFF_GROUND");
        double s_left_flip = 1 - s_left;
        double s_right_flip = 1 - s_left;
        F_leg(0) = -175 * (1 - 3 * pow(s_left_flip, 2) + 2 * pow(s_left_flip, 3)) - left_spring_force;
        F_leg(1) = -175 * (1 - 3 * pow(s_right_flip, 2) + 2 * pow(s_right_flip, 3)) - right_spring_force;

        if (current_leg_length < leg_length_des + 0.02f)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 100)
        {
          jumpTime_ = 0;
          jump_phase_ = JumpPhase::IDLE;
          lastJumpTime_ = ros::Time::now();
          ROS_INFO("[balance] Jump end");
        }
        break;
    }
  }

  // Unstick detection
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left_unstick{}, k_right_unstick{};
  k_left_unstick.setZero();
  k_right_unstick.setZero();
  k_left_unstick.block<1, 2>(1, 0) = k_left.block<1, 2>(1, 0);
  k_right_unstick.block<1, 2>(1, 0) = k_right.block<1, 2>(1, 0);

  static bool last_left_unstick{ false }, last_right_unstick{ false };
  bool left_unstick{ false }, right_unstick{ false };
  if (jump_phase_ == JumpPhase::OFF_GROUND)
  {
    left_unstick = right_unstick = true;
  }
  else if (controller->getCompleteStand() && jump_phase_ != JumpPhase::LEG_RETRACTION &&
           controller->getBaseState() == rm_msgs::ChassisCmd::FOLLOW)
  {
    left_unstick =
        unstickDetection(last_left_unstick ? F_pid_left : left_leg_state.vmc->getForceReal().F + left_spring_force,
                         u_left(LEG_Tp), left_spd.dL0, left_pos.L0, chassis_state.linear_acc.z, model_params_,
                         left_leg_state.x, leftSupportForceAveragePtr_, period);
    right_unstick =
        unstickDetection(last_right_unstick ? F_pid_right : right_leg_state.vmc->getForceReal().F + right_spring_force,
                         u_right(LEG_Tp), right_spd.dL0, right_pos.L0, chassis_state.linear_acc.z, model_params_,
                         right_leg_state.x, rightSupportForceAveragePtr_, period);
  }
  bool unstick[2]{};
  unstick[0] = left_unstick;
  unstick[1] = right_unstick;
  last_left_unstick = left_unstick;
  last_right_unstick = right_unstick;
  Matrix<double, 2, 1> F_N{};
  F_N(LEFT) = leftSupportForceAveragePtr_->output();
  F_N(RIGHT) = rightSupportForceAveragePtr_->output();
  controller->pubLQRStatus(-x_left, -x_right, x_left_ref, x_right_ref, u_left, u_right, F_N, unstick);

  updateUnstick(left_unstick, right_unstick);
  //  left_unstick = right_unstick = false;
  bool unstick_flag = left_unstick && right_unstick;
  if ((controller->getCompleteStand() && unstick_flag && jump_phase_ != JumpPhase::LEG_RETRACTION) ||
      jump_phase_ == JumpPhase::OFF_GROUND)
  {
    u_left = k_left_unstick * (-x_left);
    u_right = k_right_unstick * (-x_right);
  }

  // Control
  double left_T[2], right_T[2];
  left_leg_state.vmc->leg_conv(F_leg[LEFT], u_left(LEG_Tp) + T_theta_diff, left_T);
  right_leg_state.vmc->leg_conv(F_leg[RIGHT], u_right(LEG_Tp) - T_theta_diff, right_T);
  double left_wheel_cmd = unstick_flag ? 0. : u_left(WHEEL_T) - T_yaw - T_wheel_diff;
  double right_wheel_cmd = unstick_flag ? 0. : u_right(WHEEL_T) + T_yaw - T_wheel_diff;
  LegCommand left_cmd = { F_leg[LEFT], u_left(LEG_Tp) + T_theta_diff, { left_T[0], left_T[1] } },
             right_cmd = { F_leg[RIGHT], u_right(LEG_Tp) - T_theta_diff, { right_T[0], right_T[1] } };

  // upstairs
  if (jump_phase_ == JumpPhase::IDLE && controller->getCompleteStand() && abs(x_left(0) + x_right(0)) / 2.0f > 0.50 &&
      abs(vel_cmd_.x) > 0.1 && abs(x_left(3)) > 0.1 && ((left_pos.L0 + right_pos.L0) / 2.0f) > 0.30 &&
      leg_length_des > 0.30)
  {
    controller->setMode(BalanceMode::UPSTAIRS);
    controller->setStateChange(false);
    controller->setJumpCmd(false);
    controller->setCompleteStand(false);
    left_wheel_cmd = right_wheel_cmd = 0;
    ROS_INFO("[balance] Exit NORMAL");
  }

  if (leg_length_des < 0.22)
  {
    // Protection
    if ((abs(x_left(0)) > 0.6f || abs(x_right(0)) > 0.6f || abs(chassis_state.pitch) > 0.4 ||
         abs(chassis_state.roll) > 0.4) ||
        (abs(u_left(0)) + abs(u_right(0)) / 2.0f > 30.0f) ||
        (abs(chassis_state.x_vel - x_left_ref(VEL)) > 5.0f && abs(chassis_state.x_vel - vel_cmd_.x) > 5.0f))
    {
      protect_flag_ = true;
      leg_length_des = controller->getDefaultLegLength();
      left_leg_state.x(POS) = right_leg_state.x(POS) = 0;
      controller->setMode(BalanceMode::PROTECT);
      controller->setStateChange(false);
      controller->setCompleteStand(false);
      controller->setJumpCmd(false);
      setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
      ROS_INFO("[balance] Exit NORMAL");
    }
  }
  // Protection to sit_down
  if (abs(x_left(THETA)) > 1.0 || abs(x_right(THETA)) > 1.0 || abs(chassis_state.pitch) > 0.6 ||
      abs(chassis_state.roll) > 0.8 || controller->getOverturn() || abs(theta_diff) > 1.0 ||
      controller->getBaseState() == rm_msgs::ChassisCmd::FALLEN)
  {
    left_leg_state.x(POS) = right_leg_state.x(POS) = 0;
    controller->setMode(BalanceMode::SIT_DOWN);
    controller->setStateChange(false);
    controller->setCompleteStand(false);
    controller->setJumpCmd(false);
    setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
    ROS_INFO("[balance] Exit NORMAL");
  }
  setJointCommands(joint_handles_, left_cmd, right_cmd, left_wheel_cmd, right_wheel_cmd);
}

double Normal::calculateSupportForce(double F, double Tp, double leg_length, const double& leg_len_spd, double acc_z,
                                     Eigen::Matrix<double, STATE_DIM, 1> x,
                                     const std::shared_ptr<ModelParams>& model_params, const ros::Duration& period)
{
  static double last_ddot_zM = acc_z - model_params->g, last_dot_theta = x(1),
                last_ddot_theta = (x(1) - last_dot_theta) / period.toSec(), last_ddot_leg_len, d_leg_len[3]{};
  d_leg_len[0] = d_leg_len[1];
  d_leg_len[1] = d_leg_len[2];
  d_leg_len[2] = leg_len_spd;

  double P = F * cos(x(0)) + Tp * sin(x(0)) / leg_length;
  // lp filter
  double ddot_zM = 0.3 * (acc_z - model_params->g) + 0.7 * last_ddot_zM;
  double ddot_theta = 0.3 * ((x(1) - last_dot_theta) / period.toSec()) + 0.7 * last_ddot_theta;
  double ddot_leg_len = 0.3 * (d_leg_len[2] - d_leg_len[0]) / 2 * period.toSec() + 0.7 * last_ddot_leg_len;
  last_dot_theta = x(1);
  last_ddot_theta = ddot_theta;
  last_ddot_leg_len = ddot_leg_len;
  double ddot_zw = ddot_zM - ddot_leg_len * cos(x(0)) + 2 * leg_len_spd * x(1) * sin(x(0)) +
                   +leg_length * (ddot_theta * sin(x(0)) + leg_length * x(1) * x(1) * cos(x(0)));
  double Fn = model_params->m_w * ddot_zw + model_params->m_w * model_params->g + P;

  return Fn;
}

bool Normal::unstickDetection(const double& F_leg, const double& Tp, const double& leg_len_spd,
                              const double& leg_length, const double& acc_z,
                              const std::shared_ptr<ModelParams>& model_params, Eigen::Matrix<double, STATE_DIM, 1> x,
                              const std::shared_ptr<MovingAverageFilter<double>>& supportForceAveragePtr,
                              const ros::Duration& period)
{
  static bool maybeChange = false, last_unstick_ = false;
  static ros::Time judgeTime;
  double Fn = calculateSupportForce(F_leg, Tp, leg_length, leg_len_spd, acc_z, x, model_params, period);
  supportForceAveragePtr->input(Fn);
  bool unstick_ = supportForceAveragePtr->output() < unstick_threshold;

  if (unstick_ != last_unstick_)
  {
    if (!maybeChange)
    {
      judgeTime = ros::Time::now();
      maybeChange = true;
    }
    else
    {
      if (ros::Time::now() - judgeTime > ros::Duration(0.1))
      {
        last_unstick_ = unstick_;
      }
    }
  }
  else
  {
    maybeChange = false;
  }
  return unstick_;
}
}  // namespace rm_chassis_controllers
