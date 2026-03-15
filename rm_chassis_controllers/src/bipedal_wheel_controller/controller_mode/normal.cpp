//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
Normal::Normal(const std::vector<hardware_interface::JointHandle*>& joint_handles,
               const std::vector<control_toolbox::Pid*>& pid_legs, control_toolbox::Pid* pid_yaw_vel,
               control_toolbox::Pid* pid_theta_diff, control_toolbox::Pid* pid_roll)
  : joint_handles_(joint_handles)
  , pid_legs_(pid_legs)
  , pid_yaw_vel_(pid_yaw_vel)
  , pid_theta_diff_(pid_theta_diff)
  , pid_roll_(pid_roll)
{
  leftSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
  rightSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
}

void Normal::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  auto bias_params_ = controller->getBiasParams();
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter NORMAL");
    controller->clearStatus();
    jump_phase_ = JumpPhase::IDLE;
    pos_des_ = 0;
    controller->setStateChange(true);
  }

  if (!controller->getCompleteStand() && abs(x_left_[4]) < 0.2 && (abs(x_left_[0] + x_right_[0]) / 2.0f) < 0.15)
  {
    controller->setCompleteStand(true);
  }

  auto vel_cmd_ = controller->getVelCmd();
  if (controller->getMoveFlag() && abs(x_left_[3]) < 0.1 && abs(vel_cmd_.x) < 0.1)
  {
    controller->setMoveFlag(false);
  }

  double friction_circle = x_left_(3) * angular_vel_base_.z;
  double friction_circle_alpha = abs(friction_circle) > 3.5f ? (3.5f / abs(friction_circle)) : 1.0f;
  // PID
  double T_yaw = pid_yaw_vel_->computeCommand(friction_circle_alpha * vel_cmd_.z - angular_vel_base_.z, period);
  double T_theta_diff = pid_theta_diff_->computeCommand(right_pos_[1] - left_pos_[1], period);
  double F_roll = pid_roll_->computeCommand(0. - roll_, period);

  // LQR
  Matrix<double, 4, 12> coeffs_ = controller->getCoeffs();
  Matrix<double, 2, 6> k_left, k_right;
  k_left.setZero();
  k_right.setZero();

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      k_left(i, j) = coeffs_(0, i + 2 * j) * pow(left_pos_[0], 3) + coeffs_(1, i + 2 * j) * pow(left_pos_[0], 2) +
                     coeffs_(2, i + 2 * j) * left_pos_[0] + coeffs_(3, i + 2 * j);
      k_right(i, j) = coeffs_(0, i + 2 * j) * pow(right_pos_[0], 3) + coeffs_(1, i + 2 * j) * pow(right_pos_[0], 2) +
                      coeffs_(2, i + 2 * j) * right_pos_[0] + coeffs_(3, i + 2 * j);
    }
  }

  Eigen::Matrix<double, CONTROL_DIM, 1> u_left, u_right;
  u_left.setZero();
  u_right.setZero();
  auto x_left = x_left_;
  auto x_right = x_right_;
  Matrix<double, 6, 1> x_left_ref, x_right_ref;
  x_left_ref.setZero();
  x_right_ref.setZero();

  if (controller->getCompleteStand())
  {
    x_left_ref(2) = x_right_ref(2) = pos_des_;
    x_left_ref(3) = x_right_ref(3) = friction_circle_alpha * vel_cmd_.x;
    leg_length_des = controller->getLegCmd();
  }
  x_left(0) -= bias_params_->theta;
  x_right(0) -= bias_params_->theta;
  x_left(4) -= bias_params_->pitch;
  x_right(4) -= bias_params_->pitch;

  x_left -= x_left_ref;
  x_right -= x_right_ref;

  u_left = k_left * (-x_left);
  u_right = k_right * (-x_right);

  // Compute leg thrust
  auto model_params_ = controller->getModelParams();
  auto control_params_ = controller->getControlParams();
  //  auto f_spring_force = [](double l) { return ((2094.45f * l - 3091.28f) * l + 1408.375f) * l - 80.91f; };
  double gravity = model_params_->f_gravity, current_leg_length = (left_pos_[0] + right_pos_[0]) / 2.0f,
         spring_force = model_params_->f_spring;
  double F_inertia = model_params_->M * friction_circle;
  Eigen::Matrix<double, 2, 1> F_leg;

  // check jump
  if (jump_phase_ == JumpPhase::IDLE &&
      ros::Time::now() - lastJumpTime_ > ros::Duration(control_params_->jumpOverTime_) && controller->getJumpCmd())
  {
    jump_phase_ = JumpPhase::LEG_RETRACTION;
    ROS_INFO("[balance] Jump start");
  }
  if (jump_phase_ == JumpPhase::IDLE)
  {
    double left_length_des = controller->getCompleteStand() ? leg_length_des : controller->getDefaultLegLength();
    double right_length_des = controller->getCompleteStand() ? leg_length_des : controller->getDefaultLegLength();
    double F_pid_left = pid_legs_[LEFT]->computeCommand(left_length_des - current_leg_length, period);
    double F_pid_right = pid_legs_[RIGHT]->computeCommand(right_length_des - current_leg_length, period);
    F_pid_left = abs(F_pid_left) > 150 ? std::copysign(1, F_pid_left) * 150 : F_pid_left;
    F_pid_right = abs(F_pid_right) > 150 ? std::copysign(1, F_pid_right) * 150 : F_pid_right;
    F_leg[LEFT] = F_pid_left - F_inertia + gravity * cos(left_pos_[1]) + F_roll - spring_force;
    F_leg[RIGHT] = F_pid_right + F_inertia + gravity * cos(right_pos_[1]) - F_roll - spring_force;
  }
  else
  {
    leg_length_des = jumpLengthDes[jump_phase_].second;
    double s_left = (left_pos_[0] - 0.12) / (0.4 - 0.12);
    double s_right = (right_pos_[0] - 0.12) / (0.4 - 0.12);
    switch (jump_phase_)
    {
      case JumpPhase::LEG_RETRACTION:
      {
        ROS_INFO("[balance] ENTER LEG_RETRACTION");
        F_leg(LEFT) = pid_legs_[LEFT]->computeCommand(leg_length_des - current_leg_length, period) +
                      gravity * cos(left_pos_[1]) + F_roll - spring_force;
        F_leg(RIGHT) = pid_legs_[RIGHT]->computeCommand(leg_length_des - current_leg_length, period) +
                       gravity * cos(left_pos_[1]) - F_roll - spring_force;
        if (current_leg_length < leg_length_des + 0.01)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 6)
        {
          jumpTime_ = 0;
          jump_phase_ = JumpPhase::JUMP_UP;
        }
        break;
      }
      case JumpPhase::JUMP_UP:
        ROS_INFO("[balance] ENTER JUMP_UP");
        //        F_leg(0) = control_params_->p1_ * pow(left_pos_[0], 3) + control_params_->p2_ * pow(left_pos_[0], 2) +
        //                   control_params_->p3_ * left_pos_[0] + control_params_->p4_ + gravity;
        //        F_leg(1) = control_params_->p1_ * pow(right_pos_[0], 3) + control_params_->p2_ * pow(right_pos_[0], 2) +
        //                   control_params_->p3_ * right_pos_[0] + control_params_->p4_ + gravity;
        F_leg(0) = 400 * (1 - 3 * pow(s_left, 2) + 2 * pow(s_left, 3)) + gravity;
        F_leg(1) = 400 * (1 - 3 * pow(s_right, 2) + 2 * pow(s_right, 3)) + gravity;
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
        //        F_leg(0) = -(control_params_->p1_ * pow(left_pos_[0], 3) + control_params_->p2_ * pow(left_pos_[0], 2) +
        //                     control_params_->p3_ * left_pos_[0] + control_params_->p4_);
        //        F_leg(1) = -(control_params_->p1_ * pow(right_pos_[0], 3) + control_params_->p2_ * pow(right_pos_[0], 2) +
        //                     control_params_->p3_ * right_pos_[0] + control_params_->p4_);
        F_leg(0) = -400 * (1 - 3 * pow(s_left, 2) + 2 * pow(s_left, 3));
        F_leg(1) = -400 * (1 - 3 * pow(s_right, 2) + 2 * pow(s_right, 3));

        if (current_leg_length < leg_length_des)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 8)
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

  bool left_unstick{ false }, right_unstick{ false };
  if (controller->getCompleteStand() && jump_phase_ != JumpPhase::LEG_RETRACTION)
  {
    left_unstick = unstickDetection(F_leg[LEFT], u_left(1), left_spd_[0], left_pos_[0], linear_acc_base_.z,
                                    model_params_, x_left_, leftSupportForceAveragePtr_, period);
    right_unstick = unstickDetection(F_leg[RIGHT], u_right(1), right_spd_[0], right_pos_[0], linear_acc_base_.z,
                                     model_params_, x_right_, rightSupportForceAveragePtr_, period);
  }
  bool unstick[2]{};
  unstick[0] = left_unstick;
  unstick[1] = right_unstick;
  Matrix<double, 2, 1> F_N{};
  F_N(LEFT) = leftSupportForceAveragePtr_->output();
  F_N(RIGHT) = rightSupportForceAveragePtr_->output();
  controller->pubLQRStatus(-x_left, -x_right, x_left_ref, x_right_ref, u_left, u_right, F_N, unstick);

  updateUnstick(left_unstick, right_unstick);
  left_unstick = right_unstick = false;
  if (controller->getCompleteStand() && left_unstick && jump_phase_ != JumpPhase::LEG_RETRACTION)
  {
    F_leg[LEFT] -= F_roll;
    u_left = k_left_unstick * (-x_left);
  }
  if (controller->getCompleteStand() && right_unstick && jump_phase_ != JumpPhase::LEG_RETRACTION)
  {
    F_leg[RIGHT] += F_roll;
    u_right = k_right_unstick * (-x_right);
  }

  // Control
  double left_T[2], right_T[2];
  controller->getVMCPtr()->leg_conv(F_leg[LEFT], u_left(1) + T_theta_diff, left_angle_[0], left_angle_[1], left_T);
  controller->getVMCPtr()->leg_conv(F_leg[RIGHT], u_right(1) - T_theta_diff, right_angle_[0], right_angle_[1], right_T);
  double left_wheel_cmd = left_unstick ? 0. : u_left(0) - T_yaw;
  double right_wheel_cmd = right_unstick ? 0. : u_right(0) + T_yaw;
  LegCommand left_cmd = { F_leg[LEFT], u_left[1], { left_T[0], left_T[1] } },
             right_cmd = { F_leg[RIGHT], u_right[1], { right_T[0], right_T[1] } };

  // upstairs
  if (jump_phase_ == JumpPhase::IDLE && linear_acc_base_.z < -7.0 && controller->getCompleteStand() &&
      abs(vel_cmd_.x) > 0.1 && abs(x_left(3)) > 0.1 && ((left_pos_[0] + right_pos_[0]) / 2.0f) > 0.3 &&
      leg_length_des > 0.30)
  {
    leg_length_des = controller->getDefaultLegLength();
    controller->setMode(BalanceMode::UPSTAIRS);
    controller->setStateChange(false);
    controller->setJumpCmd(false);
    left_wheel_cmd = right_wheel_cmd = 0;
    ROS_INFO("[balance] Exit NORMAL");
  }

  // Protection
  if (abs(x_left(4)) > 0.6 || abs(x_left(0)) > 0.9 || abs(x_right(0)) > 0.9 || abs(roll_) > 1.0 ||
      controller->getOverturn() || controller->getBaseState() == 4)
  {
    leg_length_des = controller->getDefaultLegLength();
    x_left_(2) = x_right_(2) = bias_params_->x;
    controller->setMode(BalanceMode::SIT_DOWN);
    controller->setStateChange(false);
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
                last_ddot_theta = (x(1) - last_dot_theta) / period.toSec();

  double P = F * cos(x(0)) + Tp * sin(x(0)) / leg_length;
  // lp filter
  double ddot_zM = 0.7 * (acc_z - model_params->g) + 0.3 * last_ddot_zM;
  double ddot_theta = 0.7 * ((x(1) - last_dot_theta) / period.toSec()) + 0.3 * last_ddot_theta;
  last_dot_theta = x(1);
  last_ddot_theta = ddot_theta;
  double ddot_zw = ddot_zM - leg_length * cos(x(0)) + 2 * leg_len_spd * x(1) * sin(x(0)) +
                   +leg_length * (ddot_theta * sin(x(0)) + x(1) * x(1) * cos(x(0)));
  double Fn = model_params->m_w * ddot_zw + model_params->m_w * model_params->g + P;

  return Fn;
}

bool Normal::unstickDetection(const double& F_leg, const double& Tp, const double& leg_len_spd,
                              const double& leg_length, const double& acc_z,
                              const std::shared_ptr<ModelParams>& model_params, Eigen::Matrix<double, STATE_DIM, 1> x,
                              std::shared_ptr<MovingAverageFilter<double>> supportForceAveragePtr,
                              const ros::Duration& period)
{
  static bool maybeChange = false, last_unstick_ = false;
  static ros::Time judgeTime;
  double Fn = calculateSupportForce(F_leg, Tp, leg_length, leg_len_spd, acc_z, x, model_params, period);
  supportForceAveragePtr->input(Fn);
  bool unstick_ = supportForceAveragePtr->output() < 10;

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
