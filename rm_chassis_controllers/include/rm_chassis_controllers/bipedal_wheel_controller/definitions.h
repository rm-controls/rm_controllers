//
// Created by guanlin on 25-8-30.
//

#pragma once

#include "bipedal_wheel_controller/vmc/VMC.h"
#include <array>
#include <utility>

namespace rm_chassis_controllers
{
constexpr static const int STATE_DIM = 6;
constexpr static const int CONTROL_DIM = 2;

struct ModelParams
{
  double L_weight;   // Length weight to wheel axis
  double Lm_weight;  // Length weight to mass center
  double l;          // Leg rest length
  double m_w;        // Wheel mass
  double m_p;        // Leg mass
  double M;          // Body mass
  double i_w;        // Wheel inertia
  double i_p;        // Leg inertia
  double i_m;        // Body inertia
  double r;          // Wheel radius
  double g;          // Gravity acceleration
  double f_gravity;  // Gravity Force
};

struct ChassisGeometryParams
{
  double chassis_height;  // 底盘高度 (m)
  double wheel_track;     // 轮距(左右)
};

struct SpringParams
{
  double s2;
  double s3;
  double alpha_s;
  double f_spring;  // Spring Force
};

struct ControlParams
{
  double jumpOverTime_;
  double down5cmStairPitchThreshold;
  double down5cmStairThetaThreshold;
  double jump_up_force;
  double off_ground_force;
};

struct BiasParams
{
  double x;
  double theta;
  double mid_leg_len_theta;
  double high_leg_len_theta;
  double pitch;
  double roll;
  double raw_pitch;
  double raw_theta;
};

struct LegStateThresholdParams
{
  double under_lower;
  double under_upper;
  double front_lower;
  double front_upper;
  double behind_lower;
  double behind_upper;
  double upstair_des_theta;
  double upstair_des_length;
  double upstair_exit_theta_threshold;
  double upstair_exit_length_threshold;
  double unstick_threshold;
  double arrive_time_threshold;
};

struct LegCommand
{
  double force;     // Thrust
  double torque;    // Torque
  double input[2];  // input
};

enum LegOrientation
{
  UNDER,
  FRONT,
  BEHIND
};

enum JumpPhase
{
  LEG_RETRACTION,
  JUMP_UP,
  OFF_GROUND,
  IDLE,
};

enum BalanceMode
{
  NORMAL,
  STAND_UP,
  SIT_DOWN,
  RECOVER,
  UPSTAIRS,
  PROTECT
};

enum Side
{
  LEFT = 0,
  RIGHT,
};

enum
{
  WHEEL_T = 0,
  LEG_Tp,
};

enum
{
  THETA = 0,
  D_THETA,
  POS,
  VEL,
  PITCH,
  D_PITCH,
};

struct LegState
{
  Eigen::Matrix<double, STATE_DIM, 1> x;  // LQR状态量
  double angle[2];                        // [0]: hip, [1]: knee
  VMCPtr vmc{ nullptr };
  bool unstick = false;
};

struct ChassisState
{
  geometry_msgs::Vector3 angular_vel;
  geometry_msgs::Vector3 linear_acc;
  double x_vel = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double yaw_total = 0.0;
  double yaw_total_last = 0.0;
};

constexpr std::array<std::pair<JumpPhase, const double>, 3> jumpLengthDes = {
  { { JumpPhase::LEG_RETRACTION, 0.11 }, { JumpPhase::JUMP_UP, 0.34 }, { JumpPhase::OFF_GROUND, 0.11 } }
};
}  // namespace rm_chassis_controllers
