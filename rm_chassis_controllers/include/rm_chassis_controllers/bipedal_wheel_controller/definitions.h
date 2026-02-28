//
// Created by guanlin on 25-8-30.
//

#pragma once

#include <array>
#include <utility>

namespace rm_chassis_controllers
{
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
  double f_spring;   // Spring Force
  double f_gravity;  // Gravity Force
};

struct ControlParams
{
  double jumpOverTime_;
  double p1_;
  double p2_;
  double p3_;
  double p4_;
};

struct BiasParams
{
  double x;
  double theta;
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
  double upstair_exit_threshold;
};

struct LegCommand
{
  double force;     // Thrust
  double torque;    // Torque
  double input[2];  // input
};

enum LegState
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
};

enum
{
  LEFT = 0,
  RIGHT,
};

enum
{
  LEG_T = 0,
  LEG_Tp
};

constexpr std::array<std::pair<JumpPhase, const double>, 3> jumpLengthDes = {
  { { JumpPhase::LEG_RETRACTION, 0.12 }, { JumpPhase::JUMP_UP, 0.38 }, { JumpPhase::OFF_GROUND, 0.15 } }
};

constexpr static const int STATE_DIM = 6;
constexpr static const int CONTROL_DIM = 2;

}  // namespace rm_chassis_controllers
