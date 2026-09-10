//
// Created by guanlin on 25-8-27.
//

#pragma once

#include <angles/angles.h>
#include <cstddef>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Quaternion.h>
#include <hardware_interface/joint_command_interface.h>

#include "bipedal_wheel_controller/dynamics/gen_A.h"
#include "bipedal_wheel_controller/dynamics/gen_B.h"
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
static inline double get_LM(const double& l)
{
  return 0.218f * l + 0.075f;
};

static inline double get_i_p(const double& l)
{
  return 0.4f * l + 0.07f;
};

static inline double get_theta_leg_offset(const double& l)
{
  return M_PI_4 / 2;
}

/**
 * Generate continuous-time state space matrices A and B
 * @param model_params
 * @param a
 * @param b
 * @param leg_length
 */
inline void generateAB(const std::shared_ptr<ModelParams>& model_params, Eigen::Matrix<double, STATE_DIM, STATE_DIM>& a,
                       Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& b, double leg_length)
{
  double A[36] = { 0. }, B[12]{ 0. };
  //  double L = leg_length * model_params->L_weight;
  //  double Lm = leg_length * model_params->Lm_weight;
  double Lm = get_LM(leg_length);
  double L = leg_length - Lm;
  double i_p = get_i_p(leg_length);

  //  auto theta_from_length = [](double L) -> double {
  //    return -23.36693691 * L * L * L + 24.76241959 * L * L - 11.65313741 * L + 2.49258628;
  //  };
  //  double theta_L = theta_from_length(leg_length);
  //  gen_A_leg_offset(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
  //                   model_params->g, model_params->l, model_params->m_p, model_params->m_w, theta_L, A);
  //  gen_B_leg_offset(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
  //                   model_params->g, model_params->l, model_params->m_p, model_params->m_w, theta_L, B);

  //  gen_A(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
  //        model_params->g, model_params->l, model_params->m_p, model_params->m_w, A);
  //  gen_B(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
  //        model_params->l, model_params->m_p, model_params->m_w, B);

  gen_A(model_params->i_m, i_p, model_params->i_w, L, Lm, model_params->M, model_params->r, model_params->g,
        model_params->l, model_params->m_p, model_params->m_w, A);
  gen_B(model_params->i_m, i_p, model_params->i_w, L, Lm, model_params->M, model_params->r, model_params->l,
        model_params->m_p, model_params->m_w, B);

  // clang-format off
  a<< 0.  ,1.,0.,0.,0.   ,0.,
      A[1],0.,0.,0.,A[25],0.,
      0.  ,0.,0.,1.,0.   ,0.,
      A[3],0.,0.,0.,A[27],0.,
      0.  ,0.,0.,0.,0.   ,1.,
      A[5],0.,0.,0.,A[29],0.;
  b<< 0.  ,0.  ,
      B[1],B[7],
      0.  ,0.  ,
      B[3],B[9],
      0.  ,0.  ,
      B[5],B[11];
  // clang-format on
}

/**
 * Set joint commands to the joint handles
 * @param joints
 * @param left_cmd
 * @param right_cmd
 * @param wheel_left
 * @param wheel_right
 */
inline void setJointCommands(std::vector<hardware_interface::JointHandle*>& joints, const LegCommand& left_cmd,
                             const LegCommand& right_cmd, double wheel_left = 0., double wheel_right = 0.)
{
  if (joints.size() != 6)
    throw std::runtime_error("Joint handle vector size must be 6!");

  joints[0]->setCommand(left_cmd.input[0]);
  joints[1]->setCommand(left_cmd.input[1]);
  joints[2]->setCommand(right_cmd.input[0]);
  joints[3]->setCommand(right_cmd.input[1]);
  joints[4]->setCommand(wheel_left);
  joints[5]->setCommand(wheel_right);
}

/**
 * Convert quaternion to roll, pitch, yaw
 * @param q
 * @param roll
 * @param pitch
 * @param yaw
 */
inline void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
  yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  pitch = std::asin(as);
  roll = std::atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

inline void clamp(double& val, const double& minVal, const double& maxVal)
{
  if (val < minVal)
    val = minVal;
  if (val > maxVal)
    val = maxVal;
}

}  // namespace rm_chassis_controllers
