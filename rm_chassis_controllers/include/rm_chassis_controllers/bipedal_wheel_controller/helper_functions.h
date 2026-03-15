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
#include "bipedal_wheel_controller/vmc/leg_conv.h"
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
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
  double L = leg_length * model_params->L_weight;
  double Lm = leg_length * model_params->Lm_weight;
  //  auto theta_from_length = [](double L) -> double {
  //    return -23.36693691 * L * L * L + 24.76241959 * L * L - 11.65313741 * L + 2.49258628;
  //  };
  //  double theta_L = theta_from_length(leg_length);
  gen_A(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
        model_params->g, model_params->l, model_params->m_p, model_params->m_w, A);
  gen_B(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
        model_params->l, model_params->m_p, model_params->m_w, B);
  //  gen_A_leg_offset(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
  //                   model_params->g, model_params->l, model_params->m_p, model_params->m_w, theta_L, A);
  //  gen_B_leg_offset(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
  //                   model_params->g, model_params->l, model_params->m_p, model_params->m_w, theta_L, B);

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
 * Compute the leg command using PID controllers
 * @param desired_length
 * @param desired_angle
 * @param current_length
 * @param current_angle
 * @param length_pid
 * @param angle_pid
 * @param leg_angle
 * @param period
 * @param feedforward_force
 * @param overturn
 * @return
 */
[[maybe_unused]] inline LegCommand computePidLegCommand(double desired_length, double desired_angle, double leg_pos[2],
                                                        double leg_spd[2], control_toolbox::Pid& length_pid,
                                                        control_toolbox::Pid& angle_pid,
                                                        control_toolbox::Pid& angle_vel_pid, const double* leg_angle,
                                                        const int& leg_state, const ros::Duration& period,
                                                        double feedforward_force = 0.0f, const bool& overturn = false)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };
  cmd.force = length_pid.computeCommand(desired_length - leg_pos[0], period) + feedforward_force;
  if (!overturn)
  {
    if (leg_state == LegState::BEHIND || leg_state == LegState::UNDER)
    {
      cmd.torque = angle_pid.computeCommand(-angles::shortest_angular_distance(desired_angle, leg_pos[1]), period);
    }
    else
    {
      cmd.torque = angle_vel_pid.computeCommand(-5 - leg_spd[1], period);
    }
  }
  leg_conv(cmd.force, cmd.torque, leg_angle[0], leg_angle[1], cmd.input);
  return cmd;
}

[[maybe_unused]] inline LegCommand
computePidAngleVelLegCommand(double desired_length, double desired_leg_angle_vel, double leg_pos[2], double leg_spd[2],
                             control_toolbox::Pid& length_pid, control_toolbox::Pid& angle_vel_pid,
                             const double* leg_angle, const ros::Duration& period, double feedforward_force = 0.0f)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };
  cmd.force = length_pid.computeCommand(desired_length - leg_pos[0], period) + feedforward_force;
  cmd.torque = angle_vel_pid.computeCommand(desired_leg_angle_vel - leg_spd[1], period);
  leg_conv(cmd.force, 10 * desired_leg_angle_vel + cmd.torque, leg_angle[0], leg_angle[1], cmd.input);
  return cmd;
}

[[maybe_unused]] inline LegCommand computePidAngleLegCommand(double desired_length, double desired_leg_angle,
                                                             double leg_pos[2], control_toolbox::Pid& length_pid,
                                                             control_toolbox::Pid& angle_pid, const double* leg_angle,
                                                             const ros::Duration& period,
                                                             double feedforward_force = 0.0f)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };
  cmd.force = length_pid.computeCommand(desired_length - leg_pos[0], period) + feedforward_force;
  cmd.torque = angle_pid.computeCommand(-angles::shortest_angular_distance(desired_leg_angle, leg_pos[1]), period);
  leg_conv(cmd.force, cmd.torque, leg_angle[0], leg_angle[1], cmd.input);
  return cmd;
}
[[maybe_unused]] inline LegCommand computePidLenLegCommand(double desired_length, double leg_pos[2],
                                                           control_toolbox::Pid& length_pid, const double* leg_angle,
                                                           const ros::Duration& period, double feedforward_force = 0.0f)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };
  cmd.force = length_pid.computeCommand(desired_length - leg_pos[0], period) + feedforward_force;
  leg_conv(cmd.force, 0.0, leg_angle[0], leg_angle[1], cmd.input);
  return cmd;
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

}  // namespace rm_chassis_controllers
