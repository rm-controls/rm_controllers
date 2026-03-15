//
// Created by wk on 2026/2/25.
//
#pragma once

#include <memory>

namespace rm_chassis_controllers
{
class VMC
{
public:
  explicit VMC(double l1, double l2, double l5 = 0) : l1_(l1), l2_(l2), l3_(l2), l4_(l1), l5_(l5){};
  ~VMC() = default;

  /**
   * @brief Calculate the leg position (length and angle) based on joint angles.
   *
   * This function computes the forward kinematics to find the end-effector position
   * in polar coordinates (length L0 and angle Phi0).
   *
   * @param phi1 The first joint angle (e.g., hip/thigh joint).
   * @param phi4 The second joint angle (e.g., knee/calf joint), possibly relative or absolute depending on mechanism.
   * @param pos  Output array where pos[0] is the length (L0) and pos[1] is the angle (Phi0).
   */
  void leg_pos(double phi1, double phi4, double pos[2]) const;

  /**
   * @brief Calculate the leg velocity (length rate and angle rate) based on joint velocities.
   *
   * This function computes the end-effector velocity in polar coordinates
   * by mapping joint velocities through the Jacobian.
   *
   * @param dphi1 Velocity of the first joint.
   * @param dphi4 Velocity of the second joint.
   * @param phi1  Current position of the first joint.
   * @param phi4  Current position of the second joint.
   * @param spd   Output array where spd[0] is linear velocity (dL0) and spd[1] is angular velocity (dPhi0).
   */
  void leg_spd(double dphi1, double dphi4, double phi1, double phi4, double spd[2]);

  /**
   * @brief Convert Cartesian forces/torques to joint torques.
   *
   * This function maps forces acting on the leg end-effector (in polar space)
   * to the required joint torques using the transpose of the Jacobian.
   *
   * @param F    Force along the leg length (radial force).
   * @param Tp   Torque/Force corresponding to the leg angle (tangential force/torque).
   * @param phi1 Current position of the first joint.
   * @param phi4 Current position of the second joint.
   * @param T    Output array where T[0] is the torque for joint 1 and T[1] is the torque for joint 2.
   */
  void leg_conv(double F, double Tp, double phi1, double phi4, double T[2]);

private:
  /**
   * @brief Calculate the Jacobian matrix of the leg mechanism.
   *
   * The Jacobian J relates joint velocities to end-effector velocities:
   * [v_L; v_phi] = J * [dphi1; dphi4]
   *
   * @param phi1 Current position of the first joint.
   * @param phi4 Current position of the second joint.
   * @param J    Output 2x2 Jacobian matrix.
   */
  void calc_jacobian(double phi1, double phi4, double J[2][2]);

  double l1_, l2_, l3_, l4_, l5_;
};
using VMCPtr = std::shared_ptr<VMC>;
}  // namespace rm_chassis_controllers
