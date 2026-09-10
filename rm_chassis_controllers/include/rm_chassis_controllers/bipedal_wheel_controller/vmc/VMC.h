//
// Created by wk on 2026/2/25.
//
#pragma once

#include <memory>

namespace rm_chassis_controllers
{
struct LegPos
{
  double L0;     // Leg length
  double theta;  // Leg angle
};

struct LegSpd
{
  double dL0;     // Leg length rate
  double dTheta;  // Leg angle rate
};

struct LegForce
{
  double F;   // Force along leg length (radial)
  double Tp;  // Torque/Force corresponding to leg angle (tangential)
};

class VMC
{
public:
  explicit VMC(double l1, double l2, double l5 = 0) : l1_(l1), l2_(l2), l3_(l2), l4_(l1), l5_(l5){};
  ~VMC() = default;

  /**
   * @brief Calculate the leg position (length and angle) based on joint angles.
   *
   * This function computes the forward kinematics to find the end-effector position
   * in polar coordinates (length L0 and angle theta) and updates the internal position state.
   *
   * @param phi1 The first joint angle (e.g., hip/thigh joint).
   * @param phi4 The second joint angle (e.g., knee/calf joint), possibly relative or absolute depending on mechanism.
   */
  void leg_pos(double phi1, double phi4);

  /**
   * @brief Calculate the leg velocity (length rate and angle rate) based on joint velocities.
   *
   * This function computes the end-effector velocity in polar coordinates
   * by mapping joint velocities through the Jacobian and updates the internal velocity state.
   *
   * @param dphi1 Velocity of the first joint.
   * @param dphi4 Velocity of the second joint.
   */
  void leg_spd(double dphi1, double dphi4);

  /**
   * @brief Convert Cartesian forces/torques to joint torques.
   *
   * This function maps forces acting on the leg end-effector (in polar space)
   * to the required joint torques using the transpose of the Jacobian.
   *
   * @param F  Force along the leg length (radial force).
   * @param Tp Torque/Force corresponding to the leg angle (tangential force/torque).
   * @param T  Output array where T[0] is the torque for joint 1 and T[1] is the torque for joint 2.
   */
  void leg_conv(double F, double Tp, double T[2]);

  /**
   * @brief Convert joint torques back to Cartesian/Virtual forces.
   *
   * This function performs the inverse mapping of leg_conv, converting the torques
   * applied at the joints (T1, T2) into the equivalent virtual forces acting on
   * the leg end-effector in polar coordinates, and updates the internal force state.
   *
   * @param T1 Torque applied to the first joint (e.g., hip/thigh).
   * @param T2 Torque applied to the second joint (e.g., knee/calf).
   */
  void leg_conv_t(double T1, double T2);

  /**
   * @brief Calculate and update the internal Jacobian matrix of the leg mechanism.
   *
   * The Jacobian relates joint velocities to end-effector velocities.
   * This function computes the 2x2 Jacobian matrix for the current joint configuration
   * and stores it internally for subsequent velocity or force conversions.
   *
   * @param phi1 Current position of the first joint.
   * @param phi4 Current position of the second joint.
   */
  void calc_jacobian(double phi1, double phi4);

  inline double getL1() const
  {
    return l1_;
  }
  inline double getL2() const
  {
    return l2_;
  }
  inline double getL3() const
  {
    return l3_;
  }
  inline double getL4() const
  {
    return l4_;
  }
  inline double getL5() const
  {
    return l5_;
  }
  inline const LegPos& getPos() const
  {
    return pos_;
  }
  inline const LegSpd& getSpd() const
  {
    return spd_;
  }
  inline const LegForce getForceReal() const
  {
    return force_real_;
  }

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

  double J_[2][2];
  double phi1_, phi4_;
  double dphi1_, dphi4_;
  LegPos pos_;
  LegSpd spd_;
  LegForce force_real_;

  double l1_, l2_, l3_, l4_, l5_;
};
using VMCPtr = std::shared_ptr<VMC>;
}  // namespace rm_chassis_controllers
