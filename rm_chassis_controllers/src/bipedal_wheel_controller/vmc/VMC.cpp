#include "bipedal_wheel_controller/vmc/VMC.h"

#include <cmath>

namespace rm_chassis_controllers
{
VMC::VMC(double l1, double l2, double l3, double l4, double l5) : l1_(l1), l2_(l2), l3_(l3), l4_(l4), l5_(l5)
{
}

void VMC::leg_spd(double dphi1, double dphi4)
{
  const double dL0 = J_[0][0] * dphi1 + J_[0][1] * dphi4;
  const double dTheta = J_[1][0] * dphi1 + J_[1][1] * dphi4;

  if (!speed_filter_initialized_)
  {
    filtered_dL0_ = dL0;
    filtered_dTheta_ = dTheta;
    speed_filter_initialized_ = true;
  }
  else
  {
    filtered_dL0_ = 0.4 * dL0 + 0.6 * filtered_dL0_;
    filtered_dTheta_ = 0.4 * dTheta + 0.6 * filtered_dTheta_;
  }

  spd_.dL0 = filtered_dL0_;
  spd_.dTheta = filtered_dTheta_;
}

void VMC::leg_conv(double F, double Tp, double T[2]) const
{
  T[0] = J_[0][0] * F + J_[1][0] * Tp;
  T[1] = J_[0][1] * F + J_[1][1] * Tp;
}

void VMC::leg_conv_t(double T1, double T2)
{
  const double det = J_[0][0] * J_[1][1] - J_[0][1] * J_[1][0];
  if (std::abs(det) < 1e-8)
  {
    force_real_ = {};
    return;
  }

  force_real_.F = (J_[1][1] * T1 - J_[1][0] * T2) / det;
  force_real_.Tp = (-J_[0][1] * T1 + J_[0][0] * T2) / det;
}

void VMC::clearJacobian()
{
  J_[0][0] = J_[0][1] = 0.0;
  J_[1][0] = J_[1][1] = 0.0;
}

double VMC::getL1() const
{
  return l1_;
}

double VMC::getL2() const
{
  return l2_;
}

double VMC::getL3() const
{
  return l3_;
}

double VMC::getL4() const
{
  return l4_;
}

double VMC::getL5() const
{
  return l5_;
}

const LegPos& VMC::getPos() const
{
  return pos_;
}

const LegSpd& VMC::getSpd() const
{
  return spd_;
}

const LegForce& VMC::getForceReal() const
{
  return force_real_;
}
}  // namespace rm_chassis_controllers
