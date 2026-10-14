#include "bipedal_wheel_controller/vmc/TwoLinkVMC.h"

#include <cmath>

namespace rm_chassis_controllers
{
TwoLinkVMC::TwoLinkVMC(double l1, double l2) : VMC(l1, l2, l2, l1, 0.0)
{
}

void TwoLinkVMC::leg_pos(double phi1, double phi4)
{
  const double phi2 = phi1 + phi4;
  const double x = l1_ * std::cos(phi1) + l2_ * std::cos(phi2);
  const double y = l1_ * std::sin(phi1) + l2_ * std::sin(phi2);
  pos_.L0 = std::hypot(x, y);
  pos_.theta = std::atan2(y, x);
}

void TwoLinkVMC::calc_jacobian(double phi1, double phi4)
{
  const double phi2 = phi1 + phi4;
  const double c1 = std::cos(phi1);
  const double s1 = std::sin(phi1);
  const double c2 = std::cos(phi2);
  const double s2 = std::sin(phi2);
  const double x = l1_ * c1 + l2_ * c2;
  const double y = l1_ * s1 + l2_ * s2;
  const double length = std::hypot(x, y);

  if (length < 1e-8)
  {
    clearJacobian();
    return;
  }

  const double dx_dphi1 = -l1_ * s1 - l2_ * s2;
  const double dy_dphi1 = l1_ * c1 + l2_ * c2;
  const double dx_dphi4 = -l2_ * s2;
  const double dy_dphi4 = l2_ * c2;
  const double inv_length = 1.0 / length;
  const double inv_length_sq = inv_length * inv_length;

  J_[0][0] = (x * dx_dphi1 + y * dy_dphi1) * inv_length;
  J_[0][1] = (x * dx_dphi4 + y * dy_dphi4) * inv_length;
  J_[1][0] = (x * dy_dphi1 - y * dx_dphi1) * inv_length_sq;
  J_[1][1] = (x * dy_dphi4 - y * dx_dphi4) * inv_length_sq;
}
}  // namespace rm_chassis_controllers
