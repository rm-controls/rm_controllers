#include "bipedal_wheel_controller/vmc/FiveLinkVMC.h"

#include <algorithm>
#include <cmath>

namespace rm_chassis_controllers
{
FiveLinkVMC::FiveLinkVMC(double l1, double l2, double l3, double l4, double l5) : VMC(l1, l2, l3, l4, l5)
{
}

FiveLinkVMC::FiveLinkVMC(double l1, double l2, double l5) : FiveLinkVMC(l1, l2, l2, l1, l5)
{
}

bool FiveLinkVMC::solveKinematics(double phi1, double phi4, Kinematics& result) const
{
  const double xb = l1_ * std::cos(phi1);
  const double yb = l1_ * std::sin(phi1);
  const double xd = l5_ + l4_ * std::cos(phi4);
  const double yd = l4_ * std::sin(phi4);
  const double dx = xd - xb;
  const double dy = yd - yb;
  const double bd_sq = dx * dx + dy * dy;
  const double a = 2.0 * l2_ * dx;
  const double b = 2.0 * l2_ * dy;
  const double c = l2_ * l2_ + bd_sq - l3_ * l3_;
  double discriminant = a * a + b * b - c * c;

  constexpr double tolerance = 1e-12;
  if (discriminant < -tolerance)
    return false;
  discriminant = std::max(0.0, discriminant);

  result.phi2 = 2.0 * std::atan2(b + std::sqrt(discriminant), a + c);
  result.phi3 = std::atan2(yb - yd + l2_ * std::sin(result.phi2), xb - xd + l2_ * std::cos(result.phi2));

  const double xc = xb + l2_ * std::cos(result.phi2);
  const double yc = yb + l2_ * std::sin(result.phi2);
  result.x = xc - l5_ / 2.0;
  result.y = yc;
  result.length = std::hypot(result.x, result.y);
  result.polar_angle = std::atan2(result.y, result.x);
  return std::isfinite(result.phi2) && std::isfinite(result.phi3) && std::isfinite(result.length) &&
         result.length >= 1e-8;
}

void FiveLinkVMC::leg_pos(double phi1, double phi4)
{
  Kinematics kinematics{};
  if (!solveKinematics(phi1, phi4, kinematics))
  {
    pos_ = {};
    return;
  }

  pos_.L0 = kinematics.length;
  // Five-link leg angle is measured from the positive Y axis.
  pos_.theta = -std::atan2(kinematics.x, kinematics.y);
}

void FiveLinkVMC::calc_jacobian(double phi1, double phi4)
{
  Kinematics kinematics{};
  if (!solveKinematics(phi1, phi4, kinematics))
  {
    clearJacobian();
    return;
  }

  const double denominator = std::sin(kinematics.phi3 - kinematics.phi2);
  if (std::abs(denominator) < 1e-8)
  {
    clearJacobian();
    return;
  }

  J_[0][0] = l1_ * std::sin(kinematics.polar_angle - kinematics.phi3) * std::sin(phi1 - kinematics.phi2) / denominator;
  J_[0][1] = l4_ * std::sin(kinematics.polar_angle - kinematics.phi2) * std::sin(kinematics.phi3 - phi4) / denominator;
  J_[1][0] = l1_ * std::cos(kinematics.polar_angle - kinematics.phi3) * std::sin(phi1 - kinematics.phi2) /
             (kinematics.length * denominator);
  J_[1][1] = l4_ * std::cos(kinematics.polar_angle - kinematics.phi2) * std::sin(kinematics.phi3 - phi4) /
             (kinematics.length * denominator);
}
}  // namespace rm_chassis_controllers
