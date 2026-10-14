#pragma once

#include "bipedal_wheel_controller/vmc/VMC.h"

namespace rm_chassis_controllers
{
class FiveLinkVMC final : public VMC
{
public:
  FiveLinkVMC(double l1, double l2, double l3, double l4, double l5);
  FiveLinkVMC(double l1, double l2, double l5);

  void leg_pos(double phi1, double phi4) override;
  void calc_jacobian(double phi1, double phi4) override;

private:
  struct Kinematics
  {
    double phi2;
    double phi3;
    double x;
    double y;
    double length;
    double polar_angle;
  };

  bool solveKinematics(double phi1, double phi4, Kinematics& result) const;
};
}  // namespace rm_chassis_controllers
