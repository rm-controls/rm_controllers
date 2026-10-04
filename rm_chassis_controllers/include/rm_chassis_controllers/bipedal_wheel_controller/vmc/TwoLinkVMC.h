#pragma once

#include "bipedal_wheel_controller/vmc/VMC.h"

namespace rm_chassis_controllers
{
class TwoLinkVMC final : public VMC
{
public:
  TwoLinkVMC(double l1, double l2);

  void leg_pos(double phi1, double phi4) override;
  void calc_jacobian(double phi1, double phi4) override;
};
}  // namespace rm_chassis_controllers
