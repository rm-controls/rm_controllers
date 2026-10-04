#pragma once

#include <memory>

namespace rm_chassis_controllers
{
struct LegPos
{
  double L0{ 0.0 };
  double theta{ 0.0 };
};

struct LegSpd
{
  double dL0{ 0.0 };
  double dTheta{ 0.0 };
};

struct LegForce
{
  double F{ 0.0 };
  double Tp{ 0.0 };
};

class VMC
{
public:
  virtual ~VMC() = default;

  virtual void leg_pos(double phi1, double phi4) = 0;
  virtual void calc_jacobian(double phi1, double phi4) = 0;

  void leg_spd(double dphi1, double dphi4);
  void leg_conv(double F, double Tp, double T[2]) const;
  void leg_conv_t(double T1, double T2);

  double getL1() const;
  double getL2() const;
  double getL3() const;
  double getL4() const;
  double getL5() const;
  const LegPos& getPos() const;
  const LegSpd& getSpd() const;
  const LegForce& getForceReal() const;

protected:
  VMC(double l1, double l2, double l3, double l4, double l5);

  void clearJacobian();

  double J_[2][2]{};
  LegPos pos_;

  const double l1_;
  const double l2_;
  const double l3_;
  const double l4_;
  const double l5_;

private:
  LegSpd spd_;
  LegForce force_real_;
  double filtered_dL0_{ 0.0 };
  double filtered_dTheta_{ 0.0 };
  bool speed_filter_initialized_{ false };
};

using VMCPtr = std::shared_ptr<VMC>;
}  // namespace rm_chassis_controllers
