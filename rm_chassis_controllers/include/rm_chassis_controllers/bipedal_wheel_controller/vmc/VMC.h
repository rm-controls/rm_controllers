//
// Created by wk on 2026/2/25.
//

namespace rm_chassis_controllers
{
class VMC
{
public:
  explicit VMC(double l1, double l2, double l5) : l1_(l1), l2_(l2), l3_(l2), l4_(l1), l5_(l5){};
  ~VMC() = default;

  void leg_pos(double phi1, double phi4, double pos[2]) const;
  void leg_spd(double dphi1, double dphi4, double phi1, double phi4, double spd[2]);
  void leg_conv(double F, double Tp, double phi1, double phi4, double T[2]);

private:
  void calc_jacobian(double phi1, double phi4, double J[2][2]);

  double l1_, l2_, l3_, l4_, l5_;
};
}  // namespace rm_chassis_controllers
