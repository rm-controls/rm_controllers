#pragma once

#include <imu_filter_controllers/imu_filter_base.h>
#include <imu_complementary_filter/complementary_filter.h>

namespace imu_filter_controllers
{
class ComplementaryController : public ImuFilterBase
{
public:
  ComplementaryController() = default;

private:
  void filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt) override;
  void getOrientation(double& q0, double& q1, double& q2, double& q3) override;
  bool getFilterParam(ros::NodeHandle& controller_nh) override;
  // Parameters:
  bool use_mag_;
  bool publish_debug_topics_;
  // State variables:
  imu_tools::ComplementaryFilter filter_;
};

}  // namespace imu_filter_controllers
