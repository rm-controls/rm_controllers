//
// Created by yezi on 2021/11/11.
//

#pragma once

#include <imu_filter_controllers/imu_filter_base.h>
#include <imu_filter_madgwick/imu_filter.h>

namespace imu_filter_controllers
{
class MadgwickController : public ImuFilterBase
{
public:
  MadgwickController() = default;

private:
  bool getFilterParam(ros::NodeHandle& controller_nh) override;
  void filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt) override;
  void getOrientation(double& q0, double& q1, double& q2, double& q3) override;
  // Parameters:
  WorldFrame::WorldFrame world_frame_;
  bool use_mag_;
  geometry_msgs::Vector3 mag_bias_;

  ImuFilter filter_;
};

}  // namespace imu_filter_controllers
