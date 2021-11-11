#include "imu_filter_controllers/complementary_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace imu_filter_controllers
{
bool ComplementaryController::getFilterParam(ros::NodeHandle& controller_nh)
{
  double gain_acc;
  double gain_mag;
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;
  controller_nh.param("use_mag", use_mag_, false);
  controller_nh.param("gain_acc", gain_acc, 0.01);
  controller_nh.param("gain_mag", gain_mag, 0.01);
  controller_nh.param("do_bias_estimation", do_bias_estimation, true);
  controller_nh.param("bias_alpha", bias_alpha, 0.01);
  controller_nh.param("do_adaptive_gain", do_adaptive_gain, true);
  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setDoAdaptiveGain(do_adaptive_gain);
  if (!filter_.setGainAcc(gain_acc))
    ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_)
  {
    if (!filter_.setGainMag(gain_mag))
      ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation)
  {
    if (!filter_.setBiasAlpha(bias_alpha))
      ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
  }
  return true;
}

void ComplementaryController::filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt)
{
  filter_.update(ax, ay, az, wx, wy, wz, dt);
}

void ComplementaryController::getOrientation(double& q0, double& q1, double& q2, double& q3)
{
  filter_.getOrientation(q0, q1, q2, q3);
}
}  // namespace imu_filter_controllers

PLUGINLIB_EXPORT_CLASS(imu_filter_controllers::ComplementaryController, controller_interface::ControllerBase)
