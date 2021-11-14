//
// Created by yezi on 2021/11/11.
//

#include <imu_filter_controllers/madgwick_controller.h>

#include <pluginlib/class_list_macros.hpp>

namespace imu_filter_controllers
{
bool MadgwickController::getFilterParam(ros::NodeHandle& controller_nh)
{
  double gain;
  double zeta;
  std::string world_frame;
  controller_nh.param("gain", gain, 0.1);
  controller_nh.param("zeta", zeta, 0.);
  controller_nh.param("mag_bias_x", mag_bias_.x, 0.);
  controller_nh.param("mag_bias_y", mag_bias_.y, 0.);
  controller_nh.param("mag_bias_z", mag_bias_.z, 0.);
  controller_nh.param<std::string>("world_frame", world_frame, "enu");
  controller_nh.param("use_mag", use_mag_, false);
  filter_.setAlgorithmGain(gain);
  filter_.setDriftBiasGain(zeta);
  if (world_frame == "ned")
  {
    world_frame_ = WorldFrame::NED;
  }
  else if (world_frame == "nwu")
  {
    world_frame_ = WorldFrame::NWU;
  }
  else if (world_frame == "enu")
  {
    world_frame_ = WorldFrame::ENU;
  }
  else
  {
    ROS_ERROR("The parameter world_frame was set to invalid value '%s'.", world_frame.c_str());
    ROS_ERROR("Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
    world_frame_ = WorldFrame::ENU;
  }
  filter_.setWorldFrame(world_frame_);
  return true;
}

void MadgwickController::filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt)
{
  filter_.madgwickAHRSupdateIMU(wx, wy, wz, ax, ay, az, dt);
}

void MadgwickController::getOrientation(double& q0, double& q1, double& q2, double& q3)
{
  filter_.getOrientation(q0, q1, q2, q3);
}

}  // namespace imu_filter_controllers

PLUGINLIB_EXPORT_CLASS(imu_filter_controllers::MadgwickController, controller_interface::ControllerBase)
