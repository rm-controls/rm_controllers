#include "imu_filter_controllers/complementary_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace imu_filter_controllers
{
bool ComplementaryController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  initialized_filter_ = false;
  std::string name;
  double gain_acc;
  double gain_mag;
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;

  if (!controller_nh.getParam("name", name))
  {
    ROS_ERROR("Name was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("use_mag", use_mag_))
    use_mag_ = false;
  if (!controller_nh.getParam("publish_debug_topics", publish_debug_topics_))
    publish_debug_topics_ = false;
  if (!controller_nh.getParam("gain_acc", gain_acc))
    gain_acc = 0.01;
  if (!controller_nh.getParam("gain_mag", gain_mag))
    gain_mag = 0.01;
  if (!controller_nh.getParam("do_bias_estimation", do_bias_estimation))
    do_bias_estimation = true;
  if (!controller_nh.getParam("bias_alpha", bias_alpha))
    bias_alpha = 0.01;
  if (!controller_nh.getParam("do_adaptive_gain", do_adaptive_gain))
    do_adaptive_gain = true;

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
  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(name);
  imu_extra_handle_ = robot_hw->get<rm_control::ImuExtraInterface>()->getHandle(name);

  imu_data_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(controller_nh, "imu_data", 100));
  imu_temp_pub_.reset(
      new realtime_tools::RealtimePublisher<sensor_msgs::Temperature>(controller_nh, "imu_temperature", 100));
  trigger_time_pub_.reset(
      new realtime_tools::RealtimePublisher<sensor_msgs::TimeReference>(controller_nh, "imu_trigger_time", 100));

  return true;
}

void ComplementaryController::update(const ros::Time& time, const ros::Duration& period)
{
  if (!imu_extra_handle_.getGyroUpdated() || !imu_extra_handle_.getAccelUpdated())
    return;
  // Initialize.
  if (!initialized_filter_)
  {
    initialized_filter_ = true;
    last_update_ = time;
    imu_data_pub_->msg_.header.frame_id = imu_sensor_handle_.getFrameId();
    return;
  }
  const double* a = imu_sensor_handle_.getLinearAcceleration();
  const double* w = imu_sensor_handle_.getAngularVelocity();

  // Update the filter.
  filter_.update(a[0], a[1], a[2], w[0], w[1], w[2], (time - last_update_).toSec());
  last_update_ = time;
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  tf::Quaternion q(q1, q2, q3, q0);
  geometry_msgs::Quaternion quat;
  tf::quaternionTFToMsg(q, quat);
  imu_extra_handle_.setOrientation(quat.x, quat.y, quat.z, quat.w);
  if (imu_extra_handle_.getCameraTrigger())
  {
    if (imu_data_pub_->trylock())
    {
      imu_data_pub_->msg_.header.stamp = time;
      imu_data_pub_->msg_.orientation.x = quat.x;
      imu_data_pub_->msg_.orientation.y = quat.y;
      imu_data_pub_->msg_.orientation.z = quat.z;
      imu_data_pub_->msg_.orientation.w = quat.w;
      imu_temp_pub_->msg_.temperature = imu_extra_handle_.getTemperature();
      trigger_time_pub_->msg_.time_ref = time;
      imu_data_pub_->unlockAndPublish();
    }
  }
}

}  // namespace imu_filter_controllers

PLUGINLIB_EXPORT_CLASS(imu_filter_controllers::ComplementaryController, controller_interface::ControllerBase)
