//
// Created by yezi on 2021/11/10.
//

#include <imu_filter_controllers/imu_filter_base.h>

namespace imu_filter_controllers
{
bool ImuFilterBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  std::string name;
  if (!controller_nh.getParam("name", name))
  {
    ROS_ERROR("Name was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  getFilterParam(controller_nh);
  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(name);
  imu_extra_handle_ = robot_hw->get<rm_control::ImuExtraInterface>()->getHandle(name);

  imu_data_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(controller_nh, name + "/data", 100));
  imu_temp_pub_.reset(
      new realtime_tools::RealtimePublisher<sensor_msgs::Temperature>(controller_nh, name + "/temperature", 100));
  trigger_time_pub_.reset(
      new realtime_tools::RealtimePublisher<sensor_msgs::TimeReference>(controller_nh, name + "/trigger_time", 100));
  return true;
}
void ImuFilterBase::update(const ros::Time& time, const ros::Duration& period)
{
  if (!imu_extra_handle_.getGyroUpdated() || !imu_extra_handle_.getAccelUpdated())
    return;
  // For first update.
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
  filterUpdate(a[0], a[1], a[2], w[0], w[1], w[2], (time - last_update_).toSec());
  last_update_ = time;
  double q0, q1, q2, q3;
  getOrientation(q0, q1, q2, q3);
  tf::Quaternion q(q1, q2, q3, q0);
  geometry_msgs::Quaternion quat;
  tf::quaternionTFToMsg(q, quat);
  imu_extra_handle_.setOrientation(quat.x, quat.y, quat.z, quat.w);
  if (imu_extra_handle_.getCameraTrigger())
  {
    if (imu_data_pub_->trylock())
    {
      imu_data_pub_->msg_.header.stamp = time;
      imu_data_pub_->msg_.angular_velocity.x = imu_sensor_handle_.getAngularVelocity()[0];
      imu_data_pub_->msg_.angular_velocity.y = imu_sensor_handle_.getAngularVelocity()[1];
      imu_data_pub_->msg_.angular_velocity.z = imu_sensor_handle_.getAngularVelocity()[2];
      imu_data_pub_->msg_.linear_acceleration.x = imu_sensor_handle_.getLinearAcceleration()[0];
      imu_data_pub_->msg_.linear_acceleration.y = imu_sensor_handle_.getLinearAcceleration()[1];
      imu_data_pub_->msg_.linear_acceleration.z = imu_sensor_handle_.getLinearAcceleration()[2];
      imu_data_pub_->msg_.orientation.x = quat.x;
      imu_data_pub_->msg_.orientation.y = quat.y;
      imu_data_pub_->msg_.orientation.z = quat.z;
      imu_data_pub_->msg_.orientation.w = quat.w;
      imu_data_pub_->msg_.orientation_covariance = { imu_sensor_handle_.getOrientationCovariance()[0], 0., 0., 0.,
                                                     imu_sensor_handle_.getOrientationCovariance()[4], 0., 0., 0.,
                                                     imu_sensor_handle_.getOrientationCovariance()[8] };
      imu_data_pub_->msg_.angular_velocity_covariance = {
        imu_sensor_handle_.getAngularVelocityCovariance()[0], 0., 0., 0.,
        imu_sensor_handle_.getAngularVelocityCovariance()[4], 0., 0., 0.,
        imu_sensor_handle_.getAngularVelocityCovariance()[8]
      };
      imu_data_pub_->msg_.linear_acceleration_covariance = {
        imu_sensor_handle_.getLinearAccelerationCovariance()[0], 0., 0., 0.,
        imu_sensor_handle_.getLinearAccelerationCovariance()[4], 0., 0., 0.,
        imu_sensor_handle_.getLinearAccelerationCovariance()[8]
      };
      imu_data_pub_->unlockAndPublish();
    }
    if (imu_temp_pub_->trylock())
    {
      imu_temp_pub_->msg_.header.stamp = time;
      imu_temp_pub_->msg_.temperature = imu_extra_handle_.getTemperature();
      imu_temp_pub_->unlockAndPublish();
    }
    if (trigger_time_pub_->trylock())
    {
      trigger_time_pub_->msg_.time_ref = time;
      trigger_time_pub_->unlockAndPublish();
    }
  }
}

}  // namespace imu_filter_controllers
