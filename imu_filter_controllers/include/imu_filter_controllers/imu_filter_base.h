//
// Created by yezi on 2021/11/10.
//

#pragma once

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <rm_common/hardware_interface/imu_extra_interface.h>

namespace imu_filter_controllers
{
class ImuFilterBase : public controller_interface::MultiInterfaceController<hardware_interface::ImuSensorInterface,
                                                                            rm_control::ImuExtraInterface>
{
public:
  ImuFilterBase() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

protected:
  virtual void filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt) = 0;
  virtual void getOrientation(double& q0, double& q1, double& q2, double& q3) = 0;
  virtual bool getFilterParam(ros::NodeHandle& controller_nh) = 0;
  ros::Time last_update_;
  bool initialized_filter_{ false };
  // hardware_interface
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  rm_control::ImuExtraHandle imu_extra_handle_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu> > imu_data_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Temperature> > imu_temp_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::TimeReference> > trigger_time_pub_;
};
}  // namespace imu_filter_controllers
