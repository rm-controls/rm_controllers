//
// Created by luotinkai on 2022/1/2.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/hardware_interface/tof_sensor_interface.h>
#include <rm_msgs/TofSensor.h>

namespace tof_sensor_controller
{
class Controller : public controller_interface::Controller<rm_control::TofSensorInterface>
{
public:
  Controller() = default;

  bool init(rm_control::TofSensorInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  rm_control::TofSensorHandle tof_sensor_handle;
  std::vector<rm_control::TofSensorHandle> tof_sensors_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::TofSensor>> RtpublisherPtr;
  RtpublisherPtr tof_pub_;
  std::vector<RtpublisherPtr> realtime_pubs_;
};
}  // namespace tof_sensor_controller
