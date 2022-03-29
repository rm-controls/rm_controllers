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
class Controller : public controller_interface::MultiInterfaceController<rm_control::TofSensorInterface>
{
public:
  Controller() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  rm_control::TofSensorHandle tof_sensor_handle;

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::TofSensor>> tof_pub_;
};
}  // namespace tof_sensor_controller
