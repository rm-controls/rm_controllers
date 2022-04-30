//
// Created by myx on 2022/4/29.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/hardware_interface/gpio_interface.h>
#include <rm_msgs/GpioRead.h>
#include <rm_msgs/GpioWrite.h>

namespace gpio_controllers
{
class Controller
  : public controller_interface::MultiInterfaceController<rm_control::GpioReadInterface, rm_control::GpioWriteInterface>
{
public:
  Controller() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  rm_control::GpioReadHandle gpio_read_handle;
  rm_control::GpioWriteHandle gpio_write_handle;
  std::vector<rm_control::GpioReadHandle> gpio_read_;
  std::vector<rm_control::GpioWriteHandle> gpio_write_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GpioRead>> RtpublisherPtr;
  RtpublisherPtr gpio_pub_;
  std::vector<RtpublisherPtr> realtime_pubs_;
};
}  // namespace gpio_controllers
