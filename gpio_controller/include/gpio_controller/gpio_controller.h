//
// Created by myx on 2022/4/29.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/hardware_interface/gpio_interface.h>
#include <rm_msgs/GpioRead.h>
#include <rm_msgs/GpioReadArray.h>
#include <rm_msgs/GpioWrite.h>

namespace gpio_controller
{

class Controller
  : public controller_interface::MultiInterfaceController<rm_control::GpioReadInterface, rm_control::GpioWriteInterface>
{
public:
  Controller() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  bool parseGpioData(XmlRpc::XmlRpcValue& gpio_datas, ros::NodeHandle& robot_controller_nh);
  void setGpioCmd(const rm_msgs::GpioWriteConstPtr& msg);

  hardware_interface::RobotHW* robot_hw_interface;
  std::vector<rm_control::GpioReadHandle> gpio_read_handles_;
  std::vector<rm_control::GpioWriteHandle> gpio_write_handles_;
  rm_msgs::GpioReadArray gpio_read_array;
  ros::Subscriber cmd_subscriber_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GpioReadArray>> RtpublisherPtr;
  RtpublisherPtr gpio_pubs_;
};
}  // namespace gpio_controller