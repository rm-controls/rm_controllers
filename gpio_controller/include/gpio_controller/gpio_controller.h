//
// Created by myx on 2022/4/29.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/hardware_interface/gpio_interface.h>
#include <rm_msgs/GpioData.h>

namespace gpio_controller
{
class Controller : public controller_interface::MultiInterfaceController<rm_control::GpioStateInterface,
                                                                         rm_control::GpioCommandInterface>
{
public:
  Controller() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void setGpioCmd(const rm_msgs::GpioDataConstPtr& msg);

  std::vector<rm_control::GpioStateHandle> gpio_state_handles_;
  std::vector<rm_control::GpioCommandHandle> gpio_command_handles_;

  ros::Subscriber cmd_subscriber_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GpioData>> RtpublisherPtr;
  RtpublisherPtr gpio_state_pub_;
};
}  // namespace gpio_controller
