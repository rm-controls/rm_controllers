//
// Created by muyuexin on 2022/4/29.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include "gpio_controller/gpio_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace gpio_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  gpio_read_handle = robot_hw->get<rm_control::GpioReadInterface>()->getHandle("");
  gpio_write_handle = robot_hw->get<rm_control::GpioWriteInterface>()->getHandle("");
  std::string read_gpio_names = gpio_read_handle.getName();
  const std::vector<std::string> read_gpio_names;
  for (unsigned i = 0; i < read_gpio_names.size(); i++)
    ROS_DEBUG("Got gpio %s", read_gpio_names[i].c_str());

  for (unsigned i = 0; i < read_gpio_names.size(); i++)
  {
    // sensor handle
    gpio_read_.push_back(robot_hw->get<rm_control::GpioReadHandle>()->getHandle(read_gpio_names[i]));

    // realtime publisher
    gpio_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::GpioRead>(controller_nh, read_gpio_names[i] + "/data", 100));
    realtime_pubs_.push_back(gpio_pub_);
  }
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned int i = 0; i < realtime_pubs_.size(); ++i)
  {
    if (realtime_pubs_[i]->trylock())
    {
      realtime_pubs_[i]->msg_.gpio_name = gpio_read_[i].getName();
      realtime_pubs_[i]->msg_.gpio_state = gpio_read_[i].getValue();
      realtime_pubs_[i]->msg_.stamp = time;
      realtime_pubs_[i]->unlockAndPublish();
    }
  }
}

}  // namespace gpio_controller

PLUGINLIB_EXPORT_CLASS(gpio_controller::Controller, controller_interface::ControllerBase)