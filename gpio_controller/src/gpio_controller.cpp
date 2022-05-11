//
// Created by muyuexin on 2022/4/29.
//

#include <rm_common/ros_utilities.h>
#include "gpio_controller/gpio_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace gpio_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  controller_nh.getParam("gpios", xml_rpc_value);

  ROS_DEBUG_STREAM("xml_rpc_value:" << xml_rpc_value);
  for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
  {
    ROS_ASSERT(it->second.hasMember("type"));
    std::string gpioType = it->second["type"];
    std::string gpioName = it->first;

    if (gpioType.compare("in") == 0)
    {
      rm_control::GpioReadHandle read_handle_ = robot_hw->get<rm_control::GpioReadInterface>()->getHandle(gpioName);
      gpio_read_handles_.push_back(read_handle_);
    }
    else if (gpioType.compare("out") == 0)
    {
      rm_control::GpioWriteHandle write_handle_ = robot_hw->get<rm_control::GpioWriteInterface>()->getHandle(gpioName);
      gpio_write_handles_.push_back(write_handle_);
    }
    else
    {
      ROS_WARN_STREAM("Gpio " << it->first << " has no gpio_type.");
      continue;
    }
  }

  for (unsigned i = 0; i < gpio_read_handles_.size(); i++)
  {
    ROS_DEBUG("Got read_gpio %s", gpio_read_handles_[i].getName().c_str());
  }
  for (unsigned i = 0; i < gpio_write_handles_.size(); i++)
    ROS_DEBUG("Got write_gpio %s", gpio_write_handles_[i].getName().c_str());

  // realtime publisher
  gpio_pubs_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GpioData>(controller_nh, "data", 100));

  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::GpioData>("gpio_command", 1, &Controller::setGpioCmd, this);
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned i = 0; i < gpio_read_handles_.size(); i++)
  {
    gpio_pubs_->msg_.header[i].stamp = time;
    gpio_pubs_->msg_.gpio_name[i] = gpio_read_handles_[i].getName();
    gpio_pubs_->msg_.gpio_state[i] = gpio_read_handles_[i].getValue();
  }
  if (gpio_read_handles_.size() != 0)
  {
    gpio_pubs_->unlockAndPublish();
  }
}

void Controller::setGpioCmd(const rm_msgs::GpioDataConstPtr& msg)
{
  for (unsigned i = 0; i < gpio_write_handles_.size(); i++)
  {
    for (unsigned j = 0; msg->gpio_name.size(); j++)
    {
      if ((msg->gpio_name[j].compare(gpio_write_handles_[i].getName())))
      {
        gpio_write_handles_[i].setCommand(msg->gpio_state[j]);
        return;
      }
    }
    ROS_WARN("Not this %s", msg->gpio_name[i].c_str());
    return;
  }
}

}  // namespace gpio_controller

PLUGINLIB_EXPORT_CLASS(gpio_controller::Controller, controller_interface::ControllerBase)