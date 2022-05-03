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
  robot_hw_interface = robot_hw;
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (!controller_nh.getParam("gpio_control", xml_rpc_value))
  {
    ROS_WARN("No gpio_control specified");
  }
  else
  {
    parseGpioData(xml_rpc_value, controller_nh);
  }

  for (unsigned i = 0; i < gpio_read_handles_.size(); i++)
  {
    ROS_DEBUG("Got read_gpio %s", gpio_read_handles_[i].getName().c_str());
    gpio_read_array.GpioReads[i].gpio_name = gpio_read_handles_[i].getName();
    gpio_read_array.GpioReads[i].gpio_state = gpio_read_handles_[i].getValue();
  }

  for (unsigned i = 0; i < gpio_write_handles_.size(); i++)
    ROS_DEBUG("Got write_gpio %s", gpio_write_handles_[i].getName().c_str());

  // realtime publisher
  gpio_pubs_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GpioReadArray>(controller_nh, "gpios/data", 100));

  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::GpioWrite>("command", 1, &Controller::setGpioCmd, this);
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  std_msgs::Header header;
  header.stamp = time;
  for (unsigned i = 0; i < gpio_read_handles_.size(); i++)
  {
    gpio_read_array.GpioReads[i].gpio_name = gpio_read_handles_[i].getName();
    gpio_read_array.GpioReads[i].gpio_state = gpio_read_handles_[i].getValue();
  }
  gpio_read_array.header = header;
  gpio_pubs_->unlockAndPublish();
}

bool Controller::parseGpioData(XmlRpc::XmlRpcValue& gpio_datas, ros::NodeHandle& robot_controller_nh)
{
  for (auto it = gpio_datas.begin(); it != gpio_datas.end(); ++it)
  {
    std::string gpioType = it->second;
    if (!it->second.hasMember("out") && !it->second.hasMember("in"))
    {
      ROS_ERROR_STREAM("Gpio " << it->first << " has no gpio_type.");
      continue;
    }
    if (gpioType.compare("in"))
    {
      rm_control::GpioReadHandle read_handle_ =
          robot_hw_interface->get<rm_control::GpioReadInterface>()->getHandle(it->first);
      gpio_read_handles_.push_back(read_handle_);
    }
    if (gpioType.compare("out"))
    {
      rm_control::GpioWriteHandle write_handle_ =
          robot_hw_interface->get<rm_control::GpioWriteInterface>()->getHandle(it->first);
      gpio_write_handles_.push_back(write_handle_);
    }
  }
}

void Controller::setGpioCmd(const rm_msgs::GpioWriteConstPtr& msg)
{
  for (auto it = gpio_write_handles_.begin(); it != gpio_write_handles_.end(); ++it)
  {
    if (it->getName().compare(msg->gpio_name))
    {
      //      it->setCommand(msg->gpio_state);
      return;
    }
  }
  ROS_WARN("Not this %s", msg->gpio_name.c_str());
}

}  // namespace gpio_controller

PLUGINLIB_EXPORT_CLASS(gpio_controller::Controller, controller_interface::ControllerBase)