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
    //    ROS_ASSERT(it->second.hasMember("name"));
    std::string gpioName = it->second["name"];
    rm_control::GpioStateHandle state_handle_ = robot_hw->get<rm_control::GpioStateInterface>()->getHandle(gpioName);
    gpio_state_handles_.push_back(state_handle_);
    ROS_ERROR_STREAM("state_handle_.getType():" << state_handle_.getType());
    if (state_handle_.getType() == "out")
    {
      rm_control::GpioCommandHandle command_handle_ =
          robot_hw->get<rm_control::GpioCommandInterface>()->getHandle(gpioName);
      gpio_command_handles_.push_back(command_handle_);
    }
  }

  for (unsigned i = 0; i < gpio_state_handles_.size(); i++)
  {
    ROS_INFO("Got state_gpio %s", gpio_state_handles_[i].getName().c_str());
  }
  for (unsigned i = 0; i < gpio_command_handles_.size(); i++)
    ROS_INFO("Got command_gpio %s", gpio_command_handles_[i].getName().c_str());

  // realtime publisher
  gpio_pubs_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GpioData>(controller_nh, "gpio_state", 100));

  controller_nh.subscribe<rm_msgs::GpioData>("gpio_command", 1, &Controller::setGpioCmd, this);
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned i = 0; i < gpio_state_handles_.size(); i++)
  {
    ROS_INFO_STREAM("gpio Controller::update");
    ROS_INFO_STREAM(gpio_state_handles_[i].getName());
    ROS_INFO_STREAM(gpio_state_handles_[i].getValue());
    ROS_INFO_STREAM(gpio_state_handles_[i].getType());
    ROS_INFO_STREAM("gpio_pubs_->msg_.gpio_name[i]" << gpio_pubs_->msg_.gpio_name.back());
    ROS_INFO_STREAM("gpio_state_handles_[i].getName()" << gpio_state_handles_[i].getName());
    ROS_INFO_STREAM("llll");
    gpio_pubs_->msg_.gpio_name[i] = gpio_state_handles_[i].getName();
    gpio_pubs_->unlockAndPublish();
  }
}

void Controller::setGpioCmd(const rm_msgs::GpioDataConstPtr& msg)
{
  //  ROS_INFO("into setGpioCmd");
  //  for (unsigned i = 0; i < gpio_command_handles_.size(); i++)
  //  {
  //    for (unsigned j = 0; j < msg->gpio_name.size(); j++)
  //    {
  //      if ((msg->gpio_name[j].find(gpio_command_handles_[i].getName()) != std::string::npos))
  //      {
  //        ROS_INFO("setGpio_0");
  //        gpio_command_handles_[i].setCommand(msg->gpio_state[j]);
  //      }
  //    }
  //  }
  return;
}

}  // namespace gpio_controller

PLUGINLIB_EXPORT_CLASS(gpio_controller::Controller, controller_interface::ControllerBase)
