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
  ROS_ASSERT(xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // realtime publisher
  gpio_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GpioData>(controller_nh, "gpio_states", 100));

  for (int i = 0; i < xml_rpc_value.size(); ++i)
  {
    ROS_ASSERT(xml_rpc_value[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string gpioName = xml_rpc_value[i];
    rm_control::GpioStateHandle state_handle_ = robot_hw->get<rm_control::GpioStateInterface>()->getHandle(gpioName);
    gpio_state_handles_.push_back(state_handle_);
    gpio_state_pub_->msg_.gpio_name.push_back(state_handle_.getName());
    gpio_state_pub_->msg_.gpio_state.push_back(state_handle_.getValue());
    if (state_handle_.getType() == rm_control::OUTPUT)
    {
      gpio_state_pub_->msg_.gpio_type.push_back("out");
    }
    else
    {
      gpio_state_pub_->msg_.gpio_type.push_back("in");
    }
    ROS_INFO("Got state_gpio %s", gpioName.c_str());
    if (state_handle_.getType() == rm_control::OUTPUT)
    {
      rm_control::GpioCommandHandle command_handle_ =
          robot_hw->get<rm_control::GpioCommandInterface>()->getHandle(gpioName);
      gpio_command_handles_.push_back(command_handle_);
      ROS_INFO("Got command_gpio %s", gpioName.c_str());
    }
  }

  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::GpioData>("command", 1, &Controller::setGpioCmd, this);
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  if (gpio_state_pub_->trylock())
  {
    for (unsigned i = 0; i < gpio_state_handles_.size(); i++)
    {
      gpio_state_pub_->msg_.gpio_state[i] = gpio_state_handles_[i].getValue();
    }
    gpio_state_pub_->msg_.header.stamp = time;
    gpio_state_pub_->unlockAndPublish();
  }
}

void Controller::setGpioCmd(const rm_msgs::GpioDataConstPtr& msg)
{
  for (unsigned i = 0; i < gpio_command_handles_.size(); i++)
  {
    for (unsigned j = 0; j < msg->gpio_name.size(); j++)
    {
      if ((msg->gpio_name[j].find(gpio_command_handles_[i].getName()) != std::string::npos))
      {
        gpio_command_handles_[i].setCommand(msg->gpio_state[j]);
      }
    }
  }
  return;
}

}  // namespace gpio_controller

PLUGINLIB_EXPORT_CLASS(gpio_controller::Controller, controller_interface::ControllerBase)
