//
// Created by guanlin on 23-3-16.
//

#include "rm_calibration_controllers/gpio_calibration_base.h"

namespace rm_calibration_controllers
{
bool GpioCalibrationBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                               ros::NodeHandle& controller_nh)
{
  CalibrationBase::init(robot_hw, root_nh, controller_nh);

  XmlRpc::XmlRpcValue gpios, initial_gpio_states;
  if (!controller_nh.getParam("gpios", gpios))
  {
    ROS_ERROR("No gpios given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("initial_gpio_states", initial_gpio_states))
  {
    ROS_ERROR("No initial gpio states given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("vel_threshold", vel_threshold_))
  {
    ROS_ERROR("Velocity threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < gpios.size(); i++)
  {
    ROS_ASSERT(initial_gpio_states[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
    ROS_ASSERT(gpios[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string gpio_name = gpios[i];
    rm_control::GpioStateHandle state_handle = robot_hw->get<rm_control::GpioStateInterface>()->getHandle(gpio_name);
    gpio_state_handles_.push_back(state_handle);
    initial_gpio_states_.push_back(initial_gpio_states[i]);
  }
  return true;
}
}  // namespace rm_calibration_controllers
