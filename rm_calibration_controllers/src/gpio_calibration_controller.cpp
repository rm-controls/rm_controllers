//
// Created by guanlin on 22-11-7.
//

#include "rm_calibration_controllers/gpio_calibration_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool GpioCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                     ros::NodeHandle& controller_nh)
{
  CalibrationBase::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle pos_nh(controller_nh, "position");
  position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), pos_nh);
  if (!pos_nh.getParam("pos_threshold", position_threshold_))
  {
    ROS_ERROR("Position threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!pos_nh.getParam("backward_radius", backward_radius))
  {
    ROS_ERROR("Backward radius was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("velocity/forward_velocity", forward_velocity_))
  {
    ROS_ERROR("Forward velocity was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
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

void GpioCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      if (gpio_state_handles_[0].getValue() != initial_gpio_states_[0])
      {
        start_retreat_pos_ = velocity_ctrl_.joint_.getPosition();
        velocity_ctrl_.setCommand(0);
        state_ = RETREAT;
      }
      else
        velocity_ctrl_.update(time, period);
      break;
    }
    case RETREAT:
    {
      position_ctrl_.setCommand(start_retreat_pos_ - backward_radius);
      position_ctrl_.update(time, period);
      if (std::abs(position_ctrl_.command_struct_.position_ - position_ctrl_.joint_.getPosition()) < position_threshold_)
      {
        state_ = FORWARD;
        position_ctrl_.stopping(time);
      }
      break;
    }
    case FORWARD:
    {
      velocity_ctrl_.setCommand(forward_velocity_);
      if (gpio_state_handles_[0].getValue() != initial_gpio_states_[0])
        velocity_ctrl_.setCommand(0);
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_)
      {
        actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        state_ = CALIBRATED;
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case CALIBRATED:
      calibration_success_ = true;
  }
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::GpioCalibrationController, controller_interface::ControllerBase)
