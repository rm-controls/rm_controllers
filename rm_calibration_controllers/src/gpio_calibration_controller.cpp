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
  if (!pos_nh.getParam("backward_angle", backward_angle_))
  {
    ROS_ERROR("Backward angle was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("velocity/slow_forward_velocity", slow_forward_velocity_))
  {
    ROS_ERROR("Slow forward velocity was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  std::string gpio{};
  if (!controller_nh.getParam("gpio", gpio))
  {
    ROS_ERROR("No gpio given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("initial_gpio_state", initial_gpio_state_))
  {
    ROS_ERROR("No initial gpio states given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  gpio_state_handle_ = robot_hw->get<rm_control::GpioStateInterface>()->getHandle(gpio);
  return true;
}

void GpioCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      state_ = FAST_FORWARD;
      break;
    }
    case FAST_FORWARD:
    {
      if (gpio_state_handle_.getValue() != initial_gpio_state_)
      {
        start_retreat_position_ = velocity_ctrl_.joint_.getPosition();
        velocity_ctrl_.setCommand(0);
        state_ = RETREAT;
      }
      else
        velocity_ctrl_.update(time, period);
      break;
    }
    case RETREAT:
    {
      position_ctrl_.setCommand(start_retreat_position_ - backward_angle_);
      position_ctrl_.update(time, period);
      if (std::abs(position_ctrl_.command_struct_.position_ - position_ctrl_.joint_.getPosition()) < position_threshold_)
        state_ = SLOW_FORWARD;
      break;
    }
    case SLOW_FORWARD:
    {
      velocity_ctrl_.setCommand(slow_forward_velocity_);
      if (gpio_state_handle_.getValue() != initial_gpio_state_)
      {
        velocity_ctrl_.setCommand(0);
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
