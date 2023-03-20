//
// Created by guanlin on 22-11-7.
//

#include "rm_calibration_controllers/hall_switch_calibration_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool HallSwitchCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                           ros::NodeHandle& controller_nh)
{
  GpioCalibrationBase::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle pos_nh(controller_nh, "position");
  position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), pos_nh);
  if (!controller_nh.getParam("pos_threshold", position_threshold_))
  {
    ROS_ERROR("Position threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  return true;
}

void HallSwitchCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      velocity_ctrl_.update(time, period);
      if (gpio_state_handles_[0].getValue() != initial_gpio_states_[0])
      {
        last_gpio_state_ = gpio_state_handles_[0].getValue();
        velocity_ctrl_.update(time, period);
      }
      else
      {
        last_gpio_state_ = gpio_state_handles_[0].getValue();
        state_ = MOVING_AROUND;
      }
      break;
    }
    case MOVING_AROUND:
    {
      if (gpio_state_handles_[0].getValue() != last_gpio_state_)
      {
        if (gpio_state_handles_[0].getValue() != initial_gpio_states_[0] && !is_returned_)
        {
          enter_pos_ = velocity_ctrl_.joint_.getPosition();
          last_gpio_state_ = gpio_state_handles_[0].getValue();
        }
        if (gpio_state_handles_[0].getValue() == initial_gpio_states_[0] && enter_pos_ != 0)
        {
          exit_pos_ = velocity_ctrl_.joint_.getPosition();
          last_gpio_state_ = gpio_state_handles_[0].getValue();
        }
      }
      if (enter_pos_ != 0. && exit_pos_ != 0.)
      {
        velocity_ctrl_.setCommand(0.);
        position_ctrl_.setCommand((enter_pos_ + exit_pos_) / 2);
        enter_pos_ = 0;
        exit_pos_ = 0;
        state_ = RETURN;
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case RETURN:
    {
      is_returned_ = true;
      position_ctrl_.update(time, period);
      if (((std::abs(position_ctrl_.joint_.getPosition() - position_ctrl_.command_struct_.position_)) <
           position_threshold_) &&
          (position_ctrl_.joint_.getVelocity() < velocity_threshold_))
      {
        actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        state_ = CALIBRATED;
      }
      break;
    }
    case CALIBRATED:
    {
      is_returned_ = false;
      calibration_success_ = true;
      position_ctrl_.setCommand(position_ctrl_.joint_.getPosition());
      position_ctrl_.update(time, period);
    }
  }
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::HallSwitchCalibrationController, controller_interface::ControllerBase)
