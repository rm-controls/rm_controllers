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
  ros::NodeHandle vel_nh(controller_nh, "velocity");
  ros::NodeHandle pos_nh(controller_nh, "position");
  velocity_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), vel_nh);
  position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), pos_nh);
  gpio_sub_ = controller_nh.subscribe<rm_msgs::GpioData>("/controllers/gpio_controller/gpio_states", 100,
                                                         &GpioCalibrationController::gpioStateCB, this);
  XmlRpc::XmlRpcValue actuator;
  if (!controller_nh.getParam("actuator", actuator))
  {
    ROS_ERROR("No actuator given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  actuator_ = robot_hw->get<rm_control::ActuatorExtraInterface>()->getHandle(actuator[0]);
  if (!controller_nh.getParam("search_velocity", velocity_search_))
  {
    ROS_ERROR("Search velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("vel_gain", vel_gain_))
  {
    ROS_ERROR("Velocity gain was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("vel_threshold", vel_threshold_))
  {
    ROS_ERROR("Velocity threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("initial_gpio_state", initial_gpio_state_))
  {
    ROS_ERROR("Initial gpio state was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  // advertise service to check calibration
  is_calibrated_srv_ = controller_nh.advertiseService("is_calibrated", &GpioCalibrationController::isCalibrated, this);
  return true;
}

void GpioCalibrationController::starting(const ros::Time& time)
{
  actuator_.setCalibrated(false);
  state_ = INITIALIZED;
  if (actuator_.getCalibrated())
    ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f",
             velocity_ctrl_.getJointName().c_str(), actuator_.getOffset());
}

void GpioCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      velocity_ctrl_.update(time, period);
      state_ = MOVING_AROUND;
      break;
    }
    case MOVING_AROUND:
    {
      if (enter_flag_)
      {
        enter_flag_ = false;
        enter_pos_ = velocity_ctrl_.joint_.getPosition();
      }
      if (exit_flag_)
      {
        exit_flag_ = false;
        exit_pos_ = velocity_ctrl_.joint_.getPosition();
      }
      if (enter_pos_ != 0. && exit_pos_ != 0.)
      {
        velocity_ctrl_.setCommand(0.);
        can_returned_ = true;
      }
      if (can_returned_)
      {
        position_ctrl_.setCommand((enter_pos_ + exit_pos_) / 2);
        enter_pos_ = 0;
        exit_pos_ = 0;
        can_returned_ = false;
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
          (position_ctrl_.joint_.getVelocity() < vel_threshold_))
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
      position_ctrl_.setCommand(position_ctrl_.joint_.getPosition());
      position_ctrl_.update(time, period);
    }
  }
}

bool GpioCalibrationController::isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                                             control_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED);
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}

void GpioCalibrationController::gpioStateCB(const rm_msgs::GpioDataConstPtr& msg)
{
  if (msg->gpio_state[0] != initial_gpio_state_)
  {
    if (!initial_gpio_state_ && !is_returned_)
    {
      enter_flag_ = true;
    }

    if (initial_gpio_state_ && enter_pos_ != 0)
      exit_flag_ = true;
    initial_gpio_state_ = !initial_gpio_state_;
  }
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::GpioCalibrationController, controller_interface::ControllerBase)
