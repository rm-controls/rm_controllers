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
  velocity_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), controller_nh);
  gpio_sub_ =
      controller_nh.subscribe<rm_msgs::GpioData>("gpio_states", 100, &GpioCalibrationController::gpioStateCB, this);
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
      countdown_ = 100;
      state_ = MOVING_TO_CENTER;
      break;
    }
    case MOVING_TO_CENTER:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < vel_threshold_)
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ != 0)
      {
        if (gpio_state_change_)
        {
          velocity_ctrl_.setCommand(velocity_ctrl_.command_ * vel_gain_ * -1.);
          velocity_ctrl_.update(time, period);
        }
        else
          velocity_ctrl_.update(time, period);
      }
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0.);
        actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        velocity_ctrl_.update(time, period);
        state_ = CALIBRATED;
      }
      break;
    }
    case CALIBRATED:
    {
      velocity_ctrl_.update(time, period);
      break;
    }
  }
}

bool GpioCalibrationController::isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                                             control_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED && on_center_);
  resp.is_calibrated = (state_ == CALIBRATED && on_center_);
  return true;
}

void GpioCalibrationController::gpioStateCB(const rm_msgs::GpioDataConstPtr& msg)
{
  if (msg->gpio_state[0] != initial_gpio_state_)
    gpio_state_change_ = true;
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::GpioCalibrationController, controller_interface::ControllerBase)
