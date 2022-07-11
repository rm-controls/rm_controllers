/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 5/16/21.
//

#include "rm_calibration_controllers/joint_calibration_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool JointCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                      ros::NodeHandle& controller_nh)
{
  velocity_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), controller_nh);
  XmlRpc::XmlRpcValue actuator;
  is_return_ = is_center_ = false;
  controller_nh.getParam("center", is_center_);
  if (!controller_nh.getParam("actuator", actuator))
  {
    ROS_ERROR("No actuator given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  actuator_ = robot_hw->get<rm_control::ActuatorExtraInterface>()->getHandle(actuator[0]);
  if (!controller_nh.getParam("search_velocity", velocity_search_))
  {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("threshold", velocity_threshold_))
  {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (velocity_threshold_ < 0)
  {
    velocity_threshold_ *= -1.;
    ROS_ERROR("Negative velocity threshold is not supported for joint %s. Making the velocity threshold positive.",
              velocity_ctrl_.getJointName().c_str());
  }
  if (controller_nh.hasParam("return"))
  {
    ros::NodeHandle nh_return(controller_nh, "return");
    position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), nh_return);
    if (!nh_return.getParam("target_position", target_position_))
    {
      ROS_ERROR("Position value was not specified (namespace: %s)", nh_return.getNamespace().c_str());
      return false;
    }
    if (!controller_nh.getParam("threshold", position_threshold_))
    {
      ROS_ERROR("Position value was not specified (namespace: %s)", nh_return.getNamespace().c_str());
      return false;
    }
    is_return_ = true;
    returned_ = false;
  }
  // advertise service to check calibration
  is_calibrated_srv_ = controller_nh.advertiseService("is_calibrated", &JointCalibrationController::isCalibrated, this);
  return true;
}

void JointCalibrationController::starting(const ros::Time& time)
{
  actuator_.setCalibrated(false);
  state_ = INITIALIZED;
  if (actuator_.getCalibrated())
    ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f",
             velocity_ctrl_.getJointName().c_str(), actuator_.getOffset());
}

void JointCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  // TODO: Add GPIO switch support
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      break;
    }
    case MOVING_POSITIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_ && !actuator_.getHalted())
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0);
        if (!is_center_)
        {
          actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
          actuator_.setCalibrated(true);
          ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
          state_ = CALIBRATED;
          if (is_return_)
            position_ctrl_.joint_.setCommand(target_position_);
          else
          {
            velocity_ctrl_.joint_.setCommand(0.);
            returned_ = true;
          }
        }
        else
        {
          positive_position_ = actuator_.getPosition();
          countdown_ = 100;
          velocity_ctrl_.setCommand(-velocity_search_);
          state_ = MOVING_NEGATIVE;
        }
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case MOVING_NEGATIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_)
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0);
        negative_position_ = actuator_.getPosition();
        actuator_.setOffset(-(positive_position_ + negative_position_) / 2 + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        state_ = CALIBRATED;
        if (is_return_)
          position_ctrl_.joint_.setCommand(target_position_);
        else
        {
          velocity_ctrl_.joint_.setCommand(0.);
          returned_ = true;
        }
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case CALIBRATED:
    {
      if (is_return_)
      {
        if ((std::abs(position_ctrl_.joint_.getPosition()) - target_position_) < position_threshold_)
          returned_ = true;
        position_ctrl_.update(time, period);
      }
      else
        velocity_ctrl_.update(time, period);
      break;
    }
  }
}

bool JointCalibrationController::isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                                              control_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED && returned_);
  resp.is_calibrated = (state_ == CALIBRATED && returned_);
  return true;
}

}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::JointCalibrationController, controller_interface::ControllerBase)
