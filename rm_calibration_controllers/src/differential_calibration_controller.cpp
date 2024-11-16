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
// Created by cch on 24-8-7.
//

#include "rm_calibration_controllers/differential_calibration_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool DifferentialCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                             ros::NodeHandle& controller_nh)
{
  CalibrationBase::init(robot_hw, root_nh, controller_nh);
  XmlRpc::XmlRpcValue actuator;
  ros::NodeHandle nh2(controller_nh, "joint2");
  position_ctrl2_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), nh2);
  if (!controller_nh.getParam("max_calibretion_time", max_calibretion_time_))
  {
    max_calibretion_time_ = 5.0;
    ROS_ERROR("No given max calibration time, set to default (5s).");
  }
  if (!controller_nh.getParam("actuator", actuator))
  {
    ROS_ERROR("No actuator given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  actuator2_ = robot_hw->get<rm_control::ActuatorExtraInterface>()->getHandle(actuator[1]);
  if (!controller_nh.getParam("velocity/vel_threshold", velocity_threshold_))
  {
    ROS_ERROR("Velocity threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (velocity_threshold_ < 0)
  {
    velocity_threshold_ *= -1.;
    ROS_ERROR("Negative velocity threshold is not supported for joint %s. Making the velocity threshold positive.",
              velocity_ctrl_.getJointName().c_str());
  }
  calibration_success_ = false;
  return true;
}

void DifferentialCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:
    {
      start_time_ = ros::Time::now();
      velocity_ctrl_.setCommand(velocity_search_);
      position_ctrl2_.setCommand(position_ctrl2_.getPosition());
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      break;
    }
    case MOVING_POSITIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_)
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0 || (ros::Time::now() - start_time_).toSec() > max_calibretion_time_)
      {
        velocity_ctrl_.setCommand(0);
        actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
        actuator2_.setOffset(-actuator2_.getPosition() + actuator2_.getOffset());
        actuator_.setCalibrated(true);
        actuator2_.setCalibrated(true);
        ROS_INFO_STREAM("Joint " << velocity_ctrl_.getJointName() << " and " << position_ctrl2_.getJointName()
                                 << " are calibrated.");
        state_ = CALIBRATED;
        velocity_ctrl_.joint_.setCommand(0.);
        position_ctrl2_.joint_.setCommand(0.);
        position_ctrl2_.setCommand(0.);
        calibration_success_ = true;
      }
      velocity_ctrl_.update(time, period);
      position_ctrl2_.update(time, period);
      break;
    }
    case CALIBRATED:
    {
      velocity_ctrl_.update(time, period);
      position_ctrl2_.update(time, period);
      break;
    }
  }
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::DifferentialCalibrationController,
                       controller_interface::ControllerBase)
