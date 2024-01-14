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

#pragma once

#include <effort_controllers/joint_position_controller.h>
#include "rm_calibration_controllers/calibration_base.h"

namespace rm_calibration_controllers
{
class MechanicalCalibrationController
  : public CalibrationBase<rm_control::ActuatorExtraInterface, hardware_interface::EffortJointInterface>
{
public:
  MechanicalCalibrationController() = default;
  /** @brief Get necessary params from param server. Init joint_calibration_controller.
   *
   * Get params from param server and check whether these params are set.Init JointVelocityController.Check
   * whether threshold is set correctly.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if init successful, false when failed.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /** @brief Execute corresponding action according to current calibration controller state.
   *
   * Execute corresponding action according to current joint state. If INITIALIZED, target joint will be set
   * a vel_search_ and countdown_ to move, and switch state to MOVING. If MOVING, target joint will move until
   * current velocity lower than threshold last for a while, and switch state to CALIBRATED. If CALIBRATED,
   * target joint velocity will be set to zero and wait for next command.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  enum State
  {
    MOVING_POSITIVE = 3,
    MOVING_NEGATIVE,
  };
  int countdown_{};
  double velocity_threshold_{}, position_threshold_{};
  double positive_position_{}, negative_position_{}, target_position_{};
  bool is_return_{}, is_center_{};
};

}  // namespace rm_calibration_controllers
