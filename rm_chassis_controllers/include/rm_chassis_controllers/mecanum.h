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
// Created by flying on 2021/1/18.
//

#pragma once

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
class MecanumController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>
{
public:
  MecanumController() = default;
  /** @brief Execute ChassisBase::init. Get necessary handles.
   *
   * Execute ChassisBase::init. Init lf, lb, rf, rb handles so that we can controls four wheels conveniently.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  /** @brief Calculate correct command and set it to four wheels.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  /** @brief Calculate current linear_x, linear_y and angular_z according to current velocity.
   *
   * @return Calculated vel_data included linear_x, linear_y and angular_z.
   */
  geometry_msgs::Twist forwardKinematics() override;

  effort_controllers::JointVelocityController ctrl_lf_, ctrl_rf_, ctrl_lb_, ctrl_rb_;
};
}  // namespace rm_chassis_controllers
