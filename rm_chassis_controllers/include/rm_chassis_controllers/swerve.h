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
// Created by qiayuan on 4/23/21.
//

#pragma once

#include "rm_chassis_controllers/chassis_base.h"

#include <rm_common/eigen_types.h>
#include <effort_controllers/joint_position_controller.h>
#include "rm_msgs/PowerHeatData.h"
#include <sensor_msgs/JointState.h>

namespace rm_chassis_controllers
{
struct Module
{
  Vec2<double> position_;
  double pivot_offset_, pivot_buffer_threshold_, pivot_effort_threshold_, pivot_position_error_threshold_,
      pivot_max_reduce_cnt_, wheel_radius_;
  effort_controllers::JointPositionController* ctrl_pivot_;
  effort_controllers::JointVelocityController* ctrl_wheel_;
};

class SwerveController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>
{
public:
  SwerveController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void powerManagerCallback(const rm_msgs::PowerHeatData::ConstPtr& data);
  bool isPivotBlock(double cur_effort, double position_error, Module module);
  void reduceTargetPosition(double& target_pos, double& position_error, Module module);
  geometry_msgs::Twist odometry() override;
  std::vector<Module> modules_;
  ros::Subscriber power_manager_sub_;
  int chassis_power_buffer_ = 60;
  int pivot_block_cnt_ = 0;
};

}  // namespace rm_chassis_controllers
