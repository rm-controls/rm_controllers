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
// Created by qiayuan on 1/3/21.
//

#pragma once

#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/tf_rt_broadcaster.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace robot_state_controller
{
class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, std::string p_root, std::string p_tip)
    : segment(p_segment), root(std::move(p_root)), tip(std::move(p_tip))
  {
  }

  KDL::Segment segment{};
  std::string root, tip;
};

class RobotStateController
  : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                                          rm_control::RobotStateInterface>
{
public:
  RobotStateController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;

private:
  void addChildren(KDL::SegmentMap::const_iterator segment);
  void tfSubCallback(const tf2_msgs::TFMessageConstPtr& msg);
  void staticSubCallback(const tf2_msgs::TFMessageConstPtr& msg);

  urdf::Model model_{};
  std::map<std::string, urdf::JointMimicSharedPtr>* mimic_{};
  unsigned int num_hw_joints_{};
  bool use_tf_static_{};
  bool ignore_timestamp_{};
  double publish_rate_{};
  ros::Time last_update_;
  ros::Time last_publish_time_;

  std::map<std::string, hardware_interface::JointStateHandle> jnt_states_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;

  tf2_ros::Buffer* tf_buffer_{};
  rm_common::TfRtBroadcaster tf_broadcaster_;
  rm_common::StaticTfRtBroadcaster static_tf_broadcaster_;
  // Do not use tf2_ros::TransformListener because it will lead to setTransform calling twice when publishing the transform
  ros::Subscriber tf_sub_;
  ros::Subscriber tf_static_sub_;
  realtime_tools::RealtimeBuffer<tf2_msgs::TFMessage> tf_msg_;
  realtime_tools::RealtimeBuffer<tf2_msgs::TFMessage> tf_static_msg_;
};

}  // namespace robot_state_controller
