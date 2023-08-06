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
// Created by huakang on 2021/3/21.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/filters/filters.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <rm_msgs/ChassisCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_chassis_controllers
{
struct Command
{
  geometry_msgs::Twist cmd_vel_;
  rm_msgs::ChassisCmd cmd_chassis_;
  ros::Time stamp_;
};
template <typename... T>
class ChassisBase : public controller_interface::MultiInterfaceController<T...>
{
public:
  ChassisBase() = default;
  /** @brief Get and check params for covariances. Setup odometry realtime publisher + odom message constant fields.
   * init odom tf.
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
  /** @brief Receive real_time command from manual. Execute different action according to current mode. Set
   * necessary params of chassis. Execute power limit.
   *
   * Receive real_time command from manual and check whether it is normally, if can not receive command from manual
   * for a while, chassis's velocity will be set zero to avoid out of control. Execute different action according
   * to current mode such as RAW, FOLLOW, TWIST.(Their specific usage will be explain in the next). UpdateOdom,Set
   * necessary params such as Acc and vel_tfed. Execute moving action and powerlimit.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

protected:
  /** @briefThe The mode RAW: original state.
   *
   *  The mode raw: original state. Linear velocity will be set zero to stop move.
   */
  void raw();
  /** @brief The mode FOLLOW: chassis will follow gimbal.
   *
   * The mode FOLLOW: The chassis's direct will follow gimbal's direct all the time.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void follow(const ros::Time& time, const ros::Duration& period);
  /** @brief The mode TWIST: Just moving chassis.
   *
   * The mode TWIST: Chassis will move independent and will not effect by gimbal's move.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void twist(const ros::Time& time, const ros::Duration& period);
  virtual void moveJoint(const ros::Time& time, const ros::Duration& period) = 0;
  virtual geometry_msgs::Twist odometry() = 0;
  /** @brief Init frame on base_link. Integral vel to pos and angle.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void updateOdom(const ros::Time& time, const ros::Duration& period);
  /** @brief Set chassis velocity to zero.
   */
  void recovery();
  /** @brief Transform tf velocity to base link frame.
   *
   * @param from The father frame.
   */
  void tfVelToBase(const std::string& from);
  /** @brief To limit the chassis power according to current power limit.
   *
   * Receive power limit from command. Set max_effort command to chassis to avoid exceed power limit.
   */
  void powerLimit();
  /** @brief Write current command from rm_msgs::ChassisCmd.
   *
   * @param msg This message contains various state parameter settings for basic chassis control
   */
  void cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr& msg);
  /** @brief Write current command from  geometry_msgs::Twist.
   *
   * @param msg This expresses velocity in free space broken into its linear and angular parts.
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void outsideOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  rm_control::RobotStateHandle robot_state_handle_{};
  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};

  double wheel_radius_{}, publish_rate_{}, twist_angular_{}, timeout_{}, effort_coeff_{}, velocity_coeff_{},
      power_offset_{};
  double max_odom_vel_;
  bool enable_odom_tf_ = false;
  bool topic_update_ = false;
  bool publish_odom_tf_ = false;
  bool state_changed_ = true;
  enum
  {
    RAW,
    FOLLOW,
    TWIST
  };
  int state_ = RAW;
  RampFilter<double>*ramp_x_{}, *ramp_y_{}, *ramp_w_{};
  std::string follow_source_frame_{}, command_source_frame_{};

  ros::Time last_publish_time_;
  geometry_msgs::TransformStamped odom2base_{};
  tf2::Transform world2odom_;
  geometry_msgs::Vector3 vel_cmd_{};  // x, y
  control_toolbox::Pid pid_follow_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  rm_common::TfRtBroadcaster tf_broadcaster_{};
  ros::Subscriber outside_odom_sub_;
  ros::Subscriber cmd_chassis_sub_;
  ros::Subscriber cmd_vel_sub_;
  Command cmd_struct_;
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buffer_;
};

}  // namespace rm_chassis_controllers
