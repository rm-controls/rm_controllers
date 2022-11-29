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
// Created by qiayuan on 1/16/21.
//

#pragma once

#include <effort_controllers/joint_position_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/GimbalDesError.h>
#include <dynamic_reconfigure/server.h>
#include <rm_gimbal_controllers/bullet_solver.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Eigen>
#include <rm_common/filters/filters.h>

namespace rm_gimbal_controllers
{
class Vector3WithFilter
{
public:
  Vector3WithFilter(int num_data)
  {
    for (int i = 0; i < 3; i++)
      filter_vector_.push_back(std::make_shared<MovingAverageFilter<double>>(num_data));
  }
  void input(double vector[3], double period)
  {
    for (int i = 0; i < 3; i++)
    {
      if (period < 0)
        return;
      if (period > 0.1)
        filter_vector_[i]->clear();
      filter_vector_[i]->input(vector[i]);
    }
  }
  double x()
  {
    return filter_vector_[0]->output();
  }
  double y()
  {
    return filter_vector_[1]->output();
  }
  double z()
  {
    return filter_vector_[2]->output();
  }

private:
  std::vector<std::shared_ptr<MovingAverageFilter<double>>> filter_vector_;
};
class ChassisVel
{
public:
  ChassisVel(const ros::NodeHandle& nh)
  {
    double num_data;
    nh.param("num_data", num_data, 20.0);
    nh.param("debug", is_debug_, true);
    linear_ = std::make_shared<Vector3WithFilter>(num_data);
    angular_ = std::make_shared<Vector3WithFilter>(num_data);
    if (is_debug_)
    {
      real_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "real", 1));
      filtered_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "filtered", 1));
    }
  }
  std::shared_ptr<Vector3WithFilter> linear_;
  std::shared_ptr<Vector3WithFilter> angular_;
  void update(double linear_vel[3], double angular_vel[3], double period)
  {
    linear_->input(linear_vel, period);
    angular_->input(angular_vel, period);
    if (is_debug_ && loop_count_ % 10 == 0)
    {
      if (real_pub_->trylock())
      {
        real_pub_->msg_.linear.x = linear_vel[0];
        real_pub_->msg_.linear.y = linear_vel[1];
        real_pub_->msg_.linear.z = linear_vel[2];
        real_pub_->msg_.angular.x = angular_vel[0];
        real_pub_->msg_.angular.y = angular_vel[1];
        real_pub_->msg_.angular.z = angular_vel[2];

        real_pub_->unlockAndPublish();
      }
      if (filtered_pub_->trylock())
      {
        filtered_pub_->msg_.linear.x = linear_->x();
        filtered_pub_->msg_.linear.y = linear_->y();
        filtered_pub_->msg_.linear.z = linear_->z();
        filtered_pub_->msg_.angular.x = angular_->x();
        filtered_pub_->msg_.angular.y = angular_->y();
        filtered_pub_->msg_.angular.z = angular_->z();

        filtered_pub_->unlockAndPublish();
      }
    }
    loop_count_++;
  }

private:
  bool is_debug_;
  int loop_count_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> real_pub_{}, filtered_pub_{};
};

class Controller : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,
                                                                         hardware_interface::ImuSensorInterface,
                                                                         hardware_interface::EffortJointInterface>
{
public:
  Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void setDes(const ros::Time& time, double yaw_des, double pitch_des);

private:
  void rate(const ros::Time& time, const ros::Duration& period);
  void track(const ros::Time& time);
  void direct(const ros::Time& time);
  bool setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                       const urdf::JointConstSharedPtr& joint_urdf);
  void moveJoint(const ros::Time& time, const ros::Duration& period);
  double feedForward(const ros::Time& time);
  void updateChassisVel();
  void commandCB(const rm_msgs::GimbalCmdConstPtr& msg);
  void trackCB(const rm_msgs::TrackDataConstPtr& msg);

  rm_control::RobotStateHandle robot_state_handle_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  bool has_imu_ = true;
  effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

  std::shared_ptr<BulletSolver> bullet_solver_;

  // ROS Interface
  ros::Time last_publish_time_{};
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>> error_pub_;
  ros::Subscriber cmd_gimbal_sub_;
  ros::Subscriber data_track_sub_;
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rm_msgs::TrackData> track_rt_buffer_;

  rm_msgs::GimbalCmd cmd_gimbal_;
  rm_msgs::TrackData data_track_;
  std::string gimbal_des_frame_id_{}, imu_name_{};
  double publish_rate_{};
  bool state_changed_{};

  // Transform
  geometry_msgs::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, last_odom2base_;

  // Gravity Compensation
  geometry_msgs::Vector3 mass_origin_;
  double gravity_;
  bool enable_gravity_compensation_;

  // Chassis
  double k_chassis_vel_;
  std::shared_ptr<ChassisVel> chassis_vel_;

  enum
  {
    RATE,
    TRACK,
    DIRECT
  };
  int state_ = RATE;
};

}  // namespace rm_gimbal_controllers
