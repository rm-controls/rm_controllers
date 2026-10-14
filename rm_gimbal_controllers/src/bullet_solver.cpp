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
// Created by qiayuan on 8/14/20.
//

#include "rm_gimbal_controllers/bullet_solver.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/PowerHeatData.h>
#include <angles/angles.h>

namespace rm_gimbal_controllers
{
BulletSolver::BulletSolver(ros::NodeHandle& controller_nh)
{
  config_ = {
    .resistance_coff_qd_10 = getParam(controller_nh, "resistance_coff_qd_10", 0.),
    .resistance_coff_qd_15 = getParam(controller_nh, "resistance_coff_qd_15", 0.),
    .resistance_coff_qd_16 = getParam(controller_nh, "resistance_coff_qd_16", 0.),
    .resistance_coff_qd_18 = getParam(controller_nh, "resistance_coff_qd_18", 0.),
    .resistance_coff_qd_30 = getParam(controller_nh, "resistance_coff_qd_30", 0.),
    .resistance_coff_qd_800 = getParam(controller_nh, "resistance_coff_qd_800", 0.001),
    .g = getParam(controller_nh, "g", 0.),
    .delay = getParam(controller_nh, "delay", 0.),
    .outpost_delay = getParam(controller_nh, "outpost_delay", 0.),
    .center_delay = getParam(controller_nh, "center_delay", 0.0),
    .switch_angle_offset = getParam(controller_nh, "switch_angle_offset", 0.0),
    .outpost_switch_angle_offset = getParam(controller_nh, "outpost_switch_angle_offset", 0.0),
    .min_shoot_beforehand_vel = getParam(controller_nh, "min_shoot_beforehand_vel", 2.5),
    .track_rotate_target_delay = getParam(controller_nh, "track_rotate_target_delay", 0.),
    .track_move_target_delay = getParam(controller_nh, "track_move_target_delay", 0.),
    .yaw_max_acc = getParam(controller_nh, "yaw_max_acc", 60.),
    .track_rotate_outpost_delay = getParam(controller_nh, "track_rotate_outpost_delay", 0.),
    .min_fit_switch_count = getParam(controller_nh, "min_fit_switch_count", 3),
    .traject_start_fit_ = getParam(controller_nh, "traject_start_fit_", 45),
    .traject_ahead_ = getParam(controller_nh, "traject_ahead_", 1.),
    .clean_shoot_num_ = getParam(controller_nh, "clean_shoot_num_", 1),
    .end_pos_offset = getParam(controller_nh, "end_pos_offset", 0.),
    .traject_k_effort = getParam(controller_nh, "traject_k_effort", 0.0),
    .traject_k_vel_ = getParam(controller_nh, "traject_k_vel", 0.65),
  };
  max_track_target_vel_ = getParam(controller_nh, "max_track_target_vel", 5.0);
  switch_hysteresis_ = getParam(controller_nh, "switch_hysteresis", 1.0);
  config_rt_buffer_.initRT(config_);

  marker_desire_.header.frame_id = "odom";
  marker_desire_.ns = "model";
  marker_desire_.action = visualization_msgs::Marker::ADD;
  marker_desire_.type = visualization_msgs::Marker::POINTS;
  marker_desire_.scale.x = 0.02;
  marker_desire_.scale.y = 0.02;
  marker_desire_.color.r = 1.0;
  marker_desire_.color.g = 0.0;
  marker_desire_.color.b = 0.0;
  marker_desire_.color.a = 1.0;

  marker_real_ = marker_desire_;
  marker_real_.color.r = 0.0;
  marker_real_.color.g = 1.0;

  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  path_desire_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_desire", 10));
  path_real_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_real", 10));
  fly_time_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "fly_time", 10));
  bullet_solver_pub.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::BulletSolverData>(controller_nh, "bullet_solver_data", 10));
  identified_target_change_sub_ =
      controller_nh.subscribe<std_msgs::Bool>("/change", 10, &BulletSolver::identifiedTargetChangeCB, this);
  shoot_state_sub_ = controller_nh.subscribe<rm_msgs::LocalHeatState>("/local_heat_state/shooter_state", 50,
                                                                      &BulletSolver::heatCB, this);
}

double BulletSolver::getResistanceCoefficient(double target_distance) const
{
  double resistance_coff;
  // hero
  resistance_coff = config_.resistance_coff_qd_800;
  return resistance_coff;
}

bool BulletSolver::solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw,
                         double v_yaw, double r1, double r2, double dz, double armors_num, double start_vel,
                         double track_id)
{
  while (yaw > M_PI / 2)
    yaw -= M_PI;
  while (yaw < -M_PI / 2)
    yaw += M_PI;
  if (armors_num == 3)
  {
    armors_num = 3.6;
  }
  config_ = *config_rt_buffer_.readFromRT();
  bullet_speed_ = bullet_speed;
  if (abs(yaw - last_yaw_) > 1.)
    filtered_yaw_ = yaw;
  else if (last_yaw_ != yaw)
    filtered_yaw_ = filtered_yaw_ + (yaw - filtered_yaw_) * (0.001 / (0.01 + 0.001));
  last_yaw_ = yaw;
  filtered_v_yaw_ = filtered_v_yaw_ + (v_yaw - filtered_v_yaw_) * 0.05;
  if (track_count_ <= 300)
  {
    track_target_ = false;
    track_count_++;
  }
  if (track_count_ > 300)
  {
    if (std::abs(filtered_v_yaw_) >= max_track_target_vel_ + switch_hysteresis_)
      track_target_ = false;
    else if (std::abs(filtered_v_yaw_) <= max_track_target_vel_ - switch_hysteresis_)
      track_target_ = true;
    if (sqrt(pow(pos.x, 2) + pow(pos.y, 2)) < 1.2)
      track_target_ = false;
  }

  if (identified_target_change_)
  {
    for (int i = 0; i < 150; i++)
    {
      count_[i] = 0;
      next_count_[i] = 0;
    }
    identified_target_change_ = false;
  }
  for (int i = 0; i < 150; i++)
  {
    double temp_z = pos.z;
    if (track_target_)
    {
      if (track_id == 6)
      {
        yaw_[i] = yaw + filtered_v_yaw_ * (config_.track_rotate_outpost_delay + i * 0.001);
      }
      else
      {
        yaw_[i] = yaw + filtered_v_yaw_ * (config_.track_rotate_target_delay + i * 0.001);
      }
    }
    else
      yaw_[i] = yaw;
    pos_x[i] = pos.x + vel.x * (config_.track_move_target_delay + i * 0.001);
    pos_y[i] = pos.y + vel.y * (config_.track_move_target_delay + i * 0.001);
    double target_rho = std::sqrt(std::pow(pos_x[i], 2) + std::pow(pos_y[i], 2));
    resistance_coff_ = getResistanceCoefficient(target_rho) != 0 ? getResistanceCoefficient(target_rho) : 0.001;
    double output_yaw_central = std::atan2(pos_y[i], pos_x[i]);
    output_pitch_[i] = std::atan2(temp_z, std::sqrt(std::pow(pos_x[i], 2) + std::pow(pos_y[i], 2)));
    double rough_fly_time =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_[i])))) /
        resistance_coff_;
    if (bullet_speed == 0.0)
    {
      ROS_ERROR("solve bullet speed is zero");
      return false;
    }
    double r = r1;
    double z = pos.z;

    switch_armor_angle = acos(r1 / target_rho) - config_.switch_angle_offset;
    if (track_id == 6)
    {
      switch_armor_angle = acos(r1 / target_rho) - config_.outpost_switch_angle_offset;
    }
    yaw_subtract_ = yaw_[i] - output_yaw_central;
    // while (yaw_subtract_ > M_PI)
    //   yaw_subtract_ -= 2 * M_PI;
    // while (yaw_subtract_ < -M_PI)
    //   yaw_subtract_ += 2 * M_PI;
    double after_fly_yaw_subtract_ = yaw_subtract_ + v_yaw * rough_fly_time;
    if (abs(v_yaw) < 2.)
      selected_armor_[i] = 0;
    else if ((after_fly_yaw_subtract_ < switch_armor_angle - 0.4 && filtered_v_yaw_ > 2.) ||
             (after_fly_yaw_subtract_ > -switch_armor_angle + 0.4 && filtered_v_yaw_ < -2.))
      selected_armor_[i] = 0;

    if ((after_fly_yaw_subtract_ > switch_armor_angle && filtered_v_yaw_ > 2.) ||
        (after_fly_yaw_subtract_ < -switch_armor_angle && filtered_v_yaw_ < -2.))
    {
      count_[i]++;
      if (count_[i] >= config_.min_fit_switch_count)
      {
        selected_armor_[i] = v_yaw > 0. ? -1 : 1;
        r = armors_num == 4 ? r2 : r1;
        z = pos.z + dz;
        if ((after_fly_yaw_subtract_ - 2 * M_PI / armors_num > switch_armor_angle) && v_yaw > 0.)
        {
          selected_armor_[i] = -2;
          r = armors_num == 4 ? r1 : r2;
          z = pos.z;
          next_count_[i]++;
          if (next_count_[i] == config_.min_fit_switch_count)
          {
            switch_armor_time_ = ros::Time::now();
            is_aheading_two_[i] = true;
          }
        }
        else if ((after_fly_yaw_subtract_ + 2 * M_PI / armors_num < -switch_armor_angle) && v_yaw < 0.)
        {
          selected_armor_[i] = 2;
          r = armors_num == 4 ? r1 : r2;
          z = pos.z;
          next_count_[i]++;
          if (next_count_[i] == config_.min_fit_switch_count)
          {
            switch_armor_time_ = ros::Time::now();
            is_aheading_two_[i] = true;
          }
        }
        if (count_[i] == config_.min_fit_switch_count)
        {
          if (is_aheading_two_[i])
          {
            is_aheading_two_[i] = false;
          }
          else
          {
            switch_armor_time_ = ros::Time::now();
          }
        }
      }
    }
    int count{};
    double error = 999;
    if (track_target_)
    {
      target_pos_[i].x = pos_x[i] - r * cos(yaw_[i] + selected_armor_[i] * 2 * M_PI / armors_num);
      target_pos_[i].y = pos_y[i] - r * sin(yaw_[i] + selected_armor_[i] * 2 * M_PI / armors_num);
    }
    else
    {
      target_pos_[i].x = pos_x[i] - r * cos(atan2(pos_y[i], pos_x[i]));
      target_pos_[i].y = pos_y[i] - r * sin(atan2(pos_y[i], pos_x[i]));
      selected_armor_[i] = 0;
      if ((filtered_v_yaw_ > 2.0 && (yaw_subtract_ + filtered_v_yaw_ * (fly_time_[i] + config_.center_delay)) > 0.7) ||
          (filtered_v_yaw_ < -2.0 && (yaw_subtract_ + filtered_v_yaw_ * (fly_time_[i] + config_.center_delay)) < -0.7))
      {
        selected_armor_[i] = filtered_v_yaw_ > 0. ? -1 : 1;
      }
      if (((filtered_v_yaw_ > 2.0 && (yaw_subtract_ + filtered_v_yaw_ * (fly_time_[i] + config_.center_delay) -
                                      1 * 2 * M_PI / armors_num) > 0.7) ||
           (filtered_v_yaw_ < -2.0 && (yaw_subtract_ + filtered_v_yaw_ * (fly_time_[i] + config_.center_delay) +
                                       1 * 2 * M_PI / armors_num) < -0.7)))
      {
        selected_armor_[i] = filtered_v_yaw_ > 0. ? -2 : 2;
      }
      if (((filtered_v_yaw_ > 2.0 && (yaw_subtract_ + filtered_v_yaw_ * (fly_time_[i] + config_.center_delay) -
                                      2 * 2 * M_PI / armors_num) > 0.7) ||
           (filtered_v_yaw_ < -2.0 && (yaw_subtract_ + filtered_v_yaw_ * (fly_time_[i] + config_.center_delay) +
                                       2 * 2 * M_PI / armors_num) < -0.7)))
      {
        selected_armor_[i] = filtered_v_yaw_ > 0. ? -3 : 3;
      }

      if (selected_armor_[i] % 2 == 0)
      {
        r = r1;
        z = pos.z;
      }
      else
      {
        r = r2;
        z = pos.z + dz;
      }
    }
    target_pos_[i].z = z;
    while (error >= 0.001)
    {
      output_yaw_[i] = std::atan2(target_pos_[i].y, target_pos_[i].x);
      output_pitch_[i] = std::atan2(temp_z, std::sqrt(std::pow(target_pos_[i].x, 2) + std::pow(target_pos_[i].y, 2)));
      target_rho = std::sqrt(std::pow(target_pos_[i].x, 2) + std::pow(target_pos_[i].y, 2));
      fly_time_[i] = (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_[i])))) /
                     resistance_coff_;
      double real_z = (bullet_speed_ * std::sin(output_pitch_[i]) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_[i])) / resistance_coff_ -
                      config_.g * fly_time_[i] / resistance_coff_;
      if (track_target_)
      {
        target_pos_[i].x = pos_x[i] + vel.x * fly_time_[i] -
                           r * cos(yaw_[i] + v_yaw * fly_time_[i] + selected_armor_[i] * 2 * M_PI / armors_num);
        target_pos_[i].y = pos_y[i] + vel.y * fly_time_[i] -
                           r * sin(yaw_[i] + v_yaw * fly_time_[i] + selected_armor_[i] * 2 * M_PI / armors_num);
      }
      else
      {
        double target_pos_after_fly_time[2];
        target_pos_after_fly_time[0] = pos_x[i] + vel.x * fly_time_[i];
        target_pos_after_fly_time[1] = pos_y[i] + vel.y * fly_time_[i];
        target_pos_[i].x =
            target_pos_after_fly_time[0] - r * cos(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
        target_pos_[i].y =
            target_pos_after_fly_time[1] - r * sin(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
      }
      target_pos_[i].z = z + vel.z * fly_time_[i];

      double target_yaw = std::atan2(target_pos_[i].y, target_pos_[i].x);
      double error_theta = target_yaw - output_yaw_[i];
      double error_z = target_pos_[i].z - real_z;
      temp_z += error_z;
      error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
      count++;
      if (count >= 20 || std::isnan(error))
        return false;
    }
  }

  if (fly_time_pub_->trylock())
  {
    fly_time_pub_->msg_.data = fly_time_[0];
    fly_time_pub_->unlockAndPublish();
  }
  if (selected_armor_[0] == 1 || selected_armor_[0] == -1 || selected_armor_[0] == 3 || selected_armor_[0] == -3)
  {
    r_traject_ = r1;
  }
  else
  {
    r_traject_ = r2;
  }
  for (int i = 1; i <= config_.traject_start_fit_; i++)
  {
    if (selected_armor_[i] != selected_armor_[0] && using_traject_ == false && abs(v_yaw) > 4.5)
    {
      start_traject_ = true;
      using_traject_ = true;
      start_using_traject_time = ros::Time::now();
      break;
    }
  }

  if (start_traject_ && using_traject_)
  {
    traject_max_acc_ = 400.;
    switchtime = 0.05;
    while (abs(traject_max_acc_) > config_.yaw_max_acc)
    {
      switchtime += 0.005;
      if (filtered_v_yaw_ > 0)
      {
        after_traject_output_yaw_.x = pos_x[0] + vel.x * (fly_time_[0] + switchtime) -
                                      r_traject_ * cos(yaw_[0] + v_yaw * (fly_time_[0] + switchtime) +
                                                       (selected_armor_[0] - 1) * 2 * M_PI / armors_num);
        after_traject_output_yaw_.y = pos_y[0] + vel.y * (fly_time_[0] + switchtime) -
                                      r_traject_ * sin(yaw_[0] + v_yaw * (fly_time_[0] + switchtime) +
                                                       (selected_armor_[0] - 1) * 2 * M_PI / armors_num);
      }
      else
      {
        after_traject_output_yaw_.x = pos_x[0] + vel.x * (fly_time_[0] + switchtime) -
                                      r_traject_ * cos(yaw_[0] + v_yaw * (fly_time_[0] + switchtime) +
                                                       (selected_armor_[0] + 1) * 2 * M_PI / armors_num);
        after_traject_output_yaw_.y = pos_y[0] + vel.y * (fly_time_[0] + switchtime) -
                                      r_traject_ * sin(yaw_[0] + v_yaw * (fly_time_[0] + switchtime) +
                                                       (selected_armor_[0] + 1) * 2 * M_PI / armors_num);
      }
      stauts_limit_.start_pos = output_yaw_[0];
      stauts_limit_.start_vel = start_vel;
      stauts_limit_.end_pos =
          std::atan2(after_traject_output_yaw_.y, after_traject_output_yaw_.x) + config_.end_pos_offset;
      stauts_limit_.end_pos =
          stauts_limit_.start_pos + angles::shortest_angular_distance(stauts_limit_.start_pos, stauts_limit_.end_pos);
      stauts_limit_.end_vel = stauts_limit_.start_vel;

      if (switchtime > 0.15)
      {
        break;
      }
      trajectory_function_coefficients.a0 = stauts_limit_.start_pos;
      trajectory_function_coefficients.a1 = stauts_limit_.start_vel;

      Eigen::Matrix2d A;
      A << std::pow(switchtime, 2), std::pow(switchtime, 3), 2 * switchtime, 3 * std::pow(switchtime, 2);
      Eigen::Vector2d B;
      B << stauts_limit_.end_pos - (stauts_limit_.start_pos + stauts_limit_.start_vel * switchtime),
          stauts_limit_.end_vel - stauts_limit_.start_vel;

      Eigen::Vector2d X = A.colPivHouseholderQr().solve(B);

      trajectory_function_coefficients.a2 = X(0);
      trajectory_function_coefficients.a3 = X(1);
      traject_max_acc_ = abs(2 * trajectory_function_coefficients.a2);
    }
    start_traject_ = false;
  }
  if (using_traject_)
  {
    ros::Time temp = ros::Time::now();
    traject_output_yaw_ = planningPoint(temp, start_using_traject_time, v_yaw);
    while (traject_output_yaw_ > M_PI)
      traject_output_yaw_ -= 2 * M_PI;
    while (traject_output_yaw_ < -M_PI)
      traject_output_yaw_ += 2 * M_PI;
    if (ros::Time::now() - start_using_traject_time > ros::Duration(switchtime))
    {
      using_traject_ = false;
    }
  }
  if (!using_traject_ || !track_target_)
  {
    traject_output_yaw_ = output_yaw_[0];
  }
  double vel_des = (traject_output_yaw_ - last_output_yaw_) / (ros::Time::now().toSec() - last_output_time_.toSec());
  filtered_vel_des = filtered_vel_des + (vel_des - filtered_vel_des) * (0.001 / (0.01 + 0.001));
  last_output_time_ = ros::Time::now();
  last_output_yaw_ = traject_output_yaw_;
  if (bullet_solver_pub->trylock())
  {
    for (int i = 0; i < 150; ++i)
    {
      bullet_solver_pub->msg_.selected_armor_[i] = static_cast<int16_t>(selected_armor_[i]);
      bullet_solver_pub->msg_.count_number_[i] = static_cast<int16_t>(count_[i]);
    }
    bullet_solver_pub->msg_.start_vel = stauts_limit_.start_vel;
    bullet_solver_pub->msg_.switchtime = switchtime;
    bullet_solver_pub->msg_.traject_output_yaw_ = traject_output_yaw_;
    bullet_solver_pub->msg_.traject_acc_ = traject_effort_ff_;
    bullet_solver_pub->msg_.traject_vel_ = filtered_vel_des;
    bullet_solver_pub->msg_.using_traject_ = using_traject_;
    bullet_solver_pub->msg_.test_number[0] = yaw_[0];
    bullet_solver_pub->unlockAndPublish();
  }
  return true;
}

void BulletSolver::getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                             geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw,
                                             double v_yaw, double r1, double r2, double dz, double armors_num)
{
  if (armors_num == 3)
  {
    armors_num = 3.6;
  }
  double r = r1, z = pos.z;
  if (armors_num == 4 && selected_armor_[0] % 2 != 0)
  {
    r = r2;
    z = pos.z + dz;
  }
  pos.x += vel.x * (config_.track_move_target_delay + fly_time_[0]);
  pos.y += vel.y * (config_.track_move_target_delay + fly_time_[0]);
  if (track_target_)
  {
    armor_pos.x = pos.x - r * cos(yaw + v_yaw * (fly_time_[0] + config_.track_rotate_target_delay) +
                                  selected_armor_[0] * 2 * M_PI / armors_num);
    armor_pos.y = pos.y - r * sin(yaw + v_yaw * (fly_time_[0] + config_.track_rotate_target_delay) +
                                  selected_armor_[0] * 2 * M_PI / armors_num);
    armor_pos.z = z;
    armor_vel.x = vel.x + v_yaw * r *
                              sin(yaw + v_yaw * (fly_time_[0] + config_.track_rotate_target_delay) +
                                  selected_armor_[0] * 2 * M_PI / armors_num);
    armor_vel.y = vel.y - v_yaw * r *
                              cos(yaw + v_yaw * (fly_time_[0] + config_.track_rotate_target_delay) +
                                  selected_armor_[0] * 2 * M_PI / armors_num);
    armor_vel.z = vel.z;
  }
  else
  {
    armor_pos = pos;
    armor_pos.z = z;
    armor_vel = vel;
  }
}

void BulletSolver::bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time)
{
  marker_desire_.points.clear();
  marker_real_.points.clear();
  double roll{}, pitch{}, yaw{};
  quatToRPY(odom2pitch.transform.rotation, roll, pitch, yaw);
  geometry_msgs::Point point_desire{}, point_real{};
  double target_rho = std::sqrt(std::pow(target_pos_[0].x, 2) + std::pow(target_pos_[0].y, 2));
  int point_num = int(target_rho * 20);
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time = (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_[0])))) /
                      resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(output_pitch_[0]) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_desire.x = rt_bullet_rho * std::cos(output_yaw_[0]) + odom2pitch.transform.translation.x;
    point_desire.y = rt_bullet_rho * std::sin(output_yaw_[0]) + odom2pitch.transform.translation.y;
    point_desire.z = rt_bullet_z + odom2pitch.transform.translation.z;
    marker_desire_.points.push_back(point_desire);
  }
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time =
        (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(-pitch)))) / resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(-pitch) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_real.x = rt_bullet_rho * std::cos(yaw) + odom2pitch.transform.translation.x;
    point_real.y = rt_bullet_rho * std::sin(yaw) + odom2pitch.transform.translation.y;
    point_real.z = rt_bullet_z + odom2pitch.transform.translation.z;
    marker_real_.points.push_back(point_real);
  }
  marker_desire_.header.stamp = time;
  if (path_desire_pub_->trylock())
  {
    path_desire_pub_->msg_ = marker_desire_;
    path_desire_pub_->unlockAndPublish();
  }
  marker_real_.header.stamp = time;
  if (path_real_pub_->trylock())
  {
    path_real_pub_->msg_ = marker_real_;
    path_real_pub_->unlockAndPublish();
  }
}

double BulletSolver::getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
                                    double r1, double r2, double dz, double armors_num, double yaw_real,
                                    double pitch_real, double bullet_speed)
{
  if (armors_num == 3)
  {
    armors_num = 3.6;
  }
  config_ = *config_rt_buffer_.readFromRT();
  double delay;
  delay = track_target_ ? 0. : config_.center_delay;
  double r, z;
  if (selected_armor_[0] % 2 == 0)
  {
    r = r1;
    z = pos.z;
  }
  else
  {
    r = armors_num == 4 ? r2 : r1;
    z = pos.z + dz;
  }
  double error;
  if (track_target_)
  {
    double bullet_rho =
        bullet_speed * std::cos(pitch_real) * (1 - std::exp(-resistance_coff_ * fly_time_[0])) / resistance_coff_;
    double bullet_x = bullet_rho * std::cos(yaw_real);
    double bullet_y = bullet_rho * std::sin(yaw_real);
    double bullet_z = (bullet_speed * std::sin(pitch_real) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_[0])) / resistance_coff_ -
                      config_.g * fly_time_[0] / resistance_coff_;
    error = std::sqrt(std::pow(target_pos_[0].x - bullet_x, 2) + std::pow(target_pos_[0].y - bullet_y, 2) +
                      std::pow(target_pos_[0].z - bullet_z, 2));
  }
  else
  {
    geometry_msgs::Point target_pos_after_fly_time_and_delay{};
    target_pos_after_fly_time_and_delay.x =
        pos.x + vel.x * (fly_time_[0] + delay) -
        r * cos(yaw + v_yaw * (fly_time_[0] + delay) + selected_armor_[0] * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.y =
        pos.y + vel.y * (fly_time_[0] + delay) -
        r * sin(yaw + v_yaw * (fly_time_[0] + delay) + selected_armor_[0] * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.z = z + vel.z * (fly_time_[0] + delay);
    error = std::sqrt(std::pow(target_pos_[0].x - target_pos_after_fly_time_and_delay.x, 2) +
                      std::pow(target_pos_[0].y - target_pos_after_fly_time_and_delay.y, 2) +
                      std::pow(target_pos_[0].z - target_pos_after_fly_time_and_delay.z, 2));
  }
  return error;
}

void BulletSolver::identifiedTargetChangeCB(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data)
    identified_target_change_ = true;
  else if (!msg->data && identified_target_change_)
  {
    identified_target_change_ = false;
  }
}

uint8_t BulletSolver::judgeShootBeforehand(double v_yaw, int id)
{
  if (!track_target_)
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  else if (std::abs(v_yaw) > config_.min_shoot_beforehand_vel)
  {
    int ban_shoot_delay_number = static_cast<int>(55 + config_.delay * 1000);
    if (id == 6)
    {
      ban_shoot_delay_number = static_cast<int>(55 + config_.outpost_delay * 1000);
    }
    ban_shoot_delay_number = ban_shoot_delay_number > 149 ? 149 : ban_shoot_delay_number;
    if (selected_armor_[ban_shoot_delay_number] != selected_armor_[0])
    {
      ban_shoot_count_++;
      shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
      if (ban_shoot_count_ == 1)
      {
        ban_shoot_start_time_ = ros::Time::now();
      }
    }
    else if (using_traject_ && ros::Time::now().toSec() < ban_shoot_start_time_.toSec() + switchtime)
    {
      shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
    }
    else if (ros::Time::now().toSec() > ban_shoot_start_time_.toSec() + switchtime &&
             ros::Time::now().toSec() < ban_shoot_start_time_.toSec() + switchtime + config_.delay)
    {
      shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::ALLOW_SHOOT;
      ban_shoot_count_ = 0;
    }
    else
    {
      shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
      ban_shoot_count_ = 0;
    }
  }
  else
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  return static_cast<uint8_t>(shoot_beforehand_cmd_);
}

double BulletSolver::planningPoint(ros::Time& time, ros::Time& start_trajectory_time_, double v_yaw)
{
  double a0 = trajectory_function_coefficients.a0;
  double a1 = trajectory_function_coefficients.a1;
  double a2 = trajectory_function_coefficients.a2;
  double a3 = trajectory_function_coefficients.a3;
  double n_time_ = (time - start_trajectory_time_).toSec();
  double plansetpoint = a0 + a1 * n_time_ + a2 * pow(n_time_, 2) + a3 * pow(n_time_, 3);
  traject_effort_ff_ = (2 * a2 + 6 * a3 * n_time_) / (2 * a2) * 1.0 * config_.traject_k_effort;
  traject_vel_ = (a1 + 2 * a2 * n_time_ + 3 * a3 * pow(n_time_, 2)) * config_.traject_k_vel_;
  return plansetpoint;
}

void BulletSolver::heatCB(const rm_msgs::LocalHeatStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(heat_mutex_);
  if (msg->has_shoot && last_shoot_state_ != msg->has_shoot)
  {
    shoot_num_ += 1;
  }
  last_shoot_state_ = msg->has_shoot;
  config_ = *config_rt_buffer_.readFromRT();
  if (config_.clean_shoot_num_ == 0)
  {
    shoot_num_ = 0;
  }
}
void BulletSolver::reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Bullet Solver] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer_.readFromNonRT();  // config init use yaml
    config.resistance_coff_qd_10 = init_config.resistance_coff_qd_10;
    config.resistance_coff_qd_15 = init_config.resistance_coff_qd_15;
    config.resistance_coff_qd_16 = init_config.resistance_coff_qd_16;
    config.resistance_coff_qd_18 = init_config.resistance_coff_qd_18;
    config.resistance_coff_qd_30 = init_config.resistance_coff_qd_30;
    config.g = init_config.g;
    config.delay = init_config.delay;
    config.outpost_delay = init_config.outpost_delay;
    config.center_delay = init_config.center_delay;
    config.switch_angle_offset = init_config.switch_angle_offset;
    config.outpost_switch_angle_offset = init_config.outpost_switch_angle_offset;
    config.min_shoot_beforehand_vel = init_config.min_shoot_beforehand_vel;
    config.track_rotate_target_delay = init_config.track_rotate_target_delay;
    config.track_move_target_delay = init_config.track_move_target_delay;
    config.yaw_max_acc = init_config.yaw_max_acc;
    config.track_rotate_outpost_delay = init_config.track_rotate_outpost_delay;
    config.min_fit_switch_count = init_config.min_fit_switch_count;
    config.traject_start_fit_ = init_config.traject_start_fit_;
    config.traject_ahead_ = init_config.traject_ahead_;
    config.clean_shoot_num_ = init_config.clean_shoot_num_;
    config.end_pos_offset = init_config.end_pos_offset;
    config.traject_k_effort = init_config.traject_k_effort;
    config.traject_k_vel = init_config.traject_k_vel_;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .resistance_coff_qd_10 = config.resistance_coff_qd_10,
                        .resistance_coff_qd_15 = config.resistance_coff_qd_15,
                        .resistance_coff_qd_16 = config.resistance_coff_qd_16,
                        .resistance_coff_qd_18 = config.resistance_coff_qd_18,
                        .resistance_coff_qd_30 = config.resistance_coff_qd_30,
                        .resistance_coff_qd_800 = config.resistance_coff_qd_800,
                        .g = config.g,
                        .delay = config.delay,
                        .outpost_delay = config.outpost_delay,
                        .center_delay = config.center_delay,
                        .switch_angle_offset = config.switch_angle_offset,
                        .outpost_switch_angle_offset = config.outpost_switch_angle_offset,
                        .min_shoot_beforehand_vel = config.min_shoot_beforehand_vel,
                        .track_rotate_target_delay = config.track_rotate_target_delay,
                        .track_move_target_delay = config.track_move_target_delay,
                        .yaw_max_acc = config.yaw_max_acc,
                        .track_rotate_outpost_delay = config.track_rotate_outpost_delay,
                        .min_fit_switch_count = config.min_fit_switch_count,
                        .traject_start_fit_ = config.traject_start_fit_,
                        .traject_ahead_ = config.traject_ahead_,
                        .clean_shoot_num_ = config.clean_shoot_num_,
                        .end_pos_offset = config.end_pos_offset,
                        .traject_k_effort = config.traject_k_effort,
                        .traject_k_vel_ = config.traject_k_vel };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers
