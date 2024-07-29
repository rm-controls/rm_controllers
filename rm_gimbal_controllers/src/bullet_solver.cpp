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

namespace rm_gimbal_controllers
{
BulletSolver::BulletSolver(ros::NodeHandle& controller_nh)
{
  config_ = { .resistance_coff_qd_10 = getParam(controller_nh, "resistance_coff_qd_10", 0.),
              .resistance_coff_qd_15 = getParam(controller_nh, "resistance_coff_qd_15", 0.),
              .resistance_coff_qd_16 = getParam(controller_nh, "resistance_coff_qd_16", 0.),
              .resistance_coff_qd_18 = getParam(controller_nh, "resistance_coff_qd_18", 0.),
              .resistance_coff_qd_30 = getParam(controller_nh, "resistance_coff_qd_30", 0.),
              .g = getParam(controller_nh, "g", 0.),
              .delay = getParam(controller_nh, "delay", 0.),
              .wait_next_armor_delay = getParam(controller_nh, "wait_next_armor_delay", 0.105),
              .wait_diagonal_armor_delay = getParam(controller_nh, "wait_diagonal_armor_delay", 0.105),
              .dt = getParam(controller_nh, "dt", 0.),
              .timeout = getParam(controller_nh, "timeout", 0.),
              .ban_shoot_duration = getParam(controller_nh, "ban_shoot_duration", 0.0),
              .gimbal_switch_duration = getParam(controller_nh, "gimbal_switch_duration", 0.0),
              .max_switch_angle = getParam(controller_nh, "max_switch_angle", 40.0),
              .min_switch_angle = getParam(controller_nh, "min_switch_angle", 2.0),
              .min_shoot_beforehand_vel = getParam(controller_nh, "min_shoot_beforehand_vel", 4.5),
              .max_chassis_angular_vel = getParam(controller_nh, "max_chassis_angular_vel", 8.5),
              .track_rotate_target_delay = getParam(controller_nh, "track_rotate_target_delay", 0.),
              .track_move_target_delay = getParam(controller_nh, "track_move_target_delay", 0.),
              .min_fit_switch_count = getParam(controller_nh, "min_fit_switch_count", 3) };
  max_track_target_vel_ = getParam(controller_nh, "max_track_target_vel", 5.0);
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
  shoot_beforehand_cmd_pub_.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::ShootBeforehandCmd>(controller_nh, "shoot_beforehand_cmd", 10));
  fly_time_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "fly_time", 10));

  identified_target_change_sub_ =
      controller_nh.subscribe<std_msgs::Bool>("/change", 10, &BulletSolver::identifiedTargetChangeCB, this);
}

double BulletSolver::getResistanceCoefficient(double bullet_speed) const
{
  // bullet_speed have 5 value:10,15,16,18,30
  double resistance_coff;
  if (bullet_speed < 12.5)
    resistance_coff = config_.resistance_coff_qd_10;
  else if (bullet_speed < 15.5)
    resistance_coff = config_.resistance_coff_qd_15;
  else if (bullet_speed < 17)
    resistance_coff = config_.resistance_coff_qd_16;
  else if (bullet_speed < 24)
    resistance_coff = config_.resistance_coff_qd_18;
  else
    resistance_coff = config_.resistance_coff_qd_30;
  return resistance_coff;
}

bool BulletSolver::solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw,
                         double v_yaw, double r1, double r2, double dz, int armors_num, double chassis_angular_vel_z)
{
  config_ = *config_rt_buffer_.readFromRT();
  bullet_speed_ = bullet_speed;
  resistance_coff_ = getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;

  double temp_z = pos.z;
  double target_rho = std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2));
  output_yaw_ = std::atan2(pos.y, pos.x);
  output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2)));
  double rough_fly_time =
      (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
  selected_armor_ = 0;
  double r = r1;
  double z = pos.z;
  double max_switch_angle = config_.max_switch_angle / 180 * M_PI;
  double min_switch_angle = config_.min_switch_angle / 180 * M_PI;
  track_target_ = std::abs(v_yaw) < max_track_target_vel_;
  if (std::abs(chassis_angular_vel_z) >= config_.max_chassis_angular_vel)
    track_target_ = 0;
  double switch_armor_angle =
      track_target_ ?
          (acos(r / target_rho) - max_switch_angle +
           (-acos(r / target_rho) + (max_switch_angle + min_switch_angle)) * std::abs(v_yaw) / max_track_target_vel_) *
                  (1 - std::abs(chassis_angular_vel_z) / config_.max_chassis_angular_vel) +
              min_switch_angle * std::abs(chassis_angular_vel_z) / config_.max_chassis_angular_vel :
          min_switch_angle;
  if (((((yaw + v_yaw * rough_fly_time) > output_yaw_ + switch_armor_angle) && v_yaw > 0.) ||
       (((yaw + v_yaw * rough_fly_time) < output_yaw_ - switch_armor_angle) && v_yaw < 0.)) &&
      std::abs(v_yaw) >= 1.0)
  {
    count_++;
    if (identified_target_change_)
    {
      count_ = 0;
      identified_target_change_ = false;
    }
    if (count_ >= config_.min_fit_switch_count)
    {
      if (count_ == config_.min_fit_switch_count)
        switch_armor_time_ = ros::Time::now();
      selected_armor_ = v_yaw > 0. ? -1 : 1;
      r = armors_num == 4 ? r2 : r1;
      z = armors_num == 4 ? pos.z + dz : pos.z;
    }
  }
  is_in_delay_before_switch_ =
      (((((yaw + selected_armor_ * 2 * M_PI / armors_num) + v_yaw * (rough_fly_time + config_.delay)) >
         output_yaw_ + switch_armor_angle) &&
        v_yaw > 0.) ||
       ((((yaw + selected_armor_ * 2 * M_PI / armors_num) + v_yaw * (rough_fly_time + config_.delay)) <
         output_yaw_ - switch_armor_angle) &&
        v_yaw < 0.)) &&
      track_target_;
  if (track_target_)
    yaw += v_yaw * config_.track_rotate_target_delay;
  pos.x += vel.x * config_.track_move_target_delay;
  pos.y += vel.y * config_.track_move_target_delay;
  int count{};
  double error = 999;
  if (track_target_)
  {
    target_pos_.x = pos.x - r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_.y = pos.y - r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
  }
  else
  {
    target_pos_.x = pos.x - r * cos(atan2(pos.y, pos.x));
    target_pos_.y = pos.y - r * sin(atan2(pos.y, pos.x));
    if ((v_yaw > 1.0 && (yaw + v_yaw * (fly_time_ + config_.wait_next_armor_delay) +
                         selected_armor_ * 2 * M_PI / armors_num) > output_yaw_) ||
        (v_yaw < -1.0 && (yaw + v_yaw * (fly_time_ + config_.wait_next_armor_delay) +
                          selected_armor_ * 2 * M_PI / armors_num) < output_yaw_))
      selected_armor_ = v_yaw > 0. ? -2 : 2;
    if (selected_armor_ % 2 == 0)
    {
      r = r1;
      z = pos.z;
    }
  }
  target_pos_.z = z;
  while (error >= 0.001)
  {
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2)));
    target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    fly_time_ =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                    config_.g * fly_time_ / resistance_coff_;

    if (track_target_)
    {
      target_pos_.x =
          pos.x + vel.x * fly_time_ - r * cos(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
      target_pos_.y =
          pos.y + vel.y * fly_time_ - r * sin(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
    }
    else
    {
      double target_pos_after_fly_time[2];
      target_pos_after_fly_time[0] = pos.x + vel.x * fly_time_;
      target_pos_after_fly_time[1] = pos.y + vel.y * fly_time_;
      target_pos_.x =
          target_pos_after_fly_time[0] - r * cos(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
      target_pos_.y =
          target_pos_after_fly_time[1] - r * sin(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
    }
    target_pos_.z = z + vel.z * fly_time_;

    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    double error_theta = target_yaw - output_yaw_;
    double error_z = target_pos_.z - real_z;
    temp_z += error_z;
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    count++;

    if (count >= 20 || std::isnan(error))
      return false;
  }
  if (fly_time_pub_->trylock())
  {
    fly_time_pub_->msg_.data = fly_time_;
    fly_time_pub_->unlockAndPublish();
  }
  return true;
}

void BulletSolver::getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                             geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw,
                                             double v_yaw, double r1, double r2, double dz, int armors_num)
{
  double r = r1, z = pos.z;
  if (armors_num == 4 && selected_armor_ != 0)
  {
    r = r2;
    z = pos.z + dz;
  }
  if (track_target_)
  {
    armor_pos.x = pos.x - r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
    armor_pos.y = pos.y - r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
    armor_pos.z = z;
    armor_vel.x = vel.x + v_yaw * r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
    armor_vel.y = vel.y - v_yaw * r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
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
  double target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
  int point_num = int(target_rho * 20);
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time = (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) /
                      resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_desire.x = rt_bullet_rho * std::cos(output_yaw_) + odom2pitch.transform.translation.x;
    point_desire.y = rt_bullet_rho * std::sin(output_yaw_) + odom2pitch.transform.translation.y;
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
                                    double r1, double r2, double dz, int armors_num, double yaw_real, double pitch_real,
                                    double bullet_speed)
{
  double delay;
  if (track_target_)
    delay = 0.;
  else
    delay = selected_armor_ % 2 == 0 ? config_.wait_diagonal_armor_delay : config_.wait_next_armor_delay;
  double r, z;
  if (selected_armor_ % 2 == 0)
  {
    r = r1;
    z = pos.z;
  }
  else
  {
    r = armors_num == 4 ? r2 : r1;
    z = armors_num == 4 ? pos.z + dz : pos.z;
  }
  double error;
  if (track_target_)
  {
    double bullet_rho =
        bullet_speed * std::cos(pitch_real) * (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_;
    double bullet_x = bullet_rho * std::cos(yaw_real);
    double bullet_y = bullet_rho * std::sin(yaw_real);
    double bullet_z = (bullet_speed * std::sin(pitch_real) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                      config_.g * fly_time_ / resistance_coff_;
    error = std::sqrt(std::pow(target_pos_.x - bullet_x, 2) + std::pow(target_pos_.y - bullet_y, 2) +
                      std::pow(target_pos_.z - bullet_z, 2));
  }
  else
  {
    geometry_msgs::Point target_pos_after_fly_time_and_delay{};
    target_pos_after_fly_time_and_delay.x =
        pos.x + vel.x * (fly_time_ + delay) -
        r * cos(yaw + v_yaw * (fly_time_ + delay) + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.y =
        pos.y + vel.y * (fly_time_ + delay) -
        r * sin(yaw + v_yaw * (fly_time_ + delay) + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.z = z + vel.z * (fly_time_ + delay);
    error = std::sqrt(std::pow(target_pos_.x - target_pos_after_fly_time_and_delay.x, 2) +
                      std::pow(target_pos_.y - target_pos_after_fly_time_and_delay.y, 2) +
                      std::pow(target_pos_.z - target_pos_after_fly_time_and_delay.z, 2));
  }
  return error;
}

void BulletSolver::identifiedTargetChangeCB(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data)
    identified_target_change_ = true;
}

void BulletSolver::judgeShootBeforehand(const ros::Time& time, double v_yaw)
{
  if (!track_target_)
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  else if ((ros::Time::now() - switch_armor_time_).toSec() < ros::Duration(config_.ban_shoot_duration).toSec())
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
  else if (((ros::Time::now() - switch_armor_time_).toSec() < ros::Duration(config_.gimbal_switch_duration).toSec()) &&
           std::abs(v_yaw) > config_.min_shoot_beforehand_vel)
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::ALLOW_SHOOT;
  else if (is_in_delay_before_switch_)
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
  else
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  if (shoot_beforehand_cmd_pub_->trylock())
  {
    shoot_beforehand_cmd_pub_->msg_.stamp = time;
    shoot_beforehand_cmd_pub_->msg_.cmd = shoot_beforehand_cmd_;
    shoot_beforehand_cmd_pub_->unlockAndPublish();
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
    config.wait_next_armor_delay = init_config.wait_next_armor_delay;
    config.wait_diagonal_armor_delay = init_config.wait_diagonal_armor_delay;
    config.dt = init_config.dt;
    config.timeout = init_config.timeout;
    config.ban_shoot_duration = init_config.ban_shoot_duration;
    config.gimbal_switch_duration = init_config.gimbal_switch_duration;
    config.max_switch_angle = init_config.max_switch_angle;
    config.min_switch_angle = init_config.min_switch_angle;
    config.min_shoot_beforehand_vel = init_config.min_shoot_beforehand_vel;
    config.max_chassis_angular_vel = init_config.max_chassis_angular_vel;
    config.track_rotate_target_delay = init_config.track_rotate_target_delay;
    config.track_move_target_delay = init_config.track_move_target_delay;
    config.min_fit_switch_count = init_config.min_fit_switch_count;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .resistance_coff_qd_10 = config.resistance_coff_qd_10,
                        .resistance_coff_qd_15 = config.resistance_coff_qd_15,
                        .resistance_coff_qd_16 = config.resistance_coff_qd_16,
                        .resistance_coff_qd_18 = config.resistance_coff_qd_18,
                        .resistance_coff_qd_30 = config.resistance_coff_qd_30,
                        .g = config.g,
                        .delay = config.delay,
                        .wait_next_armor_delay = config.wait_next_armor_delay,
                        .wait_diagonal_armor_delay = config.wait_diagonal_armor_delay,
                        .dt = config.dt,
                        .timeout = config.timeout,
                        .ban_shoot_duration = config.ban_shoot_duration,
                        .gimbal_switch_duration = config.gimbal_switch_duration,
                        .max_switch_angle = config.max_switch_angle,
                        .min_switch_angle = config.min_switch_angle,
                        .min_shoot_beforehand_vel = config.min_shoot_beforehand_vel,
                        .max_chassis_angular_vel = config.max_chassis_angular_vel,
                        .track_rotate_target_delay = config.track_rotate_target_delay,
                        .track_move_target_delay = config.track_move_target_delay,
                        .min_fit_switch_count = config.min_fit_switch_count };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers
