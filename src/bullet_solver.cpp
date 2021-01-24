//
// Created by qiayuan on 8/14/20.
//

#include "rm_gimbal_controller/bullet_solver.h"
#include <cmath>
#include <rm_base/hardware_interface/robot_state_interface.h>
#include <tf/transform_datatypes.h>
///////////////////////////BulletSolver/////////////////////////////

///////////////////////////Bullet2DSolver/////////////////////////////
//2D solver need to make some changes to use it
bool Bullet2DSolver::solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped map2pitch,
                           realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) {
  double pitch_point = std::atan2(target_z_, target_x_);
  double error_point = computeError(pitch_point);
  double error_init = computeError(angle_init[0]);
  //compare pitch angle which direct pointing to target and angle provide by user
  pitch_solved_ = error_init > error_point ? pitch_point : angle_init[0];
  double error_z = error_init > error_point ? error_point : error_init;
  double temp_z = this->target_x_ * std::tan(pitch_solved_);

  int count = 0;
  while (std::abs(error_z) >= 0.000001) {
    temp_z = temp_z + error_z;
    pitch_solved_ = std::atan2(temp_z, this->target_x_);
    error_z = computeError(pitch_solved_);
    if (count >= 20 || error_z == 999999.) {
      return false;
    }
    count++;
  }
  return true;
}

///////////////////////////Iter2DSolver/////////////////////////////
double Iter2DSolver::computeError(double pitch) {
  double rt_target_x = this->target_x_;
  double rt_target_z = this->target_z_;
  double rt_bullet_x{}, rt_bullet_z{};
  double bullet_v_x = this->bullet_speed_ * std::cos(pitch);
  double bullet_v_z = this->bullet_speed_ * std::sin(pitch);

  this->fly_time_ = 0;
  while (rt_bullet_x <= rt_target_x) {
    this->fly_time_ += config_.dt;
    rt_bullet_x = (1 / config_.resistance_coff) * bullet_v_x
        * (1 - std::exp(-this->fly_time_ * config_.resistance_coff));
    rt_target_x += this->target_dx_ * config_.dt;

    //avoid keep looping cause by null solution
    if (this->fly_time_ > config_.timeout) {
      return 999999.;
    }
  }
  rt_bullet_z = (1 / config_.resistance_coff)
      * (bullet_v_z + config_.g / config_.resistance_coff)
      * (1 - std::exp(-this->fly_time_ * config_.resistance_coff))
      - this->fly_time_ * config_.g / config_.resistance_coff;
  rt_target_z = this->target_z_ + this->target_dz_ * this->fly_time_;

  return rt_target_z - rt_bullet_z;
}

///////////////////////////Approx2DSolver/////////////////////////////
double Approx2DSolver::computeError(double pitch) {

  this->fly_time_ = (-log(1 - this->target_x_ * config_.resistance_coff
      / (this->bullet_speed_ * cos(pitch) - this->target_dx_)))
      / config_.resistance_coff;
  if (std::isnan(this->fly_time_))
    return 999999.;
  double rt_bullet_z =
      ((this->bullet_speed_ * sin(pitch) - this->target_dz_)
          + (config_.g / config_.resistance_coff))
          * (1 - std::exp(-config_.resistance_coff * this->fly_time_))
          / config_.resistance_coff - config_.g * this->fly_time_
          / config_.resistance_coff;

  return this->target_z_ - rt_bullet_z;
}

///////////////////////////Bullet3DSolver/////////////////////////////
bool Bullet3DSolver::solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped map2pitch,
                           realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) {
  config_ = *config_rt_buffer_.readFromRT();
  map2pitch_offset_x_ = map2pitch.transform.translation.x;
  map2pitch_offset_y_ = map2pitch.transform.translation.y;
  map2pitch_offset_z_ = map2pitch.transform.translation.z;
  pos_[0] = cmd_track_rt_buffer.readFromRT()->target_position_x - map2pitch_offset_x_;
  pos_[1] = cmd_track_rt_buffer.readFromRT()->target_position_y - map2pitch_offset_y_;
  pos_[2] = cmd_track_rt_buffer.readFromRT()->target_position_z - map2pitch_offset_z_;
  vel_[0] = cmd_track_rt_buffer.readFromRT()->target_speed_x;
  vel_[1] = cmd_track_rt_buffer.readFromRT()->target_speed_y;
  vel_[2] = cmd_track_rt_buffer.readFromRT()->target_speed_z;

  this->setBulletSpeed(cmd_track_rt_buffer.readFromRT()->bullet_speed);
  setTarget(pos_, vel_);

  double error_theta_z_init[2]{}, error_theta_z_point[2]{};
  double yaw_point = std::atan2(target_y_, target_x_);
  double pitch_point = std::atan2(
      target_z_, std::sqrt(std::pow(target_x_, 2) + std::pow(target_y_, 2)));
  double error_init = computeError(angle_init[0], angle_init[1], error_theta_z_init);
  double error_point = computeError(yaw_point, pitch_point, error_theta_z_point);

  //compare pitch and yaw angle which direct pointing to target and angle provide by user
  if (error_init > error_point) {
    yaw_solved_ = yaw_point;
    pitch_solved_ = pitch_point;
  } else {
    yaw_solved_ = angle_init[0];
    pitch_solved_ = angle_init[1];
  }

  double error_theta_z[2] =
      {error_init > error_point ? error_theta_z_init[0]
                                : error_theta_z_point[0],
       error_init > error_point ? error_theta_z_init[1]
                                : error_theta_z_point[1]};
  double temp_z = target_x_ / cos(yaw_solved_) * tan(pitch_solved_);

  int count = 0;
  double error = 999999;
  while (error >= 0.00001) {
    error = computeError(yaw_solved_, pitch_solved_, error_theta_z);
    yaw_solved_ = yaw_solved_ + error_theta_z[0];
    temp_z = temp_z + error_theta_z[1];
    pitch_solved_ = std::atan2(temp_z, std::sqrt(std::pow(target_x_, 2)
                                                     + std::pow(target_y_, 2)));
    if (count >= 20 || std::isnan(error))
      return false;
    count++;
  }

  return true;
}

std::vector<Vec3<double>> Bullet3DSolver::getPointData3D() {
  double target_x = this->target_x_;
  double target_y = this->target_y_;
  double target_rho =
      std::sqrt(std::pow(target_x, 2) + std::pow(target_y, 2));
  double target_v_rho =
      std::cos(yaw_solved_) * this->target_dx_ + std::sin(yaw_solved_) * this->target_dy_;
  double bullet_v_rho = this->bullet_speed_ * std::cos(pitch_solved_) - target_v_rho;
  double bullet_v_z = this->bullet_speed_ * std::sin(pitch_solved_) - this->target_dz_;
  Vec3<double> point_data{};
  std::vector<Vec3<double>> model_data{};
  for (int i = 0; i < 20; i++) {
    double rt_bullet_rho = target_rho * i / 19;
    this->fly_time_ = (-std::log(1 - rt_bullet_rho * config_.resistance_coff
        / bullet_v_rho)) / config_.resistance_coff;
    double rt_bullet_z =
        (bullet_v_z + (config_.g / config_.resistance_coff))
            * (1 - std::exp(-config_.resistance_coff * this->fly_time_))
            / config_.resistance_coff - config_.g * this->fly_time_
            / config_.resistance_coff;
    point_data[0] = rt_bullet_rho * std::cos(yaw_solved_);
    point_data[1] = rt_bullet_rho * std::sin(yaw_solved_);
    point_data[2] = rt_bullet_z;
    model_data.push_back(point_data);
  }
  return model_data;
}

void Bullet3DSolver::modelRviz(double x_offset, double y_offset, double z_offset) {
  geometry_msgs::Point point;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "model";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (int i = 0; i < 20; i++) {
    point.x = model_data_[i][0] + x_offset;
    point.y = model_data_[i][1] + y_offset;
    point.z = model_data_[i][2] + z_offset;
    marker.points.push_back(point);
  }
  marker.header.stamp = ros::Time::now();
  this->path_pub_.publish(marker);
}

geometry_msgs::TransformStamped Bullet3DSolver::getResult(const ros::Time &time) {
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    model_data_ = getPointData3D();
    modelRviz(map2pitch_offset_x_,
              map2pitch_offset_y_,
              map2pitch_offset_z_);
    last_publish_time_ = time;
  }
  map2gimbal_des_.transform.rotation =
      tf::createQuaternionMsgFromRollPitchYaw(0, -pitch_solved_, yaw_solved_);
  map2gimbal_des_.header.stamp = time;
  return map2gimbal_des_;
}

///////////////////////////Iter3DSolver/////////////////////////////
double Iter3DSolver::computeError(double yaw, double pitch, double *error) {
  double rt_target_x = this->target_x_;
  double rt_target_y = this->target_y_;
  double rt_target_rho =
      std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));
  double rt_bullet_rho{}, rt_bullet_z{};

  double bullet_v_rho = this->bullet_speed_ * std::cos(pitch);
  double bullet_v_z = this->bullet_speed_ * std::sin(pitch);

  this->fly_time_ = 0;
  while (rt_bullet_rho <= rt_target_rho) {
    this->fly_time_ += config_.dt;

    rt_bullet_rho = (1 / config_.resistance_coff) * bullet_v_rho
        * (1 - std::exp(-this->fly_time_ * config_.resistance_coff));

    rt_target_x += this->target_dx_ * config_.dt;
    rt_target_y += this->target_dy_ * config_.dt;
    rt_target_rho =
        std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));

    //avoid keep looping cause by null solution
    if (this->fly_time_ > config_.timeout)
      return 999999.;

  }

  double rt_target_theta = std::atan2(rt_target_y, rt_target_x);
  double rt_target_z = this->target_z_ + this->target_dz_ * this->fly_time_;

  double rt_bullet_theta = yaw;
  rt_bullet_z = (1 / config_.resistance_coff)
      * (bullet_v_z + config_.g / config_.resistance_coff)
      * (1 - std::exp(-this->fly_time_ * config_.resistance_coff))
      - this->fly_time_ * config_.g / config_.resistance_coff;

  error[0] = rt_target_theta - rt_bullet_theta;
  error[1] = rt_target_z - rt_bullet_z;

  return std::sqrt(
      std::pow(error[0] * rt_bullet_rho, 2) + std::pow(error[1], 2));
}

double Approx3DSolver::computeError(double yaw, double pitch, double *error) {
  double rt_target_x = this->target_x_;
  double rt_target_y = this->target_y_;
  double rt_target_rho =
      std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));

  double target_v_rho =
      std::cos(yaw) * this->target_dx_ + std::sin(yaw) * this->target_dy_;
  double bullet_v_rho = this->bullet_speed_ * std::cos(pitch) - target_v_rho;
  double bullet_v_z = this->bullet_speed_ * std::sin(pitch) - this->target_dz_;

  this->fly_time_ = (-std::log(1 - rt_target_rho * config_.resistance_coff
      / bullet_v_rho)) / config_.resistance_coff;
  if (std::isnan(this->fly_time_))
    return 999999.;
  double rt_bullet_z =
      (bullet_v_z + (config_.g / config_.resistance_coff))
          * (1 - std::exp(-config_.resistance_coff * this->fly_time_))
          / config_.resistance_coff - config_.g * this->fly_time_
          / config_.resistance_coff;

  double rt_target_theta =
      std::atan2(this->target_y_ + this->target_dy_ * this->fly_time_,
                 this->target_x_ + this->target_dx_ * this->fly_time_);

  error[0] = rt_target_theta - yaw;
  error[1] = this->target_z_ - rt_bullet_z;
  return std::sqrt(
      std::pow(error[0] * rt_target_rho, 2) + std::pow(error[1], 2));
}