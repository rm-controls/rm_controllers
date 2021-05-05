//
// Created by qiayuan on 8/14/20.
//

#include "rm_gimbal_controller/bullet_solver.h"
#include <cmath>
#include <tf/transform_datatypes.h>

///////////////////////////BulletSolver/////////////////////////////
void BulletSolver::setResistanceCoefficient(double bullet_speed, Config config) {
  //bullet_speed have 5 value:10,15,16,18,30
  if (bullet_speed < 12.5)
    resistance_coff_ = config.resistance_coff_qd_10;
  else if (bullet_speed < 15.5)
    resistance_coff_ = config.resistance_coff_qd_15;
  else if (bullet_speed < 17)
    resistance_coff_ = config.resistance_coff_qd_16;
  else if (bullet_speed < 24)
    resistance_coff_ = config.resistance_coff_qd_18;
  else
    resistance_coff_ = config.resistance_coff_qd_30;
}

///////////////////////////Bullet3DSolver/////////////////////////////
bool Bullet3DSolver::solve(const DVec<double> &angle_init,
                           double target_position_x, double target_position_y, double target_position_z,
                           double target_speed_x, double target_speed_y, double target_speed_z, double bullet_speed) {
  config_ = *config_rt_buffer_.readFromRT();
  setResistanceCoefficient(bullet_speed, config_);
  pos_[0] = target_position_x;
  pos_[1] = target_position_y;
  pos_[2] = target_position_z;
  vel_[0] = target_speed_x;
  vel_[1] = target_speed_y;
  vel_[2] = target_speed_z;

  this->setBulletSpeed(bullet_speed);
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

  double error_theta_z[2]{};
  double temp_z = target_x_ / cos(yaw_solved_) * tan(pitch_solved_);

  int count = 0;
  double error = 999999;
  while (error >= 0.001) {
    error = computeError(yaw_solved_, pitch_solved_, error_theta_z);
    yaw_solved_ = yaw_solved_ + error_theta_z[0];
    temp_z = temp_z + error_theta_z[1];
    pitch_solved_ = std::atan2(temp_z, std::sqrt(std::pow(target_x_, 2)
                                                     + std::pow(target_y_, 2)));
    if (count >= 20 || std::isnan(error)) {
      if (solve_success_) {
        angle_result_[0] = angle_init[0];
        angle_result_[1] = -angle_init[1];
        solve_success_ = false;
      }
      return false;
    }
    count++;
  }
  solve_success_ = true;
  angle_result_[0] = yaw_solved_;
  angle_result_[1] = -pitch_solved_;

  return true;
}

double Bullet3DSolver::gimbalError(const DVec<double> &angle, double target_position_x,
                                   double target_position_y, double target_position_z,
                                   double target_speed_x, double target_speed_y,
                                   double target_speed_z, double bullet_speed) {
  config_ = *config_rt_buffer_.readFromRT();
  setResistanceCoefficient(bullet_speed, config_);
  pos_[0] = target_position_x;
  pos_[1] = target_position_y;
  pos_[2] = target_position_z;
  vel_[0] = target_speed_x;
  vel_[1] = target_speed_y;
  vel_[2] = target_speed_z;

  this->setBulletSpeed(bullet_speed);
  setTarget(pos_, vel_);
  double error_polar[2]{};
  double error = computeError(angle[0], angle[1], error_polar);

  return error;
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
    this->fly_time_ = (-std::log(1 - rt_bullet_rho * resistance_coff_
        / bullet_v_rho)) / resistance_coff_;
    double rt_bullet_z =
        (bullet_v_z + (config_.g / resistance_coff_))
            * (1 - std::exp(-resistance_coff_ * this->fly_time_))
            / resistance_coff_ - config_.g * this->fly_time_
            / resistance_coff_;
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

Vec2<double> Bullet3DSolver::getResult(const ros::Time &time, geometry_msgs::TransformStamped map2pitch) {
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    model_data_ = getPointData3D();
    modelRviz(map2pitch.transform.translation.x, map2pitch.transform.translation.y, map2pitch.transform.translation.z);
    last_publish_time_ = time;
  }

  return angle_result_;
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

    rt_bullet_rho = (1 / resistance_coff_) * bullet_v_rho
        * (1 - std::exp(-this->fly_time_ * resistance_coff_));

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
  rt_bullet_z = (1 / resistance_coff_)
      * (bullet_v_z + config_.g / resistance_coff_)
      * (1 - std::exp(-this->fly_time_ * resistance_coff_))
      - this->fly_time_ * config_.g / resistance_coff_;

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

  this->fly_time_ = (-std::log(1 - rt_target_rho * resistance_coff_
      / bullet_v_rho)) / resistance_coff_;
  if (std::isnan(this->fly_time_))
    return 999999.;
  double rt_bullet_z =
      (bullet_v_z + (config_.g / resistance_coff_))
          * (1 - std::exp(-resistance_coff_ * this->fly_time_))
          / resistance_coff_ - config_.g * this->fly_time_
          / resistance_coff_;

  double rt_target_theta =
      std::atan2(this->target_y_ + this->target_dy_ * this->fly_time_,
                 this->target_x_ + this->target_dx_ * this->fly_time_);

  error[0] = rt_target_theta - yaw;
  error[1] = this->target_z_ - rt_bullet_z;
  return std::sqrt(
      std::pow(error[0] * rt_target_rho, 2) + std::pow(error[1], 2));
}