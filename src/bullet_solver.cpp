//
// Created by qiayuan on 8/14/20.
//

#include "rm_gimbal_controller/bullet_solver.h"
#include <cmath>
#include <rm_base/hardware_interface/robot_state_interface.h>
#include <tf2/exceptions.h>
#include <tf/transform_datatypes.h>
#include "rm_gimbal_controller/standard.h"
///////////////////////////BulletSolver/////////////////////////////
template
class BulletSolver<double>;
template
class BulletSolver<float>;

///////////////////////////Bullet2DSolver/////////////////////////////
template<typename T>
void Bullet2DSolver<T>::solve(const DVec<T> &angle_init) {
  T pitch_point = std::atan2(target_z_, target_x_);
  T error_point = computeError(pitch_point);
  T error_init = computeError(angle_init[0]);
  //compare pitch angle which direct pointing to target and angle provide by user
  pitch_solved_ = error_init > error_point ? pitch_point : angle_init[0];
  T error_z = error_init > error_point ? error_point : error_init;
  T temp_z = this->target_x_ * std::tan(pitch_solved_);

  int count = 0;
  while (std::abs(error_z) >= 0.000001) {
    temp_z = temp_z + error_z;
    pitch_solved_ = std::atan2(temp_z, this->target_x_);
    error_z = computeError(pitch_solved_);
    if (count >= 20 || error_z == 999999.) {
      pitch_solved_ = angle_init[0];
      break;
    }
    count++;
  }
}

template
class Bullet2DSolver<double>;
template
class Bullet2DSolver<float>;

///////////////////////////Iter2DSolver/////////////////////////////
template<typename T>
T Iter2DSolver<T>::computeError(T pitch) {
  T rt_target_x = this->target_x_;
  T rt_target_z = this->target_z_;
  T rt_bullet_x{}, rt_bullet_z{};
  T bullet_v_x = this->bullet_speed_ * std::cos(pitch);
  T bullet_v_z = this->bullet_speed_ * std::sin(pitch);

  this->fly_time_ = 0;
  while (rt_bullet_x <= rt_target_x) {
    this->fly_time_ += this->dt_;
    rt_bullet_x = (1 / this->resistance_coff_) * bullet_v_x
        * (1 - std::exp(-this->fly_time_ * this->resistance_coff_));
    rt_target_x += this->target_dx_ * this->dt_;

    //avoid keep looping cause by null solution
    if (this->fly_time_ > this->timeout_) {
      return 999999.;
    }
  }
  rt_bullet_z = (1 / this->resistance_coff_)
      * (bullet_v_z + this->g_ / this->resistance_coff_)
      * (1 - std::exp(-this->fly_time_ * this->resistance_coff_))
      - this->fly_time_ * this->g_ / this->resistance_coff_;
  rt_target_z = this->target_z_ + this->target_dz_ * this->fly_time_;

  return rt_target_z - rt_bullet_z;
}

template
class Iter2DSolver<double>;
template
class Iter2DSolver<float>;

///////////////////////////Approx2DSolver/////////////////////////////
template<typename T>
T Approx2DSolver<T>::computeError(T pitch) {

  this->fly_time_ = (-log(1 - this->target_x_ * this->resistance_coff_
      / (this->bullet_speed_ * cos(pitch) - this->target_dx_)))
      / this->resistance_coff_;
  if (std::isnan(this->fly_time_))
    return 999999.;
  T rt_bullet_z =
      ((this->bullet_speed_ * sin(pitch) - this->target_dz_)
          + (this->g_ / this->resistance_coff_))
          * (1 - std::exp(-this->resistance_coff_ * this->fly_time_))
          / this->resistance_coff_ - this->g_ * this->fly_time_
          / this->resistance_coff_;

  return this->target_z_ - rt_bullet_z;
}

template
class Approx2DSolver<double>;
template
class Approx2DSolver<float>;

///////////////////////////Bullet3DSolver/////////////////////////////
template<typename T>
void Bullet3DSolver<T>::solve(const DVec<T> &angle_init) {
  T error_theta_z_init[2]{}, error_theta_z_point[2]{};

  T error_init = computeError(angle_init[0], angle_init[1], error_theta_z_init);

  T yaw_point = std::atan2(target_y_, target_x_);
  T pitch_point = (T) std::atan2(
      target_z_, std::sqrt(std::pow(target_x_, 2) + std::pow(target_y_, 2)));
  T error_point = computeError(yaw_point, pitch_point, error_theta_z_point);

  //compare pitch and yaw angle which direct pointing to target and angle provide by user
  if (error_init > error_point) {
    yaw_solved_ = yaw_point;
    pitch_solved_ = pitch_point;
  } else {
    yaw_solved_ = angle_init[0];
    pitch_solved_ = angle_init[1];
  }

  T error_theta_z[2] =
      {error_init > error_point ? error_theta_z_init[0]
                                : error_theta_z_point[0],
       error_init > error_point ? error_theta_z_init[1]
                                : error_theta_z_point[1]};
  T temp_z = target_x_ / cos(yaw_solved_) * tan(pitch_solved_);

  int count = 0;
  T error = 999999;
  while (error >= 0.00001) {
    error = computeError(yaw_solved_, pitch_solved_, error_theta_z);
    yaw_solved_ = yaw_solved_ + error_theta_z[0];
    temp_z = temp_z + error_theta_z[1];
    pitch_solved_ = std::atan2(temp_z, std::sqrt(std::pow(target_x_, 2)
                                                     + std::pow(target_y_, 2)));
    if (count >= 20) {
      yaw_solved_ = angle_init[0];
      pitch_solved_ = angle_init[1];
      break;
    }
    count++;
  }
}

template<typename T>
std::vector<Vec3<T>> Bullet3DSolver<T>::getPointData3D() {
  T target_x = this->target_x_;
  T target_y = this->target_y_;
  T target_rho =
      std::sqrt(std::pow(target_x, 2) + std::pow(target_y, 2));
  T target_v_rho =
      std::cos(yaw_solved_) * this->target_dx_ + std::sin(yaw_solved_) * this->target_dy_;
  T bullet_v_rho = this->bullet_speed_ * std::cos(pitch_solved_) - target_v_rho;
  T bullet_v_z = this->bullet_speed_ * std::sin(pitch_solved_) - this->target_dz_;
  Vec3<T> point_data{};
  std::vector<Vec3<T>> model_data{};
  for (int i = 0; i < 20; i++) {
    T rt_bullet_rho = target_rho * i / 19;
    this->fly_time_ = (-std::log(1 - rt_bullet_rho * this->resistance_coff_
        / bullet_v_rho)) / this->resistance_coff_;
    T rt_bullet_z =
        (bullet_v_z + (this->g_ / this->resistance_coff_))
            * (1 - std::exp(-this->resistance_coff_ * this->fly_time_))
            / this->resistance_coff_ - this->g_ * this->fly_time_
            / this->resistance_coff_;
    point_data[0] = rt_bullet_rho * std::cos(yaw_solved_);
    point_data[1] = rt_bullet_rho * std::sin(yaw_solved_);
    point_data[2] = rt_bullet_z;
    model_data.push_back(point_data);
  }
  return model_data;
}
template<typename T>
void Bullet3DSolver<T>::modelRviz(double x_deviation, double y_deviation, double z_deviation) {
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
    point.x = model_data_[i][0] + x_deviation;
    point.y = model_data_[i][1] + y_deviation;
    point.z = model_data_[i][2] + z_deviation;
    marker.points.push_back(point);
  }
  marker.header.stamp = ros::Time::now();
  this->path_pub_.publish(marker);
}

template<typename T>
geometry_msgs::TransformStamped Bullet3DSolver<T>::run(const ros::Time &time,
                                                       realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) {
  geometry_msgs::TransformStamped world2pitch;
  geometry_msgs::TransformStamped world2gimbal_des;
  hardware_interface::RobotStateHandle robot_state_handle;
  world2pitch = robot_state_handle.lookupTransform("world", "pitch", ros::Time(0));

  pos_[0] = cmd_track_rt_buffer.readFromRT()->target_position_x - world2pitch.transform.translation.x;
  pos_[1] = cmd_track_rt_buffer.readFromRT()->target_position_y - world2pitch.transform.translation.y;
  pos_[2] = cmd_track_rt_buffer.readFromRT()->target_position_z - world2pitch.transform.translation.z;
  vel_[0] = cmd_track_rt_buffer.readFromRT()->target_speed_x;
  vel_[1] = cmd_track_rt_buffer.readFromRT()->target_speed_y;
  vel_[2] = cmd_track_rt_buffer.readFromRT()->target_speed_z;

  this->setBulletSpeed(cmd_track_rt_buffer.readFromRT()->bullet_speed);
  setTarget(pos_, vel_);
  solve(angle_init_);
  output(angle_solved_);
  model_data_ = getPointData3D();

  world2gimbal_des.transform.rotation =
      tf::createQuaternionMsgFromRollPitchYaw(0, angle_solved_[0], -angle_solved_[1]);
  world2gimbal_des.header.stamp = time;

  modelRviz(world2pitch.transform.translation.x,
            world2pitch.transform.translation.y,
            world2pitch.transform.translation.z);
  return world2gimbal_des;
}

template
class Bullet3DSolver<double>;
template
class Bullet3DSolver<float>;

///////////////////////////Iter3DSolver/////////////////////////////
template<typename T>
double Iter3DSolver<T>::computeError(T yaw, T pitch, T *error) {
  T rt_target_x = this->target_x_;
  T rt_target_y = this->target_y_;
  T rt_target_rho =
      std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));
  T rt_bullet_rho{}, rt_bullet_z{};

  T bullet_v_rho = this->bullet_speed_ * std::cos(pitch);
  T bullet_v_z = this->bullet_speed_ * std::sin(pitch);

  this->fly_time_ = 0;
  while (rt_bullet_rho <= rt_target_rho) {
    this->fly_time_ += this->dt_;

    rt_bullet_rho = (1 / this->resistance_coff_) * bullet_v_rho
        * (1 - std::exp(-this->fly_time_ * this->resistance_coff_));

    rt_target_x += this->target_dx_ * this->dt_;
    rt_target_y += this->target_dy_ * this->dt_;
    rt_target_rho =
        std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));

    //avoid keep looping cause by null solution
    if (this->fly_time_ > this->timeout_)
      return 999999.;

  }

  T rt_target_theta = std::atan2(rt_target_y, rt_target_x);
  T rt_target_z = this->target_z_ + this->target_dz_ * this->fly_time_;

  T rt_bullet_theta = yaw;
  rt_bullet_z = (1 / this->resistance_coff_)
      * (bullet_v_z + this->g_ / this->resistance_coff_)
      * (1 - std::exp(-this->fly_time_ * this->resistance_coff_))
      - this->fly_time_ * this->g_ / this->resistance_coff_;

  error[0] = rt_target_theta - rt_bullet_theta;
  error[1] = rt_target_z - rt_bullet_z;

  return std::sqrt(
      std::pow(error[0] * rt_bullet_rho, 2) + std::pow(error[1], 2));
}

template
class Iter3DSolver<double>;
template
class Iter3DSolver<float>;

template<typename T>
double Approx3DSolver<T>::computeError(T yaw, T pitch, T *error) {
  T rt_target_x = this->target_x_;
  T rt_target_y = this->target_y_;
  T rt_target_rho =
      std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));

  T target_v_rho =
      std::cos(yaw) * this->target_dx_ + std::sin(yaw) * this->target_dy_;
  T bullet_v_rho = this->bullet_speed_ * std::cos(pitch) - target_v_rho;
  T bullet_v_z = this->bullet_speed_ * std::sin(pitch) - this->target_dz_;

  this->fly_time_ = (-std::log(1 - rt_target_rho * this->resistance_coff_
      / bullet_v_rho)) / this->resistance_coff_;
  if (std::isnan(this->fly_time_))
    return 999999.;
  T rt_bullet_z =
      (bullet_v_z + (this->g_ / this->resistance_coff_))
          * (1 - std::exp(-this->resistance_coff_ * this->fly_time_))
          / this->resistance_coff_ - this->g_ * this->fly_time_
          / this->resistance_coff_;

  T rt_target_theta =
      std::atan2(this->target_y_ + this->target_dy_ * this->fly_time_,
                 this->target_x_ + this->target_dx_ * this->fly_time_);

  error[0] = rt_target_theta - yaw;
  error[1] = this->target_z_ - rt_bullet_z;
  return std::sqrt(
      std::pow(error[0] * rt_target_rho, 2) + std::pow(error[1], 2));
}

template
class Approx3DSolver<double>;
template
class Approx3DSolver<float>;
