//
// Created by chenzheng on 2021/4/17.
//

#include "rm_gimbal_controller/kalman_filter.h"
#include <rm_common/ori_tool.h>

#define  SAMPLE_RATE 1000.

namespace kalman_filter {
KalmanFilterTrack::KalmanFilterTrack(ros::NodeHandle &nh, int id) {
  is_debug_ = getParam(nh, "kalman_debug", false);
  int moving_average_data_num = getParam(nh, "moving_average_data_num", 30);

  // init config
  config_ = {
      .q_x_pos = getParam(nh, "q_x_pos", 1.),
      .q_y_pos = getParam(nh, "q_y_pos", 1.),
      .q_z_pos = getParam(nh, "q_z_pos", 1.),
      .q_yaw_pos = getParam(nh, "q_yaw_pos", 1.),
      .q_x_vel = getParam(nh, "q_x_vel", 1.),
      .q_y_vel = getParam(nh, "q_y_vel", 1.),
      .q_z_vel = getParam(nh, "q_z_vel", 1.),
      .q_yaw_vel = getParam(nh, "q_yaw_vel", 1.),
      .r_x_pos = getParam(nh, "r_x_pos", 1.),
      .r_y_pos = getParam(nh, "r_y_pos", 1.),
      .r_z_pos = getParam(nh, "r_z_pos", 1.),
      .r_yaw_pos = getParam(nh, "r_yaw_pos", 1.),
      .r_x_vel = getParam(nh, "r_x_vel", 1.),
      .r_y_vel = getParam(nh, "r_y_vel", 1.),
      .r_z_vel = getParam(nh, "r_z_vel", 1.),
      .r_yaw_vel = getParam(nh, "r_yaw_vel", 1.)
  };
  config_rt_buffer_.initRT(config_);

  a_ <<
     1., 1. / SAMPLE_RATE, 0., 0., 0., 0., 0., 0.,
      0., 1., 0., 0., 0., 0., 0., 0.,
      0., 0., 1., 1. / SAMPLE_RATE, 0., 0., 0., 0.,
      0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0., 0., 0., 1., 1. / SAMPLE_RATE, 0., 0.,
      0., 0., 0., 0., 0., 1., 0., 0.,
      0., 0., 0., 0., 0., 0., 1., 1. / SAMPLE_RATE,
      0., 0., 0., 0., 0., 0., 0., 1.;
  b_ <<
     0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0.;

  h_ <<
     1., 0., 0., 0., 0., 0., 0., 0.,
      0., 1., 0., 0., 0., 0., 0., 0.,
      0., 0., 1., 0., 0., 0., 0., 0.,
      0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0., 0., 0., 1., 0., 0., 0.,
      0., 0., 0., 0., 0., 1., 0., 0.,
      0., 0., 0., 0., 0., 0., 1., 0.,
      0., 0., 0., 0., 0., 0., 0., 1.;
  q_ <<
     config_.q_x_pos, 0., 0., 0., 0., 0., 0., 0.,
      0., config_.q_x_vel, 0., 0., 0., 0., 0., 0.,
      0., 0., config_.q_y_pos, 0., 0., 0., 0., 0.,
      0., 0., 0., config_.q_y_vel, 0., 0., 0., 0.,
      0., 0., 0., 0., config_.q_z_pos, 0., 0., 0.,
      0., 0., 0., 0., 0., config_.q_z_vel, 0., 0.,
      0., 0., 0., 0., 0., 0., config_.q_yaw_pos, 0.,
      0., 0., 0., 0., 0., 0., 0., config_.q_yaw_vel;
  r_ <<
     config_.r_x_pos, 0., 0., 0., 0., 0., 0., 0.,
      0., config_.r_x_vel, 0., 0., 0., 0., 0., 0.,
      0., 0., config_.r_y_pos, 0., 0., 0., 0., 0.,
      0., 0., 0., config_.r_y_vel, 0., 0., 0., 0.,
      0., 0., 0., 0., config_.r_z_pos, 0., 0., 0.,
      0., 0., 0., 0., 0., config_.r_z_vel, 0., 0.,
      0., 0., 0., 0., 0., 0., config_.r_yaw_pos, 0.,
      0., 0., 0., 0., 0., 0., 0., config_.r_yaw_vel;
  x_ << 0., 0., 0., 0., 0., 0., 0., 0.;
  x_hat_ << 0., 0., 0., 0., 0., 0., 0., 0.;
  u_ << 0., 0., 0., 0., 0., 0., 0., 0.;

  d_srv_ =
      new dynamic_reconfigure::Server<rm_gimbal_controllers::KalmanConfig>(ros::NodeHandle(nh, std::to_string(id)));
  dynamic_reconfigure::Server<rm_gimbal_controllers::KalmanConfig>::CallbackType
      cb = [this](auto &&PH1, auto &&PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  kalman_filter_ = new KalmanFilter<double>(a_, b_, h_, q_, r_);
  kalman_filter_->clear(x_);
  moving_average_filter_x_ = new MovingAverageFilter<double>(moving_average_data_num);
  moving_average_filter_x_->clear();
  moving_average_filter_y_ = new MovingAverageFilter<double>(moving_average_data_num);
  moving_average_filter_y_->clear();
  moving_average_filter_z_ = new MovingAverageFilter<double>(moving_average_data_num);
  moving_average_filter_z_->clear();

  if (is_debug_)
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::KalmanData>(nh, "id" + std::to_string(id), 100));
}

void KalmanFilterTrack::input(const geometry_msgs::TransformStamped &map2detection) {
  Vec8<double> x0{};
  double dt = std::abs(map2detection.header.stamp.toSec() - map2detection_last_.header.stamp.toSec());
  if (dt > 0.5) {
    map2detection_ = map2detection;
    map2detection_now_ = map2detection;
    map2detection_last_ = map2detection;
    last_pos_hat_ = map2detection.transform.translation;
    last_last_pos_hat_ = map2detection.transform.translation;
    x0[0] = map2detection.transform.translation.x;
    x0[2] = map2detection.transform.translation.y;
    x0[4] = map2detection.transform.translation.z;
    kalman_filter_->clear(x0);
    moving_average_filter_x_->clear();
    moving_average_filter_y_->clear();
    moving_average_filter_z_->clear();
    is_filter_ = false;
    is_gyro_ = false;
    switch_count_ = 0;
    return;
  }
  is_filter_ = true;

  //abandon obvious error data
  map2detection_now_.header.stamp = map2detection.header.stamp;
  double delta_x = map2detection.transform.translation.x - map2detection_last_.transform.translation.x;
  double delta_y = map2detection.transform.translation.y - map2detection_last_.transform.translation.y;
  double delta_z = map2detection.transform.translation.z - map2detection_last_.transform.translation.z;
  if (std::abs(delta_x) < 0.5 && std::abs(delta_y) < 0.5 && std::abs(delta_z) < 0.5)
    map2detection_now_.transform = map2detection.transform;
  map2detection_ = map2detection_now_;

  double delta = map2detection_now_.transform.translation.y - map2detection_last_.transform.translation.y;
  //If true, the target armor is switching
  if (std::abs(delta) > 0.1) {
    x0[0] = map2detection_now_.transform.translation.x;
    x0[2] = map2detection_now_.transform.translation.y;
    x0[4] = map2detection_now_.transform.translation.z;
    last_last_pos_hat_.y = map2detection_now_.transform.translation.y;
    last_pos_hat_.y = last_last_pos_hat_.y + dt * x_hat_[3];
    x0[3] = (last_pos_hat_.y - last_last_pos_hat_.y) / dt;
    kalman_filter_->clear(x0);

    //if switch number bigger than 3, so it is in gyro
    if (switch_count_ <= 3)
      switch_count_++;
    else {
      enter_gyro_time_ = map2detection_now_.header.stamp;
      is_gyro_ = true;
    }
  } else
    is_gyro_ = std::abs(map2detection_now_.header.stamp.toSec() - enter_gyro_time_.toSec()) <= 1.0;

  moving_average_filter_x_->input(map2detection_now_.transform.translation.x);
  moving_average_filter_y_->input(map2detection_now_.transform.translation.y);
  moving_average_filter_z_->input(map2detection_now_.transform.translation.z);
  center_.x = moving_average_filter_x_->output();
  center_.y = moving_average_filter_y_->output();
  center_.z = moving_average_filter_z_->output();

  double roll_now{}, pitch_now{}, yaw_now{}, roll_last{}, pitch_last{}, yaw_last{};
  quatToRPY(map2detection_now_.transform.rotation, roll_now, pitch_now, yaw_now);
  quatToRPY(map2detection_last_.transform.rotation, roll_last, pitch_last, yaw_last);
  yaw_now = yaw_now > 0.0 ? yaw_now - 3.14 : yaw_now + 3.14;
  yaw_last = yaw_last > 0.0 ? yaw_last - 3.14 : yaw_last + 3.14;

  x_[0] = map2detection_now_.transform.translation.x;
  x_[2] = map2detection_now_.transform.translation.y;
  x_[4] = map2detection_now_.transform.translation.z;
  x_[6] = yaw_now;

  x_[1] = std::abs((last_pos_hat_.x - last_last_pos_hat_.x) / dt - x_[1]) < 1.0 ?
          (last_pos_hat_.x - last_last_pos_hat_.x) / dt : x_[1];
  x_[3] = std::abs((last_pos_hat_.y - last_last_pos_hat_.y) / dt - x_[3]) < 1.0 ?
          (last_pos_hat_.y - last_last_pos_hat_.y) / dt : x_[3];
  x_[5] = std::abs((last_pos_hat_.z - last_last_pos_hat_.z) / dt - x_[5]) < 1.0 ?
          (last_pos_hat_.z - last_last_pos_hat_.z) / dt : x_[5];
  x_[7] = std::abs((yaw_now - yaw_last) / dt) < 10.0 ? (yaw_now - yaw_last) / dt : x_[7];

  updateQR();
  kalman_filter_->predict(u_, q_);
  kalman_filter_->update(x_, r_);
  x_hat_ = kalman_filter_->getState();

  last_last_pos_hat_ = last_pos_hat_;
  last_pos_hat_.x = x_hat_[0];
  last_pos_hat_.y = x_hat_[2];
  last_pos_hat_.z = x_hat_[4];
  map2detection_last_ = map2detection_now_;

  if (is_debug_) {
    kalman_data_.header.stamp = map2detection_now_.header.stamp;

    kalman_data_.real_detection_pose.position.x = x_[0];
    kalman_data_.real_detection_pose.position.y = x_[2];
    kalman_data_.real_detection_pose.position.z = x_[4];
    kalman_data_.real_detection_pose.orientation.z = x_[6];
    kalman_data_.real_detection_twist.linear.x = x_[1];
    kalman_data_.real_detection_twist.linear.y = x_[3];
    kalman_data_.real_detection_twist.linear.z = x_[5];
    kalman_data_.real_detection_twist.angular.z = x_[7];

    kalman_data_.filtered_detection_pose.position.x = x_hat_[0];
    kalman_data_.filtered_detection_pose.position.y = x_hat_[2];
    kalman_data_.filtered_detection_pose.position.z = x_hat_[4];
    kalman_data_.filtered_detection_pose.orientation.z = x_hat_[6];
    kalman_data_.filtered_detection_twist.linear.x = x_hat_[1];
    kalman_data_.filtered_detection_twist.linear.y = x_hat_[3];
    kalman_data_.filtered_detection_twist.linear.z = x_hat_[5];
    kalman_data_.filtered_detection_twist.angular.z = x_hat_[7];

    if (realtime_pub_->trylock()) {
      realtime_pub_->msg_ = kalman_data_;
      realtime_pub_->unlockAndPublish();
    }
  }
}

geometry_msgs::TransformStamped KalmanFilterTrack::getTransform() {
  if (is_filter_) {
    map2detection_.transform.translation.x = x_hat_[0];
    map2detection_.transform.translation.y = x_hat_[2];
    map2detection_.transform.translation.z = x_hat_[4];
  }
  return map2detection_;
}

geometry_msgs::Twist KalmanFilterTrack::getTwist() {
  geometry_msgs::Twist target_vel;
  if (is_filter_) {
    target_vel.linear.x = x_hat_[1];
    target_vel.linear.y = x_hat_[3];
    target_vel.linear.z = x_hat_[5];
    target_vel.angular.z = x_hat_[7];
  }
  return target_vel;
}

geometry_msgs::Vector3 KalmanFilterTrack::getCenter() const {
  return center_;
}

bool KalmanFilterTrack::isGyro() const {
  return is_gyro_;
}

void KalmanFilterTrack::updateQR() {
  config_ = *config_rt_buffer_.readFromRT();
  q_ <<
     config_.q_x_pos, 0., 0., 0., 0., 0., 0., 0.,
      0., config_.q_x_vel, 0., 0., 0., 0., 0., 0.,
      0., 0., config_.q_y_pos, 0., 0., 0., 0., 0.,
      0., 0., 0., config_.q_y_vel, 0., 0., 0., 0.,
      0., 0., 0., 0., config_.q_z_pos, 0., 0., 0.,
      0., 0., 0., 0., 0., config_.q_z_vel, 0., 0.,
      0., 0., 0., 0., 0., 0., config_.q_yaw_pos, 0.,
      0., 0., 0., 0., 0., 0., 0., config_.q_yaw_vel;
  r_ <<
     config_.r_x_pos, 0., 0., 0., 0., 0., 0., 0.,
      0., config_.r_x_vel, 0., 0., 0., 0., 0., 0.,
      0., 0., config_.r_y_pos, 0., 0., 0., 0., 0.,
      0., 0., 0., config_.r_y_vel, 0., 0., 0., 0.,
      0., 0., 0., 0., config_.r_z_pos, 0., 0., 0.,
      0., 0., 0., 0., 0., config_.r_z_vel, 0., 0.,
      0., 0., 0., 0., 0., 0., config_.r_yaw_pos, 0.,
      0., 0., 0., 0., 0., 0., 0., config_.r_yaw_vel;
}

void KalmanFilterTrack::perdict() {
  kalman_filter_->predict(u_, q_);
}

void KalmanFilterTrack::reconfigCB(rm_gimbal_controllers::KalmanConfig &config, uint32_t) {
  ROS_INFO("[Track] Dynamic params change");
  if (!dynamic_reconfig_initialized_) {
    Config init_config = *config_rt_buffer_.readFromNonRT(); // config init use yaml
    config.q_x_pos = init_config.q_x_pos;
    config.q_y_pos = init_config.q_y_pos;
    config.q_z_pos = init_config.q_z_pos;
    config.q_yaw_pos = init_config.q_yaw_pos;
    config.q_x_vel = init_config.q_x_vel;
    config.q_y_vel = init_config.q_y_vel;
    config.q_z_vel = init_config.q_z_vel;
    config.q_yaw_vel = init_config.q_yaw_vel;
    config.r_x_pos = init_config.r_x_pos;
    config.r_y_pos = init_config.r_y_pos;
    config.r_z_pos = init_config.r_z_pos;
    config.r_yaw_pos = init_config.r_yaw_pos;
    config.r_x_vel = init_config.r_x_vel;
    config.r_y_vel = init_config.r_y_vel;
    config.r_z_vel = init_config.r_z_vel;
    config.r_yaw_vel = init_config.r_yaw_vel;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{
      .q_x_pos = config.q_x_pos,
      .q_y_pos = config.q_y_pos,
      .q_z_pos = config.q_z_pos,
      .q_yaw_pos = config.q_yaw_pos,
      .q_x_vel = config.q_x_vel,
      .q_y_vel = config.q_y_vel,
      .q_z_vel = config.q_z_vel,
      .q_yaw_vel = config.q_yaw_vel,
      .r_x_pos = config.r_x_pos,
      .r_y_pos = config.r_y_pos,
      .r_z_pos = config.r_z_pos,
      .r_yaw_pos = config.r_yaw_pos,
      .r_x_vel = config.r_x_vel,
      .r_y_vel = config.r_y_vel,
      .r_z_vel = config.r_z_vel,
      .r_yaw_vel = config.r_yaw_vel
  };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}
