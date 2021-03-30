//
// Created by liudec on 2021/3/29.
//

#include "rm_gimbal_controller/kalman_filter.h"

namespace rm_gimbal_controllers {


TranTarget::TranTarget(double dt, double q_x, double q_dx,
                                  double r_x, double r_dx) {
  a_ <<
     1., dt, 0., 0., 0., 0., 0., 0.,
      0., 1., 0., 0., 0., 0., 0., 0.,
      0., 0., 1., dt, 0., 0., 0., 0.,
      0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0., 0., 0., 1., dt, 0., 0.,
      0., 0., 0., 0., 0., 1., 0., 0.,
      0., 0., 0., 0., 0., 0., 1., dt,
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
     q_x, 0., 0., 0., 0., 0., 0., 0.,
      0., q_dx, 0., 0., 0., 0., 0., 0.,
      0., 0., q_x, 0., 0., 0., 0., 0.,
      0., 0., 0., q_dx, 0., 0., 0., 0.,
      0., 0., 0., 0., q_x, 0., 0., 0.,
      0., 0., 0., 0., 0., q_dx, 0., 0.,
      0., 0., 0., 0., 0., 0., q_x, 0.,
      0., 0., 0., 0., 0., 0., 0., q_dx;
  r_ <<
     r_x, 0., 0., 0., 0., 0., 0., 0.,
      0., r_dx, 0., 0., 0., 0., 0., 0.,
      0., 0., r_x, 0., 0., 0., 0., 0.,
      0., 0., 0., r_dx, 0., 0., 0., 0.,
      0., 0., 0., 0., r_x, 0., 0., 0.,
      0., 0., 0., 0., 0., r_dx, 0., 0.,
      0., 0., 0., 0., 0., 0., r_x, 0.,
      0., 0., 0., 0., 0., 0., 0., r_dx;
  x_ << 0., 0., 0., 0., 0., 0., 0., 0.;
  u_ << 0., 0., 0., 0., 0., 0., 0., 0.;

  kf_ = new KalmanFilter<double>(a_, b_, h_, q_, r_);
  kf_->clear(x_);

}

KalmanFilterTrack::KalmanFilterTrack(hardware_interface::RobotStateHandle &robot_state_handle, ros::NodeHandle &controller_nh) {
  robot_state_handle_ = robot_state_handle;
  d_srv_ =
      new dynamic_reconfigure::Server<KalmanConfig>(ros::NodeHandle(controller_nh,"kalman"));
  dynamic_reconfigure::Server<KalmanConfig>::CallbackType
      cb = boost::bind(&KalmanFilterTrack::reconfigCB, this, _1, _2);
  d_srv_->setCallback(cb);
  track_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>(controller_nh,"/track", 10));
}

void KalmanFilterTrack::getState() {
  if (!updated_) {
    return;
  } else {
    updated_ = false;
    rm_msgs::TrackDataArray track_data_array;
    for (const auto &item :id2detection_){
      item.second->predict();
      track_data_array.header.stamp = ros::Time::now();
      rm_msgs::TrackData track_data;
      track_data.id = item.first;
      Vec6<double> state = item.second->getState();
      track_data.pose.position.x = state[0];
      track_data.pose.position.y = state[2];
      track_data.pose.position.z = state[4];
      track_data.pose.orientation.z = state[6];
      track_data.speed.linear.x = state[1];
      track_data.speed.linear.y = state[3];
      track_data.speed.linear.z = state[5];
      track_data.speed.angular.z = state[7];
      track_data_array.tracks.emplace_back(track_data);
    }
    track_pub_->msg_.tracks.clear();
    if (track_pub_->trylock()) {
      track_pub_->msg_ = track_data_array;
      track_pub_->unlockAndPublish();
    }
  }
}
void KalmanFilterTrack::update(realtime_tools::RealtimeBuffer<rm_msgs::TargetDetectionArray>&  detection_rt_buffer) {
  ros::Time now_detection_time = detection_rt_buffer.readFromRT()->header.stamp;
  if(!begin_flag_){
    begin_flag_ = true;
    last_detection_time_ = now_detection_time;
    return;
  }
  if(now_detection_time == last_detection_time_ ){
    updated_ = false;
    return;
  }
  for (const auto &detection : detection_rt_buffer.readFromRT()->detections) {
    geometry_msgs::TransformStamped map2detection_last, map2detection_now;
    if (id2detection_.find(detection.id) == id2detection_.end())
      //do we need init with the x,y,z...
      id2detection_.insert(
          std::make_pair(detection.id, new TranTarget(0.001, q_x_, q_dx_,
                                                      r_x_, r_dx_)));
    try {
      map2detection_last =
          robot_state_handle_.lookupTransform("map", "detection" + std::to_string(detection.id),
                                              last_detection_time_);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      map2detection_last.header.stamp = last_detection_time_;
    }
    try {
      map2detection_now =
          robot_state_handle_.lookupTransform("map", "detection" + std::to_string(detection.id),
                                              now_detection_time);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      map2detection_now.header.stamp = now_detection_time;
    }
    //update last_detection_time_
    last_detection_time_ = now_detection_time;
    double dt = map2detection_now.header.stamp.toSec()
                - map2detection_last.header.stamp.toSec();
    if (dt <= 0.0005)
      return;
    Vec6<double> z; //observe value
    double x_diff = map2detection_now.transform.translation.x
                    - map2detection_last.transform.translation.x;
    double y_diff = map2detection_now.transform.translation.x
                    - map2detection_last.transform.translation.x;
    double z_diff = map2detection_now.transform.translation.x
                    - map2detection_last.transform.translation.x;

    if((std::abs(x_diff) + std::abs(y_diff) + std::abs(z_diff)) > jump_thresh_){
      updated_ = false;
      Vec6<double> x;
      x << map2detection_now.transform.translation.x, 0.,
            map2detection_now.transform.translation.y, 0.,
            map2detection_now.transform.translation.z, 0.,
            0., 0., 0., 0.;
      id2detection_[detection.id]->clear(x);
      return;
    }else{
      z << map2detection_now.transform.translation.x,
          x_diff/dt,
          map2detection_now.transform.translation.y,
          y_diff/dt,
          map2detection_now.transform.translation.z,
          z_diff/dt,
          map2detection_now.transform.rotation.z,
          (map2detection_now.transform.rotation.z-map2detection_last.transform.rotation.z)/dt;
      id2detection_[detection.id]->predict();
      id2detection_[detection.id]->update(z);
      updated_ = true;
    }
  }
}

void KalmanFilterTrack::reconfigCB(const KalmanConfig &config,
                                   uint32_t level) {
  ROS_INFO("[Track] Dynamic params change");
  (void) level;
  q_x_ = config.q_x;
  q_dx_ = config.q_dx;
  r_x_ = config.r_x;
  r_dx_ = config.r_dx;
  jump_thresh_ = config.jump_thresh;
  for (const auto &item:id2detection_) {
    delete item.second;
  }
  id2detection_.clear();
}

}
