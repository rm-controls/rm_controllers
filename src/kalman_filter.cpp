//
// Created by liudec on 2021/3/29.
//

#include "rm_gimbal_controller/kalman_filter.h"
#include <rm_common/ori_tool.h>

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

KalmanFilterTrack::KalmanFilterTrack(hardware_interface::RobotStateHandle &robot_state_handle,
                                     ros::NodeHandle &controller_nh) {
  robot_state_handle_ = robot_state_handle;
  d_srv_ =
      new dynamic_reconfigure::Server<KalmanConfig>(ros::NodeHandle(controller_nh, "kalman"));
  dynamic_reconfigure::Server<KalmanConfig>::CallbackType
      cb = boost::bind(&KalmanFilterTrack::reconfigCB, this, _1, _2);
  d_srv_->setCallback(cb);
  ros::NodeHandle global_nh("~/");
  track_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>(global_nh, "/track", 100));
  track_test_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>(global_nh, "/track_test", 100));
  tf_broadcaster_.init(global_nh);
}

void KalmanFilterTrack::getStateAndPub() {

  rm_msgs::TrackDataArray track_data_array;
  for (const auto &item :id2detection_) {
    item.second->predict();
    track_data_array.header.stamp = ros::Time::now();
    rm_msgs::TrackData track_data;
    track_data.id = item.first;
    Vec8<double> state = item.second->getState();
    track_data.pose2map.position.x = state[0];
    track_data.pose2map.position.y = state[2];
    track_data.pose2map.position.z = state[4];
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, state[6]);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    track_data.pose2map.orientation = quat_msg;

    track_data.speed2map.linear.x = state[1];
    track_data.speed2map.linear.y = state[3];
    track_data.speed2map.linear.z = state[5];
    track_data.speed2map.angular.z = state[7];

    tf2::Transform camera2detection_tf, map2detection_tf;
    geometry_msgs::TransformStamped map2detection, map2camera, camera2detection;
    tf2::fromMsg(track_data.pose2map, map2detection_tf);
    camera2detection_tf = map2camera_tf_.inverse() * map2detection_tf;
    camera2detection.transform = tf2::toMsg(camera2detection_tf);

    track_data.pose2camera.position.x = camera2detection.transform.translation.x;
    track_data.pose2camera.position.y = camera2detection.transform.translation.y;
    track_data.pose2camera.position.z = camera2detection.transform.translation.z;

    track_data_array.tracks.emplace_back(track_data);
  }
  track_pub_->msg_.tracks.clear();
  if (track_pub_->trylock()) {
    track_pub_->msg_ = track_data_array;
    track_pub_->unlockAndPublish();
  }
}

void KalmanFilterTrack::update(realtime_tools::RealtimeBuffer<rm_msgs::TargetDetectionArray> &detection_rt_buffer) {
//  ROS_INFO("I update, ok!");
  ros::Time detection_time = detection_rt_buffer.readFromRT()->header.stamp;
  if (last_detection_time_ != detection_time) {
    last_detection_time_ = detection_time;
    if (!begin_flag_) {
      //get the map2detections_last_
      begin_flag_ = true;
      std::map<int, double> id2distance;
      for (const auto &detection : detection_rt_buffer.readFromRT()->detections) {
        geometry_msgs::TransformStamped map2detection, map2camera;
        tf2::Transform camera2detection_tf, map2detection_tf;
        try {
          tf2::fromMsg(detection.pose, camera2detection_tf);
          map2camera = robot_state_handle_.lookupTransform("map",
                                                           "camera",
                                                           detection_rt_buffer.readFromRT()->header.stamp -
                                                               ros::Duration(time_compensation_));
          tf2::fromMsg(map2camera.transform, map2camera_tf_);
          map2detection_tf = map2camera_tf_ * camera2detection_tf;
          map2detection.transform.translation.x = map2detection_tf.getOrigin().x();
          map2detection.transform.translation.y = map2detection_tf.getOrigin().y();
          map2detection.transform.translation.z = map2detection_tf.getOrigin().z();
          map2detection.transform.rotation.x = map2detection_tf.getRotation().x();
          map2detection.transform.rotation.y = map2detection_tf.getRotation().y();
          map2detection.transform.rotation.z = map2detection_tf.getRotation().z();
          map2detection.transform.rotation.w = map2detection_tf.getRotation().w();
          map2detection.header.stamp = detection_rt_buffer.readFromRT()->header.stamp;
          map2detection.header.frame_id = "map";
          map2detection.child_frame_id = "detection" + std::to_string(detection.id);
          tf_broadcaster_.sendTransform(map2detection);
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
        }
        if (id2distance.find(detection.id) == id2distance.end()) {
          double distance_square = std::pow(detection.pose.position.x, 2) +
              std::pow(detection.pose.position.y, 2) +
              std::pow(detection.pose.position.z, 2);
          id2distance.insert(std::make_pair(detection.id, distance_square));
          map2detections_last_.insert(std::make_pair(detection.id, map2detection));
        } else {
          double dis_square_new = std::pow(detection.pose.position.x, 2) +
              std::pow(detection.pose.position.y, 2) + std::pow(detection.pose.position.z, 2);
          double dis_square_store = id2distance[detection.id];
          if (dis_square_new < dis_square_store) {
            map2detections_last_[detection.id] = map2detection;
          }
        }
      }
      return;
    }

    //get the update map2detections_now
    std::map<int, geometry_msgs::TransformStamped> map2detections_now;
    std::map<int, double> id2distance;
    for (const auto &detection : detection_rt_buffer.readFromRT()->detections) {
      geometry_msgs::TransformStamped map2detection, map2camera;
      tf2::Transform camera2detection_tf, map2detection_tf;
      try {
        tf2::fromMsg(detection.pose, camera2detection_tf);
        map2camera = robot_state_handle_.lookupTransform("map",
                                                         "camera",
                                                         detection_rt_buffer.readFromRT()->header.stamp -
                                                             ros::Duration(time_compensation_));
        tf2::fromMsg(map2camera.transform, map2camera_tf_);
        map2detection_tf = map2camera_tf_ * camera2detection_tf;
        map2detection.transform.translation.x = map2detection_tf.getOrigin().x();
        map2detection.transform.translation.y = map2detection_tf.getOrigin().y();
        map2detection.transform.translation.z = map2detection_tf.getOrigin().z();
        map2detection.transform.rotation.x = map2detection_tf.getRotation().x();
        map2detection.transform.rotation.y = map2detection_tf.getRotation().y();
        map2detection.transform.rotation.z = map2detection_tf.getRotation().z();
        map2detection.transform.rotation.w = map2detection_tf.getRotation().w();
        map2detection.header.stamp = detection_rt_buffer.readFromRT()->header.stamp;
        map2detection.header.frame_id = "map";
        map2detection.child_frame_id = "detection" + std::to_string(detection.id);
        tf_broadcaster_.sendTransform(map2detection);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
      }
      //last have no , now have
      if (map2detections_last_.find(detection.id) == map2detections_last_.end()) {
        map2detections_now.insert(std::make_pair(detection.id, map2detection));
      } else if (id2distance.find(detection.id) == id2distance.end()) {
        //last have , now have
        double last_x = map2detections_last_[detection.id].transform.translation.x;
        double last_y = map2detections_last_[detection.id].transform.translation.y;
        double last_z = map2detections_last_[detection.id].transform.translation.z;
        double distance_square = std::pow((map2detection.transform.translation.x - last_x), 2) +
            std::pow((map2detection.transform.translation.y - last_y), 2) +
            std::pow((map2detection.transform.translation.z - last_z), 2);
        id2distance.insert(std::make_pair(detection.id, distance_square));
        map2detections_now.insert(std::make_pair(detection.id, map2detection));
      } else {
        double last_x = map2detections_last_[detection.id].transform.translation.x;
        double last_y = map2detections_last_[detection.id].transform.translation.y;
        double last_z = map2detections_last_[detection.id].transform.translation.z;
        double distance_square = std::pow((map2detection.transform.translation.x - last_x), 2) +
            std::pow((map2detection.transform.translation.y - last_y), 2) +
            std::pow((map2detection.transform.translation.z - last_z), 2);
        if (distance_square < id2distance[detection.id]) {
          map2detections_now[detection.id] = map2detection;
        }
      }
    }

    rm_msgs::TrackDataArray track_data_array;
    for (const auto &map2detection_last : map2detections_last_) {
      //last have,we create something that haven't created
      if (id2detection_.find(map2detection_last.first) == id2detection_.end()) {
        id2detection_.insert(
            std::make_pair(map2detection_last.first, new TranTarget(0.001, q_x_, q_dx_,
                                                                    r_x_, r_dx_)));
      }
      //last have, now have no, we delete;
      if (map2detections_now.find(map2detection_last.first) == map2detections_now.end()) {
//        id2detection_[map2detection_last.first]->predict();
        id2detection_.erase(map2detection_last.first);
      } else {
        //last have, now have, clear or update;
        geometry_msgs::TransformStamped map2detection_now = map2detections_now[map2detection_last.first];
        double time_diff, distance_diff;
        time_diff = (map2detection_now.header.stamp - map2detection_last.second.header.stamp).toSec();
        distance_diff =
            std::pow(map2detection_now.transform.translation.x - map2detection_last.second.transform.translation.x, 2) +
                std::pow(map2detection_now.transform.translation.y - map2detection_last.second.transform.translation.y,
                         2)
                +
                    std::pow(
                        map2detection_now.transform.translation.z - map2detection_last.second.transform.translation.z,
                        2);
        if (time_diff > time_thresh_ || distance_diff > distance_thresh_) {
          Vec8<double> x;
          x << 0., 0., 0., 0., 0., 0., 0., 0.;
          id2detection_[map2detection_last.first]->clear(x);
          ROS_INFO("i am %d, i clear", map2detection_last.first);
          continue;
        } else if (time_diff < 0.0001) {
          id2detection_[map2detection_last.first]->predict();
          ROS_INFO("i am %d, i predict", map2detection_last.first);
          continue;
        } else {
          double dt = map2detection_now.header.stamp.toSec()
              - map2detection_last.second.header.stamp.toSec();
          Vec8<double> z; //observe value
          double now_pos_x, now_pos_y, now_pos_z;
          double last_pos_x, last_pos_y, last_pos_z;
          now_pos_x = map2detection_now.transform.translation.x;
          now_pos_y = map2detection_now.transform.translation.y;
          now_pos_z = map2detection_now.transform.translation.z;
          double now_roll, now_pitch, now_yaw;
          quatToRPY(map2detection_now.transform.rotation, now_roll, now_pitch, now_yaw);

          last_pos_x = map2detection_last.second.transform.translation.x;
          last_pos_y = map2detection_last.second.transform.translation.y;
          last_pos_z = map2detection_last.second.transform.translation.z;
          double last_roll, last_pitch, last_yaw;
          quatToRPY(map2detection_last.second.transform.rotation, last_roll, last_pitch, last_yaw);

          rm_msgs::TrackData track_data;
          track_data.id = map2detection_last.first;
          track_data.pose2map.position.x = now_pos_x;
          track_data.pose2map.position.y = now_pos_y;
          track_data.pose2map.position.z = now_pos_z;
          track_data.pose2map.orientation.x = now_yaw;
          track_data.pose2map.orientation.y = (now_yaw - last_yaw) / dt;
          track_data.pose2camera.position.x = (now_pos_x - last_pos_x) / dt;
          track_data.pose2camera.position.y = (now_pos_y - last_pos_y) / dt;
          track_data.pose2camera.position.z = (now_pos_z - last_pos_z) / dt;
          track_data_array.header.stamp = map2detection_last.second.header.stamp;
          track_data_array.tracks.emplace_back(track_data);
          z << now_pos_x, (now_pos_x - last_pos_x) / dt,
              now_pos_y, (now_pos_y - last_pos_y) / dt,
              now_pos_z, (now_pos_z - last_pos_z) / dt,
              now_yaw, (now_yaw - last_yaw) / dt;
          id2detection_[map2detection_last.first]->update(z);
          ROS_INFO("i am %d, i update", map2detection_last.first);
        }
      }
    }

    track_test_pub_->msg_.tracks.clear();
    if (track_test_pub_->trylock()) {
      track_test_pub_->msg_ = track_data_array;
      track_test_pub_->unlockAndPublish();
    }

    map2detections_last_ = map2detections_now;

  } else {
    for (const auto &item: id2detection_) {
      item.second->predict();
      ROS_INFO("i am %d, i predict", item.first);
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
  time_thresh_ = config.time_thresh;
  distance_thresh_ = config.distance_thresh;
  time_compensation_ = config.time_compensation;
  for (const auto &item:id2detection_) {
    delete item.second;
  }
  id2detection_.clear();
}

}
