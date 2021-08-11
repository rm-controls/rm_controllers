//
// Created by qiayuan on 1/16/21.
//
#include "rm_gimbal_controller/standard.h"

#include <string>
#include <angles/angles.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_gimbal_controllers {
bool Controller::init(hardware_interface::RobotHW *robot_hw,
                      ros::NodeHandle &root_nh,
                      ros::NodeHandle &controller_nh) {
  ros::NodeHandle nh_bullet_solver = ros::NodeHandle(controller_nh, "bullet_solver");
  ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  nh_moving_average_filter_ = ros::NodeHandle(controller_nh, "moving_average_filter");

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");

  int chassis_angular_data_num{};
  std::string detection_topic{}, camera_topic{};
  detection_topic = getParam(controller_nh, "detection_topic", std::string("/detection"));
  camera_topic = getParam(controller_nh, "camera_topic", std::string("/camera/camera_info"));
  detection_frame_ = getParam(controller_nh, "detection_frame", std::string("detection"));
  if (!controller_nh.getParam("publish_rate", publish_rate_) ||
      !controller_nh.getParam("chassis_angular_data_num", chassis_angular_data_num)) {
    ROS_ERROR("Some gimbal params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  if (!ctrl_yaw_.init(effort_joint_interface_, nh_yaw) ||
      !ctrl_pitch_.init(effort_joint_interface_, nh_pitch))
    return false;

  gimbal_des_frame_id_ = ctrl_pitch_.joint_urdf_->child_link_name + "_des";
  map2gimbal_des_.header.frame_id = "map";
  map2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  map2gimbal_des_.transform.rotation.w = 1.;
  map2pitch_.header.frame_id = "map";
  map2pitch_.child_frame_id = ctrl_pitch_.joint_urdf_->child_link_name;
  map2pitch_.transform.rotation.w = 1.;
  map2base_.header.frame_id = "map";
  map2base_.child_frame_id = ctrl_yaw_.joint_urdf_->parent_link_name;
  map2base_.transform.rotation.w = 1.;

  cmd_gimbal_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this);
  data_detection_sub_ =
      root_nh.subscribe<rm_msgs::TargetDetectionArray>(detection_topic, 1, &Controller::detectionCB, this);
  camera_sub_ = root_nh.subscribe<sensor_msgs::CameraInfo>(camera_topic, 1, &Controller::cameraCB, this);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error_des", 100));
  track_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>(controller_nh, "track", 100));
  tf_broadcaster_.init(root_nh);

  // init config
  config_ = {.time_compensation = getParam(controller_nh, "time_compensation", 0.)};
  config_rt_buffer_.initRT(config_);

  d_srv_ =
      new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>::CallbackType
      cb = [this](auto &&PH1, auto &&PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  bullet_solver_ = new bullet_solver::BulletSolver(nh_bullet_solver);
  ma_filter_chassis_angular_ = new MovingAverageFilter<double>(chassis_angular_data_num);

  return true;
}

void Controller::starting(const ros::Time &) {
  state_ = RATE;
  state_changed_ = true;
};

void Controller::update(const ros::Time &time, const ros::Duration &period) {
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  if (!updateTf())
    return;
  updateChassisVel();

  if (state_ != cmd_gimbal_.mode) {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }
  switch (state_) {
    case RATE: rate(time, period);
      break;
    case TRACK: track(time);
      break;
    case DIRECT: direct(time);
      break;
  }
  moveJoint(time, period);
}

void Controller::rate(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    map2gimbal_des_.transform.rotation = map2pitch_.transform.rotation;
    map2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controller");
  } else {
    double roll{}, pitch{}, yaw{};
    quatToRPY(map2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
  }
}

void Controller::track(const ros::Time &time) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  bool solve_success = false;
  double yaw_compute{}, pitch_compute;
  double roll_real, pitch_real, yaw_real;
  quatToRPY(map2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);

  int target_id = cmd_gimbal_.target_id;
  if (moving_average_filters_track_.find(target_id) != moving_average_filters_track_.end()) {
    geometry_msgs::Point target_pos_solve{}, target_pos_compute{};
    geometry_msgs::Vector3 target_vel_solve{}, target_vel_compute{};

    if (moving_average_filters_track_.find(target_id)->second->isGyro()) {
      target_pos_solve.x = center_pos_.find(target_id)->second.x - map2pitch_.transform.translation.x;
      target_pos_solve.y = center_pos_.find(target_id)->second.y - map2pitch_.transform.translation.y;
      target_pos_solve.z = center_pos_.find(target_id)->second.z - map2pitch_.transform.translation.z;

      target_pos_compute.x = center_pos_observation_.find(target_id)->second.x;
      target_pos_compute.y = detection_pos_observation_.find(target_id)->second.y;
      target_pos_compute.z = center_pos_observation_.find(target_id)->second.z;
      target_vel_compute.y = gyro_vel_.find(target_id)->second;
      pitch_compute = -pitch_real;
    } else {
      target_pos_solve.x = detection_pos_.find(target_id)->second.x - map2pitch_.transform.translation.x;
      target_pos_solve.y = detection_pos_.find(target_id)->second.y - map2pitch_.transform.translation.y;
      target_pos_solve.z = detection_pos_.find(target_id)->second.z - map2pitch_.transform.translation.z;
      target_vel_solve.x = detection_vel_.find(target_id)->second.x - chassis_vel_.linear.x;
      target_vel_solve.y = detection_vel_.find(target_id)->second.y - chassis_vel_.linear.y;

      target_pos_compute = target_pos_solve;
      target_vel_compute = target_vel_solve;
      yaw_compute = yaw_real;
      pitch_compute = -pitch_real;
    }

    solve_success = bullet_solver_->solve(target_pos_solve, target_vel_solve, cmd_gimbal_.bullet_speed);

    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
      if (error_pub_->trylock()) {
        double error, error_delta;
        error = bullet_solver_->getGimbalError(
            target_pos_compute, target_vel_compute, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
        target_pos_compute.y += moving_average_filters_track_.find(target_id)->second->getDelta();
        error_delta = bullet_solver_->getGimbalError(
            target_pos_compute, target_vel_compute, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
        error = error < error_delta ? error : error_delta;

        error_pub_->msg_.stamp = time;
        error_pub_->msg_.error = solve_success ? error : 1.0;
        error_pub_->unlockAndPublish();
      }
      bullet_solver_->bulletModelPub(map2pitch_, time);
      last_publish_time_ = time;
    }
  }
  if (solve_success)
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
  else {
    map2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controller");
  }
}

void Controller::direct(const ros::Time &time) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter DIRECT");
  }
  geometry_msgs::Point aim_point_map{};
  double yaw{}, pitch{};
  try {
    tf2::doTransform(cmd_gimbal_.aim_point.point, aim_point_map,
                     robot_state_handle_.lookupTransform("map",
                                                         cmd_gimbal_.aim_point.header.frame_id,
                                                         cmd_gimbal_.aim_point.header.stamp));
    yaw = std::atan2(aim_point_map.y - map2pitch_.transform.translation.y,
                     aim_point_map.x - map2pitch_.transform.translation.x);
    pitch = -std::atan2(aim_point_map.z - map2pitch_.transform.translation.z,
                        std::sqrt(std::pow(aim_point_map.x - map2pitch_.transform.translation.x, 2)
                                      + std::pow(aim_point_map.y - map2pitch_.transform.translation.y, 2)));
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
  setDes(time, yaw, pitch);
}

void Controller::setDes(const ros::Time &time, double yaw_des, double pitch_des) {
  double roll_base{}, pitch_base{}, yaw_base{};
  quatToRPY(map2base_.transform.rotation, roll_base, pitch_base, yaw_base);
  double roll_now{}, pitch_now{}, yaw_now{};
  quatToRPY(map2gimbal_des_.transform.rotation, roll_now, pitch_now, yaw_now);
  double pitch_delta = angles::shortest_angular_distance(pitch_base, pitch_des);
  double yaw_delta = angles::shortest_angular_distance(yaw_base, yaw_des);
  map2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw
      (0.,
       pitch_delta <= ctrl_pitch_.joint_urdf_->limits->upper && pitch_delta >= ctrl_pitch_.joint_urdf_->limits->lower ?
       pitch_des : pitch_now,
       yaw_delta <= ctrl_yaw_.joint_urdf_->limits->upper && yaw_delta >= ctrl_yaw_.joint_urdf_->limits->lower ?
       yaw_des : yaw_now);
  map2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controller");
}

void Controller::moveJoint(const ros::Time &time, const ros::Duration &period) {
  geometry_msgs::TransformStamped base_frame2des;
  try {
    base_frame2des = robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name,
                                                         gimbal_des_frame_id_, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(base_frame2des.transform.rotation, roll_des, pitch_des, yaw_des);

  ctrl_yaw_.setCommand(yaw_des, -chassis_vel_.angular.z);
  ctrl_pitch_.setCommand(pitch_des, 0);
  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
}

bool Controller::updateTf() {
  try {
    map2pitch_ = robot_state_handle_.lookupTransform("map", ctrl_pitch_.joint_urdf_->child_link_name, ros::Time(0));
    map2base_ = robot_state_handle_.lookupTransform("map", ctrl_yaw_.joint_urdf_->parent_link_name, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  std::map<int, geometry_msgs::Pose> now_detection;
  double now_distance, last_distance;

  // Select the only target pose
  for (const auto &detection:detection_rt_buffer_.readFromRT()->detections) {
    if (now_detection.find(detection.id) == now_detection.end())
      now_detection.insert(std::make_pair(detection.id, detection.pose));
    else {
      now_distance = std::sqrt(
          std::pow(detection.pose.position.x - last_detection_.find(detection.id)->second.position.x, 2)
              + std::pow(detection.pose.position.y - last_detection_.find(detection.id)->second.position.y, 2)
              + std::pow(detection.pose.position.z - last_detection_.find(detection.id)->second.position.z, 2));
      last_distance = std::sqrt(
          std::pow(now_detection.find(detection.id)->second.position.x
                       - last_detection_.find(detection.id)->second.position.x, 2) +
              std::pow(now_detection.find(detection.id)->second.position.y
                           - last_detection_.find(detection.id)->second.position.y, 2) +
              std::pow(now_detection.find(detection.id)->second.position.z
                           - last_detection_.find(detection.id)->second.position.z, 2));
      if (now_distance < last_distance)
        now_detection[detection.id] = detection.pose;
    }
  }
  last_detection_ = now_detection;

  // Filtering the targets with different id
  ros::Time detection_time = detection_rt_buffer_.readFromRT()->header.stamp;
  if (last_detection_time_ != detection_time) {
    last_detection_time_ = detection_time;
    track_pub_->msg_.tracks.clear();
    for (const auto &detection:now_detection) {
      if (moving_average_filters_track_.find(detection.first) == moving_average_filters_track_.end())
        moving_average_filters_track_.insert(std::make_pair(detection.first,
                                                            new moving_average_filter::MovingAverageFilterTrack(
                                                                nh_moving_average_filter_,
                                                                detection.first, robot_state_handle_)));
      config_ = *config_rt_buffer_.readFromRT();
      geometry_msgs::TransformStamped map2camera, map2detection;
      tf2::Transform camera2detection_tf, map2camera_tf, map2detection_tf;
      try {
        tf2::fromMsg(detection.second, camera2detection_tf);
        map2camera = robot_state_handle_.lookupTransform("map",
                                                         detection_rt_buffer_.readFromRT()->header.frame_id,
                                                         detection_time - ros::Duration(config_.time_compensation));
        tf2::fromMsg(map2camera.transform, map2camera_tf);
        map2detection_tf = map2camera_tf * camera2detection_tf;
        map2detection.transform = tf2::toMsg(map2detection_tf);
        map2detection.header.stamp = detection_time;
        map2detection.header.frame_id = "map";
        map2detection.child_frame_id = detection_frame_ + std::to_string(detection.first);

        moving_average_filters_track_.find(detection.first)->second->input(map2detection,
                                                                           ctrl_pitch_.joint_urdf_->child_link_name);
        detection_pos_[detection.first] = moving_average_filters_track_.find(detection.first)->second->getPos();
        detection_vel_[detection.first] = moving_average_filters_track_.find(detection.first)->second->getVel();
        center_pos_[detection.first] = moving_average_filters_track_.find(detection.first)->second->getCenter();
        detection_pos_observation_[detection.first] =
            moving_average_filters_track_.find(detection.first)->second->getPosObservation();
        center_pos_observation_[detection.first] =
            moving_average_filters_track_.find(detection.first)->second->getCenterObservation();
        gyro_vel_[detection.first] = moving_average_filters_track_.find(detection.first)->second->getGyroVel();

        tf_broadcaster_.sendTransform(moving_average_filters_track_.find(detection.first)->second->getTransform());
        updateTrack(detection.first);
      }
      catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
    }
  }

  ros::Time camera_time = camera_rt_buffer_.readFromRT()->header.stamp;
  if (last_camera_time_ != camera_time) {
    last_camera_time_ = camera_time;
    if (track_pub_->trylock()) {
      track_pub_->msg_.header.stamp = camera_time;
      track_pub_->unlockAndPublish();
    }
  }

  return true;
}

void Controller::updateTrack(int id) {
  geometry_msgs::TransformStamped camera2detection;
  try {
    tf2::doTransform(moving_average_filters_track_.find(id)->second->getTransform(), camera2detection,
                     robot_state_handle_.lookupTransform(detection_rt_buffer_.readFromRT()->header.frame_id,
                                                         "map", ros::Time(0)));
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

  rm_msgs::TrackData track_data;
  track_data.id = id;
  track_data.stamp = moving_average_filters_track_.find(id)->second->getTransform().header.stamp;
  track_data.is_gyro = moving_average_filters_track_.find(id)->second->isGyro();
  track_data.camera2detection.x = camera2detection.transform.translation.x;
  track_data.camera2detection.y = camera2detection.transform.translation.y;
  track_data.camera2detection.z = camera2detection.transform.translation.z;
  track_data.detection_vel = detection_vel_.find(id)->second;

  track_pub_->msg_.tracks.push_back(track_data);
}

void Controller::updateChassisVel() {
  double tf_period = map2base_.header.stamp.toSec() - last_map2base_.header.stamp.toSec();
  if (tf_period > 0.0 && tf_period < 1.0) {
    chassis_vel_.linear.x = (map2base_.transform.translation.x - last_map2base_.transform.translation.x) / tf_period;
    chassis_vel_.linear.y = (map2base_.transform.translation.y - last_map2base_.transform.translation.y) / tf_period;
    if (std::abs(yawFromQuat(map2base_.transform.rotation) - yawFromQuat(last_map2base_.transform.rotation)) < 6.0) {
      ma_filter_chassis_angular_->input(
          (yawFromQuat(map2base_.transform.rotation) - yawFromQuat(last_map2base_.transform.rotation)) / tf_period);
      chassis_vel_.angular.z = ma_filter_chassis_angular_->output();
    }
  }
  last_map2base_ = map2base_;
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr &msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::detectionCB(const rm_msgs::TargetDetectionArrayConstPtr &msg) {
  detection_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::cameraCB(const sensor_msgs::CameraInfoConstPtr &msg) {
  camera_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::reconfigCB(rm_gimbal_controllers::GimbalConfig &config, uint32_t) {
  ROS_INFO("[Gimbal] Dynamic params change");
  if (!dynamic_reconfig_initialized_) {
    Config init_config = *config_rt_buffer_.readFromNonRT(); // config init use yaml
    config.time_compensation = init_config.time_compensation;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{.time_compensation = config.time_compensation};
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}

} // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
