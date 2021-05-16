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

  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_yaw_ =
      effort_jnt_interface->getHandle(getParam(nh_yaw, "joint_name", std::string("joint_yaw")));
  joint_pitch_ =
      effort_jnt_interface->getHandle(getParam(nh_pitch, "joint_name", std::string("joint_pitch")));

  upper_yaw_ = getParam(nh_yaw, "upper", 1e9);
  lower_yaw_ = getParam(nh_yaw, "lower", -1e9);
  upper_pitch_ = getParam(nh_pitch, "upper", 1e9);
  lower_pitch_ = getParam(nh_pitch, "lower", -1e9);

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");

  if (!pid_yaw_.init(ros::NodeHandle(nh_yaw, "pid")) ||
      !pid_pitch_.init(ros::NodeHandle(nh_pitch, "pid")))
    return false;

  map2gimbal_des_.header.frame_id = "map";
  map2gimbal_des_.child_frame_id = "gimbal_des";
  map2gimbal_des_.transform.rotation.w = 1.;
  controller_nh.param("publish_rate_error", publish_rate_, 100.0);

  cmd_gimbal_sub_ = root_nh.subscribe<rm_msgs::GimbalCmd>("cmd_gimbal", 1, &Controller::commandCB, this);
  data_detection_sub_ =
      root_nh.subscribe<rm_msgs::TargetDetectionArray>("detection", 1, &Controller::detectionCB, this);
  camera_sub_ = root_nh.subscribe<sensor_msgs::CameraInfo>("galaxy_camera/camera_info", 1, &Controller::cameraCB, this);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(root_nh, "error_des", 100));
  track_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>(root_nh, "track", 100));
  tf_broadcaster_.init(root_nh);

  // init config
  config_ = {.time_compensation = getParam(controller_nh, "time_compensation", 0.)};
  config_rt_buffer_.initRT(config_);

  d_srv_ =
      new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>::CallbackType
      cb = [this](auto &&PH1, auto &&PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  bullet_solver_ = new bullet_solver::Approx3DSolver(nh_bullet_solver);
  lp_filter_yaw_ = new LowPassFilter(nh_yaw);
  lp_filter_pitch_ = new LowPassFilter(nh_pitch);

  return true;
}

void Controller::update(const ros::Time &time, const ros::Duration &period) {
  cmd_ = *cmd_rt_buffer_.readFromRT();
  updateTf();

  if (state_ != cmd_.mode) {
    state_ = StandardState(cmd_.mode);
    state_changed_ = true;
  }

  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == RATE)
      rate(time, period);
    else if (state_ == TRACK)
      track(time);
    moveJoint(time, period);
  }
}

void Controller::passive() {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter PASSIVE");
  }

  joint_yaw_.setCommand(0);
  joint_pitch_.setCommand(0);
  pid_yaw_.reset();
}

void Controller::rate(const ros::Time &time, const ros::Duration &period) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    map2gimbal_des_.transform = map2pitch_.transform;
    map2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controller");
  } else {
    double roll{}, pitch{}, yaw{};
    quatToRPY(map2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time,
           yaw + period.toSec() * cmd_rt_buffer_.readFromRT()->rate_yaw,
           pitch + period.toSec() * cmd_rt_buffer_.readFromRT()->rate_pitch);
  }
}

void Controller::track(const ros::Time &time) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  bool solve_success = false;
  double roll, pitch, yaw;
  Vec2<double> angle_init{};
  Vec2<double> gyro_angle_init{};
  geometry_msgs::TransformStamped yaw2detection, yaw2pitch;

  if (last_solve_success_) {
    quatToRPY(map2pitch_.transform.rotation, roll, pitch, yaw);
    angle_init[0] = yaw;
    angle_init[1] = -pitch;
    gyro_angle_init = angle_init;
  }

  int target_id = cmd_rt_buffer_.readFromRT()->target_id;
  try {
    yaw2detection = robot_state_handle_.lookupTransform(
        "yaw", "detection" + std::to_string(target_id),
        ros::Time(0));
    yaw2pitch = robot_state_handle_.lookupTransform("yaw", "pitch", detection_rt_buffer_.readFromRT()->header.stamp);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  if (moving_average_filters_track_.find(target_id) != moving_average_filters_track_.end()) {
    geometry_msgs::Vector3 target_pos[2]{};
    geometry_msgs::Vector3 target_vel[2]{};
    geometry_msgs::Point center_pos_yaw;

    if (moving_average_filters_track_.find(target_id)->second->isGyro()) {
      try {
        tf2::doTransform(center_pos_.find(target_id)->second, center_pos_yaw, robot_state_handle_.lookupTransform(
            "yaw", "map", detection_rt_buffer_.readFromRT()->header.stamp));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
      }
      target_pos[0].x = center_pos_.find(target_id)->second.x - map2pitch_.transform.translation.x;
      target_pos[0].y = center_pos_.find(target_id)->second.y - map2pitch_.transform.translation.y;
      target_pos[0].z = center_pos_.find(target_id)->second.z - map2pitch_.transform.translation.z;
      target_pos[1].x = center_pos_yaw.x - yaw2pitch.transform.translation.x;
      target_pos[1].y = yaw2detection.transform.translation.y - yaw2pitch.transform.translation.y;
      target_pos[1].z = center_pos_yaw.z - yaw2pitch.transform.translation.z;

      target_vel[1].y = gyro_vel_.find(target_id)->second;
      gyro_angle_init[0] = 0;
    } else {
      target_pos[0].x = detection_pos_.find(target_id)->second.x - map2pitch_.transform.translation.x;
      target_pos[0].y = detection_pos_.find(target_id)->second.y - map2pitch_.transform.translation.y;
      target_pos[0].z = detection_pos_.find(target_id)->second.z - map2pitch_.transform.translation.z;
      target_pos[1] = target_pos[0];

      target_vel[0] = detection_vel_.find(target_id)->second;
      target_vel[1] = target_vel[0];
    }

    solve_success = bullet_solver_->solve(
        angle_init,
        target_pos[0].x, target_pos[0].y, target_pos[0].z,
        target_vel[0].x, target_vel[0].y, 0,
        cmd_.bullet_speed);

    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
      if (error_pub_->trylock()) {
        double error, error_delta;
        error = bullet_solver_->gimbalError(
            gyro_angle_init,
            target_pos[1].x, target_pos[1].y, target_pos[1].z,
            0., target_vel[1].y, 0,
            cmd_.bullet_speed);
        error_delta = bullet_solver_->gimbalError(
            gyro_angle_init, target_pos[1].x,
            target_pos[1].y + moving_average_filters_track_.find(target_id)->second->getDelta(),
            target_pos[1].z, 0., target_vel[1].y, 0, cmd_.bullet_speed);
        error = error < error_delta ? error : error_delta;

        error_pub_->msg_.stamp = time;
        error_pub_->msg_.error = solve_success ? error : 10;
        error_pub_->unlockAndPublish();
      }
      last_publish_time_ = time;
    }
  }
  if (solve_success)
    setDes(time, bullet_solver_->getResult(time, map2pitch_)[0], bullet_solver_->getResult(time, map2pitch_)[1]);
  else
    setDes(time, angle_init[0], -angle_init[1]);
  last_solve_success_ = solve_success;
}

void Controller::setDes(const ros::Time &time, double yaw, double pitch) {
  if (pitch <= upper_pitch_ && pitch >= lower_pitch_ && yaw <= upper_yaw_ && yaw >= lower_yaw_)
    map2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
  map2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controller");
}

void Controller::moveJoint(const ros::Time &time, const ros::Duration &period) {
  geometry_msgs::TransformStamped base2des;
  try {
    base2des = robot_state_handle_.lookupTransform("base_link", "gimbal_des", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  double error_yaw, error_pitch;
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(base2des.transform.rotation, roll_des, pitch_des, yaw_des);
  error_yaw = angles::shortest_angular_distance(joint_yaw_.getPosition(), yaw_des);
  error_pitch = angles::shortest_angular_distance(joint_pitch_.getPosition(), pitch_des);
  lp_filter_yaw_->input(error_yaw, time);
  lp_filter_pitch_->input(error_pitch, time);
  pid_yaw_.computeCommand(lp_filter_yaw_->output(), period);
  pid_pitch_.computeCommand(lp_filter_pitch_->output(), period);
  joint_yaw_.setCommand(pid_yaw_.getCurrentCmd());
  joint_pitch_.setCommand(pid_pitch_.getCurrentCmd());
}

void Controller::updateTf() {
  try {
    map2pitch_ = robot_state_handle_.lookupTransform("map", "pitch", ros::Time(0));
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

  std::map<int, geometry_msgs::Pose> now_detection;
  double now_distance, last_distance;

  //Select the only target pose
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

  //Filtering the targets with different id
  ros::Time detection_time = detection_rt_buffer_.readFromRT()->header.stamp;
  if (last_detection_time_ != detection_time) {
    last_detection_time_ = detection_time;
    track_pub_->msg_.tracks.clear();
    for (const auto &detection:now_detection) {
      if (moving_average_filters_track_.find(detection.first) == moving_average_filters_track_.end())
        moving_average_filters_track_.insert(std::make_pair(detection.first,
                                                            new moving_average_filter::MovingAverageFilterTrack(
                                                                nh_moving_average_filter_,
                                                                detection.first,
                                                                robot_state_handle_)));
      config_ = *config_rt_buffer_.readFromRT();
      geometry_msgs::TransformStamped map2camera, map2detection;
      tf2::Transform camera2detection_tf, map2camera_tf, map2detection_tf;
      try {
        tf2::fromMsg(detection.second, camera2detection_tf);
        map2camera = robot_state_handle_.lookupTransform("map",
                                                         "camera_optical_frame",
                                                         detection_time - ros::Duration(config_.time_compensation));
        tf2::fromMsg(map2camera.transform, map2camera_tf);
        map2detection_tf = map2camera_tf * camera2detection_tf;
        map2detection.transform.translation.x = map2detection_tf.getOrigin().x();
        map2detection.transform.translation.y = map2detection_tf.getOrigin().y();
        map2detection.transform.translation.z = map2detection_tf.getOrigin().z();
        map2detection.transform.rotation.x = map2detection_tf.getRotation().x();
        map2detection.transform.rotation.y = map2detection_tf.getRotation().y();
        map2detection.transform.rotation.z = map2detection_tf.getRotation().z();
        map2detection.transform.rotation.w = map2detection_tf.getRotation().w();
        map2detection.header.stamp = detection_time;
        map2detection.header.frame_id = "map";
        map2detection.child_frame_id = "detection" + std::to_string(detection.first);

        moving_average_filters_track_.find(detection.first)->second->input(map2detection);
        detection_pos_[detection.first] =
            moving_average_filters_track_.find(detection.first)->second->getTransform().transform.translation;
        detection_vel_[detection.first] = moving_average_filters_track_.find(detection.first)->second->getVel();
        center_pos_[detection.first] = moving_average_filters_track_.find(detection.first)->second->getCenter();
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
}

void Controller::updateTrack(int id) {
  geometry_msgs::TransformStamped camera2detection, map2detection;
  try {
    camera2detection = robot_state_handle_.lookupTransform("camera_optical_frame",
                                                           "detection" + std::to_string(id),
                                                           ros::Time(0));
    map2detection = robot_state_handle_.lookupTransform("map",
                                                        "detection" + std::to_string(id),
                                                        ros::Time(0));
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

  rm_msgs::TrackData track_data;
  track_data.id = id;
  track_data.map2detection.position.x = map2detection.transform.translation.x;
  track_data.map2detection.position.y = map2detection.transform.translation.y;
  track_data.map2detection.position.z = map2detection.transform.translation.z;
  track_data.map2detection.orientation = map2detection.transform.rotation;
  track_data.camera2detection.position.x = camera2detection.transform.translation.x;
  track_data.camera2detection.position.y = camera2detection.transform.translation.y;
  track_data.camera2detection.position.z = camera2detection.transform.translation.z;
  track_data.camera2detection.orientation = camera2detection.transform.rotation;

  track_pub_->msg_.tracks.push_back(track_data);
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
};

} // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
