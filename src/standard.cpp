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

  cmd_subscriber_ = root_nh.subscribe<rm_msgs::GimbalCmd>("cmd_gimbal", 1, &Controller::commandCB, this);
  cmd_sub_track_ =
      root_nh.subscribe<rm_msgs::TargetDetectionArray>("detection", 1, &Controller::detectionCB, this);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error_des", 100));
  track_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::TrackDataArray>(controller_nh, "track", 100));

  bullet_solver_ = new Approx3DSolver(nh_bullet_solver);
  kalman_filter_track = new KalmanFilterTrack(robot_state_handle_, controller_nh);
  lp_filter_yaw_ = new LowPassFilter(nh_yaw);
  lp_filter_pitch_ = new LowPassFilter(nh_pitch);
  tf_broadcaster_.init(root_nh);

  config_ = {
      .time_compensation = getParam(controller_nh, "time_compensation", 0.)};
  config_rt_gimbal_buffer_.initRT(config_);

  d_srv_ =
      new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalTimeCompensationConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalTimeCompensationConfig>::CallbackType
      cb = boost::bind(&Controller::reconfigCB, this, _1, _2);
  d_srv_->setCallback(cb);

  return true;
}

void Controller::update(const ros::Time &time, const ros::Duration &period) {
  cmd_ = *cmd_rt_buffer_.readFromRT();
//  updateTf();
//  updateTrack();
  updateDetectionTf();

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
  }

  double roll{}, pitch{}, yaw{};
  quatToRPY(map2gimbal_des_.transform.rotation, roll, pitch, yaw);
  setDes(time,
         yaw + period.toSec() * cmd_rt_buffer_.readFromRT()->rate_yaw,
         pitch + period.toSec() * cmd_rt_buffer_.readFromRT()->rate_pitch);
}

void Controller::track(const ros::Time &time) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    error_yaw_ = 999;
    error_pitch_ = 999;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  bool solve_success = false;
  double roll, pitch, yaw;
  try {
    quatToRPY(map2pitch_.transform.rotation, roll, pitch, yaw);
    angle_init_[0] = yaw;
    angle_init_[1] = -pitch;
    geometry_msgs::TransformStamped map2detection =
        robot_state_handle_.lookupTransform("map",
                                            "detection" + std::to_string(cmd_rt_buffer_.readFromRT()->target_id),
                                            ros::Time(0));
    solve_success = bullet_solver_->solve(
        angle_init_,
        map2detection.transform.translation.x - map2pitch_.transform.translation.x,
        map2detection.transform.translation.y - map2pitch_.transform.translation.y,
        map2detection.transform.translation.z - map2pitch_.transform.translation.z,
        0, 0, 0, cmd_.bullet_speed);
  }
  catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    if (error_pub_->trylock()) {
      error_pub_->msg_.error_pitch = solve_success ? error_pitch_ : 999;
      error_pub_->msg_.error_yaw = solve_success ? error_yaw_ : 999;
      error_pub_->unlockAndPublish();
    }
    last_publish_time_ = time;
  }
  if (solve_success)
    setDes(time, bullet_solver_->getResult(time, map2pitch_)[0], bullet_solver_->getResult(time, map2pitch_)[1]);
  else
    setDes(time, yaw, pitch);
}

void Controller::setDes(const ros::Time &time, double yaw, double pitch) {
  if (pitch <= upper_pitch_ && pitch >= lower_pitch_ && yaw <= upper_yaw_ && pitch >= lower_yaw_)
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
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(base2des.transform.rotation, roll_des, pitch_des, yaw_des);
  error_yaw_ = angles::shortest_angular_distance(joint_yaw_.getPosition(), yaw_des);
  error_pitch_ = angles::shortest_angular_distance(joint_pitch_.getPosition(), pitch_des);
  lp_filter_yaw_->input(error_yaw_, time);
  lp_filter_pitch_->input(error_pitch_, time);
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

  for (const auto &detection:detection_rt_buffer_.readFromRT()->detections) {
    geometry_msgs::TransformStamped map2camera, map2detection;
    tf2::Transform camera2detection_tf, map2camera_tf, map2detection_tf;
    ros::Time detection_time = detection_rt_buffer_.readFromRT()->header.stamp;
    config_ = *config_rt_gimbal_buffer_.readFromRT();
    if (last_detection_time_ != detection_time) {
      last_detection_time_ = detection_time;
      try {
        tf2::fromMsg(detection.pose, camera2detection_tf);
        map2camera = robot_state_handle_.lookupTransform("map",
                                                         "camera",
                                                         detection_time - ros::Duration(config_.time_compensation));
        tf2::fromMsg(map2camera.transform, map2camera_tf);
        map2detection_tf = map2camera_tf * camera2detection_tf;
        map2detection.transform.translation.x = map2detection_tf.getOrigin().x();
        map2detection.transform.translation.y = map2detection_tf.getOrigin().y();
        map2detection.transform.translation.z = map2detection_tf.getOrigin().z();
        map2detection.transform.rotation.w = map2detection_tf.getRotation().z();
        map2detection.header.stamp = detection_time;
        map2detection.header.frame_id = "map";
        map2detection.child_frame_id = "detection" + std::to_string(detection.id);
        tf_broadcaster_.sendTransform(map2detection);
      }
      catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
    }
  }
}

void Controller::updateDetectionTf() {
  kalman_filter_track->update(detection_rt_buffer_, config_.time_compensation);
  kalman_filter_track->getStateAndPub();
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr &msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::detectionCB(const rm_msgs::TargetDetectionArrayConstPtr &msg) {
  detection_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::reconfigCB(rm_gimbal_controllers::GimbalTimeCompensationConfig &config, uint32_t) {
  ROS_INFO("[Gimbal] Dynamic params change");
  if (!dynamic_reconfig_initialized_) {
    Config init_config = *config_rt_gimbal_buffer_.readFromNonRT(); // config init use yaml
    config.time_compensation = init_config.time_compensation;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{
      .time_compensation=config.time_compensation
  };
  config_rt_gimbal_buffer_.writeFromNonRT(config_non_rt);
}

} // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
