//
// Created by qiayuan on 1/16/21.
//

#include <ros_utilities.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include "rm_gimbal_controller/standard.h"

namespace rm_gimbal_controllers {
bool GimbalStandardController::init(hardware_interface::RobotHW *robot_hw,
                                    ros::NodeHandle &root_nh,
                                    ros::NodeHandle &controller_nh) {
  auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_yaw_ = effort_jnt_interface->getHandle(
      getParam(controller_nh, "joint_yaw_name", std::string("joint_yaw")));
  joint_pitch_ =
      effort_jnt_interface->getHandle(
          getParam(controller_nh, "joint_pitch_name", std::string("joint_pitch")));

  robot_state_handle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");

  if (!pid_yaw_.init(ros::NodeHandle(controller_nh, "pid_yaw")) ||
      !pid_pitch_.init(ros::NodeHandle(controller_nh, "pid_pitch")))
    return false;

  map2gimbal_des_.header.frame_id = "map";
  map2gimbal_des_.child_frame_id = "gimbal_des";

  cmd_subscriber_ = root_nh.subscribe<rm_msgs::GimbalCmd>("cmd_gimbal", 1, &GimbalStandardController::commandCB, this);
  cmd_sub_track_ = root_nh.subscribe<rm_msgs::GimbalTrackCmd>
      ("cmd_gimbal_track", 1, &GimbalStandardController::cmdTrackCB, this);
  bullet_solver_ = new Approx3DSolver(controller_nh);
  return true;
}

void GimbalStandardController::update(const ros::Time &time, const ros::Duration &period) {
  cmd_ = *cmd_rt_buffer_.readFromRT();

  if (!*cmd_msgs_type_rt_buffer_.readFromRT()) {
    if (state_ != cmd_.mode) {
      state_ = StandardState(cmd_.mode);
      state_changed_ = true;
    }
  } else {
    if (state_ != TRACK) {
      state_ = TRACK;
      state_changed_ = true;
    }
  }

  if (state_ == PASSIVE)
    passive();
  else {
    if (state_ == RATE)
      rate(time, period);
    else if (state_ == TRACK)
      track(time);
    moveJoint(period);
  }
}

void GimbalStandardController::passive() {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter PASSIVE");

    joint_yaw_.setCommand(0);
    joint_pitch_.setCommand(0);
    pid_yaw_.reset();
  }
}

void GimbalStandardController::rate(const ros::Time &time, const ros::Duration &period) {
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

void GimbalStandardController::track(const ros::Time &time) {
  if (state_changed_) { //on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  geometry_msgs::TransformStamped map2pitch;
  try {
    map2pitch = robot_state_handle_.lookupTransform("map", "pitch", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  if (bullet_solver_->solve(angle_init_, map2pitch, cmd_track_rt_buffer_))
    robot_state_handle_.setTransform(bullet_solver_->getResult(time), "rm_gimbal_controller");
  else {
    double roll{}, pitch{}, yaw{};
    quatToRPY(map2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw, pitch);
  }
}

void GimbalStandardController::setDes(const ros::Time &time, double yaw, double pitch) {
  //pitch = minAbs(pitch, M_PI_2 - 0.1); //avoid gimbal lock
  map2gimbal_des_.transform.rotation =
      tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
  map2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controller");
}

void GimbalStandardController::moveJoint(const ros::Duration &period) {
  geometry_msgs::TransformStamped pitch2des;
  try {
    pitch2des = robot_state_handle_.lookupTransform("base_link", "gimbal_des", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(pitch2des.transform.rotation, roll_des, pitch_des, yaw_des);
  double yaw_error = angles::shortest_angular_distance(joint_yaw_.getPosition(), yaw_des);
  double pitch_error = angles::shortest_angular_distance(joint_pitch_.getPosition(), pitch_des);
  pid_yaw_.computeCommand(yaw_error, period);
  pid_pitch_.computeCommand(pitch_error, period);
  joint_yaw_.setCommand(pid_yaw_.getCurrentCmd());
  joint_pitch_.setCommand(pid_pitch_.getCurrentCmd());
}

void GimbalStandardController::commandCB(const rm_msgs::GimbalCmdConstPtr &msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
  cmd_msgs_type_rt_buffer_.writeFromNonRT(false);
}
void GimbalStandardController::cmdTrackCB(const rm_msgs::GimbalTrackCmdConstPtr &msg) {
  cmd_track_rt_buffer_.writeFromNonRT(*msg);
  cmd_msgs_type_rt_buffer_.writeFromNonRT(true);
}

} // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::GimbalStandardController, controller_interface::ControllerBase)
