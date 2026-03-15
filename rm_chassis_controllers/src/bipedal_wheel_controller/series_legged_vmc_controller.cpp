//
// Created by wiselook on 7/27/25.
//
#include "bipedal_wheel_controller/series_legged_vmc_controller.h"
#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <angles/angles.h>

namespace rm_chassis_controllers
{
bool VMCController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  cmdLegLengthSubscriber_ =
      controller_nh.subscribe<std_msgs::Float64>("command/leg_length", 1, &VMCController::commandLegLengthCB, this);
  cmdLegAngleSubscriber_ =
      controller_nh.subscribe<std_msgs::Float64>("command/leg_angle", 1, &VMCController::commandLegAngleCB, this);
  statePublisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("state", 1);
  jointCmdStatePublisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("joint_cmd_state", 1);
  if (controller_nh.hasParam("pid_length"))
  {
    if (!pidLength_.init(ros::NodeHandle(controller_nh, "pid_length")))
    {
      ROS_ERROR("Load param fail, check the resist of pid_length");
      return false;
    }
  }
  if (controller_nh.hasParam("pid_angle"))
  {
    if (!pidAngle_.init(ros::NodeHandle(controller_nh, "pid_angle")))
    {
      ROS_ERROR("Load param fail, check the resist of pid_angle");
      return false;
    }
  }
  if (!controller_nh.getParam("vmc_bias_angle", vmcBiasAngle_))
  {
    ROS_ERROR("Load param fail, check the resist of vmc_bias_angle");
    return false;
  }
  std::string thighJoint, kneeJoint;
  if (!(controller_nh.getParam("thigh_joint", thighJoint) && controller_nh.getParam("knee_joint", kneeJoint)))
  {
    ROS_ERROR("Load param fail, check the resist of thigh_joint or knee_joint");
    return false;
  }
  if (!controller_nh.getParam("spring_force", spring_force_))
  {
    ROS_ERROR("Load param fail, check the resist of spring_force");
    return false;
  }
  double l1, l2;
  if (!controller_nh.getParam("l1", l1) || !controller_nh.getParam("l2", l2))
  {
    ROS_ERROR("Load param fail, check the resist of l1 or l2");
    return false;
  }
  vmcPtr_ = std::make_unique<VMC>(l1, l2, 0);

  jointThigh_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(thighJoint);
  jointKnee_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(kneeJoint);
  return true;
}

void VMCController::starting(const ros::Time& /*time*/)
{
  angleCmd_ = 0.;
  lengthCmd_ = 0.20;
}

void VMCController::update(const ros::Time& time, const ros::Duration& period)
{
  double knee_angle = 0, thigh_angle = 0, position[2], speed[2];

  // series leg vmc
  // gazebo
  //  thigh_angle = jointThigh_.getPosition() + M_PI_2;
  //  knee_angle = jointKnee_.getPosition() - M_PI_2;

  // five link vmc
  thigh_angle = jointThigh_.getPosition() + M_PI;
  knee_angle = jointKnee_.getPosition();
  vmcPtr_->leg_pos(thigh_angle, knee_angle, position);
  vmcPtr_->leg_spd(jointThigh_.getVelocity(), jointKnee_.getVelocity(), thigh_angle, knee_angle, speed);

  double effortCmd[2], jointCmd[2];
  static double angleSinCmd_ = 0;
  angleSinCmd_ -= 0.001;
  if (angleSinCmd_ <= -M_PI)
  {
    angleSinCmd_ = M_PI;
  }
  //  angleCmd_ = angleSinCmd_;
  double angle_error = angles::shortest_angular_distance(position[1], angleCmd_);

  effortCmd[0] = pidLength_.computeCommand(lengthCmd_ - position[0], period) - spring_force_;
  effortCmd[1] = pidAngle_.computeCommand(angle_error, period);

  vmcPtr_->leg_conv(effortCmd[0], effortCmd[1], thigh_angle, knee_angle, jointCmd);
  std_msgs::Float64MultiArray state;
  state.data.push_back(thigh_angle);
  state.data.push_back(knee_angle);
  state.data.push_back(position[0]);
  state.data.push_back(position[1]);
  state.data.push_back(speed[0]);
  state.data.push_back(speed[1]);
  state.data.push_back(angle_error);
  state.data.push_back(effortCmd[0]);
  state.data.push_back(effortCmd[1]);
  state.data.push_back(jointCmd[0]);
  state.data.push_back(jointCmd[1]);
  statePublisher_.publish(state);

  std_msgs::Float64MultiArray jointCmdState;
  jointCmdState.data.push_back(jointCmd[0]);
  jointCmdState.data.push_back(jointCmd[1]);
  jointCmdStatePublisher_.publish(jointCmdState);

  jointThigh_.setCommand(jointCmd[0]);
  jointKnee_.setCommand(jointCmd[1]);
}

}  // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::VMCController, controller_interface::ControllerBase)
