//
// Created by wiselook on 7/27/25.
//
#include "bipedal_wheel_controller/series_legged_vmc_controller.h"
#include "bipedal_wheel_controller/vmc/TwoLinkVMC.h"
#include "bipedal_wheel_controller/vmc/FiveLinkVMC.h"
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
  leg_gravity_compensation_debug_ = controller_nh.param("leg_gravity_compensation_debug", false);
  leg_mass_ = controller_nh.param("leg_mass", 1.55);
  s2_ = controller_nh.param("s2", 0.0775);
  s3_ = controller_nh.param("s3", 0.205);
  alpha_s_ = controller_nh.param("alpha_s", 0.2);
  // vmcPtr_ = std::make_unique<TwoLinkVMC>(l1, l2);
  vmcPtr_ = std::make_unique<FiveLinkVMC>(l1, l2, l2, l1, 0);

  jointThigh_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(thighJoint);
  jointKnee_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(kneeJoint);

  debugPub_ = std::make_shared<DebugDataPublisher>(controller_nh, "vmc_debug_data");
  return true;
}

void VMCController::starting(const ros::Time& /*time*/)
{
  angleCmd_ = 0.;
  lengthCmd_ = 0.20;
}

void VMCController::update(const ros::Time& time, const ros::Duration& period)
{
  double knee_angle = 0, thigh_angle = 0;

  // series leg vmc
  // gazebo
  //  thigh_angle = jointThigh_.getPosition() + M_PI_2;
  //  knee_angle = jointKnee_.getPosition() - M_PI_2;

  // five link vmc
  thigh_angle = jointThigh_.getPosition() + M_PI;
  knee_angle = jointKnee_.getPosition();
  vmcPtr_->calc_jacobian(thigh_angle, knee_angle);
  vmcPtr_->leg_pos(thigh_angle, knee_angle);
  vmcPtr_->leg_spd(jointThigh_.getVelocity(), jointKnee_.getVelocity());

  const auto& leg_pos = vmcPtr_->getPos();
  const auto& leg_spd = vmcPtr_->getSpd();

  double effortCmd[2], jointCmd[2];
  double angle_error = angles::shortest_angular_distance(leg_pos.theta, angleCmd_);
  double f_spring_force_comp = f_spring_force(leg_pos.L0);
  if (leg_gravity_compensation_debug_)
  {
    double Tp_leg_comp{}, F_leg_comp{}, beta{};
    double G_leg = leg_mass_ * g_;
    double l_leg = get_LM(leg_pos.L0);
    double theta_leg_offset = get_theta_leg_offset(leg_pos.L0);
    if (leg_pos.theta > -M_PI_2 && leg_pos.theta < M_PI_2)
    {
      beta = leg_pos.theta + theta_leg_offset;
      F_leg_comp = -G_leg * l_leg * cos(beta);
    }
    else
    {
      if (leg_pos.theta > -M_PI && leg_pos.theta < -M_PI_2)
      {
        beta = -leg_pos.theta - M_PI - theta_leg_offset;
      }
      else if (leg_pos.theta > M_PI_2 && leg_pos.theta < M_PI)
      {
        beta = M_PI - leg_pos.theta - theta_leg_offset;
      }
      F_leg_comp = G_leg * l_leg * cos(beta);
    }
    Tp_leg_comp = G_leg * l_leg * sin(beta);
    effortCmd[0] = F_leg_comp - f_spring_force_comp;
    effortCmd[1] = Tp_leg_comp;
    debugPub_->add("F_leg_comp", F_leg_comp);
    debugPub_->add("Tp_leg_comp", Tp_leg_comp);
  }
  else
  {
    effortCmd[0] = pidLength_.computeCommand(lengthCmd_ - leg_pos.L0, period) - f_spring_force(leg_pos.L0);
    effortCmd[1] = pidAngle_.computeCommand(angle_error, period);
  }

  vmcPtr_->leg_conv(effortCmd[0], effortCmd[1], jointCmd);
  std_msgs::Float64MultiArray state;
  state.data.push_back(thigh_angle);
  state.data.push_back(knee_angle);
  state.data.push_back(leg_pos.L0);
  state.data.push_back(leg_pos.theta);
  state.data.push_back(leg_spd.dL0);
  state.data.push_back(leg_spd.dTheta);
  state.data.push_back(angle_error);
  state.data.push_back(effortCmd[0]);
  state.data.push_back(effortCmd[1]);
  state.data.push_back(jointCmd[0]);
  state.data.push_back(jointCmd[1]);
  statePublisher_.publish(state);

  debugPub_->add("f_spring_force", f_spring_force_comp);
  debugPub_->add("F_effortCmd", effortCmd[0]);
  debugPub_->publish();

  std_msgs::Float64MultiArray jointCmdState;
  jointCmdState.data.push_back(jointCmd[0]);
  jointCmdState.data.push_back(jointCmd[1]);
  jointCmdStatePublisher_.publish(jointCmdState);

  jointThigh_.setCommand(jointCmd[0]);
  jointKnee_.setCommand(jointCmd[1]);
}

double VMCController::f_spring_force(double L0)
{
  double l1 = vmcPtr_->getL1(), l2 = vmcPtr_->getL2(), Fs = spring_force_, s2 = s2_, s3 = s3_, alpha_s = alpha_s_;
  double cos_theta3, theta3, ls, Fv;
  cos_theta3 = (l1 * l1 + l2 * l2 - L0 * L0) / (2 * l1 * l2);
  theta3 = acos(cos_theta3);
  ls = sqrt(s2 * s2 + s3 * s3 - 2 * s2 * s3 * cos(theta3 - alpha_s));
  Fv = Fs * (L0 * s2 * s3 * sin(theta3 - alpha_s)) / (ls * l1 * l2 * sin(theta3));
  return Fv;

  // return ((2094.45f * L0 - 3091.28f) * L0 + 1408.375f) * L0 - 80.91f;
}

}  // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::VMCController, controller_interface::ControllerBase)
