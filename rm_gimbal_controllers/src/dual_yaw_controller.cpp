#include "rm_gimbal_controllers/dual_yaw_controller.h"

#include <angles/angles.h>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <urdf/model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_gimbal_controllers
{
bool DualYawController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  if (!initBaseYaw(robot_hw, controller_nh))
    return false;
  return Controller::init(robot_hw, root_nh, controller_nh);
}

bool DualYawController::initBaseYaw(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  ros::NodeHandle base_yaw_nh(controller_nh, "controllers/base_yaw");
  ros::NodeHandle base_yaw_pid_nh(controller_nh, "controllers/base_yaw/pid_pos");

  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", controller_nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  base_yaw_joint_ = urdf.getJoint(getParam(base_yaw_nh, "joint", std::string("base_yaw_joint")));
  if (!base_yaw_joint_)
  {
    ROS_ERROR("Could not find base_yaw joint in urdf");
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  base_yaw_ctrl_ = std::make_unique<effort_controllers::JointVelocityController>();
  base_yaw_pid_pos_ = std::make_unique<control_toolbox::Pid>();
  base_yaw_pos_state_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>>(base_yaw_nh, "pos_state", 1);
  return base_yaw_ctrl_->init(effort_joint_interface, base_yaw_nh) && base_yaw_pid_pos_->init(base_yaw_pid_nh);
}

bool DualYawController::shouldInitializeController(const std::string& /*name*/,
                                                   const urdf::JointConstSharedPtr& joint_urdf, int axis) const
{
  return !(axis == 2 && base_yaw_joint_ && joint_urdf->name == base_yaw_joint_->name);
}

bool DualYawController::shouldApplyJointLimit(int axis) const
{
  return axis != 2;
}

std::string DualYawController::getBaseFrameID(const std::unordered_map<int, urdf::JointConstSharedPtr>& joint_urdfs)
{
  if (base_yaw_joint_)
    return base_yaw_joint_->parent_link_name.c_str();
  return Controller::getBaseFrameID(joint_urdfs);
}

void DualYawController::updateYawJoint(const ros::Time& time, const ros::Duration& period,
                                       const geometry_msgs::Vector3& angular_vel, const double pos_real[3],
                                       const double pos_des[3], const double /*pos_des_temp*/[3],
                                       const double vel_des[3], const double angle_error[3],
                                       const double traject_pos_des[3], const double traject_angle_error[3])
{
  if (pid_pos_.find(2) != pid_pos_.end() && ctrls_.find(2) != ctrls_.end())
  {
    updateGimbalYawController(time, period, angular_vel, *ctrls_.at(2), *pid_pos_.at(2), config_.yaw_k_v_, vel_des,
                              angle_error, traject_angle_error);
  }

  if (!base_yaw_ctrl_ || !base_yaw_pid_pos_)
    return;

  double base_yaw_pos_real = 0.;
  try
  {
    geometry_msgs::TransformStamped odom2base_yaw =
        robot_state_handle_.lookupTransform("odom", base_yaw_joint_->child_link_name, time);
    double roll{}, pitch{};
    quatToRPY(odom2base_yaw.transform.rotation, roll, pitch, base_yaw_pos_real);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  double armor_set_point{};
  double base_yaw_set_point = getTrackArmorSetPoint(time, armor_set_point) ? armor_set_point : pos_des[2];
  double base_yaw_error = angles::shortest_angular_distance(base_yaw_pos_real, base_yaw_set_point);
  base_yaw_pid_pos_->computeCommand(base_yaw_error, period);
  base_yaw_ctrl_->setCommand(base_yaw_pid_pos_->getCurrentCmd() + base_yaw_ctrl_->joint_.getVelocity() - angular_vel.z);
  base_yaw_ctrl_->update(time, period);
  publishBaseYawState(time, base_yaw_pos_real, base_yaw_set_point, vel_des, base_yaw_error);
}

bool DualYawController::getTrackArmorSetPoint(const ros::Time& time, double& armor_set_point)
{
  if (state_ != TRACK || !base_yaw_joint_ || data_track_.header.stamp.isZero())
    return false;

  geometry_msgs::Point target_pos = data_track_.position;
  if (!std::isfinite(target_pos.x) || !std::isfinite(target_pos.y) || !std::isfinite(target_pos.z))
    return false;

  try
  {
    geometry_msgs::TransformStamped odom2base_yaw =
        robot_state_handle_.lookupTransform("odom", base_yaw_joint_->child_link_name, time);
    geometry_msgs::Point base_yaw_pos_odom;
    base_yaw_pos_odom.x = odom2base_yaw.transform.translation.x;
    base_yaw_pos_odom.y = odom2base_yaw.transform.translation.y;
    base_yaw_pos_odom.z = odom2base_yaw.transform.translation.z;

    armor_set_point = std::atan2(target_pos.y - base_yaw_pos_odom.y, target_pos.x - base_yaw_pos_odom.x);
    if (!std::isfinite(armor_set_point))
      return false;

    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

void DualYawController::publishBaseYawState(const ros::Time& time, double base_yaw_pos_real, double base_yaw_set_point,
                                            const double vel_des[3], double base_yaw_error)
{
  if (base_yaw_pos_state_pub_ && loop_count_ % 10 == 0 && base_yaw_pos_state_pub_->trylock())
  {
    base_yaw_pos_state_pub_->msg_.header.stamp = time;
    base_yaw_pos_state_pub_->msg_.set_point = base_yaw_set_point;
    base_yaw_pos_state_pub_->msg_.traject_set_point = base_yaw_set_point;
    base_yaw_pos_state_pub_->msg_.set_point_dot = vel_des[2];
    base_yaw_pos_state_pub_->msg_.process_value = base_yaw_pos_real;
    base_yaw_pos_state_pub_->msg_.error = base_yaw_error;
    base_yaw_pos_state_pub_->msg_.command = base_yaw_pid_pos_->getCurrentCmd();
    base_yaw_pos_state_pub_->msg_.shoot_number = bullet_solver_->getShootnum();
    base_yaw_pos_state_pub_->unlockAndPublish();
  }
}

}  // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::DualYawController, controller_interface::ControllerBase)
