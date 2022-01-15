//
// Created by yezi on 2021/12/3.
//

#include <rm_chassis_controllers/omni.h>
#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
bool OmniController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  if (!controller_nh.getParam("chassis_radius", chassis_radius_))
  {
    ROS_ERROR("chassis_radius is not set");
    return false;
  }
  ros::NodeHandle nh_lf = ros::NodeHandle(controller_nh, "left_front");
  ros::NodeHandle nh_rf = ros::NodeHandle(controller_nh, "right_front");
  ros::NodeHandle nh_lb = ros::NodeHandle(controller_nh, "left_back");
  ros::NodeHandle nh_rb = ros::NodeHandle(controller_nh, "right_back");
  if (!ctrl_lf_.init(effort_joint_interface_, nh_lf) || !ctrl_rf_.init(effort_joint_interface_, nh_rf) ||
      !ctrl_lb_.init(effort_joint_interface_, nh_lb) || !ctrl_rb_.init(effort_joint_interface_, nh_rb))
    return false;
  joint_handles_.push_back(ctrl_lf_.joint_);
  joint_handles_.push_back(ctrl_rf_.joint_);
  joint_handles_.push_back(ctrl_lb_.joint_);
  joint_handles_.push_back(ctrl_rb_.joint_);
  return true;
}

void OmniController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  ctrl_rf_.setCommand(((vel_cmd_.x + vel_cmd_.y + sqrt(2) * vel_cmd_.z * chassis_radius_) / sqrt(2)) / wheel_radius_);
  ctrl_lf_.setCommand(((-vel_cmd_.x + vel_cmd_.y + sqrt(2) * vel_cmd_.z * chassis_radius_) / sqrt(2)) / wheel_radius_);
  ctrl_lb_.setCommand(((-vel_cmd_.x - vel_cmd_.y + sqrt(2) * vel_cmd_.z * chassis_radius_) / sqrt(2)) / wheel_radius_);
  ctrl_rb_.setCommand(((vel_cmd_.x - vel_cmd_.y + sqrt(2) * vel_cmd_.z * chassis_radius_) / sqrt(2)) / wheel_radius_);
  ctrl_lf_.update(time, period);
  ctrl_rf_.update(time, period);
  ctrl_lb_.update(time, period);
  ctrl_rb_.update(time, period);
}

geometry_msgs::Twist OmniController::forwardKinematics()
{
  geometry_msgs::Twist vel_data;
  double k = wheel_radius_ / 2;
  double lf_velocity = ctrl_lf_.joint_.getVelocity();
  double rf_velocity = ctrl_rf_.joint_.getVelocity();
  double lb_velocity = ctrl_lb_.joint_.getVelocity();
  double rb_velocity = ctrl_rb_.joint_.getVelocity();
  vel_data.linear.x = k * (-lf_velocity + rf_velocity - lb_velocity + rb_velocity) / sqrt(2);
  vel_data.linear.y = k * (lf_velocity + rf_velocity - lb_velocity - rb_velocity) / sqrt(2);
  vel_data.angular.z = k * (lf_velocity + rf_velocity + lb_velocity + rb_velocity) / (2 * chassis_radius_);
  return vel_data;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::OmniController, controller_interface::ControllerBase)
