//
// Created by qiayuan on 4/23/21.
//

#include "rm_chassis_controllers/swerve.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers {

bool SwerveController::init(hardware_interface::RobotHW *robot_hw,
                            ros::NodeHandle &root_nh,
                            ros::NodeHandle &controller_nh) {
  if (!ChassisBase::init(robot_hw, root_nh, controller_nh))
    return false;
  XmlRpc::XmlRpcValue modules;
  controller_nh.getParam("modules", modules);
  ROS_ASSERT(modules.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (const auto &module:modules) {
    ROS_ASSERT(module.second.hasMember("position"));
    ROS_ASSERT(module.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(module.second["position"].size() == 2);
    ROS_ASSERT(module.second.hasMember("pivot"));
    ROS_ASSERT(module.second["pivot"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(module.second.hasMember("wheel"));
    ROS_ASSERT(module.second["wheel"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(module.second["wheel"].hasMember("radius"));

    Module m{.position_= Vec2<double>(module.second["position"][0], module.second["position"][1]),
        .pivot_offset_ = module.second["pivot"]["offset"],
        .wheel_radius_ = module.second["wheel"]["radius"],
        .ctrl_pivot_ = new effort_controllers::JointPositionController(),
        .ctrl_wheel_ = new effort_controllers::JointVelocityController()};
    ros::NodeHandle nh_pivot = ros::NodeHandle(controller_nh, "modules/" + module.first + "/pivot");
    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "modules/" + module.first + "/wheel");
    if (!m.ctrl_pivot_->init(effort_joint_interface_, nh_pivot) ||
        !m.ctrl_wheel_->init(effort_joint_interface_, nh_wheel))
      return false;
    if (module.second["pivot"].hasMember("offset"))
      m.pivot_offset_ = module.second["pivot"]["offset"];
    joint_handles_.push_back(m.ctrl_pivot_->joint_);
    joint_handles_.push_back(m.ctrl_wheel_->joint_);
    modules_.push_back(m);
  }
  return true;
}

// Ref: https://dominik.win/blog/programming-swerve-drive/

void SwerveController::moveJoint(const ros::Time &time, const ros::Duration &period) {
  Vec2<double> vel_center(ramp_x->output(), ramp_y->output());
  for (auto &module:modules_) {
    Vec2<double> vel = vel_center + ramp_w->output() * Vec2<double>(-module.position_.y(), module.position_.x());
    double vel_angle = std::atan2(vel.y(), vel.x()) + module.pivot_offset_;
    // Direction flipping and Stray module mitigation
    double a = angles::shortest_angular_distance(module.ctrl_pivot_->joint_.getPosition(), vel_angle);
    double b = angles::shortest_angular_distance(module.ctrl_pivot_->joint_.getPosition(), vel_angle + M_PI);
    if (std::abs(vel_tfed_.x) + std::abs(vel_tfed_.y) + std::abs(vel_tfed_.z) >= 0.1)
      module.ctrl_pivot_->setCommand(std::abs(a) < std::abs(b) ? vel_angle : vel_angle + M_PI);
    module.ctrl_wheel_->setCommand(vel.norm() / module.wheel_radius_ * std::cos(a));
    module.ctrl_pivot_->update(time, period);
    module.ctrl_wheel_->update(time, period);
  }
}

geometry_msgs::Twist SwerveController::forwardKinematics() {
  geometry_msgs::Twist vel;
  vel.linear.x = vel_tfed_.x;
  vel.linear.y = vel_tfed_.y;
  vel.angular.z = vel_tfed_.z;
  return vel;
}

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SwerveController, controller_interface::ControllerBase)
}
