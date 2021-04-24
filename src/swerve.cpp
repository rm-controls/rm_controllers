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
    ROS_ASSERT(module.second["pivot"].hasMember("name"));
    ROS_ASSERT(module.second.hasMember("wheel"));
    ROS_ASSERT(module.second["wheel"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(module.second["wheel"].hasMember("name"));
    ROS_ASSERT(module.second["wheel"].hasMember("radius"));

    auto *effort_jnt_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    Module m{.position_= Vec2<double>(module.second["position"][0], module.second["position"][1]),
        .wheel_radius_ = module.second["wheel"]["radius"],
        .joint_pivot_ =effort_jnt_interface->getHandle(module.second["pivot"]["name"]),
        .joint_wheel_ =effort_jnt_interface->getHandle(module.second["wheel"]["name"]),
        .pid_pivot_ = control_toolbox::Pid(),
        .pid_wheel_ = control_toolbox::Pid()};
    if (!m.pid_pivot_.init(ros::NodeHandle(controller_nh, "modules/" + module.first + "/pivot/pid")) ||
        !m.pid_wheel_.init(ros::NodeHandle(controller_nh, "modules/" + module.first + "/wheel/pid")))
      return false;
    if (module.second["pivot"].hasMember("offset"))
      m.pivot_offset_ = module.second["pivot"]["offset"];

    joint_handles_.push_back(effort_jnt_interface->getHandle(module.second["pivot"]["name"]));
    joint_handles_.push_back(effort_jnt_interface->getHandle(module.second["wheel"]["name"]));
    wheel_pids_.push_back(&m.pid_wheel_);
    modules_.push_back(m);
  }
  return true;
}

// Ref: https://dominik.win/blog/programming-swerve-drive/

void SwerveController::moveJoint(const ros::Duration &period) {
  Vec2<double> vel_center(vel_tfed_.vector.x, vel_tfed_.vector.y);
  for (auto &module:modules_) {
    Vec2<double> vel = vel_center + vel_tfed_.vector.z * Vec2<double>(-module.position_.y(), module.position_.x());
    double vel_angle = std::atan2(vel.y(), vel.x()) + module.pivot_offset_;
    // Direction flipping and Stray module mitigation
    double error_pivot = std::min(
        angles::shortest_angular_distance(module.joint_pivot_.getPosition(), vel_angle),
        angles::shortest_angular_distance(module.joint_pivot_.getPosition(), vel_angle + M_PI));
    double wheel_des = vel.norm() / module.wheel_radius_
        * std::cos(angles::shortest_angular_distance(module.joint_pivot_.getPosition(), vel_angle));
    double error_wheel = wheel_des - module.joint_wheel_.getVelocity();
    // PID
    module.pid_pivot_.computeCommand(error_pivot, period);
    module.pid_wheel_.computeCommand(error_wheel, period);
    module.joint_pivot_.setCommand(module.pid_pivot_.getCurrentCmd());
  }
  // Effort limit
  double scale = getEffortLimitScale();
  for (auto &module:modules_)
    module.joint_wheel_.setCommand(scale * module.pid_wheel_.getCurrentCmd());
}

geometry_msgs::Twist SwerveController::forwardKinematics() {
  return geometry_msgs::Twist();
}

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SwerveController, controller_interface::ControllerBase)
}
