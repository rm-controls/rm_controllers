//
// Created by yezi on 2021/12/3.
//

#include <rm_chassis_controllers/omni.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
bool OmniController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  XmlRpc::XmlRpcValue modules;
  controller_nh.getParam("modules", modules);
  ROS_ASSERT(modules.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (const auto& module : modules)
  {
    XmlRpc::XmlRpcValue direction_component, wheel_frame_vel, chassis_frame_vel;
    controller_nh.getParam("modules/" + module.first + "/direction_component", direction_component);
    controller_nh.getParam("modules/" + module.first + "/wheel_frame_vel", wheel_frame_vel);
    controller_nh.getParam("modules/" + module.first + "/chassis_frame_vel", chassis_frame_vel);
    ROS_ASSERT(module.second.hasMember("wheel_radius") && module.second.hasMember("chassis_frame_vel") &&
               module.second.hasMember("wheel_frame_vel") && module.second.hasMember("direction_component"));
    ROS_ASSERT(chassis_frame_vel.getType() == XmlRpc::XmlRpcValue::TypeArray &&
               wheel_frame_vel.getType() == XmlRpc::XmlRpcValue::TypeArray &&
               direction_component.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(chassis_frame_vel.size() == 2 && wheel_frame_vel.size() == 2 && direction_component.size() == 2);
    Module m;
    controller_nh.getParam("modules/" + module.first + "/wheel_radius", m.wheel_radius_);
    m.ctrl_wheel_ = new effort_controllers::JointVelocityController();
    m.direction_component_.setZero();
    m.wheel_frame_vel_.setZero();
    m.chassis_frame_vel_.setZero();
    m.direction_component_ = Eigen::Matrix<double, 1, 2>(direction_component[0], direction_component[1]);
    m.chassis_frame_vel_ << chassis_frame_vel[0][0], chassis_frame_vel[0][1], chassis_frame_vel[0][2],
        chassis_frame_vel[1][0], chassis_frame_vel[1][1], chassis_frame_vel[1][2];
    m.wheel_frame_vel_ << wheel_frame_vel[0][0], wheel_frame_vel[0][1], wheel_frame_vel[1][0], wheel_frame_vel[1][1];
    m.h = (1 / m.wheel_radius_) * m.direction_component_ * m.wheel_frame_vel_ * m.chassis_frame_vel_;
    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "modules/" + module.first);
    if (!m.ctrl_wheel_->init(effort_joint_interface_, nh_wheel))
      return false;
    joint_handles_.push_back(m.ctrl_wheel_->joint_);
    modules_.push_back(m);
  }
  return true;
}

void OmniController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  chassis_vel_.setZero();
  chassis_vel_(0, 0) = vel_cmd_.z;
  chassis_vel_(1, 0) = vel_cmd_.x;
  chassis_vel_(2, 0) = vel_cmd_.y;
  for (auto& module : modules_)
  {
    module.ctrl_wheel_->setCommand(module.h * chassis_vel_);
    module.ctrl_wheel_->update(time, period);
  }
}

geometry_msgs::Twist OmniController::forwardKinematics()
{
  geometry_msgs::Twist vel_data{};
  chassis_vel_.setZero();

  return vel_data;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::OmniController, controller_interface::ControllerBase)
