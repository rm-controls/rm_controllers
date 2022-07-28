//
// Created by yezi on 2021/12/3.
//

#pragma once

#include <rm_chassis_controllers/chassis_base.h>

#include <rm_common/eigen_types.h>
#include <effort_controllers/joint_position_controller.h>

namespace rm_chassis_controllers
{
struct Module
{
  double wheel_radius_;
  Mat2<double> wheel_frame_vel_{};
  Eigen::Matrix<double, 1, 2> direction_component_{};
  Eigen::Matrix<double, 2, 3> chassis_frame_vel_{};
  Eigen::Matrix<double, 1, 3> h{};
  effort_controllers::JointVelocityController* ctrl_wheel_;
};
class OmniController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>
{
public:
  OmniController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  geometry_msgs::Twist forwardKinematics() override;
  std::vector<Module> modules_;
  Vec3<double> chassis_vel_;
};
}  // namespace rm_chassis_controllers
