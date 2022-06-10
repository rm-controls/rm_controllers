//
// Created by qiayuan on 2022/4/17.
//
// Reference: The Cubli: A Cube that can Jump Up and Balance

#pragma once

#include <rm_common/lqr.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class ReactionWheelController
  : public controller_interface::MultiInterfaceController<hardware_interface::ImuSensorInterface,
                                                          hardware_interface::EffortJointInterface,
                                                          hardware_interface::JointStateInterface>
{
public:
  ReactionWheelController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  static const int STATE_DIM = 3;
  static const int CONTROL_DIM = 1;
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle joint_handle_;
  hardware_interface::JointStateHandle left_wheel_handle_, right_wheel_handle_;
  double alpha_, torque_g_, pitch_offset_;
};

}  // namespace rm_chassis_controllers
