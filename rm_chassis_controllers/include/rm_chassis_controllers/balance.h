//
// Created by yezi on 2022/11/15.
//

#pragma once

#include <rm_common/lqr.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_chassis_controllers/QRConfig.h>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
public:
  BalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void reconfigCB(rm_chassis_controllers::QRConfig& config, uint32_t /*level*/);
  geometry_msgs::Twist odometry() override;
  static const int STATE_DIM = 10;
  static const int CONTROL_DIM = 4;
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;
  double wheel_radius_, wheel_base_;
  double position_des_ = 0;
  double yaw_des_ = 0;

  // dynamic reconfigure
  double q_dynamic_[STATE_DIM], r_dynamic_[CONTROL_DIM], q_config_[STATE_DIM], r_config_[CONTROL_DIM];
  bool dynamic_reconfig_initialized_ = false;
  dynamic_reconfigure::Server<rm_chassis_controllers::QRConfig>* reconf_server_;

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_,
      left_momentum_block_joint_handle_, right_momentum_block_joint_handle_;
  ros::Publisher state_pub_;
};

}  // namespace rm_chassis_controllers
