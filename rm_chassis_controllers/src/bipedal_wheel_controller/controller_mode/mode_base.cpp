//
// Created by guanlin on 25-9-4.
//

#include "bipedal_wheel_controller/controller_mode/mode_base.h"

namespace rm_chassis_controllers
{
void ModeBase::updateEstimation(const Eigen::Matrix<double, STATE_DIM, 1>& x_left,
                                const Eigen::Matrix<double, STATE_DIM, 1>& x_right)
{
  x_left_ = x_left;
  x_right_ = x_right;
}

void ModeBase::updateLegKinematics(double* left_angle, double* right_angle, double* left_pos, double* left_spd,
                                   double* right_pos, double* right_spd)
{
  std::memcpy(left_pos_, left_pos, 2 * sizeof(double));
  std::memcpy(left_spd_, left_spd, 2 * sizeof(double));
  std::memcpy(right_pos_, right_pos, 2 * sizeof(double));
  std::memcpy(right_spd_, right_spd, 2 * sizeof(double));
  std::memcpy(left_angle_, left_angle, 2 * sizeof(double));
  std::memcpy(right_angle_, right_angle, 2 * sizeof(double));
}

void ModeBase::updateBaseState(const geometry_msgs::Vector3& angular_vel_base,
                               const geometry_msgs::Vector3& linear_acc_base, const double& roll, const double& pitch,
                               const double& yaw)
{
  angular_vel_base_ = angular_vel_base;
  linear_acc_base_ = linear_acc_base;
  roll_ = roll;
  pitch_ = pitch;
  yaw_ = yaw;
  yaw_total_last_ = yaw_total_;
  yaw_total_ = yaw_total_last_ + angles::shortest_angular_distance(yaw_total_last_, yaw_);
}

void ModeBase::updateUnstick(const bool& left_unstick, const bool& right_unstick)
{
  left_unstick_ = left_unstick;
  right_unstick_ = right_unstick;
}

}  // namespace rm_chassis_controllers
