//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <ros/time.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <angles/angles.h>

#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
class BipedalController;

class ModeBase
{
public:
  virtual void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) = 0;
  virtual const char* name() const = 0;
  virtual ~ModeBase() = default;
  void updateEstimation(const Eigen::Matrix<double, STATE_DIM, 1>& x_left,
                        const Eigen::Matrix<double, STATE_DIM, 1>& x_right);
  void updateLegKinematics(double* left_angle, double* right_angle, double* left_pos, double* left_spd,
                           double* right_pos, double* right_spd);
  void updateBaseState(const geometry_msgs::Vector3& angular_vel_base, const geometry_msgs::Vector3& linear_acc_base,
                       const double& roll, const double& pitch, const double& yaw);

  void updateUnstick(const bool& left_unstick, const bool& right_unstick);
  inline double getRealxVel()
  {
    return x_left_[3];
  }

  inline double getRealYawVel()
  {
    return angular_vel_base_.z;
  }

  inline bool getUnstick()
  {
    return (left_unstick_ && right_unstick_);
  }

protected:
  Eigen::Matrix<double, STATE_DIM, 1> x_left_{}, x_right_{};
  double left_angle_[2], right_angle_[2], left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];
  bool left_unstick_{ false }, right_unstick_{ false };
  geometry_msgs::Vector3 angular_vel_base_{}, linear_acc_base_{};
  double roll_, pitch_, yaw_;
  double yaw_total_{}, yaw_total_last_{};
};

}  // namespace rm_chassis_controllers
