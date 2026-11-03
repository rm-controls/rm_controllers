//
// Created by qiayuan on 2022/7/29.
//

#pragma once

#include <Eigen/Dense>
#include <rm_common/filters/lp_filter.h>
#include <rm_common/rls.h>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
class OmniController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>
{
public:
  OmniController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

protected:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;

private:
  geometry_msgs::Twist odometry() override;

  std::vector<std::shared_ptr<effort_controllers::JointVelocityController>> joints_;
  Eigen::MatrixXd chassis2joints_;
  void powerLimit() override;
  void updatePowerStatus() override;
  virtual void stateJudge();

  std::array<LowPassFilter*, 4> motor_lp_filters_{};
  std::unique_ptr<Rls<double>> rls_{};
};

}  // namespace rm_chassis_controllers
