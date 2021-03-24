//
// Created by flying on 2021/1/18.
//

#ifndef RM_CHASSIS_CONTROLLER_STANDARD_H
#define RM_CHASSIS_CONTROLLER_STANDARD_H

#include <rm_chassis_controller/chassis_base.h>
#include <rm_common/tf_rt_broadcaster.h>

namespace rm_chassis_controllers {

class StandardController : public rm_chassis_base::ChassisBase {
 public:
  StandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  using rm_chassis_base::ChassisBase::passive;
  using rm_chassis_base::ChassisBase::recovery;
  using rm_chassis_base::ChassisBase::raw;
  using rm_chassis_base::ChassisBase::cmdChassisCallback;
  using rm_chassis_base::ChassisBase::cmdVelCallback;
  using rm_chassis_base::ChassisBase::tfVelFromYawToBase;

 private:
  void follow(const ros::Time &time, const ros::Duration &period) override;
  void twist(const ros::Time &time, const ros::Duration &period);
  void gyro(const ros::Time &time, const ros::Duration &period);
  void moveJoint(const ros::Duration &period) override;
  void updateOdom(const ros::Time &time, const ros::Duration &period) override;
  geometry_msgs::Twist iKine(const ros::Duration &period) override;

  control_toolbox::Pid pid_rf_, pid_lf_, pid_rb_, pid_lb_;
  control_toolbox::Pid pid_follow_, pid_twist_;
  hardware_interface::JointHandle joint_rf_, joint_lf_, joint_rb_, joint_lb_;
  double wheel_track_{};
  bool enable_odom_tf_{};
  robot_state_controller::TfRtBroadcaster tf_broadcaster_{};
};
} // namespace rm_chassis_controllers

#endif // RM_CHASSIS_CONTROLLER_STANDARD_H
