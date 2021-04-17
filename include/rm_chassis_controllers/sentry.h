//
// Created by flying on 2021/1/18.
//

#ifndef RM_CHASSIS_CONTROLLER_SENTRY_H
#define RM_CHASSIS_CONTROLLER_SENTRY_H

#include <rm_chassis_controllers/chassis_base.h>
#include <rm_common/tf_rt_broadcaster.h>

namespace rm_chassis_controllers {

class SentryController : public rm_chassis_base::ChassisBase {
 public:
  SentryController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  using rm_chassis_base::ChassisBase::passive;
  using rm_chassis_base::ChassisBase::raw;
  using rm_chassis_base::ChassisBase::recovery;
  using rm_chassis_base::ChassisBase::cmdChassisCallback;
  using rm_chassis_base::ChassisBase::cmdVelCallback;

 private:
  void moveJoint(const ros::Duration &period) override;
  void updateOdom(const ros::Time &time, const ros::Duration &period) override;
  geometry_msgs::Twist iKine(const ros::Duration &period) override;

  control_toolbox::Pid pid_wheel_;
  hardware_interface::JointHandle joint_wheel_;
  geometry_msgs::TransformStamped odom2base_{};
  robot_state_controller::TfRtBroadcaster tf_broadcaster_{};
};
}
#endif //RM_CHASSIS_CONTROLLER_STANDARD_H
