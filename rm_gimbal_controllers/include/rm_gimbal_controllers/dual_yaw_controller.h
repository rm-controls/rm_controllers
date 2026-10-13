/*******************************************************************************
 * BSD 3-Clause License
 ******************************************************************************/

#pragma once

#include <rm_gimbal_controllers/gimbal_base.h>

namespace rm_gimbal_controllers
{
class DualYawController : public Controller
{
public:
  DualYawController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

protected:
  bool shouldInitializeController(const std::string& name, const urdf::JointConstSharedPtr& joint_urdf,
                                  int axis) const override;
  bool shouldApplyJointLimit(int axis) const override;
  std::string getBaseFrameID(const std::unordered_map<int, urdf::JointConstSharedPtr>& joint_urdfs) override;
  void updateYawJoint(const ros::Time& time, const ros::Duration& period, const geometry_msgs::Vector3& angular_vel,
                      const double pos_real[3], const double pos_des[3], const double pos_des_temp[3],
                      const double vel_des[3], const double angle_error[3], const double traject_pos_des[3],
                      const double traject_angle_error[3]) override;

private:
  bool initBaseYaw(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh);
  bool getTrackArmorSetPoint(const ros::Time& time, double& armor_set_point);
  void publishBaseYawState(const ros::Time& time, double base_yaw_pos_real, double base_yaw_set_point,
                           const double vel_des[3], double base_yaw_error);

  urdf::JointConstSharedPtr base_yaw_joint_;
  std::unique_ptr<effort_controllers::JointVelocityController> base_yaw_ctrl_;
  std::unique_ptr<control_toolbox::Pid> base_yaw_pid_pos_;
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>> base_yaw_pos_state_pub_;
};
}  // namespace rm_gimbal_controllers
