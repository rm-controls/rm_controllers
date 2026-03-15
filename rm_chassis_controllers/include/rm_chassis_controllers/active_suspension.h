//
// Created by xuncheng on 2025/11/29.
//

#include "rm_chassis_controllers/chassis_base.h"
#include "rm_chassis_controllers/omni.h"
#include <effort_controllers/joint_position_controller.h>
#include <rm_msgs/ChassisActiveSusCmd.h>
#include <std_msgs/Bool.h>
#include <rm_common/ros_utilities.h>
#include <string>
#include <Eigen/QR>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
enum class State
{
  DOWN = 0,
  MID = 1,
  UP = 2
};

class ActiveSuspensionController : public OmniController
{
public:
  ActiveSuspensionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void ActiveSuspensionCallBack(const rm_msgs::ChassisActiveSusCmd& msg);

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;

  std::vector<std::shared_ptr<effort_controllers::JointPositionController>> active_suspension_joints_;
  std::vector<hardware_interface::JointHandle> active_suspension_joint_handles_{};

  ros::Subscriber active_suspension_sub_;
  double current_pos_{ 0. };
  double target_pos_{ 0. };
  double feedforward_offset{ 0. };
  double feedforward_effort{ 0. };
  double stretch_coff_A_{ 0. };
  double stretch_coff_k_{ 0. };
  double shrink_coff_A_{ 0. };
  double shrink_coff_k_{ 0. };
  double feedforward_effect_time_{ 0. };
  double static_effort{ 0. };

  ros::Time feedforward_timer;
  State current_state_{ State::DOWN };
  State last_state_{ State::DOWN };
};
}  // namespace rm_chassis_controllers
