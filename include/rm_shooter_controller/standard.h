//
// Created by huakang on 2021/1/18.
//

#ifndef SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
#define SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_base/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rm_shooter_controllers/ShooterStandardConfig.h>
#include <rm_msgs/ShootCmd.h>

namespace rm_shooter_controllers {
enum StandardState {
  PASSIVE,
  READY,
  PUSH,
  BLOCK
};

class ShooterStandardController
    : public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface,
        hardware_interface::RobotStateInterface> {
 public:
  ShooterStandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void shoot(int num, double freq);
  void setSpeed(double speed);
 protected:
  void passive();
  void ready(const ros::Duration &period);
  void push(const ros::Time &time, const ros::Duration &period);
  void block(const ros::Time &time, const ros::Duration &period);
  void moveJoint(const ros::Duration &period);
  void commandCB(const rm_msgs::ShootCmdConstPtr &msg);
  void reconfigCB(const rm_shooter_controllers::ShooterStandardConfig &config,
                  uint32_t level);

  control_toolbox::Pid pid_fiction_l_, pid_fiction_r_, pid_trigger_;
  hardware_interface::JointHandle joint_fiction_l_, joint_fiction_r_,
      joint_trigger_;
  hardware_interface::RobotStateHandle robot_state_handle_;

  double fric_qd_des_{};
  double push_angle_{};
  double friction_radius_{};
  double bullet_speed_{};
  double trigger_des_{};
  double ff_coff_{};
  double block_coff_{};
//  double anti_block_duration_{};
//  ros::Time anti_block_time_;

  int shoot_num_{};
  double shoot_freq_{};
  ros::Time last_shoot_time_;

  bool state_changed_{};
  bool shoot_num_change_ = false;
  StandardState state_ = PASSIVE;
  ros::Subscriber cmd_subscriber_;
  realtime_tools::RealtimeBuffer<rm_msgs::ShootCmd> cmd_rt_buffer_;
  rm_msgs::ShootCmd cmd_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterStandardConfig>
      *d_srv_;
};
} // namespace rm_shooter_controllers
#endif //SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
