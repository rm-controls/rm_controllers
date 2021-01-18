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
#include <rm_msgs/ShootCmd.h>

namespace rm_shooter_controllers {
enum StandardState {
  PASSIVE,
  FEED,
  READY,
  PUSH,
  BLOCK
};

class ShooterStandardBaseController
    : public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface,
        hardware_interface::RobotStateInterface> {
 public:
  ShooterStandardBaseController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &conctroller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void shoot(int num, double freq);
  viod setSpeed(double speed);
 protected:
  void passive();
  void ready();
  void push();
  void block();
  void commandCB(const rm_msgs::ShootCmdConstPtr &msg);

  control_toolbox::Pid pid_wheel_, pid_trigger_;
  hardware_interface::JointHandle joint_wheel_l_, joint_wheel_r_,
      joint_trigger_;
  hardware_interface::RobotStateHandle robot_state_handle_;

  double fric_qd_des_{};
  double push_angle_{};
  double friction_radius_{};
  double ff_coff_{};
  double ff_duration_{};
  double bullet_speed_{};
  double block_coff_{};
  double block_duration_{};
  double anti_block_speed_{};
  double anti_block_duration_{};
  int shoot_num_{};
  double shoot_freq_{};

  bool state_changed_{};
  StandardState state_ = PASSIVE;
  ros::Subscriber cmd_subscriber_;
  realtime_tools::RealtimeBuffer<rm_msgs::ShootCmd> cmd_rt_buffer_;
  rm_msgs::ShootCmd cmd_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterStandardConfig>
      *d_srv_;
};
}
#endif //SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
