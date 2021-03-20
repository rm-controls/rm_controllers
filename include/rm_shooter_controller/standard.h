//
// Created by huakang on 2021/1/18.
//

#ifndef SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
#define SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rm_shooter_controllers/ShooterStandardConfig.h>
#include <rm_msgs/ShootCmd.h>

namespace rm_shooter_controllers {

enum State {
  PASSIVE = 0,
  READY = 1,
  PUSH = 2,
  STOP = 3,
  BLOCK = 4
};

struct Config {
  double push_angle, block_effort, block_duration, block_speed, anti_block_angle, anti_block_error;
  double qd_10, qd_15, qd_16, qd_18, qd_30;
};

class ShooterStandardController
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                            hardware_interface::RobotStateInterface> {

 public:
  ShooterStandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
 protected:
  void passive();
  void ready(const ros::Duration &period);
  void push(const ros::Time &time, const ros::Duration &period);
  void block(const ros::Time &time, const ros::Duration &period);
  void stop(const ros::Time &time, const ros::Duration &period);
  void moveJoint(const ros::Duration &period);
  void commandCB(const rm_msgs::ShootCmdConstPtr &msg);
  void reconfigCB(rm_shooter_controllers::ShooterStandardConfig &config, uint32_t /*level*/);

  hardware_interface::JointHandle joint_friction_l_, joint_friction_r_, joint_trigger_;
  control_toolbox::Pid pid_friction_l_, pid_friction_r_, pid_trigger_;

  double friction_qd_des_{};
  double trigger_q_des_{};
  bool dynamic_reconfig_initialized_ = false;
  bool state_changed_ = false;
  bool is_start_block_time_ = false;

  ros::Time last_shoot_time_;
  ros::Time block_time_;

  State state_ = PASSIVE;
  Config config_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer;
  realtime_tools::RealtimeBuffer<rm_msgs::ShootCmd> cmd_rt_buffer_;
  rm_msgs::ShootCmd cmd_;
  ros::Subscriber cmd_subscriber_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterStandardConfig> *d_srv_{};
};

} // namespace rm_shooter_controllers
#endif //SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
