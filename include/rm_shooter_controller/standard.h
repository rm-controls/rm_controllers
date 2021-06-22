//
// Created by huakang on 2021/1/18.
//

#ifndef SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
#define SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_

#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/ros_utilities.h>
#include <realtime_tools/realtime_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rm_shooter_controllers/ShooterConfig.h>
#include <rm_msgs/ShootCmd.h>

#include <utility>

namespace rm_shooter_controllers {
enum {
  PASSIVE = 0,
  READY = 1,
  PUSH = 2,
  STOP = 3,
  BLOCK = 4
};

struct Config {
  double block_effort, block_speed, block_duration, anti_block_angle, anti_block_threshold;
  double qd_10, qd_15, qd_16, qd_18, qd_30;
};

class Controller
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                            hardware_interface::RobotStateInterface> {
 public:
  Controller() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle
            &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time & /*time*/) override;

 private:
  void passive();
  void stop(const ros::Time &time, const ros::Duration &period);
  void ready(const ros::Duration &period);
  void push(const ros::Time &time, const ros::Duration &period);
  void block(const ros::Time &time, const ros::Duration &period);
  void setSpeed(const rm_msgs::ShootCmd &cmd);
  void normalize();
  void commandCB(const rm_msgs::ShootCmdConstPtr &msg) { cmd_rt_buffer_.writeFromNonRT(*msg); }
  void reconfigCB(rm_shooter_controllers::ShooterConfig &config, uint32_t /*level*/);

  hardware_interface::EffortJointInterface *effort_joint_interface_{};
  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
  effort_controllers::JointPositionController ctrl_trigger_;
  int push_per_rotation_{};
  double push_qd_threshold_{};
  bool dynamic_reconfig_initialized_ = false;
  bool state_changed_ = false;
  bool maybe_block_ = false;

  ros::Time last_shoot_time_, block_time_;
  int state_ = PASSIVE;
  Config config_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer;
  realtime_tools::RealtimeBuffer<rm_msgs::ShootCmd> cmd_rt_buffer_;
  rm_msgs::ShootCmd cmd_;
  ros::Subscriber cmd_subscriber_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig> *d_srv_{};
};

} // namespace rm_shooter_controllers
#endif //SRC_RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_STANDARD_H_
