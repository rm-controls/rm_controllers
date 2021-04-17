//
// Created by huakang on 2021/3/28.
//

#ifndef RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_SHOOTER_BASE_H_
#define RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_SHOOTER_BASE_H_

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rm_shooter_controllers/ShooterBaseConfig.h>
#include <rm_msgs/ShootCmd.h>

namespace rm_shooter_base {

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

class ShooterBase : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                          hardware_interface::RobotStateInterface> {
 public:
  ShooterBase() = default;
  virtual bool init(hardware_interface::RobotHW *robot_hw,
                    ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  virtual void update(const ros::Time &time, const ros::Duration &period) override;
 protected:
  virtual void passive();
  virtual void ready(const ros::Duration &period);
  virtual void push(const ros::Time &time, const ros::Duration &period);
  virtual void block(const ros::Time &time, const ros::Duration &period);
  virtual void stop(const ros::Time &time, const ros::Duration &period) {};
  virtual void moveJoint(const ros::Duration &period) = 0;
  virtual void commandCB(const rm_msgs::ShootCmdConstPtr &msg);
  virtual void reconfigCB(rm_shooter_controllers::ShooterBaseConfig &config, uint32_t /*level*/);

  std::vector<hardware_interface::JointHandle> joint_friction_vector_{}, joint_trigger_vector_{};
  std::vector<control_toolbox::Pid> pid_friction_vector_{}, pid_trigger_vector_{};

  double friction_qd_des_{}, trigger_q_des_{}, last_trigger_q_des_{};
  double enter_push_qd_coef_{}, push_angle_error_{};
  bool dynamic_reconfig_initialized_ = false;
  bool state_changed_ = false;
  bool is_start_block_time_ = false;
  bool is_out_from_block_ = false;

  ros::Time last_shoot_time_;
  ros::Time block_time_;

  State state_ = PASSIVE;
  Config config_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer;
  realtime_tools::RealtimeBuffer<rm_msgs::ShootCmd> cmd_rt_buffer_;
  rm_msgs::ShootCmd cmd_;
  ros::Subscriber cmd_subscriber_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterBaseConfig> *d_srv_{};
};

} // namespace rm_shooter_base
#endif //RM_SHOOTER_CONTROLLERS_INCLUDE_RM_SHOOTER_CONTROLLER_SHOOTER_BASE_H_
