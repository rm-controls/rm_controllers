//
// Created by guanlin on 25-8-28.
//

#pragma once

#include <rm_common/lqr.h>
#include <rm_msgs/LegCmd.h>
#include <rm_msgs/LeggedChassisStatus.h>
#include <rm_msgs/LeggedLQRStatus.h>
#include <rm_msgs/LQRkMatrix.h>
#include <rm_msgs/LeggedChassisMode.h>
#include <rm_msgs/LeggedUpstairStatus.h>
#include <rm_common/filters/kalman_filter.h>
#include <rm_common/filters/lp_filter.h>
#include <rm_common/DebugDataPublisher.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "rm_chassis_controllers/chassis_base.h"
#include <dynamic_reconfigure/server.h>
#include <rm_chassis_controllers/LQRWeightConfig.h>

#include "bipedal_wheel_controller/helper_functions.h"
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/controller_mode/mode_manager.h"
#include "bipedal_wheel_controller/vmc/VMC.h"
#include "bipedal_wheel_controller/controller_interface.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;

struct LQRConfig
{
  double Q_theta{}, Q_d_theta{}, Q_x{}, Q_dx{}, Q_phi{}, Q_d_phi{};
  double R_T{}, R_Tp{};
};

class BipedalController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>,
                          public BipedalControllerInterface
{
public:
  BipedalController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;
  // clang-format off
  // BipedalControllerInterface implementations
  bool getOverturn() const override { return overturn_; }
  bool getStateChange() const override { return balance_state_changed_; }
  bool getCompleteStand() const override { return complete_stand_; }
  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> getCoeffs() override { return coeffs_; }
  const std::shared_ptr<ModelParams>& getModelParams() const override { return model_params_; }
  const std::shared_ptr<ControlParams>& getControlParams() const override { return control_params_; }
  const std::shared_ptr<BiasParams>& getBiasParams() const override { return bias_params_; }
  const std::shared_ptr<LegStateThresholdParams>& getLegThresholdParams() const override { return leg_threshold_params_; }
  const std::shared_ptr<ChassisGeometryParams>& getChassisGeometryParams() const override { return chassis_geometry_params_; }
  double getLegCmd() const override { return legCmd_; }
  double getJumpCmd() const override { return jumpCmd_; }
  int getBaseState() const override { return state_; }
  inline double getDefaultLegLength() const override { return default_leg_length_;}
  geometry_msgs::Vector3 getVelCmd() override { return vel_cmd_; }
  inline bool getMoveFlag() const override { return move_flag_; }
  inline void setMoveFlag(const bool& move_flag) override { move_flag_ = move_flag; }
  inline const ChassisState& getChassisState() override { return chassis_state_; };
  inline LegState& getLegState(Side side) override { return leg_state_[side]; };
  inline void setStateChange(bool state) override { balance_state_changed_ = state; }
  inline void setCompleteStand(bool state) override { complete_stand_ = state; }
  void setJumpCmd(bool cmd) override { jumpCmd_ = cmd; }
  void setMode(int mode) override { balance_mode_ = mode; }
  inline void clearRecoveryFlag() override { overturn_ = false; }
  double f_spring_force(double L0) override;
  void pubState() override;
  void pubLQRStatus(Eigen::Matrix<double, STATE_DIM, 1> left_error, Eigen::Matrix<double, STATE_DIM, 1> right_error,
                    Eigen::Matrix<double, STATE_DIM, 1> left_ref, Eigen::Matrix<double, STATE_DIM, 1> right_ref,
                    Eigen::Matrix<double, CONTROL_DIM, 1> u_left, Eigen::Matrix<double, CONTROL_DIM, 1> u_right,
                    Eigen::Matrix<double, CONTROL_DIM, 1> F_leg_, const bool unstick[2]) const override;
  void pubLegLenStatus(const bool& upstair_flag) override;
  void clearStatus() override;
  void pubDebugData(const std::string& name, double value) override{ debugPub_->add(name, value); };
  void setRecoveryLegSpdTurnback(bool recovery_leg_spd_turnback) override { recovery_leg_spd_turnback_ = recovery_leg_spd_turnback; }
  bool getRecoveryLegSpdTurnback() const override { return recovery_leg_spd_turnback_;}
  // clang-format on
private:
  void updateEstimation(const ros::Time& time, const ros::Duration& period);

  bool setupLQR(ros::NodeHandle& controller_nh);
  bool setupParams(ros::NodeHandle& controller_nh);
  bool setupModelParams(ros::NodeHandle& controller_nh);
  bool setupControlParams(ros::NodeHandle& controller_nh);
  bool setupBiasParams(ros::NodeHandle& controller_nh);
  bool setupThresholdParams(ros::NodeHandle& controller_nh);
  bool setupSpringParams(ros::NodeHandle& controller_nh);
  bool setupChassisGeometryParams(ros::NodeHandle& controller_nh);
  void polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
               Eigen::Matrix<double, 4, 12>& coeffs);
  geometry_msgs::Twist odometry() override;

  void reconfigCB(rm_chassis_controllers::LQRWeightConfig& config, uint32_t level);

  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> coeffs_;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> q_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};

  std::shared_ptr<ModelParams> model_params_;
  std::shared_ptr<ControlParams> control_params_;
  std::shared_ptr<BiasParams> bias_params_;
  std::shared_ptr<SpringParams> spring_params_;
  std::shared_ptr<ChassisGeometryParams> chassis_geometry_params_;
  std::shared_ptr<LegStateThresholdParams> leg_threshold_params_;

  int balance_mode_ = BalanceMode::SIT_DOWN;
  bool balance_state_changed_ = false;
  std::shared_ptr<ModeManager> mode_manager_;

  // Slippage_detection
  double leftWheelVel{}, rightWheelVel{}, leftWheelVelAbsolute{}, rightWheelVelAbsolute{}, slip_alpha_{ 2.0 },
      slip_R_wheel_{}, R_wheel_{};
  int itor = 0, sample_times_ = 0;
  bool slip_flag_{ false };
  Eigen::Matrix<double, 2, 2> A_, B_, H_, Q_, R_;
  Eigen::Matrix<double, 2, 1> X_, U_;
  std::shared_ptr<KalmanFilter<double>> kalmanFilterPtr_;

  ChassisState chassis_state_;
  LegState leg_state_[2];
  //  Eigen::Matrix<double, STATE_DIM, 1> x_left_{}, x_right_{};
  double default_leg_length_{ 0.12 };
  bool move_flag_{ false };
  // stand up
  bool complete_stand_ = false, overturn_ = false;
  // recovery
  bool recovery_leg_spd_turnback_{ false };

  // handles
  hardware_interface::ImuSensorHandle imu_handle_, gimbal_imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_;
  hardware_interface::JointHandle left_hip_joint_handle_, left_knee_joint_handle_, right_hip_joint_handle_,
      right_knee_joint_handle_;
  std::vector<hardware_interface::JointHandle*> joint_handles_;

  // Leg Cmd
  double legCmd_{ 0.2 };
  bool jumpCmd_{ false };

  // ROS Interface
  dynamic_reconfigure::Server<rm_chassis_controllers::LQRWeightConfig>* d_srv_;
  realtime_tools::RealtimeBuffer<LQRConfig> config_rt_buffer_;
  LQRConfig config_{};
  bool dynamic_reconfig_initialized_{ false };
  ros::Subscriber leg_cmd_sub_, recovery_leg_spd_turnback_sub_;
  ros::Publisher unstick_pub_, upstair_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisStatus>> legged_chassis_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>> legged_chassis_mode_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedLQRStatus>> lqr_status_pub_;
  ros::Time cmd_update_time_;
  std::shared_ptr<DebugDataPublisher> debugPub_;
};
}  // namespace rm_chassis_controllers
