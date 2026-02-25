//
// Created by guanlin on 25-8-28.
//

#include "bipedal_wheel_controller/controller.h"

#include <angles/angles.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pluginlib/class_list_macros.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include "bipedal_wheel_controller/vmc/leg_params.h"
#include "bipedal_wheel_controller/vmc/leg_conv.h"
#include "bipedal_wheel_controller/vmc/leg_spd.h"
#include "bipedal_wheel_controller/vmc/leg_pos.h"

namespace rm_chassis_controllers
{
bool BipedalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  const std::pair<const char*, hardware_interface::JointHandle*> table[] = {
    { "left_hip_joint", &left_hip_joint_handle_ },     { "left_knee_joint", &left_knee_joint_handle_ },
    { "right_hip_joint", &right_hip_joint_handle_ },   { "right_knee_joint", &right_knee_joint_handle_ },
    { "left_wheel_joint", &left_wheel_joint_handle_ }, { "right_wheel_joint", &right_wheel_joint_handle_ }
  };
  auto* joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  for (const auto& t : table)
  {
    *t.second = joint_interface->getHandle(t.first);
    joint_handles_.push_back(t.second);
  }

  mode_manager_ = std::make_unique<ModeManager>(controller_nh, joint_handles_);
  model_params_ = std::make_shared<ModelParams>();
  control_params_ = std::make_shared<ControlParams>();
  bias_params_ = std::make_shared<BiasParams>();
  leg_threshold_params_ = std::make_shared<LegStateThresholdParams>();

  if (!setupModelParams(controller_nh) || !setupLQR(controller_nh) || !setupBiasParams(controller_nh) ||
      !setupControlParams(controller_nh) || !setupThresholdParams(controller_nh))
    return false;

  auto legCmdCallback = [this](const rm_msgs::LegCmd::ConstPtr& msg) {
    legCmd_ = msg->leg_length;
    jumpCmd_ = msg->jump;
  };
  leg_cmd_sub_ = controller_nh.subscribe<rm_msgs::LegCmd>("/leg_cmd", 1, legCmdCallback);

  unstick_pub_ = controller_nh.advertise<std_msgs::Bool>("unstick", 1);
  upstair_status_pub_ = controller_nh.advertise<rm_msgs::LeggedUpstairStatus>("upstair_status", 1);
  legged_chassis_status_pub_ = controller_nh.advertise<rm_msgs::LeggedChassisStatus>("legged_chassis_status", 1);
  legged_chassis_mode_pub_ = controller_nh.advertise<rm_msgs::LeggedChassisMode>("legged_chassis_mode", 1);
  lqr_status_pub_ = controller_nh.advertise<rm_msgs::LeggedLQRStatus>("lqr_status", 1);
  x_left_.setZero();
  x_right_.setZero();

  // Slippage detection
  A_ << 1, 0.0, 0, 1;
  H_ << 1, 0, 0, 1;
  Q_ << 1, 0, 0, 1;
  R_ << 200, 0, 0, 200;
  //  Q_ << 25, 0, 0, 2000;
  //  R_ << 800, 0, 0, 0.01;
  R_wheel_ = R_(0, 0);
  slip_R_wheel_ = slip_alpha_ * R_wheel_;
  B_.setZero();
  X_.setZero();
  U_.setZero();
  kalmanFilterPtr_ = std::make_shared<KalmanFilter<double>>(A_, B_, H_, Q_, R_);
  kalmanFilterPtr_->clear(X_);

  //  left_leg_angle_lpFilterPtr_ = std::make_shared<LowPassFilter>(100);
  //  right_leg_angle_lpFilterPtr_ = std::make_shared<LowPassFilter>(100);

  left_leg_angle_vel_lpFilterPtr_ = std::make_shared<LowPassFilter>(60);
  right_leg_angle_vel_lpFilterPtr_ = std::make_shared<LowPassFilter>(60);
  return true;
}

void BipedalController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  if (!balance_state_changed_)
    mode_manager_->switchMode(balance_mode_);
  if (getBaseState() == 4)
  {
    balance_mode_ = SIT_DOWN;
    mode_manager_->switchMode(SIT_DOWN);
  }
  updateEstimation(time, period);
  mode_manager_->getModeImpl()->execute(this, time, period);
  pubState();
}

void BipedalController::clearStatus()
{
  x_left_(2) = x_right_(2) = -bias_params_->x;
}

void BipedalController::updateEstimation(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, acc;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  acc.x = imu_handle_.getLinearAcceleration()[0];
  acc.y = imu_handle_.getLinearAcceleration()[1];
  acc.z = imu_handle_.getLinearAcceleration()[2];
  tf2::Transform odom2imu, imu2base, odom2base;
  geometry_msgs::Vector3 angular_vel_base{}, linear_acc_base{};
  double roll{}, pitch{}, yaw{};
  try
  {
    tf2::doTransform(gyro, angular_vel_base,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
    tf2::Quaternion odom2imu_quaternion;
    tf2::Vector3 odom2imu_origin;
    odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                                 imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
    odom2imu_origin.setValue(0, 0, 0);
    odom2imu.setOrigin(odom2imu_origin);
    odom2imu.setRotation(odom2imu_quaternion);
    odom2base = odom2imu * imu2base;
    quatToRPY(toMsg(odom2base).rotation, roll, pitch, yaw);

    tf_msg.transform = tf2::toMsg(odom2imu.inverse());
    tf_msg.header.stamp = time;
    tf2::doTransform(acc, linear_acc_base, tf_msg);

    tf2::Vector3 z_body(0, 0, 1);
    tf2::Vector3 z_world = tf2::quatRotate(odom2base.getRotation(), z_body);
    overturn_ = (abs(pitch) > 0.65 || abs(roll) > 0.4) && z_world.z() < 0.0;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
    return;
  }

  // vmc
  double left_angle[2]{}, right_angle[2]{}, left_pos[2]{}, left_spd[2]{}, right_pos[2]{}, right_spd[2]{};
  // [0]:hip_vmc_joint [1]:knee_vmc_joint
  left_angle[0] = left_hip_joint_handle_.getPosition() + M_PI;
  left_angle[1] = left_knee_joint_handle_.getPosition();
  right_angle[0] = right_hip_joint_handle_.getPosition() + M_PI;
  right_angle[1] = right_knee_joint_handle_.getPosition();

  // gazebo
  //  left_angle[0] = left_hip_joint_handle_.getPosition() + M_PI_2;
  //  left_angle[1] = left_knee_joint_handle_.getPosition() - M_PI_2;
  //  right_angle[0] = right_hip_joint_handle_.getPosition() + M_PI_2;
  //  right_angle[1] = right_knee_joint_handle_.getPosition() - M_PI_2;

  // [0] is length, [1] is angle
  leg_pos(left_angle[0], left_angle[1], left_pos);
  leg_pos(right_angle[0], right_angle[1], right_pos);
  leg_spd(left_hip_joint_handle_.getVelocity(), left_knee_joint_handle_.getVelocity(), left_angle[0], left_angle[1],
          left_spd);
  leg_spd(right_hip_joint_handle_.getVelocity(), right_knee_joint_handle_.getVelocity(), right_angle[0], right_angle[1],
          right_spd);
  left_leg_angle_vel_lpFilterPtr_->input(left_spd[1]);
  right_leg_angle_vel_lpFilterPtr_->input(right_spd[1]);
  //  left_leg_angle_lpFilterPtr_->input(left_pos[1]);
  //  right_leg_angle_lpFilterPtr_->input(right_pos[1]);

  //  left_pos[1] = left_leg_angle_lpFilterPtr_->output();
  //  right_pos[1] = right_leg_angle_lpFilterPtr_->output();
  left_spd[1] = left_leg_angle_vel_lpFilterPtr_->output();
  right_spd[1] = right_leg_angle_vel_lpFilterPtr_->output();

  // Slippage_detection
  leftWheelVel = (left_wheel_joint_handle_.getVelocity() + angular_vel_base.y + left_spd[1]) * wheel_radius_;
  rightWheelVel = (right_wheel_joint_handle_.getVelocity() + angular_vel_base.y + right_spd[1]) * wheel_radius_;
  leftWheelVelAbsolute =
      leftWheelVel + left_pos[0] * left_spd[1] * cos(left_pos[1] + pitch_) + left_spd[0] * sin(left_pos[1] + pitch_);
  rightWheelVelAbsolute = rightWheelVel + right_pos[0] * right_spd[1] * cos(right_pos[1] + pitch_) +
                          right_spd[0] * sin(right_pos[1] + pitch_);

  double wheel_vel_aver = (leftWheelVelAbsolute + rightWheelVelAbsolute) / 2.;
  R_(0, 0) = slip_flag_ ? slip_R_wheel_ : R_wheel_;
  if (i >= sample_times_)
  {  // oversampling
    i = 0;
    X_(0) = wheel_vel_aver;
    X_(1) = linear_acc_base.x;
    kalmanFilterPtr_->predict(U_);
    kalmanFilterPtr_->update(X_, R_);
  }
  else
  {
    kalmanFilterPtr_->predict(U_);
    i++;
  }
  auto x_hat_vel = kalmanFilterPtr_->getState();
  slip_flag_ = abs(x_hat_vel(0) - wheel_vel_aver) > 3.0;

  // update state
  x_left_[3] = state_ != RAW ? x_hat_vel(0) : 0;
  if (abs(x_left_[3]) <= 0.6f && abs(vel_cmd_.x) <= 0.1f)
  {
    x_left_[2] += state_ != RAW ? x_left_[3] * period.toSec() : 0;
  }
  else
  {
    x_left_[2] = -bias_params_->x;
  }
  x_left_[0] = (left_pos[1] + pitch);
  x_left_[1] = left_spd[1] + angular_vel_base.y;
  x_left_[4] = -pitch;
  x_left_[5] = -angular_vel_base.y;
  x_right_ = x_left_;
  x_right_[0] = (right_pos[1] + pitch);
  x_right_[1] = right_spd[1] + angular_vel_base.y;

  // ros msg
  rm_msgs::LeggedChassisStatus legged_chassis_status_msg;
  legged_chassis_status_msg.roll = wheel_vel_aver;
  legged_chassis_status_msg.pitch = x_left_[4];
  legged_chassis_status_msg.d_pitch = x_left_[5];
  legged_chassis_status_msg.yaw = yaw;
  legged_chassis_status_msg.d_yaw = angular_vel_base.z;
  legged_chassis_status_msg.left_leg_length = left_pos[0];
  legged_chassis_status_msg.right_leg_length = right_pos[0];
  legged_chassis_status_msg.x = x_left_[2];
  legged_chassis_status_msg.x_dot = x_left_[3];
  legged_chassis_status_msg.left_leg_theta = x_left_[0];
  legged_chassis_status_msg.left_leg_theta_dot = x_left_[1];
  legged_chassis_status_msg.right_leg_theta = x_right_[0];
  legged_chassis_status_msg.right_leg_theta_dot = x_right_[1];
  legged_chassis_status_msg.linear_acc_base.push_back(linear_acc_base.x);
  legged_chassis_status_msg.linear_acc_base.push_back(linear_acc_base.y);
  legged_chassis_status_msg.linear_acc_base.push_back(linear_acc_base.z);
  legged_chassis_status_pub_.publish(legged_chassis_status_msg);

  rm_msgs::LeggedChassisMode legged_chassis_mode_msg;
  legged_chassis_mode_msg.mode = balance_mode_;
  legged_chassis_mode_pub_.publish(legged_chassis_mode_msg);

  mode_manager_->getModeImpl()->updateEstimation(x_left_, x_right_);
  mode_manager_->getModeImpl()->updateLegKinematics(left_angle, right_angle, left_pos, left_spd, right_pos, right_spd);
  mode_manager_->getModeImpl()->updateBaseState(angular_vel_base, linear_acc_base, roll, pitch, yaw);
}

void BipedalController::pubState()
{
  std_msgs::Bool msg;
  msg.data = mode_manager_->getModeImpl()->getUnstick();
  unstick_pub_.publish(msg);
}

void BipedalController::stopping(const ros::Time& time)
{
  balance_mode_ = BalanceMode::SIT_DOWN;
  balance_state_changed_ = false;
  setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });

  ROS_INFO("[balance] Controller Stop");
}

bool BipedalController::setupModelParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = { { "m_w", &model_params_->m_w },
                                                  { "m_p", &model_params_->m_p },
                                                  { "M", &model_params_->M },
                                                  { "i_w", &model_params_->i_w },
                                                  { "i_m", &model_params_->i_m },
                                                  { "i_p", &model_params_->i_p },
                                                  { "l", &model_params_->l },
                                                  { "L_weight", &model_params_->L_weight },
                                                  { "Lm_weight", &model_params_->Lm_weight },
                                                  { "g", &model_params_->g },
                                                  { "wheel_radius", &model_params_->r },
                                                  { "spring_force", &model_params_->f_spring },
                                                  { "gravity_force", &model_params_->f_gravity } };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }

  if (!controller_nh.getParam("default_leg_length", default_leg_length_))
  {
    ROS_ERROR("Param %s not given (namespace: %s)", "default_leg_length", controller_nh.getNamespace().c_str());
    return false;
  }

  return true;
}

bool BipedalController::setupLQR(ros::NodeHandle& controller_nh)
{
  // Set up weight matrices
  auto loadWeightMatrix = [](ros::NodeHandle& nh, const char* key, int dim) -> Eigen::VectorXd {
    std::vector<double> v;
    if (!nh.getParam(key, v) || static_cast<int>(v.size()) != dim)
      return Eigen::VectorXd::Constant(dim, std::numeric_limits<double>::quiet_NaN());
    return Eigen::VectorXd::Map(v.data(), dim);
  };
  Eigen::VectorXd q_diag = loadWeightMatrix(controller_nh, "q", STATE_DIM);
  Eigen::VectorXd r_diag = loadWeightMatrix(controller_nh, "r", CONTROL_DIM);
  if (!q_diag.allFinite() || !r_diag.allFinite())
    return false;
  q_.setZero();
  r_.setZero();
  q_.diagonal() = q_diag;
  r_.diagonal() = r_diag;

  // Continuous model \dot{x} = A x + B u
  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>> ks;
  for (int i = 10; i < 40; i++)
  {
    double length = i / 100.;
    lengths.push_back(length);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b{};
    generateAB(model_params_, a, b, length);
    Lqr<double> lqr(a, b, q_, r_);
    if (!lqr.computeK())
    {
      ROS_ERROR("Failed to compute K of LQR.");
      return false;
    }
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k = lqr.getK();
    ks.push_back(k);
  }
  polyfit(ks, lengths, coeffs_);
  return true;
}

bool BipedalController::setupBiasParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = {
    { "x_bias", &bias_params_->x },
    { "theta_bias", &bias_params_->theta },
    { "pitch_bias", &bias_params_->pitch },
    { "roll_bias", &bias_params_->roll },
  };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

// [will unused]
bool BipedalController::setupControlParams(ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("jumpOverTime", control_params_->jumpOverTime_) ||
      !controller_nh.getParam("p1", control_params_->p1_) || !controller_nh.getParam("p2", control_params_->p2_) ||
      !controller_nh.getParam("p3", control_params_->p3_) || !controller_nh.getParam("p4", control_params_->p4_))
  {
    ROS_ERROR("Load param fail, check the resist of jump_over_time, p1, p2, p3, p4");
    return false;
  }
  return true;
}

bool BipedalController::setupThresholdParams(ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("under_lower_threshold", leg_threshold_params_->under_lower) ||
      !controller_nh.getParam("under_upper_threshold", leg_threshold_params_->under_upper) ||
      !controller_nh.getParam("front_lower_threshold", leg_threshold_params_->front_lower) ||
      !controller_nh.getParam("front_upper_threshold", leg_threshold_params_->front_upper) ||
      !controller_nh.getParam("behind_lower_threshold", leg_threshold_params_->behind_lower) ||
      !controller_nh.getParam("behind_upper_threshold", leg_threshold_params_->behind_upper) ||
      !controller_nh.getParam("upstair_exit_threshold", leg_threshold_params_->upstair_exit_threshold) ||
      !controller_nh.getParam("upstair_des_theta", leg_threshold_params_->upstair_des_theta))
  {
    ROS_ERROR("Load threshold param fail, check the resist of  "
              "under_threshold, front_threshold, behind_threshold");
    return false;
  }
  return true;
}

void BipedalController::polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
                                Eigen::Matrix<double, 4, 12>& coeffs)
{
  int N = L0s.size();
  Eigen::MatrixXd A(N, 4), B(N, 12);
  for (int i = 0; i < N; ++i)
  {
    A.block(i, 0, 1, 4) << pow(L0s[i], 3), pow(L0s[i], 2), L0s[i], 1.0;
    Eigen::Map<const Eigen::Matrix<double, 12, 1>> flat(Ks[i].data());
    B.row(i) = flat.transpose();
  }
  coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * B);
}

geometry_msgs::Twist BipedalController::odometry()
{
  geometry_msgs::Twist twist;
  if (mode_manager_->getModeImpl() != nullptr)
  {
    twist.linear.x = mode_manager_->getModeImpl()->getRealxVel();
    twist.angular.z = mode_manager_->getModeImpl()->getRealYawVel();
  }
  else
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
  }
  return twist;
}

void BipedalController::pubLQRStatus(Eigen::Matrix<double, STATE_DIM, 1> left_error,
                                     Eigen::Matrix<double, STATE_DIM, 1> right_error,
                                     Eigen::Matrix<double, STATE_DIM, 1> left_ref,
                                     Eigen::Matrix<double, STATE_DIM, 1> right_ref,
                                     Eigen::Matrix<double, CONTROL_DIM, 1> u_left,
                                     Eigen::Matrix<double, CONTROL_DIM, 1> u_right,
                                     Eigen::Matrix<double, CONTROL_DIM, 1> F_leg_, const bool unstick[2]) const
{
  rm_msgs::LeggedLQRStatus msg;
  for (int i = 0; i < 6; ++i)
  {
    msg.left_leg_error.push_back(left_error(i));
    msg.right_leg_error.push_back(right_error(i));
    msg.left_leg_ref.push_back(left_ref(i));
    msg.right_leg_ref.push_back(right_ref(i));
  }
  for (int i = 0; i < 2; ++i)
  {
    msg.left_leg_u.push_back(u_left(i));
    msg.right_leg_u.push_back(u_right(i));
    msg.F_leg.push_back(F_leg_[i]);
    msg.unstick.push_back(unstick[i]);
  }
  lqr_status_pub_.publish(msg);
}

void BipedalController::pubLegLenStatus(const bool& upstair_flag)
{
  rm_msgs::LeggedUpstairStatus msg;
  msg.upstair_flag = upstair_flag;
  upstair_status_pub_.publish(msg);
}

// void BipedalController::follow(const ros::Time& time, const ros::Duration& period)
//{
//   static bool follow_source_frame_changed_{ false };
//   static std::string last_follow_source_frame_{ follow_source_frame_ };
//   if (state_changed_)
//   {
//     state_changed_ = false;
//     ROS_INFO("[Chassis] Enter FOLLOW");
//
//     ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
//                 hardware_interface::EffortJointInterface>::recovery();
//     pid_follow_.reset();
//   }
//
//   tfVelToBase(command_source_frame_);
//   try
//   {
//     double roll{}, pitch{}, yaw{};
//     //    double  target_yaw_bias{ 0 };
//     quatToRPY(robot_state_handle_.lookupTransform("base_link", follow_source_frame_, ros::Time(0)).transform.rotation,
//               roll, pitch, yaw);
//     double yawForwardError = angles::shortest_angular_distance(0, yaw);
//     double yawInverseError = angles::shortest_angular_distance(M_PI, yaw);
//     double yawError = abs(yawForwardError) < abs(yawInverseError) ? yawForwardError : yawInverseError;
//     if (follow_source_frame_ != last_follow_source_frame_)
//     {
//       follow_source_frame_changed_ = true;
//     }
//     if (follow_source_frame_changed_)
//     {
//       yawError = yawForwardError;
//       if (abs(yawError) < 0.1)
//       {
//         follow_source_frame_changed_ = false;
//       }
//     }
//     pid_follow_.computeCommand(yawError, period);
//     vel_cmd_.z = pid_follow_.getCurrentCmd() + cmd_rt_buffer_.readFromRT()->cmd_chassis_.follow_vel_des;
//   }
//   catch (tf2::TransformException& ex)
//   {
//     ROS_WARN("%s", ex.what());
//   }
//   last_follow_source_frame_ = follow_source_frame_;
// }

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BipedalController, controller_interface::ControllerBase)
