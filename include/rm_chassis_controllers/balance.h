//
// Created by chenzheng on 2021/2/23.
//

#ifndef RM_CHASSIS_CONTROLLER_STANDARD_H
#define RM_CHASSIS_CONTROLLER_STANDARD_H

#include "rm_chassis_controllers/chassis_base.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <rm_common/lqr.h>

namespace rm_chassis_controllers {

class BalanceController : public rm_chassis_base::ChassisBase {
 public:
  BalanceController() = default;

  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  using rm_chassis_base::ChassisBase::passive;
  using rm_chassis_base::ChassisBase::recovery;
  using rm_chassis_base::ChassisBase::cmdChassisCallback;
  using rm_chassis_base::ChassisBase::cmdVelCallback;
  using rm_chassis_base::ChassisBase::tfVelFromYawToBase;

 private:
  void getK(XmlRpc::XmlRpcValue a,
            XmlRpc::XmlRpcValue b,
            XmlRpc::XmlRpcValue q,
            XmlRpc::XmlRpcValue r);

  void raw(const ros::Duration &period) override;

  void follow(const ros::Time &time, const ros::Duration &period) override;

  void moveJoint(const ros::Duration &period) override;

  void updateOdom(const ros::Time &time, const ros::Duration &period) override;

  geometry_msgs::Twist iKine(const ros::Duration &period) override;

  void dataImuCallback(const sensor_msgs::ImuConstPtr &data);

  geometry_msgs::Twist vel_data_;
  hardware_interface::JointHandle joint_left_, joint_right_;

  ros::Subscriber data_imu_sub_;
  ros::Publisher state_real_pub_;
  realtime_tools::RealtimeBuffer<sensor_msgs::Imu> imu_rt_buffer_;
  sensor_msgs::Imu imu_data_;

  // class member about full state feedback controller
  static const size_t STATE_DIM = 4;
  static const size_t CONTROL_DIM = 2;

  Eigen::Matrix<double, STATE_DIM, 1> x_{}, x_ref_{};
  Eigen::Matrix<double, CONTROL_DIM, 1> u_{};
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  double x_pos_{}, com_pitch_offset_{};
};

}

#endif // RM_CHASSIS_CONTROLLER_STANDARD_H