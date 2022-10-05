//
// Created by bruce on 2021/5/19.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <rm_common/hardware_interface/rm_imu_sensor_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <sensor_msgs/Imu.h>

namespace rm_orientation_controller
{
class Controller : public controller_interface::MultiInterfaceController<rm_control::RmImuSensorInterface,
                                                                         rm_control::RobotStateInterface>
{
public:
  Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  bool getTransform(const ros::Time& time, geometry_msgs::TransformStamped& source2target, const double x,
                    const double y, const double z, const double w);
  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);

  rm_control::RmImuSensorHandle imu_sensor_;
  rm_control::RobotStateHandle robot_state_;

  ros::Time last_imu_update_time_;

  rm_common::TfRtBroadcaster tf_broadcaster_{};
  geometry_msgs::TransformStamped source2target_msg_;

  std::string frame_source_;
  std::string frame_target_;

  ros::Subscriber imu_data_sub_;
  bool receive_imu_msg_ = false;
};
}  // namespace rm_orientation_controller
