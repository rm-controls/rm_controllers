//
// Created by bruce on 2021/5/19.
//

#ifndef SRC_RM_SOFTWARE_RM_CONTROLLERS_RM_ORIENTATION_CONTROLLERS_INCLUDE_ORIENTATION_CONTROLLER_H_
#define SRC_RM_SOFTWARE_RM_CONTROLLERS_RM_ORIENTATION_CONTROLLERS_INCLUDE_ORIENTATION_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <sensor_msgs/Imu.h>

namespace rm_orientation_controller {
class Controller :
    public controller_interface::MultiInterfaceController<hardware_interface::ImuSensorInterface,
                                                          hardware_interface::RobotStateInterface> {
 public:
  Controller() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;

 private:
  void fixtf();

  hardware_interface::ImuSensorHandle imu_sensor_;
  hardware_interface::RobotStateHandle robot_state_;

  rm_common::TfRtBroadcaster tf_rt_broadcaster_{};
  geometry_msgs::TransformStamped source2target_msg_;
  tf2_ros::TransformBroadcaster *br_;

  ros::Time last_run_, last_br_;

  sensor_msgs::Imu data_;
  tf2_ros::Buffer *tf_;

  std::string frame_fixed_;
  std::string frame_source_;
  std::string frame_target_;
};
};

#endif //SRC_RM_SOFTWARE_RM_CONTROLLERS_RM_ORIENTATION_CONTROLLERS_INCLUDE_ORIENTATION_CONTROLLER_H_
