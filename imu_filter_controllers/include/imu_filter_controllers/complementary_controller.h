
#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H

#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "imu_complementary_filter/complementary_filter.h"
#include <rm_common/hardware_interface/complementary_interface.h>

namespace imu_filter_controllers
{
class ComplementaryController
{
public:
  ComplementaryController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& /*period*/);

private:
  // ROS-related variables.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformBroadcaster tf_broadcaster_;

  // hardware_interface
  hardware_interface::ComplementaryHandle complementary_handle_{};

  // Parameters:
  bool use_mag_;
  bool publish_tf_;
  bool reverse_tf_;
  double constant_dt_;
  bool publish_debug_topics_;
  std::string fixed_frame_;

  // msg
  geometry_msgs::Vector3 imu_angular_velocity_;
  geometry_msgs::Vector3 imu_linear_acceleration_;
  ros::Time imu_time_;
  std::string imu_frame_id_;

  // State variables:
  imu_tools::ComplementaryFilter filter_;
  ros::Time time_prev_;
  bool initialized_filter_;

  /** @brief init params
   *
   */
  void initializeParams();
  void UpdateFilter(const geometry_msgs::Vector3 imu_angular_velocity_,
                    const geometry_msgs::Vector3 imu_linear_acceleration_, const ros::Time time);

  tf::Quaternion hamiltonToTFQuaternion(double q0, double q1, double q2, double q3) const;
};

}  // namespace imu_filter_controllers

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
