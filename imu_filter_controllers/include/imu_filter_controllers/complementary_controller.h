//
// Created by ljq on 2021/9/4.
//

#ifndef SRC_COMPLEMENTARY_CONTROLLER_H
#define SRC_COMPLEMENTARY_CONTROLLER_H

#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <imu_complementary_filter/complementary_filter.h>

namespace imu_filter_controllers
{
class ComplementaryFilterROS
{
public:
  ComplementaryFilterROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~ComplementaryFilterROS();

private:
  // Convenience typedefs
  typedef sensor_msgs::Imu ImuMsg;
  typedef sensor_msgs::MagneticField MagMsg;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, MagMsg> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef message_filters::Subscriber<MagMsg> MagSubscriber;

  // ROS-related variables.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  boost::shared_ptr<Synchronizer> sync_;
  boost::shared_ptr<ImuSubscriber> imu_subscriber_;
  boost::shared_ptr<MagSubscriber> mag_subscriber_;

  ros::Publisher imu_publisher_;
  ros::Publisher rpy_publisher_;
  ros::Publisher state_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Parameters:

  bool use_mag_;
  bool publish_tf_;
  bool reverse_tf_;
  double constant_dt_;
  bool publish_debug_topics_;
  std::string fixed_frame_;

  // State variables:

  imu_tools::ComplementaryFilter filter_;
  ros::Time time_prev_;
  bool initialized_filter_;

  void initializeParams();
  void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);
  void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw, const MagMsg::ConstPtr& mav_msg);
  void publish(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);

  tf::Quaternion hamiltonToTFQuaternion(double q0, double q1, double q2, double q3) const;
};

}  // namespace imu_filter_controllers

#endif  // SRC_COMPLEMENTARY_CONTROLLER_H
