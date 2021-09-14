//
// Created by ljq on 2021/9/4.
//

#include "imu_filter_controllers/complementary_controller.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace imu_filter_controllers
{
bool ComplementaryController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  // base init
  nh_ = controller_nh;
  nh_private_ = root_nh;
  initialized_filter_ = false;

  ROS_INFO("Starting ComplementaryFilterROS");
  initializeParams();
}

void ComplementaryController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // get new data from interface
  imu_angular_velocity_ = complementary_handle_.getAngularVelocity();
  imu_linear_acceleration_ = complementary_handle_.getLinearAcceleration();
  imu_time_ = complementary_handle_.getStamp();
  imu_frame_id_ = complementary_handle_.getFrameId();

  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  // update filter
  UpdateFilter(imu_angular_velocity_, imu_linear_acceleration_, imu_time_);

  // calcute error and send data

  // Account for biases.
  if (filter_.getDoBiasEstimation())
  {
    imu_angular_velocity_.x -= filter_.getAngularVelocityBiasX();
    imu_angular_velocity_.y -= filter_.getAngularVelocityBiasY();
    imu_angular_velocity_.z -= filter_.getAngularVelocityBiasZ();
  }

  complementary_handle_.setAngularVelocity(imu_angular_velocity_);

  /*
  if (publish_debug_topics_)
  {
    // Create and publish roll, pitch, yaw angles
    geometry_msgs::Vector3Stamped rpy;
    rpy.header = imu_msg_raw->header;

    tf::Matrix3x3 M;
    M.setRotation(q);
    M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
    rpy_publisher_.publish(rpy);

    // Publish whether we are in the steady state, when doing bias estimation
    if (filter_.getDoBiasEstimation())
    {
      std_msgs::Bool state_msg;
      state_msg.data = filter_.getSteadyState();
      state_publisher_.publish(state_msg);
    }
  }
   */

  if (publish_tf_)
  {
    // Create and publish the ROS tf.
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(q);

    if (reverse_tf_)
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform.inverse(), imu_time_, imu_frame_id_, fixed_frame_));
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, imu_time_, fixed_frame_, imu_frame_id_));
    }
  }
}

void ComplementaryController::UpdateFilter(const geometry_msgs::Vector3 imu_angular_velocity_,
                                           const geometry_msgs::Vector3 imu_linear_acceleration_, const ros::Time time)
{
  const geometry_msgs::Vector3& a = imu_angular_velocity_;
  const geometry_msgs::Vector3& w = imu_linear_acceleration_;

  // Initialize.
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - time_prev_).toSec();

  time_prev_ = time;

  // Update the filter.
  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
}

/*
void ComplementaryController::UpdateMagFilter(const geometry_msgs::Vector3 imu_angular_velocity_,
                                              const geometry_msgs::Vector3 imu_linear_acceleration_,
                                              const geometry_msgs::Vector3 m)
{
  const geometry_msgs::Vector3& a = imu_angular_velocity_;
  const geometry_msgs::Vector3& w = imu_linear_acceleration_;

  // Initialize.
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - time_prev_).toSec();

  time_prev_ = time;

  // Update the filter.
  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
}
 */

// send date

tf::Quaternion ComplementaryController::hamiltonToTFQuaternion(double q0, double q1, double q2, double q3) const
{
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf::Quaternion(q1, q2, q3, q0);
}

void ComplementaryController::initializeParams()
{
  double gain_acc;
  double gain_mag;
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;

  if (!nh_private_.getParam("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam("use_mag", use_mag_))
    use_mag_ = false;
  if (!nh_private_.getParam("publish_tf", publish_tf_))
    publish_tf_ = false;
  if (!nh_private_.getParam("reverse_tf", reverse_tf_))
    reverse_tf_ = false;
  if (!nh_private_.getParam("constant_dt", constant_dt_))
    constant_dt_ = 0.0;
  if (!nh_private_.getParam("publish_debug_topics", publish_debug_topics_))
    publish_debug_topics_ = false;
  if (!nh_private_.getParam("gain_acc", gain_acc))
    gain_acc = 0.01;
  if (!nh_private_.getParam("gain_mag", gain_mag))
    gain_mag = 0.01;
  if (!nh_private_.getParam("do_bias_estimation", do_bias_estimation))
    do_bias_estimation = true;
  if (!nh_private_.getParam("bias_alpha", bias_alpha))
    bias_alpha = 0.01;
  if (!nh_private_.getParam("do_adaptive_gain", do_adaptive_gain))
    do_adaptive_gain = true;

  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setDoAdaptiveGain(do_adaptive_gain);

  if (!filter_.setGainAcc(gain_acc))
    ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_)
  {
    if (!filter_.setGainMag(gain_mag))
      ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation)
  {
    if (!filter_.setBiasAlpha(bias_alpha))
      ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
  }

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
    // otherwise, it will be constant
    ROS_WARN("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }
}

}  // namespace imu_filter_controllers
