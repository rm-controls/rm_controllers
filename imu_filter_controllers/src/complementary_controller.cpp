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
  // Get the orientation:
  double q0, q1, q2, q3;
  geometry_msgs::Vector3 imu_msg;
  filter_.getOrientation(q0, q1, q2, q3);
  tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  //
  imu_msg = complementary_handle_.getAngularVelocity();

  if (filter_.getDoBiasEstimation())
  {
    imu_msg.x -= filter_.getAngularVelocityBiasX();
    imu_msg.y -= filter_.getAngularVelocityBiasY();
    imu_msg.z -= filter_.getAngularVelocityBiasZ();
  }

  complementary_handle_.setAngularVelocity(imu_msg);

  if (publish_debug_topics_)
  {
    geometry_msgs::Vector3Stamped rpy;

    tf::Matrix3x3 M;
    M.setRotation(q);
    M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

    if (filter_.getDoBiasEstimation())
    {
      std_msgs::Bool state_msg;
      state_msg.data = filter_.getSteadyState();
      complementary_handle_.setStateMsg(state_msg);
    }
  }

  if (publish_tf_)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(q);

    if (reverse_tf_)
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform.inverse(), complementary_handle_.getStamp(),
                                                         complementary_handle_.getFrameId(), fixed_frame_));
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, complementary_handle_.getStamp(), fixed_frame_,
                                                         complementary_handle_.getFrameId()));
    }
  }
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

// maybe wrong
double ComplementaryController::CalculateDt(geometry_msgs::Vector3& a, geometry_msgs::Vector3& m,
                                            geometry_msgs::Vector3& w, ros::Time& time)
{
  double dt = (time - time_prev_).toSec();
  time_prev_ = time;
  // ros::Time t_in, t_out;
  // t_in = ros::Time::now();
  //  Update the filter.
  if (isnan(m.x) || isnan(m.y) || isnan(m.z))
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
  else
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, m.x, m.y, m.z, dt);
}

}  // namespace imu_filter_controllers
