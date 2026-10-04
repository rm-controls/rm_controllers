//
// Created by qiayuan on 2022/7/29.
//

#include <algorithm>
#include <cmath>
#include <string>
#include <Eigen/QR>

#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

#include "rm_chassis_controllers/omni.h"

namespace rm_chassis_controllers
{
bool OmniController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  // Set init value for RLS and power limiters.
  try
  {
    controller_nh.getParam("power/vel_coeff", wheel_power_limitor_.vel_coeff);
    controller_nh.getParam("power/effort_coeff", wheel_power_limitor_.effort_coeff);
    controller_nh.getParam("power/power_offset", wheel_power_limitor_.power_offset);
    controller_nh.param<bool>("power/use_rls", use_rls_, false);
    controller_nh.param<bool>("power/use_K_angle", use_K_angle_, false);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to get power limiter parameters: %s", e.what());
    return false;
  }

  wheel_power_limitor_.err_upper = 500;
  wheel_power_limitor_.err_lower = 0.01;

  rls_ = std::make_unique<Rls<double>>(2, 1, 0.99999, 1e-5);
  Eigen::Matrix<double, 2, 1> w;
  w << wheel_power_limitor_.effort_coeff, wheel_power_limitor_.vel_coeff;
  rls_->setW(w);

  for (auto& filter : motor_lp_filters_)
  {
    filter = new LowPassFilter(20);
  }

  auto epower_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64>>(controller_nh, "power/estimated", 100);
  this->epower_pub_ = std::move(epower_publisher);
  auto cpower_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64>>(controller_nh, "power/commanded", 100);
  this->cpower_pub_ = std::move(cpower_publisher);

  XmlRpc::XmlRpcValue wheels;
  controller_nh.getParam("wheels", wheels);
  chassis2joints_.resize(wheels.size(), 3);

  size_t i = 0;
  for (const auto& wheel : wheels)
  {
    ROS_ASSERT(wheel.second.hasMember("pose"));
    ROS_ASSERT(wheel.second["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(wheel.second["pose"].size() == 3);
    ROS_ASSERT(wheel.second.hasMember("roller_angle"));
    ROS_ASSERT(wheel.second.hasMember("radius"));

    // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
    Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
    double beta = (double)wheel.second["pose"][2];
    double roller_angle = (double)wheel.second["roller_angle"];
    direction << 1, tan(roller_angle);
    in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
    in_chassis << -(double)wheel.second["pose"][1], 1., 0., (double)wheel.second["pose"][0], 0., 1.;
    Eigen::MatrixXd chassis2joint = 1. / (double)wheel.second["radius"] * direction * in_wheel * in_chassis;
    chassis2joints_.block<1, 3>(i, 0) = chassis2joint;

    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "wheels/" + wheel.first);
    joints_.push_back(std::make_shared<effort_controllers::JointVelocityController>());
    if (!joints_.back()->init(effort_joint_interface_, nh_wheel))
      return false;
    wheel_joint_handles_.push_back(joints_[i]->joint_);

    i++;
  }
  return true;
}

void OmniController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  Eigen::Vector3d vel_chassis;
  vel_chassis << vel_cmd_.z, vel_cmd_.x, vel_cmd_.y;
  Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;
  for (size_t i = 0; i < joints_.size(); i++)
  {
    joints_[i]->setCommand(vel_joints(i));
    joints_[i]->update(time, period);
  }
}

geometry_msgs::Twist OmniController::odometry()
{
  Eigen::VectorXd vel_joints(joints_.size());
  for (size_t i = 0; i < joints_.size(); i++)
    vel_joints[i] = joints_[i]->joint_.getVelocity();
  Eigen::Vector3d vel_chassis =
      (chassis2joints_.transpose() * chassis2joints_).inverse() * chassis2joints_.transpose() * vel_joints;
  geometry_msgs::Twist twist;
  twist.angular.z = vel_chassis(0);
  twist.linear.x = vel_chassis(1);
  twist.linear.y = vel_chassis(2);
  return twist;
}

void OmniController::stateJudge()
{
  if (!use_K_angle_)
  {
    for (size_t i = 0; i < joints_.size() && i < 4; ++i)
    {
      wheel_power_limitor_.K_angle[i] = 1.0;
    }
    return;
  }
  double sin_pitch{};
  if (abs(pitch_) > 0.12)
  {
    sin_pitch = sin(pitch_);
  }
  else
  {
    sin_pitch = 0.0;
  }

  for (size_t i = 0; i < joints_.size() && i < 4; ++i)
  {
    auto& ctl = joints_[i];

    if (ctl->joint_.getName().find("front") != std::string::npos)
    {
      if (ctl->getJointName().find("left") != std::string::npos)
      {
        wheel_power_limitor_.K_angle[i] = 1 + sin_pitch;
      }
      if (ctl->getJointName().find("right") != std::string::npos)
      {
        wheel_power_limitor_.K_angle[i] = 1 + sin_pitch;
      }
    }
    if (ctl->joint_.getName().find("back") != std::string::npos)
    {
      if (ctl->getJointName().find("left") != std::string::npos)
      {
        wheel_power_limitor_.K_angle[i] = 1 - sin_pitch;
      }
      if (ctl->getJointName().find("right") != std::string::npos)
      {
        wheel_power_limitor_.K_angle[i] = 1 - sin_pitch;
      }
    }
  }
}

void OmniController::powerLimit()
{
  updatePowerStatus();
  stateJudge();
  // multiply K to limit power for wheel joints.
  if (wheel_power_limitor_.err_sum > wheel_power_limitor_.err_upper)
  {
    wheel_power_limitor_.K = 1;
  }
  else if (wheel_power_limitor_.err_sum < wheel_power_limitor_.err_lower)
  {
    wheel_power_limitor_.K = 0;
  }
  else
  {
    wheel_power_limitor_.K = 1 - (wheel_power_limitor_.err_sum - wheel_power_limitor_.err_lower) /
                                     (wheel_power_limitor_.err_upper - wheel_power_limitor_.err_lower);
  }
  // Set power limit to each joint according to K.
  for (int i = 0; i < 4; i++)
  {
    double wheel_zoom =
        (wheel_power_limitor_.K * abs(wheel_power_limitor_.err[i]) / wheel_power_limitor_.err_sum) +
        (1 - wheel_power_limitor_.K) * (abs(wheel_power_limitor_.power_in[i]) / wheel_power_limitor_.cmd_power);
    wheel_zoom = limit(wheel_zoom, 0.0, 1.0);
    wheel_power_limitor_.power_limit[i] = wheel_zoom * wheel_power_limitor_.max_power;
  }
  if (wheel_power_limitor_.cmd_power > wheel_power_limitor_.max_power)
  {
    for (size_t i = 0; i < joints_.size() && i < 4; ++i)
    {
      auto& ctl = joints_[i];
      auto& joint = ctl->joint_;
      double A = wheel_power_limitor_.effort_coeff;
      double B = wheel_power_limitor_.omiga[i];
      double C = abs(wheel_power_limitor_.omiga[i]) * wheel_power_limitor_.vel_coeff +
                 wheel_power_limitor_.power_offset / 4 - wheel_power_limitor_.power_limit[i];
      double Delta = square(B) - 4 * A * C;
      if (!std::isfinite(Delta) || Delta < 0.0)
        Delta = 0.0;
      if (Delta >= 0)
      {
        double Sqrt = sqrtf(Delta);
        if (wheel_power_limitor_.torque[i] >= 0)
          joint.setCommand(((-B + Sqrt) / (2 * A)) * wheel_power_limitor_.K_angle[i]);
        else
          joint.setCommand(((-B - Sqrt) / (2 * A)) * wheel_power_limitor_.K_angle[i]);
      }
      else
      {
        joint.setCommand(((-B) / (2 * A)) * wheel_power_limitor_.K_angle[i]);
      }
    }
  }
  else
  {
    for (size_t i = 0; i < joints_.size() && i < 4; ++i)
    {
      auto& ctl = joints_[i];
      auto& joint = ctl->joint_;
      joint.setCommand(wheel_power_limitor_.torque[i] * wheel_power_limitor_.K_angle[i]);
    }
  }
}

void OmniController::updatePowerStatus()
{
  double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis_.power_limit;

  wheel_power_limitor_.max_power = power_limit;
  wheel_power_limitor_.err_sum = 0;

  double ewheel_power{}, cwheel_power{};
  for (size_t i = 0; i < joints_.size() && i < 4; ++i)
  {
    auto& ctl = joints_[i];

    double cmd_torque = wheel_power_limitor_.torque[i] = ctl->joint_.getCommand();
    double cmd_vel{};
    ctl->getCommand(cmd_vel);
    double real_vel = wheel_power_limitor_.omiga[i] = ctl->joint_.getVelocity();
    motor_lp_filters_[i]->input(ctl->joint_.getEffort());
    double real_torque = motor_lp_filters_[i]->output();

    wheel_power_limitor_.err[i] = cmd_vel - real_vel;
    wheel_power_limitor_.err_sum += abs(wheel_power_limitor_.err[i]);

    ewheel_power += real_torque * real_vel + wheel_power_limitor_.effort_coeff * square(real_torque) +
                    wheel_power_limitor_.vel_coeff * abs(real_vel);
    cwheel_power += cmd_torque * real_vel + wheel_power_limitor_.effort_coeff * square(cmd_torque) +
                    wheel_power_limitor_.vel_coeff * abs(real_vel);

    wheel_power_limitor_.power_in[i] = cmd_torque * real_vel + wheel_power_limitor_.effort_coeff * square(cmd_torque) +
                                       wheel_power_limitor_.vel_coeff * abs(real_vel);
  }
  wheel_power_limitor_.cmd_power = cwheel_power + wheel_power_limitor_.power_offset;
  wheel_power_limitor_.estimated_power = ewheel_power + wheel_power_limitor_.power_offset;

  double estimated_total_power = wheel_power_limitor_.estimated_power;
  double cmd_total_power = wheel_power_limitor_.cmd_power;

  if (capacity_update_flag_ && use_rls_ && estimated_total_power > 0)
  {
    // Update Rls.
    double all_in = limit(estimated_total_power, -power_limit, power_limit);
    // Update Rls: compute regression vector x and pass actual measured power
    Eigen::Matrix<double, 2, 1> x;
    x(0) = square(wheel_power_limitor_.torque[0]) + square(wheel_power_limitor_.torque[1]) +
           square(wheel_power_limitor_.torque[2]) + square(wheel_power_limitor_.torque[3]);
    x(1) = abs(wheel_power_limitor_.omiga[0]) + abs(wheel_power_limitor_.omiga[1]) +
           abs(wheel_power_limitor_.omiga[2]) + abs(wheel_power_limitor_.omiga[3]);
    rls_->setU(all_in);
    rls_->setX(x);
    rls_->setY(chassis_power_);
    rls_->update();
    auto w = rls_->getW();
    wheel_power_limitor_.effort_coeff = std::max(w(0), 1e-3);
    wheel_power_limitor_.vel_coeff = std::max(w(1), 1e-3);
    capacity_update_flag_ = false;
  }

  // Publish power status.
  auto publishPower = [](auto& pub, const double power) {
    if (pub && pub->trylock())
    {
      pub->msg_.data = power;
      pub->unlockAndPublish();
    }
  };

  publishPower(epower_pub_, estimated_total_power);
  publishPower(cpower_pub_, cmd_total_power);
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::OmniController, controller_interface::ControllerBase)
