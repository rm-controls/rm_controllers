//
// Created by ljq on 2021/12/17.
//

#pragma once

#include <rm_common/math_utilities.h>

namespace rm_chassis_controllers
{
class PowerLimit
{
public:
  explicit PowerLimit(ros::NodeHandle& controller_nh, hardware_interface::EffortJointInterface* hardware_interface,
                      std::string keyword)
  {
    XmlRpc::XmlRpcValue xml_value;
    controller_nh.getParam("power_limit", xml_value);
    ROS_ASSERT(xml_value.getType() == XmlRpc::XmlRpcValue::Type::TypeArray);
    for (int i = 0; i < xml_value.size(); i++)
    {
      if (xml_value[i]["keyword"] == keyword)
      {
        effort_coeff_ = xml_value[i]["effort_coeff"];
        vel_coeff_ = xml_value[i]["vel_coeff"];
        power_offset_ = xml_value[i]["power_offset_"];
      }
    }
    if (keyword == "wheel")
    {
      joint_handles_.push_back(hardware_interface->getHandle("right_front_wheel_joint"));
      joint_handles_.push_back(hardware_interface->getHandle("left_front_wheel_joint"));
      joint_handles_.push_back(hardware_interface->getHandle("left_back_wheel_joint"));
      joint_handles_.push_back(hardware_interface->getHandle("right_back_wheel_joint"));
    }
    else if (keyword == "pivot")
    {
      joint_handles_.push_back(hardware_interface->getHandle("right_front_pivot_joint"));
      joint_handles_.push_back(hardware_interface->getHandle("left_front_pivot_joint"));
      joint_handles_.push_back(hardware_interface->getHandle("left_back_pivot_joint"));
      joint_handles_.push_back(hardware_interface->getHandle("right_back_pivot_joint"));
    }
    else if (keyword == "sentry")
    {
      joint_handles_.push_back(hardware_interface->getHandle("drive_wheel_joint"));
    }
    else
    {
      ROS_ERROR("no type");
    }
  }

  void limit(double power_limit)
  {
    // Three coefficients of a quadratic equation in one variable
    double a = 0., b = 0., c = 0.;
    for (const auto& joint : joint_handles_)
    {
      double cmd_effort = joint.getCommand();
      double real_vel = joint.getVelocity();
      a += square(cmd_effort);
      b += std::abs(cmd_effort * real_vel);
      c += square(real_vel);
    }
    a *= effort_coeff_;
    c = c * vel_coeff_ - power_offset_ - power_limit;
    // Root formula for quadratic equation in one variable
    double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
    for (auto joint : joint_handles_)
      joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
  }

  std::vector<hardware_interface::JointHandle> joint_handles_{};
  double effort_coeff_, vel_coeff_, power_offset_;
};

}  // namespace rm_chassis_controllers
