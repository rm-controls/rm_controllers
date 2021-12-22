//
// Created by ljq on 2021/12/21.
//

#pragma once

#include <rm_common/math_utilities.h>
namespace rm_chassis_controllers
{
class PowerLimit
{
public:
  explicit PowerLimit(XmlRpc::XmlRpcValue xml_value, hardware_interface::EffortJointInterface& hardware_interface)
  {
    if (xml_value.hasMember("keyword"))
    {
      for (long unsigned int j = 0; j < hardware_interface.getNames().size(); j++)
      {
        std::string keyword_name_ = xml_value["keyword"];
        if (hardware_interface.getNames()[j].find(keyword_name_) != std::string::npos)
        {
          effort_coeff_ = xml_value["effort_coeff"];
          vel_coeff_ = xml_value["vel_coeff"];
          power_offset_ = xml_value["power_offset"];
          joint_handles_.push_back(hardware_interface.getHandle(hardware_interface.getNames()[j]));
        }
      }
    }
    else if (xml_value.hasMember("joint"))
    {
      for (long unsigned int j = 0; j < hardware_interface.getNames().size(); j++)
      {
        for (int k = 0; k < xml_value["joint"].size(); k++)
        {
          std::string joint_name_ = xml_value["joint"][k];
          if (hardware_interface.getNames()[j] == joint_name_)
          {
            effort_coeff_ = xml_value["effort_coeff"];
            vel_coeff_ = xml_value["vel_coeff"];
            power_offset_ = xml_value["power_offset"];
            joint_handles_.push_back(hardware_interface.getHandle(joint_name_));
          }
        }
      }
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
    {
      joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
    }
  }

  double effort_coeff_, vel_coeff_, power_offset_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
};
}  // namespace rm_chassis_controllers
