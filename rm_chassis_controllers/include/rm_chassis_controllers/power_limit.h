//
// Created by ljq on 2021/12/3.
//

#pragma once

#include <effort_controllers/joint_position_controller.h>

namespace power_limit
{
class PowerLimit
{
public:
  PowerLimit(ros::NodeHandle& controller_nh, effort_controllers::JointVelocityController& joint_interface)
  {
    joint_ = joint_interface.joint_;
    if (controller_nh.hasParam("power_limit"))
    {
      XmlRpc::XmlRpcValue xml_value;
      controller_nh.getParam("power_limit", xml_value);
      ROS_ASSERT(xml_value.getType() == XmlRpc::XmlRpcValue::Type::TypeArray);
      for (int i = 0; i < xml_value.size(); ++i)
      {
        std::string arr = xml_value[i]["keyword"];
        if (joint_.getName().find(arr) != std::string::npos)
        {
          keyword_ = arr;
          effort_coeff_ = xml_value[i]["effort_coeff"];
          vel_coeff_ = xml_value[i]["vel_coeff"];
          power_offset_ = xml_value[i]["power_offset"];
        }
      }
    }
  }

  PowerLimit(ros::NodeHandle& controller_nh, effort_controllers::JointPositionController& joint_interface)
  {
    joint_ = joint_interface.joint_;
    if (controller_nh.hasParam("power_limit"))
    {
      XmlRpc::XmlRpcValue xml_value;
      controller_nh.getParam("power_limit", xml_value);
      ROS_ASSERT(xml_value.getType() == XmlRpc::XmlRpcValue::Type::TypeArray);
      for (int i = 0; i < xml_value.size(); ++i)
      {
        std::string arr = xml_value[i]["keyword"];
        if (joint_.getName().find(arr) != std::string::npos)
        {
          keyword_ = arr;
          effort_coeff_ = xml_value[i]["effort_coeff"];
          vel_coeff_ = xml_value[i]["vel_coeff"];
          power_offset_ = xml_value[i]["power_offset"];
        }
      }
    }
  }

  std::string keyword_;
  double effort_coeff_, vel_coeff_, power_offset_;
  double final_a, final_b, final_c;
  hardware_interface::JointHandle joint_;
};
}  // namespace power_limit