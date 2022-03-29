//
// Created by luotinkai on 2022/1/2.
//

#include "tof_sensor_controller/tof_sensor_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace tof_sensor_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  std::string name;
  if (!controller_nh.getParam("name", name))
  {
    ROS_ERROR("Name was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  tof_sensor_handle = robot_hw->get<rm_control::TofSensorInterface>()->getHandle(name);

  tof_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::TofSensor>(controller_nh, name + "/data", 100));
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  if (tof_pub_->trylock())
  {
    tof_pub_->msg_.distance = tof_sensor_handle.getDistance();
    tof_pub_->msg_.signal_strength = tof_sensor_handle.getStrength();
    tof_pub_->msg_.dis_status = tof_sensor_handle.getStatus();
    tof_pub_->msg_.stamp = time;
    tof_pub_->unlockAndPublish();
  }
}

}  // namespace tof_sensor_controller

PLUGINLIB_EXPORT_CLASS(tof_sensor_controller::Controller, controller_interface::ControllerBase)
