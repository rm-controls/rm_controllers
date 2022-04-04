//
// Created by luotinkai on 2022/1/2.
//

#include "tof_sensor_controller/tof_sensor_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace tof_sensor_controller
{
bool Controller::init(rm_control::TofSensorInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  const std::vector<std::string>& tof_sensor_names = hw->getNames();
  for (unsigned i = 0; i < tof_sensor_names.size(); i++)
    ROS_DEBUG("Got sensor %s", tof_sensor_names[i].c_str());

  for (unsigned i = 0; i < tof_sensor_names.size(); i++)
  {
    // sensor handle
    tof_sensors_.push_back(hw->getHandle(tof_sensor_names[i]));

    // realtime publisher
    tof_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::TofSensor>(controller_nh, tof_sensor_names[i] + "/data", 100));
    realtime_pubs_.push_back(tof_pub_);
  }
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned int i = 0; i < realtime_pubs_.size(); ++i)
  {
    if (realtime_pubs_[i]->trylock())
    {
      realtime_pubs_[i]->msg_.distance = tof_sensors_[i].getDistance();
      realtime_pubs_[i]->msg_.signal_strength = tof_sensors_[i].getStrength();
      realtime_pubs_[i]->msg_.dis_status = tof_sensors_[i].getStatus();
      realtime_pubs_[i]->msg_.stamp = time;
      realtime_pubs_[i]->unlockAndPublish();
    }
  }
}

}  // namespace tof_sensor_controller

PLUGINLIB_EXPORT_CLASS(tof_sensor_controller::Controller, controller_interface::ControllerBase)
