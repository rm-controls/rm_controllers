//
// Created by luotinkai on 2022/1/2.
//

#include "tof_radar_controller/tof_radar_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace tof_radar_controller
{
bool Controller::init(rm_control::TofRadarInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  const std::vector<std::string>& tof_radar_names = hw->getNames();
  for (const auto& tof_radar_name : tof_radar_names)
    ROS_DEBUG("Got radar %s", tof_radar_name.c_str());

  for (const auto& tof_radar_name : tof_radar_names)
  {
    // sensor handle
    tof_radar_.push_back(hw->getHandle(tof_radar_name));

    // realtime publisher
    tof_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::TofRadarData>(controller_nh, tof_radar_name + "/data", 100));
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
      realtime_pubs_[i]->msg_.distance = tof_radar_[i].getDistance() / 100.;
      realtime_pubs_[i]->msg_.strength = tof_radar_[i].getStrength();
      realtime_pubs_[i]->msg_.stamp = time;
      realtime_pubs_[i]->unlockAndPublish();
    }
  }
}

}  // namespace tof_radar_controller

PLUGINLIB_EXPORT_CLASS(tof_radar_controller::Controller, controller_interface::ControllerBase)
