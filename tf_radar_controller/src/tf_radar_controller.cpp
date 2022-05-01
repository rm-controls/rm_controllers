//
// Created by luotinkai on 2022/1/2.
//

#include "tf_radar_controller/tf_radar_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace tf_radar_controller
{
bool Controller::init(rm_control::TfRadarInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  const std::vector<std::string>& tf_radar_names = hw->getNames();
  for (const auto& tf_radar_name : tf_radar_names)
    ROS_DEBUG("Got radar %s", tf_radar_name.c_str());

  for (const auto& tf_radar_name : tf_radar_names)
  {
    // sensor handle
    tf_radar_.push_back(hw->getHandle(tf_radar_name));

    // realtime publisher
    tf_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::TfRadarData>(controller_nh, tf_radar_name + "/data", 100));
    realtime_pubs_.push_back(tf_pub_);
  }
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned int i = 0; i < realtime_pubs_.size(); ++i)
  {
    if (realtime_pubs_[i]->trylock())
    {
      realtime_pubs_[i]->msg_.distance = tf_radar_[i].getDistance() / 100.;
      realtime_pubs_[i]->msg_.strength = tf_radar_[i].getStrength();
      realtime_pubs_[i]->msg_.stamp = time;
      realtime_pubs_[i]->unlockAndPublish();
    }
  }
}

}  // namespace tf_radar_controller

PLUGINLIB_EXPORT_CLASS(tf_radar_controller::Controller, controller_interface::ControllerBase)
