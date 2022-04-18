//
// Created by luotinkai on 2022/1/2.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/hardware_interface/tf_radar_interface.h>
#include <rm_msgs/TfRadarData.h>

namespace tf_radar_controller
{
class Controller : public controller_interface::Controller<rm_control::TfRadarInterface>
{
public:
  Controller() = default;

  bool init(rm_control::TfRadarInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  std::vector<rm_control::TfRadarHandle> tf_radar_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::TfRadarData>> RtpublisherPtr;
  RtpublisherPtr tf_pub_;
  std::vector<RtpublisherPtr> realtime_pubs_;
};
}  // namespace tf_radar_controller
