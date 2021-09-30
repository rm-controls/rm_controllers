#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <rm_common/hardware_interface/imu_extra_interface.h>
#include <imu_complementary_filter/complementary_filter.h>

namespace imu_filter_controllers
{
class ComplementaryController
  : public controller_interface::MultiInterfaceController<hardware_interface::ImuSensorInterface,
                                                          rm_control::ImuExtraInterface>
{
public:
  ComplementaryController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  // Parameters:
  bool use_mag_;
  bool publish_debug_topics_;

  // hardware_interface
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  rm_control::ImuExtraHandle imu_extra_handle_;

  sensor_msgs::Temperature imu_temp_;
  sensor_msgs::TimeReference trigger_time_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu> > imu_data_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Temperature> > imu_temp_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::TimeReference> > trigger_time_pub_;

  // State variables:
  imu_tools::ComplementaryFilter filter_;
  bool initialized_filter_;
  ros::Time last_update_;
};

}  // namespace imu_filter_controllers
