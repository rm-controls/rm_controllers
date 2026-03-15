//
// Created by wiselook on 7/27/25.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <effort_controllers/joint_position_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/ros_utilities.h>
#include <std_msgs/Float64MultiArray.h>

#include "bipedal_wheel_controller/vmc/leg_params.h"
#include "bipedal_wheel_controller/vmc/leg_conv.h"
#include "bipedal_wheel_controller/vmc/leg_pos.h"
#include "bipedal_wheel_controller/vmc/leg_spd.h"

#include "bipedal_wheel_controller/vmc/VMC.h"

#include <utility>

namespace rm_chassis_controllers
{
class VMCController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                            rm_control::RobotStateInterface>
{
public:
  VMCController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& /*time*/) override;

private:
  void commandLegLengthCB(const std_msgs::Float64ConstPtr& msg)
  {
    lengthCmd_ = msg->data;
  }

  void commandLegAngleCB(const std_msgs::Float64ConstPtr& msg)
  {
    angleCmd_ = msg->data;
  }

  hardware_interface::JointHandle jointThigh_, jointKnee_;
  control_toolbox::Pid pidLength_, pidAngle_;

  double vmcBiasAngle_, spring_force_;
  double lengthCmd_, angleCmd_;

  std::unique_ptr<VMC> vmcPtr_;

  ros::Publisher statePublisher_, jointCmdStatePublisher_;
  ros::Subscriber cmdLegLengthSubscriber_, cmdLegAngleSubscriber_;
};

}  // namespace rm_chassis_controllers
