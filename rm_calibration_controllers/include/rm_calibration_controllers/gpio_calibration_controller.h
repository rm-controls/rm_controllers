//
// Created by guanlin on 22-11-7.
//

#pragma once

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <rm_common/hardware_interface/actuator_extra_interface.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_msgs/QueryCalibrationState.h>
#include <rm_msgs/GpioData.h>
namespace rm_calibration_controllers
{
class GpioCalibrationController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                          rm_control::ActuatorExtraInterface>
{
public:
  GpioCalibrationController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;

private:
  void gpioStateCB(const rm_msgs::GpioDataConstPtr& msg);
  bool isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                    control_msgs::QueryCalibrationState::Response& resp);
  enum
  {
    INITIALIZED,
    MOVING_AROUND,
    RETURN,
    CALIBRATED
  };
  int state_{};
  double velocity_search_{}, vel_gain_{}, vel_threshold_{}, enter_pos_{}, exit_pos_{};
  double position_threshold_ = 0.01;
  bool initial_gpio_state_ = false, enter_flag_ = false, exit_flag_ = false, can_returned_ = false,
       is_returned_ = false;
  rm_control::ActuatorExtraHandle actuator_;
  effort_controllers::JointVelocityController velocity_ctrl_;
  effort_controllers::JointPositionController position_ctrl_;

  ros::ServiceServer is_calibrated_srv_;
  ros::Subscriber gpio_sub_;
};
}  // namespace rm_calibration_controllers
