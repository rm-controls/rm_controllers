//
// Created by guanlin on 23-3-14.
//

#include "rm_calibration_controllers/calibration_base.h"

namespace rm_calibration_controllers
{
template class CalibrationBase<rm_control::ActuatorExtraInterface, hardware_interface::EffortJointInterface>;
template class CalibrationBase<rm_control::ActuatorExtraInterface, rm_control::GpioStateInterface,
                               hardware_interface::EffortJointInterface>;

template <typename... T>
bool CalibrationBase<T...>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                 ros::NodeHandle& controller_nh)
{
  ros::NodeHandle vel_nh(controller_nh, "velocity");
  velocity_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), vel_nh);
  XmlRpc::XmlRpcValue actuator;
  if (!controller_nh.getParam("actuator", actuator))
  {
    ROS_ERROR("No actuator given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  actuator_ = robot_hw->get<rm_control::ActuatorExtraInterface>()->getHandle(actuator[0]);
  if (!vel_nh.getParam("search_velocity", velocity_search_))
  {
    ROS_ERROR("Search velocity was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  // advertise service to check calibration
  is_calibrated_srv_ = controller_nh.advertiseService("is_calibrated", &CalibrationBase<T...>::isCalibrated, this);
  return true;
}

template <typename... T>
void CalibrationBase<T...>::starting(const ros::Time& time)
{
  actuator_.setCalibrated(false);
  state_ = INITIALIZED;
  if (actuator_.getCalibrated())
    ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f",
             velocity_ctrl_.getJointName().c_str(), actuator_.getOffset());
}

template <typename... T>
void CalibrationBase<T...>::stopping(const ros::Time& time)
{
  calibration_success_ = false;
}

template <typename... T>
bool CalibrationBase<T...>::isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                                         control_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", calibration_success_);
  resp.is_calibrated = calibration_success_;
  return true;
}
}  // namespace rm_calibration_controllers
