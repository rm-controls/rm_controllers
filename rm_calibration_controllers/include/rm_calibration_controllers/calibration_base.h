//
// Created by guanlin on 23-3-14.
//

#pragma once

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <rm_common/hardware_interface/actuator_extra_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/hardware_interface/gpio_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <control_msgs/QueryCalibrationState.h>

namespace rm_calibration_controllers
{
template <typename... T>
class CalibrationBase : public controller_interface::MultiInterfaceController<T...>
{
public:
  CalibrationBase() = default;
  /** @brief Get necessary params from param server. Init joint_calibration_controller.
   *
   * Get params from param server and check whether these params are set.Init JointVelocityController.Check
   * whether threshold is set correctly.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if init successful, false when failed.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /** @brief Switch all of the actuators state to INITIALIZED.
   *
   * Switch all of the actuator state to INITIALIZED in order to restart the calibration.
   *
   * @param time The current time.
   */
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

protected:
  /** @brief Provide a service to know the state of target actuators.
   *
   * When requesting to this server, it will return respond about whether target actuators has been calibrated.
   *
   * @param req The request of knowing the state of target actuators.
   * @param resp The respond included the state of target actuators.
   * @return True if get respond successfully, false when failed.
   */
  bool isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                    control_msgs::QueryCalibrationState::Response& resp);
  ros::ServiceServer is_calibrated_srv_;
  enum State
  {
    INITIALIZED,
    CALIBRATED
  };
  int state_{};
  double velocity_search_{};
  bool calibration_success_ = false;
  rm_control::ActuatorExtraHandle actuator_;
  effort_controllers::JointVelocityController velocity_ctrl_;
  effort_controllers::JointPositionController position_ctrl_;
};

}  // namespace rm_calibration_controllers
