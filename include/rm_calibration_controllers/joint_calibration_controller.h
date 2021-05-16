//
// Created by qiayuan on 5/16/21.
//

#ifndef RM_CALIBRATION_CONTROLLERS_JOINT_CALIBRATION_CONTROLLER_H_
#define RM_CALIBRATION_CONTROLLERS_JOINT_CALIBRATION_CONTROLLER_H_
#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <rm_common/hardware_interface/actuator_extra_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <control_msgs/QueryCalibrationState.h>

namespace rm_calibration_controllers {

class JointCalibrationController : public controller_interface::MultiInterfaceController
    <hardware_interface::EffortJointInterface, hardware_interface::ActuatorExtraInterface> {
 public:
  JointCalibrationController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time &time) override;
 private:
  bool isCalibrated(control_msgs::QueryCalibrationState::Request &req,
                    control_msgs::QueryCalibrationState::Response &resp);

  ros::Time last_publish_time_;
  ros::ServiceServer is_calibrated_srv_;
//  enum { INITIALIZED, BEGINNING, MOVING_TO_LOW, MOVING_TO_HIGH, CALIBRATED }; for GPIO switch
  enum { INITIALIZED, MOVING, CALIBRATED };
  int state_{}, countdown_{};
  double vel_search_{}, threshold_{};
  hardware_interface::ActuatorExtraHandle actuator_;
  effort_controllers::JointVelocityController velocity_ctrl_;
};

}
#endif //RM_CALIBRATION_CONTROLLERS_JOINT_CALIBRATION_CONTROLLER_H_
