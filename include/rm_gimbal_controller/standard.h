//
// Created by qiayuan on 1/16/21.
//

#ifndef RM_GIMBAL_CONTROLLER_INCLUDE_STANDARD_H_
#define RM_GIMBAL_CONTROLLER_INCLUDE_STANDARD_H_

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_base/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/GimbalCmd.h>

namespace rm_gimbal_controller {

class GimbalStandardController :
    public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface, hardware_interface::RobotStateInterface> {
 public:
  GimbalStandardController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration & /*period*/) override;
 private:
  control_toolbox::Pid pid_yaw_, pid_pitch_;
  hardware_interface::JointHandle joint_yaw_, joint_pitch_;

  ros::Subscriber sub_command_;
  void commandCB(const rm_msgs::GimbalCmdConstPtr &msg);
};
}

#endif //RM_GIMBAL_CONTROLLER_INCLUDE_STANDARD_H_
