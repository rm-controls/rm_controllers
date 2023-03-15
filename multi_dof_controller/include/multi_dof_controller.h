//
// Created by lsy on 23-3-15.
//

#pragma once

#include <effort_controllers/joint_position_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>

namespace multi_dof_controller
{
class Controller : public  controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,hardware_interface::EffortJointInterface>
{
public:
    Controller() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
private:
    void position(const ros::Time& time, const ros::Duration& period);
    void velocity(const ros::Time& time, const ros::Duration& period);
    rm_control::RobotStateHandle robot_state_handle_;
    effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

    double publish_rate_{};
    bool state_changed_{};

    enum
    {
        VELOCITY,
        POSITION
    };
    int state_ = VELOCITY;
};
}

// namespace multi_dof_controller
