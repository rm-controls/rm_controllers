//
// Created by lsy on 23-3-15.
//

#pragma once

#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <rm_msgs/MultiDofCmd.h>

namespace multi_dof_controller
{
struct Joint
{
    std::string joint_name_;
    effort_controllers::JointPositionController* ctrl_position_;
    effort_controllers::JointVelocityController* ctrl_velocity_;
};
struct Motion
{
    std::string motion_name_;
    double position_per_step_;
    double velocity_max_speed_;
    std::vector<double> position_config_;
    std::vector<double> velocity_config_;
    std::vector<double> position_need_reverse;
    std::vector<double> velocity_need_reverse;
};

class Controller : public  controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,hardware_interface::EffortJointInterface>
{
public:
    Controller() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

private:
    void position(const ros::Time& time);
    void velocity(const ros::Time& time, const ros::Duration& period);
    void moveJoint(const ros::Time& time, const ros::Duration& period);
    void commandCB(const rm_msgs::MultiDofCmdPtr& msg);

    rm_control::RobotStateHandle robot_state_handle_;
    effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

    // ROS Interface
    ros::Subscriber cmd_multi_dof_sub_;
    realtime_tools::RealtimeBuffer<rm_msgs::MultiDofCmd> cmd_rt_buffer_;
    hardware_interface::EffortJointInterface* effort_joint_interface_{};
    std::vector<hardware_interface::JointHandle> joint_handles_{};

    rm_msgs::MultiDofCmd cmd_multi_dof_;
    bool state_changed_{};

    enum
    {
        VELOCITY,
        POSITION
    };
    int state_ = VELOCITY;
    std::vector<Joint> joints_;
    std::vector<Motion> motions_;
};

}// namespace multi_dof_controller
