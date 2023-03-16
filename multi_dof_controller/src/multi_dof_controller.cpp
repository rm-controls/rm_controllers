//
// Created by lsy on 23-3-15.
//

#include "multi_dof_controller.h"

#include <string>
#include <rm_common/ros_utilities.h>
#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>


namespace multi_dof_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
    XmlRpc::XmlRpcValue xml_rpc_value;
    ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
    ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
    hardware_interface::EffortJointInterface* effort_joint_interface =
            robot_hw->get<hardware_interface::EffortJointInterface>();
    if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch))
        return false;
    robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
    cmd_multi_dof_sub_ = controller_nh.subscribe("command", 1, &Controller::commandCB, this);
    return true;
    }

void Controller::starting(const ros::Time& /*unused*/)
{
    state_ = VELOCITY;
    state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
    cmd_multi_dof_ = *cmd_rt_buffer_.readFromRT();
    if (state_)
    {
        state_changed_ = true;
    }
    switch (state_)
    {
        case VELOCITY:
            velocity(time,period);
            break;
        case POSITION:
            position(time);
            break;
    }
    moveJoint(time, period);
}

void Controller::velocity(const ros::Time &time, const ros::Duration &period)
{
    if (state_changed_)
    {  // on enter
        state_changed_ = false;
        ROS_INFO("[Multi_Fof] VELOCITY");
    }
}
void Controller::position(const ros::Time &time)
{
    if (state_changed_)
    {  // on enter
        state_changed_ = false;
        ROS_INFO("[Multi_Fof] POSITION");
    }
}

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{

}

void Controller::commandCB(const rm_msgs::MultiDofCmdPtr &msg)
{
    cmd_rt_buffer_.writeFromNonRT(*msg);
}
}