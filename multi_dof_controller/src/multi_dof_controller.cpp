//
// Created by lsy on 23-3-15.
//

#include "multi_dof_controller.h"

#include <string>
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
        return true;
    }

    void Controller::starting(const ros::Time& /*unused*/)
    {
        state_ = VELOCITY;
        state_changed_ = true;
    }

    void Controller::update(const ros::Time& time, const ros::Duration& period)
    {
        if (state_)
        {
            state_changed_ = true;
        }
        switch (state_)
        {
            case VELOCITY:
                break;
            case POSITION:
                break;
        }
    }

}