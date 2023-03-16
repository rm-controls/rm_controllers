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
    XmlRpc::XmlRpcValue joints,motions;
    controller_nh.getParam("joints", joints);
    controller_nh.getParam("motions", motions);
    ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
    for (const auto& joint : joints)
    {
        Joint j{ .joint_name_ = joint.first,
                .ctrl_position_ = new effort_controllers::JointPositionController(),
                .ctrl_velocity_ = new effort_controllers::JointVelocityController() };
        ros::NodeHandle nh_position = ros::NodeHandle(controller_nh, "joints/" + joint.first + "position");
        ros::NodeHandle nh_velocity = ros::NodeHandle(controller_nh, "joints/" + joint.first + "velocity");
        if (!j.ctrl_position_->init(effort_joint_interface_, nh_position) ||
            !j.ctrl_velocity_->init(effort_joint_interface_, nh_velocity))
            return false;
        joint_handles_.push_back(j.ctrl_position_->joint_);
        joint_handles_.push_back(j.ctrl_velocity_->joint_);
        joints_.push_back(j);
    }
    for (const auto& motion : motions)
    {
        Motion m{ .motion_name_ = motion.first,
                  .position_per_step_ = motion.second["position_per_step"],
                  .velocity_max_speed_ = motion.second["velocity_max_speed"] };
        for (int i = 0; i < motion.second["position_config"].size(); ++i)
        {
            ROS_ASSERT(motion.second["position_config"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            ROS_ASSERT(motion.second["velocity_config_"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            ROS_ASSERT(motion.second["position_need_reverse"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            ROS_ASSERT(motion.second["velocity_need_reverse"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            m.position_config_.push_back(motion.second["position_config"][i]);
            m.velocity_config_.push_back(motion.second["velocity_config_"][i]);
            m.position_need_reverse.push_back(motion.second["position_need_reverse"][i]);
            m.velocity_need_reverse.push_back(motion.second["velocity_need_reverse"][i]);
        }
        motions_.push_back(m);
    }
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
    if (state_ != cmd_multi_dof_.mode)
    {
        state_ = cmd_multi_dof_.mode;
        state_changed_ = true;
    }
    switch (state_)
    {
        case VELOCITY:
            velocity(time,period);
            break;
        case POSITION:
            position(time,period);
            break;
    }
}

void Controller::velocity(const ros::Time &time,const ros::Duration& period)
{
    if (state_changed_)
    {  // on enter
        state_changed_ = false;
        ROS_INFO("[Multi_Fof] VELOCITY");
    }
    std::vector<double> results{(double)joints_.size()};
    for (int i = 0; (int)i < joints_.size(); ++i) {

            for (int j = 0; (int)j < motions_.size(); ++j) {
                if (motions_[j].motion_name_ == cmd_multi_dof_.motion_name) {
                    results[i] = judgeReverse(getDirectionValue(cmd_multi_dof_), motions_[j].velocity_need_reverse[i]) *
                                 motions_[j].velocity_max_speed_ * motions_[j].velocity_config_[i];
                }
            }
    }
    for (int i = 0; (int)i < joints_.size(); ++i) {
        joints_[i].ctrl_velocity_->setCommand(results[i]);
        joints_[i].ctrl_velocity_->update(time,period);
    }
}
void Controller::position(const ros::Time &time,const ros::Duration& period)
{
    if (state_changed_)
    {  // on enter
        state_changed_ = false;
        ROS_INFO("[Multi_Fof] POSITION");
    }
    std::vector<double> results{(double)joints_.size()};
    for (int i = 0; (int)i < joints_.size(); ++i) {
        for (int j = 0; (int)j < motions_.size(); ++j) {
            if (motions_[j].motion_name_ == cmd_multi_dof_.motion_name)
            {
                double total_step_value = judgeReverse(getDirectionValue(cmd_multi_dof_),motions_[j].position_need_reverse[i]);
                double step_num = total_step_value / motions_[j].position_per_step_;
                results[i] = step_num * motions_[j].position_config_[i];
            }
        }
    }
    for (int i = 0; (int)i < joints_.size(); ++i) {
        joints_[i].ctrl_position_->setCommand(joints_[i].ctrl_position_->getPosition()+results[i]);
        joints_[i].ctrl_position_->update(time,period);
    }
}


double Controller::getDirectionValue()
{
    double direction_value;
    if (cmd_multi_dof_.motion_name == "x")
        direction_value = cmd_multi_dof_.values.linear.x;
    else if (cmd_multi_dof_.motion_name == "y")
        direction_value = cmd_multi_dof_.values.linear.y;
    else if (cmd_multi_dof_.motion_name == "z")
        direction_value = cmd_multi_dof_.values.linear.z;
    else if (cmd_multi_dof_.motion_name == "roll")
        direction_value = cmd_multi_dof_.values.angular.x;
    else if (cmd_multi_dof_.motion_name == "pitch")
        direction_value = cmd_multi_dof_.values.angular.y;
    else if (cmd_multi_dof_.motion_name == "yaw")
        direction_value = cmd_multi_dof_.values.linear.z;
    else
        direction_value = 0;
    return direction_value;
}

double Controller::judgeReverse(double value, bool is_need_reverse)
{
    if (!is_need_reverse)
        value = abs(value);
    return value;
}
void Controller::commandCB(const rm_msgs::MultiDofCmdPtr &msg)
{
    cmd_rt_buffer_.writeFromNonRT(*msg);
}
}