//
// Created by qiayuan on 2022/7/29.
//

#include <string>
#include <Eigen/QR>

#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

#include "rm_chassis_controllers/active_suspension.h"

namespace rm_chassis_controllers
{
    bool ActiveSuspensionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        OmniController::init(robot_hw, root_nh, controller_nh);

        active_suspension_sub_=controller_nh.subscribe("/cmd_active_suspension",10,&ActiveSuspensionController::ActiveSuspensionCallBack,this );

        XmlRpc::XmlRpcValue suspension_legs;
        controller_nh.getParam("suspension_leg", suspension_legs);

        for (const auto& suspension_leg : suspension_legs)
        {
            ros::NodeHandle nh_suspension = ros::NodeHandle(controller_nh, "suspension_leg/" + suspension_leg.first);
            active_suspension_joints_.push_back(std::make_shared<effort_controllers::JointPositionController>());
            if (!active_suspension_joints_.back()->init(effort_joint_interface_, nh_suspension))
            {
                ROS_ERROR("Failed to init active_suspension_joint");
                return false;
            }
        }
        return true;
    }
    void ActiveSuspensionController::moveJoint(const ros::Time& time, const ros::Duration& period)
    {
        OmniController::moveJoint(time, period);
        switch (state_)
        {
          case DOWN:
            suspension_pos_ = 0.0;
            break;
          case MID:
            suspension_pos_ = 0.4;
            break;
          case UP:
            suspension_pos_ = 0.85;
            break;
     }
        for (auto& joint : active_suspension_joints_)
        {
            joint->setCommand(suspension_pos_);
            joint->update(time, period);
            double pos = joint->getPosition();
            auto cmd = joint->joint_.getCommand();
            cmd += 25 - pos*25.88;
            joint->joint_.setCommand(cmd);

        }
    }
    void ActiveSuspensionController::ActiveSuspensionCallBack(const rm_msgs::ChassisActiveSusCmd& msg)
    {
      state_ = msg.mode;
    }

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ActiveSuspensionController, controller_interface::ControllerBase)
