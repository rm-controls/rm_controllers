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
        feed_forward_offset = getParam(controller_nh, "feed_forward_offset", 0.);
        stretch_coff_A_ = getParam(controller_nh,"stretch_coff_A_",0.);
        stretch_coff_k_ = getParam(controller_nh,"stretch_coff_k_",0.);
        shrink_coff_A_ = getParam(controller_nh,"shrink_coff_A_",0.);
        shrink_coff_k_ = getParam(controller_nh,"shrink_coff_k_",0.);


        suspension_pos_DOWN_ = getParam(controller_nh,"suspension_pos_DOWN_",0.);
        suspension_pos_MID_ = getParam(controller_nh,"suspension_pos_MID_",0.);
        suspension_pos_UP_ = getParam(controller_nh,"suspension_pos_UP_",0.);
        static_effort_DOWN_ = getParam(controller_nh,"static_effort_DOWN_",0.);
        static_effort_MID_ = getParam(controller_nh,"static_effort_MID_",0.);
        static_effort_UP_ = getParam(controller_nh,"static_effort_UP_",0.);
        feedforward_effect_time_ = getParam(controller_nh,"feedforward_effect_time_",0.);

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
            suspension_pos_ = suspension_pos_DOWN_;
            static_effort = -6.0;
            break;
          case MID:
            suspension_pos_ = suspension_pos_MID_;
            static_effort = 8.0;
            break;
          case UP:
            suspension_pos_ = suspension_pos_UP_;
            static_effort = 0.0;
            break;
        }
        if (state_ != check_state_)
        {
          count_ =0;
          last_state_ =  check_state_;
        }
        if (last_state_ - state_ == -2 || ((last_state_ - state_ == - 1) && (state_ ==2)))
        {
          count_ ++;
          if (count_ == 1)
          {
            state_change_time = ros::Time::now();
          }
          feed_forward_effort =  2 * (stretch_coff_A_ + stretch_coff_k_ * pos);
        }
        else if ( last_state_ - state_ == -1 && state_ == 1)
        {
          count_ ++;
          if (count_ == 1)
          {
            state_change_time = ros::Time::now();
          }
          feed_forward_effort = stretch_coff_A_ - 45.0 * pos;
        }
        else if (last_state_ - state_ == 1 || last_state_ - state_ == 2)
        {
          count_ ++;
          if (count_ == 1)
          {
            state_change_time = ros::Time::now();
          }
          feed_forward_effort = shrink_coff_A_ + shrink_coff_k_ * pos;
        }
        else
        {
          feed_forward_effort = 0.0;
        }

        for (auto& joint : active_suspension_joints_)
        {
            joint->setCommand(suspension_pos_);
            joint->update(time, period);
            pos = joint->getPosition();
            auto cmd = joint->joint_.getCommand();

            cmd += feed_forward_effort + feed_forward_offset + static_effort;
            joint->joint_.setCommand(cmd);
        }
        if (ros::Time::now().toSec() - state_change_time.toSec() > feedforward_effect_time_)
        {
          last_state_ = state_;
          count_ = 0;
        }
        check_state_ = state_;
    }
    void ActiveSuspensionController::ActiveSuspensionCallBack(const rm_msgs::ChassisActiveSusCmd& msg)
    {
      state_ = msg.mode;
    }

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ActiveSuspensionController, controller_interface::ControllerBase)
