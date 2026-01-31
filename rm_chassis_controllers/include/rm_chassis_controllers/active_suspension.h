//
// Created by xuncheng on 2025/11/29.
//


#include "rm_chassis_controllers/chassis_base.h"
#include "rm_chassis_controllers/omni.h"
#include <std_msgs/Bool.h>
#include <effort_controllers/joint_position_controller.h>
#include <rm_msgs/ChassisActiveSusCmd.h>


namespace rm_chassis_controllers
{
    class ActiveSuspensionController : public OmniController
    {
    public:
        ActiveSuspensionController() = default;
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
        void ActiveSuspensionCallBack(const rm_msgs::ChassisActiveSusCmd& msg);

    private:
        void moveJoint(const ros::Time& time, const ros::Duration& period) override;

        std::vector<std::shared_ptr<effort_controllers::JointPositionController>> active_suspension_joints_;
        std::vector<hardware_interface::JointHandle> active_suspension_joint_handles_{};

        ros::Subscriber active_suspension_sub_;

        double suspension_pos_;

      enum
      {
          DOWN,
          MID,
          UP
      };
      int state_ = DOWN;

  };
}
