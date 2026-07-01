//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <ros/time.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <angles/angles.h>

#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/controller_interface.h"

namespace rm_chassis_controllers
{
class BipedalController;

class ModeBase
{
public:
  explicit ModeBase(BipedalControllerInterface* controller_) : controller(controller_)
  {
  }
  virtual void execute(const ros::Time& time, const ros::Duration& period) = 0;
  virtual const char* name() const = 0;
  virtual ~ModeBase() = default;
  void updateUnstick(const bool& left_unstick, const bool& right_unstick);

  inline bool getUnstick()
  {
    return (left_unstick_ && right_unstick_);
  }

protected:
  bool left_unstick_{ false }, right_unstick_{ false };
  BipedalControllerInterface* controller{ nullptr };
};

}  // namespace rm_chassis_controllers
