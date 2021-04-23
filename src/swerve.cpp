//
// Created by qiayuan on 4/23/21.
//

#include "rm_chassis_controllers/swerve.h"

#include <angles/angles.h>

namespace rm_chassis_controllers {

// Ref: https://dominik.win/blog/programming-swerve-drive/

void SwerveController::moveJoint(const ros::Duration &period) {
  Vec2<double> vel_center(vel_tfed_.vector.x, vel_tfed_.vector.y);
  for (auto &module:modules_) {
    Vec2<double> vel = vel_center + vel_tfed_.vector.z * module.position_;
    double vel_angle = std::atan2(vel.y(), vel.x()) + module.pivot_offset_;
    // Direction flipping and Stray module mitigation
    double error_pivot = std::min(
        angles::shortest_angular_distance(module.joint_pivot_.getPosition(), vel_angle),
        angles::shortest_angular_distance(module.joint_pivot_.getPosition(), vel_angle + M_PI));
    double wheel_des = vel.norm() / module.wheel_radius_
        * std::cos(angles::shortest_angular_distance(module.joint_pivot_.getPosition(), vel_angle));
    double error_wheel = wheel_des - module.joint_wheel_.getVelocity();
    // PID
    module.pid_pivot_.computeCommand(error_pivot, period);
    module.pid_wheel_.computeCommand(error_wheel, period);
    module.joint_pivot_.setCommand(module.pid_pivot_.getCurrentCmd());
  }
  // Effort limit
  double scale = getEffortLimitScale();
  for (auto &module:modules_)
    module.joint_wheel_.setCommand(scale * module.pid_wheel_.getCurrentCmd());
}

}
