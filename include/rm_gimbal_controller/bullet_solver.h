//
// Created by qiayuan on 8/14/20.
//

#ifndef SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
#define SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <rm_gimbal_controllers/BulletSolverConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/eigen_types.h>
#include <rm_common/ros_utilities.h>

namespace bullet_solver {
struct Config {
  double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
      resistance_coff_qd_30, g, delay, dt, timeout;
};

class BulletSolver {
 public:
  explicit BulletSolver(ros::NodeHandle &controller_nh);

  bool solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed);
  double getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel,
                        double yaw_real, double pitch_real, double bullet_speed);
  double getResistanceCoefficient(double bullet_speed) const;
  double getYaw() const { return output_yaw_; }
  double getPitch() const { return output_pitch_; }
  void bulletModelPub(const geometry_msgs::TransformStamped &map2pitch, const ros::Time &time);
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig &config, uint32_t);
  ~BulletSolver() = default;

 private:
  ros::Time last_publish_time_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_real_pub_;
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig> *d_srv_{};
  Config config_{};
  bool dynamic_reconfig_initialized_{};
  double publish_rate_{};
  double output_yaw_{}, output_pitch_{};
  double bullet_speed_{}, resistance_coff_{};

  geometry_msgs::Point target_pos_{};
  visualization_msgs::Marker marker_desire_;
  visualization_msgs::Marker marker_real_;
};
}
#endif //SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
