//
// Created by qiayuan on 8/14/20.
//

#ifndef SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
#define SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_

#include <rm_msgs/GimbalTrackCmd.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <rm_gimbal_controllers/GimbalConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rm_base/hardware_interface/robot_state_interface.h>
#include "cpp_types.h"

class BulletSolver {
 public:
  explicit BulletSolver(ros::NodeHandle &nh) {

    world2gimbal_des_.header.frame_id = "world";
    world2gimbal_des_.child_frame_id = "gimbal_des";
    nh.param("publish_rate", publish_rate_, 50.0);
    d_srv_ =
        new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>(nh);
    dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>::CallbackType
        cb = boost::bind(&BulletSolver::reconfigCB, this, _1, _2);
    d_srv_->setCallback(cb);

    path_pub_ = nh.advertise<visualization_msgs::Marker>("bullet_model", 10);
  };
  virtual ~BulletSolver() = default;
  virtual void setTarget(const DVec<double> &pos, const DVec<double> &vel) = 0;
  virtual void setBulletSpeed(double speed) { bullet_speed_ = speed; };
  virtual void reconfigCB(const rm_gimbal_controllers::GimbalConfig &config, uint32_t level) {
    ROS_INFO("[Gimbal] Dynamic params change");
    (void) level;
    resistance_coff_ = config.resistance_coff;
    g_ = config.g;
    delay_ = config.delay;
    dt_ = config.dt;
    timeout_ = config.timeout;
  };
  virtual bool solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped world2pitch,
                     realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) = 0;
  virtual void modelRviz(double x_offset, double y_offset, double z_offset) = 0;

 protected:
  double bullet_speed_{};
  double resistance_coff_{}, g_{}, delay_{}, dt_{}, timeout_{};
  ros::Publisher path_pub_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig> *d_srv_{};
  geometry_msgs::TransformStamped world2gimbal_des_;
  double publish_rate_{};
  ros::Time last_publish_time_;
};

class Bullet2DSolver : public BulletSolver {
 public:
  using BulletSolver::BulletSolver;
  void setTarget(const DVec<double> &pos, const DVec<double> &vel) override {
    target_x_ = pos[0] + vel[0] * this->delay_;
    target_z_ = pos[1] + vel[1] * this->delay_;
    target_dx_ = vel[0];
    target_dz_ = vel[1];
  };
  bool solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped world2pitch,
             realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) override;
 protected:
  virtual double computeError(double pitch) = 0;
  double target_x_{}, target_z_{}, target_dx_{}, target_dz_{};
  double fly_time_{};
  double pitch_solved_;
};

class Iter2DSolver : public Bullet2DSolver {
 public:
  using Bullet2DSolver::Bullet2DSolver;
  using Bullet2DSolver::solve;
 private:
  double computeError(double pitch) override;
};

class Approx2DSolver : public Bullet2DSolver {
 public:
  using Bullet2DSolver::Bullet2DSolver;
  using Bullet2DSolver::solve;
 private:
  double computeError(double pitch) override;
};

class Bullet3DSolver : public BulletSolver {
 public:
  using BulletSolver::BulletSolver;
  void setTarget(const DVec<double> &pos, const DVec<double> &vel) override {
    target_x_ = pos[0] + vel[0] * this->delay_;
    target_y_ = pos[1] + vel[1] * this->delay_;
    target_z_ = pos[2] + vel[2] * this->delay_;
    target_dx_ = vel[0];
    target_dy_ = vel[1];
    target_dz_ = vel[2];
  };
  bool solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped world2pitch,
             realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) override;
  void modelRviz(double x_offset, double y_offset, double z_offset) override;
  geometry_msgs::TransformStamped getResult(const ros::Time &time);
  std::vector<Vec3<double>> getPointData3D();
 protected:
  virtual double computeError(double yaw, double pitch, double *error_polar) = 0;
  double target_x_{}, target_y_{}, target_z_{},
      target_dx_{}, target_dy_{}, target_dz_{};
  double fly_time_{};
  double pitch_solved_, yaw_solved_;
  Vec3<double> pos_{};
  Vec3<double> vel_{};
  std::vector<Vec3<double>> model_data_;
  double world2pitch_offset_x_{};
  double world2pitch_offset_y_{};
  double world2pitch_offset_z_{};
};

class Iter3DSolver : public Bullet3DSolver {
 public:
  using Bullet3DSolver::Bullet3DSolver;
  using Bullet3DSolver::solve;
  using Bullet3DSolver::getPointData3D;
  using Bullet3DSolver::getResult;
 private:
  double computeError(double yaw, double pitch, double *error_polar) override;
};

class Approx3DSolver : public Bullet3DSolver {
 public:
  using Bullet3DSolver::Bullet3DSolver;
  using Bullet3DSolver::solve;
  using Bullet3DSolver::getPointData3D;
  using Bullet3DSolver::getResult;
 private:
  double computeError(double yaw, double pitch, double *error_polar) override;
};
#endif //SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
