//
// Created by qiayuan on 8/15/20.
//

#include <iostream>
#include "rm_gimbal_controller/bullet_solver.h"
//
int main(int argc, char **argv) {
  BulletSolver<double> *bullet_solver_;
  Vec3<double> pos_{};
  Vec3<double> vel_{};
  Vec2<double> angle_init_{};
  Vec2<double> angle_solved_{};

  bullet_solver_ = new Approx3DSolver<double>
      (0.1, 9.8, 0, 0.001, 5);

  pos_[0] = 5;
  pos_[1] = 0;
  pos_[2] = 1;
  vel_[0] = 0;
  vel_[1] = 0;
  vel_[2] = 0;

  bullet_solver_->setBulletSpeed(10);
  bullet_solver_->setTarget(pos_, vel_);
  bullet_solver_->solve(angle_init_);
  bullet_solver_->output(angle_solved_);
  std::cout << "yaw:" << angle_solved_[0] << std::endl
            << "pitch:" << angle_solved_[1] << std::endl;
  std::cout << 1 << std::endl;
  return 0;
}
