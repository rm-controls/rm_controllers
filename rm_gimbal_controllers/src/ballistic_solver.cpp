//
// Created by ljyi on 25-9-20.
//

#include "rm_gimbal_controllers/ballistic_solver.h"

namespace rm_gimbal_controllers
{
BallisticSolver::BallisticSolver(ros::NodeHandle& controller_nh)
{
  config_ = { .mass = getParam(controller_nh, "mass", 0.0445),
              .radius = getParam(controller_nh, "radius", 0.02125),
              .gun_offset_x = getParam(controller_nh, "gun_offset_x", 0.2),
              .gun_offset_z = getParam(controller_nh, "gun_offset_z", 0.0),
              .Cd_value = getParam(controller_nh, "Cd_value", 0.4),
              .Cd_distance = getParam(controller_nh, "Cd_distance", 12.0),
              .Cd_slope = getParam(controller_nh, "Cd_slope", 0.0),
              .air_density = getParam(controller_nh, "air_density", 1.1),
              .g = getParam(controller_nh, "g", 9.81),
              .initial_vel = getParam(controller_nh, "initial_vel", 16.5),
              .rk4_simulate_step = getParam(controller_nh, "rk4_simulate_step", 0.01),
              .newton_convergence_tol = getParam(controller_nh, "newton_convergence_tol", 2e-5),
              .newton_pitch_epsilon = getParam(controller_nh, "newton_pitch_epsilon", 1e-4),
              .max_newton_step = getParam(controller_nh, "max_newton_step", 0.04),
              .max_newton_iterations = getParam(controller_nh, "max_newton_iterations", 5) };
  config_rt_buffer_.initRT(config_);
  XmlRpc::XmlRpcValue lut_config;
  if (controller_nh.getParam("output_pitch_match", lut_config))
    output_pitch_match_lut_.init(lut_config);
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BallisticSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BallisticSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);
  base2target_data_sub_ =
      controller_nh.subscribe<geometry_msgs::Point>("/base2target", 1, &BallisticSolver::base2targetDataCB, this);
}

void BallisticSolver::solver(const geometry_msgs::TransformStamped& base2gimbal, double& yaw, double& pitch)
{
  BallisticConfig config = *config_rt_buffer_.readFromRT();
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!base2target_data_valid_)
  {
    pitch = 0.0, yaw = 0.0;
    return;
  }
  geometry_msgs::Vector3 launch2target;
  launch2target.x = base2target_.x - base2gimbal.transform.translation.x - config.gun_offset_x;
  launch2target.y = base2target_.y - base2gimbal.transform.translation.y;
  launch2target.z = base2target_.z - base2gimbal.transform.translation.z - config.gun_offset_z;
  double target_dis = std::sqrt(launch2target.x * launch2target.x + launch2target.y * launch2target.y);
  double target_hgt = launch2target.z;
  // std::cout << "target_dis" << target_dis << std::endl << "target_hgt" << target_hgt << std::endl;
  yaw = std::atan2(launch2target.y, launch2target.x);
  double initial_pitch = -output_pitch_match_lut_.output(target_dis);
  // std::cout << "initial_pitch: " << initial_pitch << std::endl;
  auto error_function = [this, &config, &target_dis, &target_hgt](double pitch_angle) -> double {
    return simulate(pitch_angle, config.initial_vel, target_dis, target_hgt);
  };
  double current_pitch = initial_pitch;
  double error = error_function(current_pitch);
  for (int iter = 0; iter < config.max_newton_iterations; ++iter)
  {
    double error_after_step = error_function(current_pitch + config.newton_pitch_epsilon);
    double error_derivative = (error_after_step - error) / config.newton_pitch_epsilon;
    double pitch_adjustment = error / error_derivative;
    pitch_adjustment = (pitch_adjustment < -config.max_newton_step) ?
                           -config.max_newton_step :
                           (pitch_adjustment > config.max_newton_step) ? config.max_newton_step : pitch_adjustment;
    double update_pitch = current_pitch - pitch_adjustment;
    double update_error = error_function(update_pitch);
    if (std::abs(update_error) < config.newton_convergence_tol)
    {
      pitch = -update_pitch;
      return;
    }
    current_pitch = update_pitch;
    error = update_error;
  }
  pitch = -initial_pitch;
}

double BallisticSolver::simulate(double pitch_angle, double initial_vel, double target_dis, double target_hgt)
{
  BallisticConfig config = *config_rt_buffer_.readFromRT();
  double dt = config.rk4_simulate_step;
  double x_previous = 0.0, z_previous = 0.0, x_current = 0.0, z_current = 0.0;
  std::array<double, 4> state = { { 0.0, 0.0, initial_vel * std::cos(pitch_angle),
                                    initial_vel * std::sin(pitch_angle) } };
  std::array<double, 4> k1{}, k2{}, k3{}, k4{}, temp{};
  auto systemEquation = [&config, &target_dis](const std::array<double, 4>& state,
                                               std::array<double, 4>& stateDerivative) {
    double vx = state[2], vz = state[3];
    double v = std::sqrt(vx * vx + vz * vz);
    stateDerivative[0] = vx;
    stateDerivative[1] = vz;
    double fitting_Cd = config.Cd_value + config.Cd_slope * (target_dis - config.Cd_distance);
    double F_drag = 0.5 * config.air_density * fitting_Cd * M_PI * config.radius * config.radius * v * v;
    double drag_accel = F_drag / config.mass;
    stateDerivative[2] = -drag_accel * vx / v;
    stateDerivative[3] = -config.g - drag_accel * vz / v;
  };
  while (state[0] < target_dis)
  {
    x_previous = x_current;
    z_previous = z_current;
    x_current = state[0];
    z_current = state[1];
    systemEquation(state, k1);
    for (int i = 0; i < 4; ++i)
      temp[i] = state[i] + 0.5 * dt * k1[i];
    systemEquation(temp, k2);
    for (int i = 0; i < 4; ++i)
      temp[i] = state[i] + 0.5 * dt * k2[i];
    systemEquation(temp, k3);
    for (int i = 0; i < 4; ++i)
      temp[i] = state[i] + dt * k3[i];
    systemEquation(temp, k4);
    for (int i = 0; i < 4; ++i)
      state[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
  double z_at_target = z_previous + (z_current - z_previous) * (target_dis - x_previous) / (x_current - x_previous);
  return z_at_target - target_hgt;
}

void BallisticSolver::base2targetDataCB(const geometry_msgs::Point::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  base2target_ = *msg;
  if (sqrt(msg->x * msg->x + msg->y * msg->y) <= 30)
    base2target_data_valid_ = true;
  else
    base2target_data_valid_ = false;
}

void BallisticSolver::reconfigCB(rm_gimbal_controllers::BallisticSolverConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Ballistic Solver] Dynamic params changed");
  if (!dynamic_reconfig_initialized_)
  {
    BallisticConfig init_config = *config_rt_buffer_.readFromNonRT();  // config init from YAML
    config.mass = init_config.mass;
    config.radius = init_config.radius;
    config.Cd_value = init_config.Cd_value;
    config.Cd_distance = init_config.Cd_distance;
    config.Cd_slope = init_config.Cd_slope;
    config.air_density = init_config.air_density;
    config.g = init_config.g;
    config.initial_vel = init_config.initial_vel;
    config.rk4_simulate_step = init_config.rk4_simulate_step;
    config.newton_convergence_tol = init_config.newton_convergence_tol;
    config.newton_pitch_epsilon = init_config.newton_pitch_epsilon;
    config.max_newton_step = init_config.max_newton_step;
    config.max_newton_iterations = init_config.max_newton_iterations;
    dynamic_reconfig_initialized_ = true;
  }
  BallisticConfig config_non_rt{
    .mass = config.mass,
    .radius = config.radius,
    .Cd_value = config.Cd_value,
    .Cd_distance = config.Cd_distance,
    .Cd_slope = config.Cd_slope,
    .air_density = config.air_density,
    .g = config.g,
    .initial_vel = config.initial_vel,
    .rk4_simulate_step = config.rk4_simulate_step,
    .newton_convergence_tol = config.newton_convergence_tol,
    .newton_pitch_epsilon = config.newton_pitch_epsilon,
    .max_newton_step = config.max_newton_step,
    .max_newton_iterations = config.max_newton_iterations,
  };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers
