#pragma once
#include <array>
#include <iostream>

namespace integration_methods {

// neq = number of equations = number of states
// nin = number of inputs
// nargs = number of other arguments

template <typename DynFunc, typename InputFunc, std::size_t neq,
          std::size_t nin, std::size_t nargs>
concept DynamicSystem = requires(DynFunc dyn_func, InputFunc input_func,
                                 const std::array<double, neq> &x, double t,
                                 const std::array<double, nargs> &args) {
  // InputFunc must produce correct input size
  { input_func(x, t) } -> std::same_as<std::array<double, nin>>;
  // DynFunc must consume that input size and produce state derivative
  {
    dyn_func(x, input_func(x, t), args)
    } -> std::same_as<std::array<double, neq>>;
};

/* example of how you should define a function that can be passed to RK4step:*/
// std::array<double, 4> eom(const std::array<double, 4>& x, const
// std::array<double, 2>& u, const std::array<double, 1>& args){

//     std::array<double, 4> xdot = {};
//     xdot[0] = x[2];
// 	xdot[1] = x[3];
// 	xdot[2] = u[0] / args[0];
// 	xdot[3] = u[1] / args[0];

//     return xdot;
// }

// template <std::size_t neq, std::size_t nin, std::size_t nargs>
// double RK4step(DynFunc<neq, nin, nargs> func) {
//     std::cout << "RK4 step executed" << std::endl;
//     return 0.0; // Replace with actual RK4 logic
// }

template <typename DynFunc, typename InputFunc, std::size_t neq,
          std::size_t nin, std::size_t nargs>
requires DynamicSystem<DynFunc, InputFunc, neq, nin, nargs>
    std::array<double, neq> stepRK4(DynFunc dyn_func, InputFunc input_func,
                                    const double t, const double dt,
                                    const std::array<double, neq> &x_prev,
                                    const std::array<double, nargs> &args) {
  // RK4 coeffs
  static const std::array<double, 4> RK4 = {1.0 / 6.0, 2.0 / 6.0, 2.0 / 6.0,
                                            1.0 / 6.0};
  static const std::array<double, 4> RK4_time = {0., 0.5, 0.5, 1.0};

  std::array<std::array<double, neq>, 4> K = {};

  // at each timestep, forward iterations
  std::array<double, neq> xdot = {};
  std::array<double, nin> u_temp = {};

  std::array<double, neq> x_temp = x_prev;

  int step = 0;
  u_temp = input_func(x_temp, t);
  xdot = dyn_func(x_temp, u_temp, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + 0.5 * K[step][ieq];
  }

  step = 1;
  u_temp = input_func(x_temp, t + 0.5 * dt);
  xdot = dyn_func(x_temp, u_temp, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + 0.5 * K[step][ieq];
  }
  step = 2; // t0 -> t0 + dt with xdot = f(t0 + dt/2, x_step1)
  u_temp = input_func(x_temp, t + 0.5 * dt);
  xdot = dyn_func(x_temp, u_temp, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + K[step][ieq];
  }

  step = 3; // t0 -> t0 + dt with xdot = f(t0 + dt, x_step2)
  u_temp = input_func(x_temp, t + dt);
  xdot = dyn_func(x_temp, u_temp, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + RK4[0] * K[0][ieq] + RK4[1] * K[1][ieq] +
                  RK4[2] * K[2][ieq] + RK4[3] * K[3][ieq]; // update state
  }

  return x_temp;
}

} // namespace integration_methods