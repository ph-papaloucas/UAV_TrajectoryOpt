#pragma once

#include "FixedWing.hpp"
#include "SimulationResult.hpp"
#include "solvers.hpp"
#include <array>

template <typename FSolver>
requires AerodynamicSolver<FSolver>
class Simulator {
public:
  using ModelType =
      typename solver_traits<FSolver>::model_type; // deduces model
                                                   // type fro msolver
  using State = std::array<double, 12>;  // [px,py,pz, u,v,w, φ,θ,ψ, p,q,r]
  using Control = std::array<double, 4>; // [δe, δa, δr, throttle]

  struct Config {
    double dt = 0.01;
    double max_time = 10.0;
    bool save_history = true;
  };

  Simulator(const FixedWing &uav, const Config &config = {},
            FSolver force_solver = solvers::stability_derivatives)
      : uav_(uav), force_solver_(force_solver), config_(config),
        model_(uav.get_model<ModelType>()) {}

  void run(const State &initial_state, const Control &control,
           bool open_loop = true);

private:
  const FixedWing &uav_;
  FSolver force_solver_; // Store by value (functions are cheap to copy)
  const ModelType
      &model_; // Type deduced from uav_.get_model<Solver>() (multiple
               // model types encapsulated in different solver namespaces)
  Config config_;
};

#include "integration_methods.hpp"
#include "rigit_body_eom.hpp"

template <typename FSolver>
Simulator(const FixedWing &, const typename Simulator<FSolver>::Config &,
          FSolver) -> Simulator<FSolver>;

template <typename FSolver>
void Simulator<FSolver>::run(const State &initial_state, const Control &control,
                             bool open_loop) {
  double mass = uav_.get_mass();
  std::array<double, 6> inertia = uav_.get_I();
  State x = initial_state;

  if (open_loop == false) {
    throw std::runtime_error("Closed-loop control not implemented yet");
  }

  // dyn_func: (state, control) -> state_derivative
  ForcesMoments cached_fm =
      force_solver_(model_, x, control); // Convert controls to force
  cached_fm[0] +=
      control[3] - mass * 9.81 * sin(x[7]); // Add thrust in X direction
  cached_fm[2] += mass * 9.81 * cos(x[7]);  // Add weight in Z direction

  auto dyn_func = [this, mass, inertia,
                   &cached_fm](const State &x, const Control &u,
                               const auto & /*unused*/) -> State {
    cached_fm = force_solver_(model_, x, u); // Convert controls to forces
    cached_fm[0] += u[3] - mass * 9.81 * sin(x[7]); // Add thrust in X direction
    cached_fm[2] += mass * 9.81 * cos(x[7]);        // Add weight in Z direction

    return rigid_body_eom(x, cached_fm, mass, inertia); // Apply physics
  };

  // input_func: (state, time) -> control
  auto input_func = [&control](const State &x, double t) -> Control {
    return control; // Constant open-loop control
  };

  double dt = 0.05;
  double t = 0.0; // Should start at 0

  SimulationResult sim_result;
  sim_result.add_snapshot(t, x, control,
                          cached_fm); // initial state
  // Integration loop
  constexpr size_t neq = State{}.size();
  constexpr size_t nin = Control{}.size();
  constexpr size_t nargs = 0; // no extra args for dyn_func
  while (t < config_.max_time) {
    x = integration_methods::stepRK4<decltype(dyn_func), decltype(input_func),
                                     neq, nin, nargs>(
        dyn_func, input_func, t, dt, x, std::array<double, 0>{});
    t += dt;

    if (config_.save_history) {
      sim_result.add_snapshot(t, x, control, cached_fm);
    }
  }
  sim_result.save_to_csv("simulation_output.csv");
}