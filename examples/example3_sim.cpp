#include "FixedWing.hpp"
#include "Simulator.hpp"
#include "solvers.hpp"
#include <cmath>
#include <iostream>
#include <tuple>

std::tuple<std::array<double, 4>, std::array<double, 12>>
find_trim_conditions(const FixedWing &uav, Control control) {
  const auto &model = uav.get_model_stability_derivatives();
  double mass = uav.get_mass();
  double g = 9.81;
  double rho = utils::rho;

  // trim angle of attack
  // Cm = Cm_0 + Cm_alpha*alpha + Cm_delta_e*delta_e + cm_q*(c/(2V))*q = 0

  double alpha = -(model.C_m0 + model.C_m_delta_e * control[0]) /
                 model.C_m_alpha; // trim alpha

  // trim cl, cd
  double cL = model.C_L0 + model.C_L_alpha * alpha +
              model.C_L_delta_e * utils::delta_e(control);
  double cD = model.C_D0 + model.C_D_alpha * alpha +
              model.C_D_delta_e * utils::delta_e(control);

  // Keep theta separate in case you later allow gamma != 0. For level flight:
  double theta = alpha;

  // ---- Z body balance ----
  // L*cos(alpha) + D*sin(alpha) - W*cos(theta) = 0
  // ==> V from Z-balance:
  double velocity = std::sqrt(
      (2.0 * mass * g * std::cos(theta)) /
      (rho * model.S * (cL * std::cos(alpha) + cD * std::sin(alpha))));

  // Dynamic pressure & forces
  double q = 0.5 * rho * velocity * velocity;
  double lift = cL * q * model.S;
  double drag = cD * q * model.S;

  // ---- X body balance ----
  // T - D*cos(alpha) + L*sin(alpha) - W*sin(theta) = 0
  double thrust = mass * g * std::sin(theta) + drag * std::cos(alpha) -
                  lift * std::sin(alpha);

  State trim_state = {0,
                      0,
                      -100, // position NED
                      velocity * std::cos(alpha),
                      0,
                      velocity * std::sin(alpha), // velocity body
                      0,
                      alpha,
                      0, // orientation euler
                      0,
                      0,
                      0}; // angular rates body

  ForcesMoments aero_forces = solvers::stability_derivatives(
      uav.get_model_stability_derivatives(), trim_state, control);

  double thrust2 = mass * g * std::sin(theta) + aero_forces[0];
  std::cout << "Initial guess for thrust: " << thrust
            << ", from aero forces: " << thrust2 << std::endl;
  double alpha2 = utils::alpha(trim_state);
  std::cout << "Initial guess for alpha: " << alpha
            << ", from state: " << alpha2 << std::endl;

  double Xb_from_CLCD = -drag * std::cos(alpha) + lift * std::sin(alpha);
  double Xb_from_solver = aero_forces[0];

  ForcesMoments coeffs = solvers::stability_derivatives_helper::compute_aero_coeffs(
      uav.get_model_stability_derivatives(), trim_state, control);
  std::cout << "hand computed coefficients: \n";
  std::cout << "  cL = " << cL << "\n";
  std::cout << "  cD = " << cD << std::endl;
  std::cout << "Computed coefficients: \n";
  std::cout << "  cL = " << coeffs[0] << "\n";
  std::cout << "  cD = " << coeffs[1] << std::endl;

  std::cout << "Xb_from_CLCD = " << Xb_from_CLCD << "\n";
  std::cout << "Xb_from_solver = " << Xb_from_solver << "\n";
  control[3] = thrust;

  // Final elevator value is already in trim_control[0] from the loop
  return std::make_tuple(control, trim_state);
}
int main() {
  FixedWing uav = FixedWing::from_yaml(
      "../Data/plane1.yaml");

  // Controls to find trim for
  Control control = {0, 0, 0,
                     0}; // [δe, δa, δr, throttle] #throttle wiwll change

  // Find trim conditions for 25 m/s flight
  std::cout << "Finding trim conditions for Controls:...\n c0" << control[0]
            << "\nc1" << control[1] << "\nc2" << control[2] << std::endl;
  auto [trim_control, trim_state] = find_trim_conditions(uav, control);

  // Verify trim conditions
  std::cout << "\nVerifying trim conditions..." << std::endl;
  std::array<double, 6> fm = solvers::stability_derivatives(
      uav.get_model_stability_derivatives(), trim_state, trim_control);

  std::cout << "Forces: X=" << fm[0] << " Y=" << fm[1] << " Z=" << fm[2]
            << std::endl;
  std::cout << "Moments: L=" << fm[3] << " M=" << fm[4] << " N=" << fm[5]
            << std::endl;

  std::cout << "Trim control inputs:" << std::endl;
  std::cout << "  Elevator: " << trim_control[0] << " rad" << std::endl;
  std::cout << "  Aileron: " << trim_control[1] << " rad" << std::endl;
  std::cout << "  Rudder: " << trim_control[2] << " rad" << std::endl;
  std::cout << "  Throttle: " << trim_control[3] << std::endl;

  std::cout << "Trim State:" << std::endl;
  std::cout << "  Position (NED): [" << trim_state[0] << ", " << trim_state[1]
            << ", " << trim_state[2] << "] m" << std::endl;
  std::cout << "  Velocity (body): [" << trim_state[3] << ", " << trim_state[4]
            << ", " << trim_state[5] << "] m/s" << std::endl;
  std::cout << "  Orientation (Euler): [" << trim_state[6] << ", "
            << trim_state[7] << ", " << trim_state[8] << "] rad" << std::endl;
  std::cout << "  Angular rates (body): [" << trim_state[9] << ", "
            << trim_state[10] << ", " << trim_state[11] << "] rad/s"
            << std::endl;

  std::cout << "aero forces" << std::endl;

  ForcesMoments aero_forces = solvers::stability_derivatives(
      uav.get_model_stability_derivatives(), trim_state, trim_control);
  std::cout << "aero forces" << std::endl;
  std::cout << "  Fx: " << aero_forces[0] << " N" << std::endl;
  std::cout << "  Fy: " << aero_forces[1] << " N" << std::endl;
  std::cout << "  Fz: " << aero_forces[2] << " N" << std::endl;
  std::cout << "  L: " << aero_forces[3] << " Nm" << std::endl;
  std::cout << "  M: " << aero_forces[4] << " Nm" << std::endl;
  std::cout << "  N: " << aero_forces[5] << " Nm" << std::endl;

  ForcesMoments total_forces = aero_forces;

  // Add thrust (assuming thrust along +X body)
  total_forces[0] += trim_control[3] - uav.get_mass() * 9.81 * std::sin(trim_state[7]);
  total_forces[2] += uav.get_mass() * 9.81 * std::cos(trim_state[7]);

  std::cout << "total forces" << std::endl;
  std::cout << "  Fx: " << total_forces[0] << " N" << std::endl;
  std::cout << "  Fy: " << total_forces[1] << " N" << std::endl;
  std::cout << "  Fz: " << total_forces[2] << " N" << std::endl;
  std::cout << "  L: " << total_forces[3] << " Nm" << std::endl;
  std::cout << "  M: " << total_forces[4] << " Nm" << std::endl;
  std::cout << "  N: " << total_forces[5] << " Nm" << std::endl;

  // Create and run simulator
  Simulator sim(uav, {}, solvers::stability_derivatives);

  std::cout << "\nStarting simulation with trim conditions..." << std::endl;
  sim.run(trim_state, trim_control, true); // true for open-loop

  return 0;
}