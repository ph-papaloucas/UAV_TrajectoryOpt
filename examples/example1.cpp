#include <iostream>

#include "FixedWing.hpp"
#include "solvers.hpp"
#include "Simulator.hpp"

int main() {

  FixedWing uav = FixedWing::from_yaml(
      "../Data/plane1.yaml");

  //   Simulator sim(uav1, solver);

  //   StabilityDerivatives derivs =
  //       StabilityDerivatives::from_yaml("/home/lupin/phili/UAV_TrajectoryOpt/Data/plane1der.yaml",
  //       uav);

  std::array<double, 12> state = {0, 0, 0, 15, 0., -5, 0, 0, 0, 0, 0, 0};
  std::array<double, 4> control = {0, 0, 0, 0};
  std::array<double, 6> ForcesMoments = solvers::stability_derivatives(
      uav.get_model_stability_derivatives(), state, control);

  for (int ii = 0; ii < 6; ++ii) {
    std::cout << ForcesMoments[ii] << std::endl;
  }

  return 0;
}