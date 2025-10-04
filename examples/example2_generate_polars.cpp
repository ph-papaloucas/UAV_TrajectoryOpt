#include <fstream>
#include <iostream>

#include "FixedWing.hpp"
//#include "Simulator.hpp"
#include "solvers.hpp"

int main() {
  FixedWing uav = FixedWing::from_yaml(
      "../Data/plane1.yaml");

  // Open CSV file for writing
  std::ofstream csv_file("aerodynamic_coeffs.csv");

  // Write CSV header
  csv_file << "alpha,beta,elevator,aileron,rudder,cL,cD,cY,c_l,c_m,c_n\n";

  double V = 20;
  std::array<double, 10> alpha;
  std::array<double, 10> beta;
  std::array<double, 10> elevator;
  std::array<double, 10> aileron;
  std::array<double, 10> rudder;

  // Initialize test values
  for (int i = 0; i < 10; i++) {
    alpha[i] = -0.1 + i * 0.02;
    beta[i] = -0.1 + i * 0.02;
    elevator[i] = -0.1 + i * 0.02;
    aileron[i] = -0.1 + i * 0.02;
    rudder[i] = -0.1 + i * 0.02;
  }

  // Test all combinations
  for (int i_alpha = 0; i_alpha < 10; ++i_alpha) {
    for (int i_beta = 0; i_beta < 10; ++i_beta) {
      for (int i_elev = 0; i_elev < 10; ++i_elev) {
        for (int i_ail = 0; i_ail < 10; ++i_ail) {
          for (int i_rud = 0; i_rud < 10; ++i_rud) {

            // Set up state with current alpha and beta
            double u = V * cos(alpha[i_alpha]) * cos(beta[i_beta]);
            double v = V * sin(beta[i_beta]);
            double w = V * sin(alpha[i_alpha]) * cos(beta[i_beta]);

            std::array<double, 12> state = {0, 0, 0, u, v, w, 0, 0, 0, 0, 0, 0};

            // Set up control with current control surface deflections
            std::array<double, 4> control = {
                elevator[i_elev], aileron[i_ail], rudder[i_rud],
                0 // throttle (if needed)
            };

            // Compute coefficients
            std::array<double, 6> coeffs =
                solvers::stability_derivatives_helper::compute_aero_coeffs(
                    uav.get_model_stability_derivatives(), state, control);

            // Write to CSV
            csv_file << alpha[i_alpha] << "," << beta[i_beta] << ","
                     << elevator[i_elev] << "," << aileron[i_ail] << ","
                     << rudder[i_rud] << "," << coeffs[0] << "," // cL
                     << coeffs[1] << ","                         // cD
                     << coeffs[2] << ","                         // cY
                     << coeffs[3] << ","                         // c_l
                     << coeffs[4] << ","                         // c_m
                     << coeffs[5] << "\n";                       // c_n
          }
        }
      }
    }
  }

  csv_file.close();
  std::cout << "Data written to aerodynamic_coeffs.csv" << std::endl;

  return 0;
}