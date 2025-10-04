#pragma once

#include <array>
#include <cmath>
#include <iostream>
using State = std::array<double, 12>;  // [px,py,pz, u,v,w, φ,θ,ψ, p,q,r]
using Control = std::array<double, 4>; // [delta_e, delta_a, delta_r, throttle]
using ForcesMoments = std::array<double, 6>; // [Fx Fy Fz L M N] body
using Forces = std::array<double, 3>;
using Moments = std::array<double, 3>;

namespace utils {

inline double rho = 1.225;

// Aerodynamic quantities
inline double airspeed(const State &x) {
  return std::sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
}

inline double alpha(const State &x) {
  return std::atan2(x[5], x[3]); // atan2(w, u)
}

inline double beta(const State &x) {
  double V = airspeed(x);
  return std::asin(x[4] / V); // asin(v/V)
}

// Attitude helpers
inline double phi(const State &x) { return x[6]; }
inline double theta(const State &x) { return x[7]; }
inline double psi(const State &x) { return x[8]; }

// Rate helpers
inline double p(const State &x) { return x[9]; }
inline double q(const State &x) { return x[10]; }
inline double r(const State &x) { return x[11]; }

inline double delta_e(const Control &c) { return c[0]; }
inline double delta_a(const Control &c) { return c[1]; }
inline double delta_r(const Control &c) { return c[2]; }
inline double throttle(const Control &c) { return c[3]; }

inline std::array<double, 6>
transform_body_to_coeffs(const std::array<double, 6> &forces_moments,
                         const State &state, double c, double b, double S) {
  std::array<double, 6> coeffs;

  double V = airspeed(state);
  if (V == 0.) {
    return {0., 0., 0., 0., 0., 0.};
  }

  double q_inf = 0.5 * rho * V * V;

  // Extract forces and moments
  double F_x = forces_moments[0];
  double F_y = forces_moments[1];
  double F_z = forces_moments[2];
  double L_moment = forces_moments[3];
  double M_moment = forces_moments[4];
  double N_moment = forces_moments[5];

  // Transform body forces to aerodynamic coefficients
  double alpha = utils::alpha(state);
  double beta = utils::beta(state);

  double cos_a = cos(alpha), sin_a = sin(alpha);
  double cos_b = cos(beta), sin_b = sin(beta);

  // Inverse of your body force transformation
  double L = +F_x * sin_a + F_z * cos_a;
  double D = -F_x * cos_a * cos_b - F_y * sin_b - F_z * sin_a * cos_b;
  double Y = F_x * cos_a * sin_b - F_y * cos_b + F_z * sin_a * sin_b;

  // Convert to coefficients
  coeffs[0] = L / (q_inf * S);
  coeffs[1] = D / (q_inf * S);
  coeffs[2] = Y / (q_inf * S);
  coeffs[3] = L_moment / (q_inf * S * b);
  coeffs[4] = M_moment / (q_inf * S * c);
  coeffs[5] = N_moment / (q_inf * S * b);

  return coeffs;
}

inline std::array<double, 6>
transform_coeffs_to_body(const std::array<double, 6> &coeffs,
                         const State &state, double c, double b, double S) {
  std::array<double, 6> forces_moments;

  double V = airspeed(state);
  if (V == 0.) {
    return {0., 0., 0., 0., 0., 0.};
  }

  double q_inf = 0.5 * rho * V * V;

  // Extract coefficients - using same order as your transform_body_to_coeffs
  double cL = coeffs[0];  // Lift coefficient
  double cD = coeffs[1];  // Drag coefficient
  double cY = coeffs[2];  // Side force coefficient
  double c_l = coeffs[3]; // Roll moment coefficient
  double c_m = coeffs[4]; // Pitch moment coefficient
  double c_n = coeffs[5]; // Yaw moment coefficient

  // Convert coefficients to forces and moments in wind axes
  double L = cL * q_inf * S;
  double D = cD * q_inf * S;
  double Y = cY * q_inf * S;
  double L_moment = c_l * q_inf * S * b;
  double M_moment = c_m * q_inf * S * c;
  double N_moment = c_n * q_inf * S * b;

  // Transform wind axes forces to body axes (your original transformation)
  double alpha = utils::alpha(state);
  double beta = utils::beta(state);

  double cos_a = cos(alpha), sin_a = sin(alpha);
  double cos_b = cos(beta), sin_b = sin(beta);

  // This is the exact inverse of your body force transformation
  double F_x = -D * cos_a * cos_b + Y * cos_a * sin_b + L * sin_a;
  double F_y = -D * sin_b + Y * cos_b;
  double F_z = -D * sin_a * cos_b + Y * sin_a * sin_b - L * cos_a;

  // Package forces and moments
  forces_moments[0] = F_x;
  forces_moments[1] = F_y;
  forces_moments[2] = F_z;
  forces_moments[3] = L_moment;
  forces_moments[4] = M_moment;
  forces_moments[5] = N_moment;

  return forces_moments;
}

}; // namespace utils
