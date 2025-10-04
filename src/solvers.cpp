#include "solvers.hpp"
#include "utils.hpp"
#include <iostream>

ForcesMoments
solvers::stability_derivatives(const models::StabilityDerivatives &model,
                               const State &x, const Control &control) {

  double V = utils::airspeed(x);

  if (V == 0.) {
    return {0., 0., 0., 0., 0., 0.};
  }

  std::array<double, 6> ForcesMoments = utils::transform_coeffs_to_body(
      solvers::stability_derivatives_helper::compute_aero_coeffs(model, x,
                                                                 control),
      x, model.c, model.b, model.S);

  return ForcesMoments;
}

std::array<double, 6>
solvers::stability_derivatives_helper::compute_aero_coeffs(
    const models::StabilityDerivatives &model, const State &x,
    const Control &control) {
  double V = utils::airspeed(x);
  double alpha = utils::alpha(x);
  double beta = utils::beta(x);
  double c = model.c;
  double S = model.S;
  double b = model.b;

  double q_inf = 0.5 * utils::rho * V * V;

  // Forces

  double cL = model.C_L0 + model.C_L_alpha * alpha +
              0.5 * model.C_L_q * c * utils::q(x) / V +
              model.C_L_delta_e * utils::delta_e(control);

  double cD = model.C_D0 + model.C_D_alpha * alpha +
              0.5 * model.C_D_q * c * utils::q(x) / V +
              model.C_D_delta_e * utils::delta_e(control);

  double cY = model.C_Y0 + model.C_Y_beta * beta +
              0.5 * model.C_Y_p * b * utils::p(x) / V +
              0.5 * model.C_Y_r * b * utils::r(x) / V +
              model.C_Y_delta_a * utils::delta_a(control) +
              model.C_Y_delta_r * utils::delta_r(control);

  // Moments

  double c_l = model.C_l0 + model.C_l_beta * beta +
               0.5 * model.C_l_p * b * utils::p(x) / V +
               0.5 * model.C_l_r * b * utils::r(x) / V +
               model.C_l_delta_a * utils::delta_a(control) +
               model.C_l_delta_r * utils::delta_r(control);

  double c_m = model.C_m0 + model.C_m_alpha * alpha +
               0.5 * model.C_m_q * c * utils::q(x) / V +
               model.C_m_delta_e * utils::delta_e(control);

  double c_n = model.C_n0 + model.C_n_beta * beta +
               0.5 * model.C_n_p * b * utils::p(x) / V +
               0.5 * model.C_n_r * b * utils::r(x) / V +
               model.C_n_delta_a * utils::delta_a(control) +
               model.C_n_delta_r * utils::delta_r(control);

  return {cL, cD, cY, c_l, c_m, c_n};
}