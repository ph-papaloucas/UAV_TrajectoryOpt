#pragma once

#include "utils.hpp"
#include <array>
#include <yaml-cpp/yaml.h>

namespace models {

class StabilityDerivatives {
public:
  double C_L0, C_L_alpha, C_L_q, C_L_delta_e;
  double C_D0, C_D_alpha, C_D_q, C_D_delta_e;
  double C_Y0, C_Y_beta, C_Y_p, C_Y_r, C_Y_delta_a, C_Y_delta_r;

  // Moment coefficients
  double C_l0, C_l_beta, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r;
  double C_m0, C_m_alpha, C_m_q, C_m_delta_e;
  double C_n0, C_n_beta, C_n_p, C_n_r, C_n_delta_a, C_n_delta_r;

  double c, S, b;

  static models::StabilityDerivatives from_yaml_node(const YAML::Node &node);
};

} // namespace models