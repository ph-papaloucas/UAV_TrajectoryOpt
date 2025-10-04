#include "models.hpp"

models::StabilityDerivatives
models::StabilityDerivatives::from_yaml_node(const YAML::Node &node) {
  models::StabilityDerivatives model;

  // geometry
  model.S = node["S"].as<double>();
  model.b = node["b"].as<double>();
  model.c = node["c"].as<double>();

  // Lift coefficients
  model.C_L0 = node["C_L0"].as<double>();
  model.C_L_alpha = node["C_L_alpha"].as<double>();
  model.C_L_q = node["C_L_q"].as<double>();
  model.C_L_delta_e = node["C_L_delta_e"].as<double>();

  // Drag coefficients
  model.C_D0 = node["C_D0"].as<double>();
  model.C_D_alpha = node["C_D_alpha"].as<double>();
  model.C_D_q = node["C_D_q"].as<double>();
  model.C_D_delta_e = node["C_D_delta_e"].as<double>();

  // Side force coefficients
  model.C_Y0 = node["C_Y0"].as<double>();
  model.C_Y_beta = node["C_Y_beta"].as<double>();
  model.C_Y_p = node["C_Y_p"].as<double>();
  model.C_Y_r = node["C_Y_r"].as<double>();
  model.C_Y_delta_a = node["C_Y_delta_a"].as<double>();
  model.C_Y_delta_r = node["C_Y_delta_r"].as<double>();

  // Roll moment coefficients
  model.C_l0 = node["C_l0"].as<double>();
  model.C_l_beta = node["C_l_beta"].as<double>();
  model.C_l_p = node["C_l_p"].as<double>();
  model.C_l_r = node["C_l_r"].as<double>();
  model.C_l_delta_a = node["C_l_delta_a"].as<double>();
  model.C_l_delta_r = node["C_l_delta_r"].as<double>();

  // Pitch moment coefficients
  model.C_m0 = node["C_m0"].as<double>();
  model.C_m_alpha = node["C_m_alpha"].as<double>();
  model.C_m_q = node["C_m_q"].as<double>();
  model.C_m_delta_e = node["C_m_delta_e"].as<double>();

  // Yaw moment coefficients
  model.C_n0 = node["C_n0"].as<double>();
  model.C_n_beta = node["C_n_beta"].as<double>();
  model.C_n_p = node["C_n_p"].as<double>();
  model.C_n_r = node["C_n_r"].as<double>();
  model.C_n_delta_a = node["C_n_delta_a"].as<double>();
  model.C_n_delta_r = node["C_n_delta_r"].as<double>();

  return model;
}
