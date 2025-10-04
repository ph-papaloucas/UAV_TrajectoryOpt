#pragma once

#include <array>
#include <fstream>
#include <iostream>
#include <models.hpp>
#include <optional>
#include <stdexcept>
#include <yaml-cpp/yaml.h>



// // forward declaration of relevant models
// namespace solvers {
// namespace stability_derivatives {
// class Model; // just a declaration, no definition
// } // namespace stability_derivatives
// } // namespace solvers

class FixedWing {
public:
  // Add const correctness
  double get_mass() const { return mass_; }
  std::array<double, 6> get_I() const { return I_; }

  static FixedWing from_yaml(const std::string &filepath) {
    YAML::Node config = YAML::LoadFile(filepath);

    FixedWing fw;
    // Read mass and inertia from YAML
    if (config["mass_model"]) {
      const auto &mass_model = config["mass_model"];
      if (mass_model[0]["mass"]) {
      fw.mass_ = mass_model[0]["mass"].as<double>();
      } else {
      throw std::runtime_error("Missing 'mass' in mass_model");
      }
      if (mass_model[1]["I"]) {
      auto I_vec = mass_model[1]["I"];
      if (!I_vec.IsSequence() || I_vec.size() != 6) {
        throw std::runtime_error("'I' must be a sequence of 6 elements");
      }
      for (size_t i = 0; i < 6; ++i) {
        fw.I_[i] = I_vec[i].as<double>();
      }
      } else {
      throw std::runtime_error("Missing 'I' in mass_model");
      }
    } else {
      throw std::runtime_error("Missing 'mass_model' in YAML file");
    }

    // Read stability derivatives if present
    if (config["stability_derivatives"]) {
      fw.model_stability_derivatives_ =
          models::StabilityDerivatives::from_yaml_node(
              config["stability_derivatives"]);
    }

    return fw;
  }

  inline const models::StabilityDerivatives &
  get_model_stability_derivatives() const {
    if (!model_stability_derivatives_) {
      throw std::runtime_error("Stability derivatives model not loaded");
    }
    return model_stability_derivatives_.value();
  }

  // get model from using ModelType as template param
  template <typename ModelType> const ModelType &get_model() const { // default case: throw error if we didnt implement the template specialization
    static_assert(sizeof(ModelType) == 0, 
                  "get_model<Model>: No specialization for this Model type.");
  }

private:
  std::optional<models::StabilityDerivatives> model_stability_derivatives_;
  // std::optional<solvers::stability_derivatives::Model>
  //   model_stability_derivatives_;
  double mass_;
  std::array<double, 6> I_; // Inertia: {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}
};

// Template specialization for StabilityDerivatives model
template <>
inline const models::StabilityDerivatives &
FixedWing::get_model<models::StabilityDerivatives>() const {
  return get_model_stability_derivatives();
}
