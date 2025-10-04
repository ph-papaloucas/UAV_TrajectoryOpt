#pragma once

#include "models.hpp"
#include "utils.hpp"
#include <array>
#include <yaml-cpp/yaml.h>

// example: Fsolver == delctype(&solvers::stability_derivatives)

namespace solvers {

ForcesMoments stability_derivatives(const models::StabilityDerivatives &model,
                                    const State &state, const Control &control);

namespace stability_derivatives_helper {
std::array<double, 6>
compute_aero_coeffs(const models::StabilityDerivatives &model,
                    const State &state, const Control &control);
}

} // namespace solvers

//// IMPLEMENT SOLVER TRAITS SO WE MATCH SOLVER WITH MODEL
////USES:
/// 1. deduce model from solver type:
/// using ModelType = typename solver_traits<FSolver>::model_type;
/// 2. abstractly call model class constructor
/// model_ = solver_traits<FSolver>::model_type();

// --- Default solver_traits ---
template <typename Solver>
struct solver_traits { // force static assert failure if no specialization
  static_assert(sizeof(Solver) == 0,
                "solver_traits<Solver> not specialized. "
                "Add a specialization mapping this solver to its model_type.");
};

// --- Specialization for stability_derivatives ---
template <>
struct solver_traits<
    decltype(&solvers::stability_derivatives)> { // comparing function signature
  using model_type =
      models::StabilityDerivatives; // type in the scope of the struct
};

template <typename FSolver>
concept AerodynamicSolver =
    requires(FSolver solver,
             const typename solver_traits<FSolver>::model_type
                 &model, // deduces model type from solver
             const State &state, const Control &control) {
  { solver(model, state, control) } -> std::same_as<ForcesMoments>;
};