/// @file
/// @brief Shared by the bindings that run E-RANSAC (epipolar, two-view): its summary out.
///
/// An `epipolar::EssentialSummary` crosses as a dict: `run` (see `ransac_summary`),
/// `refinement` (see `nls_summary`, or None if it did not run) and `refined`. The module's
/// `codes()` carries both `termination` (RANSAC's) and `nls_termination`.
#pragma once

#include <nanobind/nanobind.h>

#include "foundations/nls_crossing.hpp"
#include "foundations/ransac_crossing.hpp"
#include "slamlab/foundations/epipolar.hpp"

namespace slamlab::bindings {

/// @brief E-RANSAC's summary as a dict.
/// @param summary The summary.
/// @return `{"run", "refinement", "refined"}`.
inline nanobind::dict essential_summary(const foundations::epipolar::EssentialSummary& summary) {
  nanobind::dict out;
  out["run"] = ransac_summary(summary.run);
  out["refinement"] =
      summary.refinement ? nanobind::object(nls_summary(*summary.refinement)) : nanobind::none();
  out["refined"] = summary.refined;
  return out;
}

/// @brief E-RANSAC's solution as a dict.
/// @param solution The solution.
/// @return `model`, `inliers` (bool) and `summary` (see `essential_summary`).
inline nanobind::dict essential_result(const foundations::epipolar::EssentialSolution& solution) {
  nanobind::dict out;
  out["model"] = matrix3_array(solution.state.model);
  out["inliers"] = mask_array(solution.state.inliers);
  out["summary"] = essential_summary(solution.summary);
  return out;
}

/// @brief The enum tables of a module running E-RANSAC: RANSAC's and NLS' terminations.
/// @return `{"termination": [...], "nls_termination": [...]}`.
inline nanobind::dict essential_codes() {
  nanobind::dict out = ransac_codes();
  out["nls_termination"] = nls_terminations();
  return out;
}

}  // namespace slamlab::bindings
