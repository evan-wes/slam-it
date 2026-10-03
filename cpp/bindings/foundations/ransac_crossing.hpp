/// @file
/// @brief Shared by every binding that runs RANSAC: its options in, its solution out.
///
/// The options cross as one dict of scalars keyed by `slamlab.foundations.ransac.Options`'
/// field names; a missing or unknown key is an error, so the two sides cannot drift apart.
/// A solution crosses as a dict: `model`, `inliers` (bool) and `summary`, a dict of the
/// summary's fields with `termination` as its index in `codes()["termination"]`.
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "slamlab/foundations/ransac.hpp"

namespace slamlab::bindings {

/// @brief `ransac::Options`' fields, as the dict's keys.
inline constexpr std::array<const char*, 10> kRansacOptionKeys = {"threshold",
                                                                  "confidence",
                                                                  "max_iterations",
                                                                  "seed",
                                                                  "stream",
                                                                  "local_optimization",
                                                                  "lo_steps",
                                                                  "lo_threshold_multiplier",
                                                                  "lo_inner_iterations",
                                                                  "lo_sample_factor"};

/// @brief Read RANSAC's options from a dict of scalars.
/// @param options The dict, with exactly `kRansacOptionKeys`.
/// @return The options (checked by the core when it runs).
/// @throws std::invalid_argument (Python `ValueError`) for a missing or unknown key.
inline foundations::ransac::Options ransac_options(const nanobind::dict& options) {
  check_keys(options, kRansacOptionKeys, "ransac.Options");
  return foundations::ransac::Options{
      .threshold = nanobind::cast<double>(options["threshold"]),
      .confidence = nanobind::cast<double>(options["confidence"]),
      .max_iterations = nanobind::cast<int>(options["max_iterations"]),
      .seed = nanobind::cast<std::uint64_t>(options["seed"]),
      .stream = nanobind::cast<std::uint64_t>(options["stream"]),
      .local_optimization = nanobind::cast<bool>(options["local_optimization"]),
      .lo_steps = nanobind::cast<int>(options["lo_steps"]),
      .lo_threshold_multiplier = nanobind::cast<double>(options["lo_threshold_multiplier"]),
      .lo_inner_iterations = nanobind::cast<int>(options["lo_inner_iterations"]),
      .lo_sample_factor = nanobind::cast<int>(options["lo_sample_factor"])};
}

/// @brief A RANSAC summary as a dict of scalars, `termination` as its code.
/// @param summary The summary.
/// @return The summary's fields (`converged` follows from `termination`).
inline nanobind::dict ransac_summary(const foundations::ransac::Summary& summary) {
  nanobind::dict out;
  out["iterations"] = summary.iterations;
  out["termination"] = static_cast<int>(summary.termination);
  out["degenerate_samples"] = summary.degenerate_samples;
  out["local_optimizations"] = summary.local_optimizations;
  out["inlier_count"] = summary.inlier_count;
  out["score"] = summary.score;
  return out;
}

/// @brief A RANSAC solution with a 3x3 model as a dict.
/// @param solution The solution.
/// @return `model`, `inliers` (bool) and `summary` (see `ransac_summary`).
inline nanobind::dict ransac_result(
    const foundations::ransac::Solution<Eigen::Matrix3d>& solution) {
  nanobind::dict out;
  out["model"] = matrix3_array(solution.state.model);
  out["inliers"] = mask_array(solution.state.inliers);
  out["summary"] = ransac_summary(solution.summary);
  return out;
}

/// @brief The enum tables a RANSAC module's `codes()` returns.
/// @return `{"termination": [names in code order]}`.
inline nanobind::dict ransac_codes() {
  nanobind::list terminations;
  for (int k = 0; k < static_cast<int>(foundations::ransac::Termination::kCount); ++k) {
    terminations.append(
        foundations::ransac::name(static_cast<foundations::ransac::Termination>(k)));
  }
  nanobind::dict out;
  out["termination"] = terminations;
  return out;
}

}  // namespace slamlab::bindings
