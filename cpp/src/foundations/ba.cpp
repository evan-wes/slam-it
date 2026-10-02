/// @file
/// @brief Bundle adjustment: the public checks and entry point.
#include "slamlab/foundations/ba.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "ba/bundle.hpp"
#include "ba/common.hpp"
#include "ba/layout.hpp"
#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/se3.hpp"

namespace slamlab::foundations::ba {

namespace {

using detail::at;
using detail::Index;

/// @brief Check that every entry of an index array lies in `[0, bound)`.
/// @param indices The indices.
/// @param bound The exclusive upper bound.
/// @param name The array's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_indices(const Indices& indices, Index bound, const char* name) {
  for (const std::int64_t i : indices) {
    if (i < 0 || at(i) >= bound) {
      return fail(ErrorKind::kInvalidInput,
                  std::string(name) + ": indices must lie in [0, " + std::to_string(bound) + ")");
    }
  }
  return {};
}

/// @brief Check the depth and pose priors' sizes and values.
/// @param problem The problem.
/// @return Nothing, or `InvalidInput` naming the offending array.
Result<void> check_priors(const Problem& problem) {
  const Index n_obs = problem.observations.rows();
  const auto n_cams = static_cast<Index>(problem.poses.size());
  const Eigen::VectorXd& prior = problem.depth_prior;
  if (prior.size() != n_obs || ((prior.array() <= 0.0) || prior.array().isInf()).any()) {
    return fail(ErrorKind::kInvalidInput, "depth_prior: expected M positive values (NaN for none)");
  }
  const Eigen::VectorXd& prior_weights = problem.depth_prior_weights;
  if (prior_weights.size() != n_obs || !prior_weights.allFinite() ||
      (prior_weights.array() < 0.0).any()) {
    return fail(ErrorKind::kInvalidInput, "depth_prior_weights: expected M non-negative values");
  }
  if (problem.depth_affine.rows() != n_cams || !problem.depth_affine.allFinite() ||
      std::cmp_not_equal(problem.fixed_depth_affine.size(), n_cams)) {
    return fail(ErrorKind::kInvalidInput, "depth_affine: expected C finite (scale, shift) rows");
  }
  if (std::cmp_not_equal(problem.pose_prior.size(), n_cams) ||
      problem.pose_prior_weights.rows() != n_cams || !problem.pose_prior_weights.allFinite() ||
      (problem.pose_prior_weights.array() < 0.0).any()) {
    return fail(ErrorKind::kInvalidInput, "pose_prior: expected C poses and C weight pairs");
  }
  for (const Eigen::Matrix4d& pose : problem.pose_prior) {
    if (auto checked = se3::check_transform(pose); !checked) {
      return checked;
    }
  }
  return {};
}

/// @brief Check the point anchors: -1, or one of the point's own observations.
/// @param problem The problem.
/// @return Nothing, or `InvalidInput`.
Result<void> check_anchors(const Problem& problem) {
  if (std::cmp_not_equal(problem.point_anchor.size(), problem.points.rows())) {
    return fail(ErrorKind::kInvalidInput, "point_anchor: expected N entries");
  }
  for (std::size_t p = 0; p < problem.point_anchor.size(); ++p) {
    const std::int64_t a = problem.point_anchor[p];
    if (a < -1 || a >= problem.observations.rows() ||
        (a >= 0 && std::cmp_not_equal(problem.pt_idx[static_cast<std::size_t>(a)], p))) {
      return fail(ErrorKind::kInvalidInput,
                  "point_anchor: a point's anchor must be one of its observations");
    }
  }
  return {};
}

}  // namespace

std::optional<Eigen::VectorXd> DenseCholesky::solve(const Eigen::MatrixXd& matrix,
                                                    const Eigen::VectorXd& rhs) {
  const Eigen::LLT<Eigen::MatrixXd> factor(matrix);
  if (factor.info() != Eigen::Success) {
    return std::nullopt;
  }
  return Eigen::VectorXd(factor.solve(rhs));
}

Result<void> check_problem(const Problem& problem) {
  const std::size_t n_cams = problem.poses.size();
  const auto n_obs = static_cast<std::size_t>(problem.observations.rows());
  const Index n_groups = problem.intrinsics.rows();
  if (n_groups == 0) {
    return fail(ErrorKind::kInvalidInput, "intrinsics: expected at least one group");
  }
  for (Index g = 0; g < n_groups; ++g) {
    if (auto checked =
            camera::check_intrinsics(problem.model, problem.intrinsics.row(g).transpose());
        !checked) {
      return checked;
    }
  }
  for (const Eigen::Matrix4d& pose : problem.poses) {
    if (auto checked = se3::check_transform(pose); !checked) {
      return checked;
    }
  }
  if (!problem.points.allFinite() || !problem.observations.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "points, observations: expected finite values");
  }
  if (std::cmp_not_equal(problem.weights.size(), n_obs) || !problem.weights.allFinite() ||
      (problem.weights.array() < 0.0).any()) {
    return fail(ErrorKind::kInvalidInput, "weights: expected M non-negative values");
  }
  if (problem.fixed_poses.size() != n_cams ||
      std::cmp_not_equal(problem.fixed_intrinsics.size(), n_groups * problem.intrinsics.cols()) ||
      problem.fixed_points.size() != static_cast<std::size_t>(problem.points.rows())) {
    return fail(ErrorKind::kInvalidInput, "fixed masks: wrong length");
  }
  if (auto checked = check_priors(problem); !checked) {
    return checked;
  }
  if (auto checked = check_anchors(problem); !checked) {
    return checked;
  }
  if (problem.cam_idx.size() != n_obs || problem.pt_idx.size() != n_obs ||
      problem.intrinsics_idx.size() != n_cams) {
    return fail(ErrorKind::kInvalidInput, "cam_idx, pt_idx, intrinsics_idx: wrong length");
  }
  if (auto checked = check_indices(problem.cam_idx, static_cast<Index>(n_cams), "cam_idx");
      !checked) {
    return checked;
  }
  if (auto checked = check_indices(problem.pt_idx, problem.points.rows(), "pt_idx"); !checked) {
    return checked;
  }
  return check_indices(problem.intrinsics_idx, n_groups, "intrinsics_idx");
}

Result<Solution> solve(const Problem& problem, const Options& options) {
  if (auto checked = check_problem(problem); !checked) {
    return std::unexpected(checked.error());
  }
  if (!std::isfinite(options.min_parallax_deg) || options.min_parallax_deg < 0.0 ||
      !std::isfinite(options.focal_std_ratio) || options.focal_std_ratio < 0.0) {
    return fail(ErrorKind::kInvalidInput,
                "min_parallax_deg, focal_std_ratio: must be non-negative and finite");
  }
  if (!options.solver.block_loss.losses.empty()) {
    return fail(ErrorKind::kInvalidInput,
                "solver.block_loss: set by bundle adjustment, not the caller");
  }
  return detail::adjust(problem, options, detail::make_layout(problem, options));
}

}  // namespace slamlab::foundations::ba
