/// @file
/// @brief Bundle adjustment: the public checks and entry point.
#include "slamlab/foundations/ba.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ba/common.hpp"
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
    if (a >= 0 && problem.fixed_points[p]) {
      return fail(ErrorKind::kInvalidInput,
                  "point_anchor: a fixed point cannot be anchored (it would move with its host)");
    }
  }
  return {};
}

}  // namespace

const char* name(Gauge gauge) {
  switch (gauge) {
    case Gauge::kNone:
      return "none";
    case Gauge::kFirstPose:
      return "first_pose";
    case Gauge::kSimilarity:
      return "similarity";
    case Gauge::kAuto:
      return "auto";
    case Gauge::kScale:
      return "scale";
    case Gauge::kPartial:
      return "partial";
    case Gauge::kCount:
      break;
  }
  return "unknown";
}

std::optional<Eigen::VectorXd> DenseCholesky::solve(const Eigen::MatrixXd& matrix,
                                                    const Eigen::VectorXd& rhs) {
  const Eigen::LLT<Eigen::MatrixXd> factor(matrix);
  if (factor.info() != Eigen::Success) {
    return std::nullopt;
  }
  return Eigen::VectorXd(factor.solve(rhs));
}

Problem Problem::with_defaults(camera::Model model, Intrinsics intrinsics, Poses poses,
                               Points points, Pixels observations, Indices cam_idx,
                               Indices pt_idx) {
  const auto n_cams = static_cast<Eigen::Index>(poses.size());
  const Index n_points = points.rows();
  const Index n_obs = observations.rows();
  const auto n_fixed = static_cast<std::size_t>(intrinsics.size());
  Problem problem{
      .model = model,
      .intrinsics = std::move(intrinsics),
      .poses = std::move(poses),
      .points = std::move(points),
      .observations = std::move(observations),
      .cam_idx = std::move(cam_idx),
      .pt_idx = std::move(pt_idx),
      .intrinsics_idx = Indices(static_cast<std::size_t>(n_cams), 0),
      .weights = Eigen::VectorXd::Ones(n_obs),
      .fixed_poses = Mask(static_cast<std::size_t>(n_cams), false),
      .fixed_intrinsics = Mask(n_fixed, true),
      .fixed_points = Mask(static_cast<std::size_t>(n_points), false),
      .depth_prior = Eigen::VectorXd::Constant(n_obs, std::numeric_limits<double>::quiet_NaN()),
      .depth_prior_weights = Eigen::VectorXd::Ones(n_obs),
      .depth_affine = {},
      .fixed_depth_affine = Mask(static_cast<std::size_t>(n_cams), true),
      .pose_prior = {},
      .pose_prior_weights =
          Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>::Zero(n_cams, 2),
      .point_anchor = Indices(static_cast<std::size_t>(n_points), -1)};
  problem.depth_affine.resize(n_cams, 2);
  problem.depth_affine.col(0).setOnes();
  problem.depth_affine.col(1).setZero();
  problem.pose_prior = problem.poses;
  return problem;
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
  if (auto checked = check_indices(problem.intrinsics_idx, n_groups, "intrinsics_idx"); !checked) {
    return checked;
  }
  // The priors and anchors index the arrays checked above, so they come last.
  if (auto checked = check_priors(problem); !checked) {
    return checked;
  }
  return check_anchors(problem);
}

}  // namespace slamlab::foundations::ba
