/// @file
/// @brief Bundle adjustment's solve (declared in slam_it/foundations/ba.hpp): the problem
///        centred on its cameras, laid out, adjusted, and shifted back.
///
/// Mirrors `slam_it.foundations.ba.solve`. It sits above every other module of the package,
/// which is why it is not in the facade's `ba.cpp`: the layer check holds this file to the
/// `solve` layer.
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <expected>
#include <vector>

#include "bundle.hpp"
#include "layout.hpp"
#include "slam_it/foundations/ba.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/nls.hpp"

namespace slam_it::foundations::ba {

Eigen::Vector3d centring_origin(const Problem& problem) {
  // The cameras with a positive-weight observation: an unobserved placeholder (an
  // unregistered camera left at the identity) must not pull the origin off the scene.
  std::vector<bool> seen(problem.poses.size(), false);
  for (Eigen::Index m = 0; m < problem.weights.size(); ++m) {
    if (problem.weights(m) > 0.0) {
      seen[static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)])] = true;
    }
  }
  const bool any_seen = std::ranges::find(seen, true) != seen.end();
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0;
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    if (seen[c] || !any_seen) {
      sum += problem.poses[c].topRightCorner<3, 1>();  // in index order, as Python sums
      ++count;
    }
  }
  return count > 0 ? Eigen::Vector3d(sum / static_cast<double>(count)) : sum;
}

namespace {

/// @brief The problem shifted by `-origin`: camera centres, points and prior centres.
/// @param problem The problem.
/// @param origin The shift.
/// @return The shifted problem.
Problem shifted(const Problem& problem, const Eigen::Vector3d& origin) {
  Problem out = problem;
  for (Eigen::Matrix4d& pose : out.poses) {
    pose.topRightCorner<3, 1>() -= origin;
  }
  for (Eigen::Matrix4d& prior : out.pose_prior) {
    prior.topRightCorner<3, 1>() -= origin;
  }
  out.points.rowwise() -= origin.transpose();
  return out;
}

}  // namespace

Result<Solution> solve(const Problem& problem, const Options& options) {
  if (auto checked = check_problem(problem); !checked) {
    return std::unexpected(checked.error());
  }
  if (!std::isfinite(options.min_parallax_deg) || options.min_parallax_deg < 0.0 ||
      !std::isfinite(options.focal_std_ratio) || options.focal_std_ratio < 0.0) {
    return fail(ErrorKind::kInvalidInput,
                "min_parallax_deg, focal_std_ratio: must be non-negative and finite");
  }
  if (options.gauge == Gauge::kPartial) {
    return fail(ErrorKind::kInvalidInput, "gauge: 'partial' is reported, not requested");
  }
  if (options.gauge >= Gauge::kCount) {
    return fail(ErrorKind::kInvalidInput, "gauge: not a gauge (kCount)");
  }
  if (options.depth_loss.kind >= nls::LossKind::kCount) {
    return fail(ErrorKind::kInvalidInput, "depth_loss: not a loss (kCount)");
  }
  if (!options.solver.block_loss.losses.empty()) {
    return fail(ErrorKind::kInvalidInput,
                "solver.block_loss: set by bundle adjustment, not the caller");
  }
  // Solve in a world centred on the cameras: exact (every residual is unchanged by a shift),
  // and it keeps the steps well conditioned far from the origin, where a step's rotation
  // about the world origin would swing a camera by about |theta|^2 |C| / 2.
  const Eigen::Vector3d origin = centring_origin(problem);
  const Problem centred = shifted(problem, origin);
  const auto layout = detail::make_layout(centred, options);
  if (!layout) {
    return std::unexpected(layout.error());
  }
  auto solution = detail::adjust(centred, options, *layout);
  if (!solution) {
    return solution;
  }
  // Shift back; what did not move (fixed or unobserved poses, fixed or dropped points) is
  // returned as given, bit for bit.
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    Eigen::Matrix4d& pose = solution->poses[c];
    if (pose == centred.poses[c]) {
      pose = problem.poses[c];
    } else {
      pose.topRightCorner<3, 1>() += origin;
    }
  }
  for (Eigen::Index p = 0; p < problem.points.rows(); ++p) {
    if (solution->points.row(p) == centred.points.row(p)) {
      solution->points.row(p) = problem.points.row(p);
    } else {
      solution->points.row(p) += origin.transpose();
    }
  }
  return solution;
}

}  // namespace slam_it::foundations::ba
