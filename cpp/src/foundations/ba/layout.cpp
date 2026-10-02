/// @file
/// @brief Bundle adjustment internals: the layout -- degeneracy, unobserved variables, gauge.
#include "layout.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

#include "slamlab/foundations/camera.hpp"

namespace slamlab::foundations::ba::detail {

namespace {

/// @brief Each point's largest angle between its first kept ray and any other (radians).
/// @param problem The problem.
/// @param obs The kept observations.
/// @return The parallax per point.
Eigen::VectorXd ray_parallax(const Problem& problem, const IndexList& obs) {
  const Index n_points = problem.points.rows();
  std::vector<Eigen::Vector3d> reference(static_cast<std::size_t>(n_points));
  std::vector<bool> seen(static_cast<std::size_t>(n_points), false);
  Eigen::VectorXd parallax = Eigen::VectorXd::Zero(n_points);
  for (const Index m : obs) {
    const auto p = static_cast<std::size_t>(problem.pt_idx[static_cast<std::size_t>(m)]);
    const Eigen::Vector3d ray =
        (problem.poses[static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)])]
             .topRightCorner<3, 1>() -
         problem.points.row(static_cast<Index>(p)).transpose())
            .normalized();
    if (!seen[p]) {
      seen[p] = true;
      reference[p] = ray;
      continue;
    }
    const double angle = std::atan2(ray.cross(reference[p]).norm(), ray.dot(reference[p]));
    parallax(static_cast<Index>(p)) = std::max(parallax(static_cast<Index>(p)), angle);
  }
  return parallax;
}

/// @brief Fix the gauge: update the free poses and record the gauge and scale poses.
/// @param problem The problem.
/// @param observed Which poses are observed.
/// @param gauge The gauge choice.
/// @param layout The layout to update.
void fix_gauge(const Problem& problem, const Mask& observed, Gauge gauge, Layout& layout) {
  if (gauge == Gauge::kNone) {
    return;
  }
  const std::size_t n_cams = problem.poses.size();
  IndexList fixed_observed;
  std::int64_t first_free = -1;
  for (std::size_t c = 0; c < n_cams; ++c) {
    if (observed[c] && problem.fixed_poses[c]) {
      fixed_observed.push_back(static_cast<Index>(c));
    } else if (observed[c] && first_free < 0) {
      first_free = static_cast<std::int64_t>(c);
    }
  }
  if (fixed_observed.empty() && first_free >= 0) {
    layout.counts.gauge_pose = first_free;
    layout.pose_free[static_cast<std::size_t>(first_free)] = false;
    fixed_observed.push_back(at(first_free));
  }
  if (gauge == Gauge::kFirstPose || fixed_observed.empty()) {
    return;
  }
  const auto pose_centre = [&](Index c) -> Eigen::Vector3d {
    return problem.poses[static_cast<std::size_t>(c)].topRightCorner<3, 1>();
  };
  const Eigen::Vector3d origin = pose_centre(fixed_observed.front());
  const double tolerance = 1e-12 * (1.0 + origin.norm());
  for (const Index c : fixed_observed) {
    if ((pose_centre(c) - origin).norm() > tolerance) {
      return;  // two fixed poses apart already pin the scale
    }
  }
  double farthest = tolerance;
  for (std::size_t c = 0; c < n_cams; ++c) {
    const double distance = (pose_centre(static_cast<Index>(c)) - origin).norm();
    if (layout.pose_free[c] && distance > farthest) {
      farthest = distance;
      layout.scale_pose = static_cast<std::int64_t>(c);
    }
  }
  if (layout.scale_pose >= 0) {
    layout.scale_reference = fixed_observed.front();
    layout.counts.scale_pose = layout.scale_pose;
    layout.pose_free[static_cast<std::size_t>(layout.scale_pose)] = false;
  }
}

/// @brief The weighted observations whose point is in front of its camera at the start.
/// @param problem The problem.
/// @param counts Incremented for each observation behind its camera.
/// @return The kept observations.
IndexList in_front(const Problem& problem, Summary& counts) {
  IndexList kept;
  for (Index m = 0; m < problem.observations.rows(); ++m) {
    if (problem.weights(m) <= 0.0) {
      continue;
    }
    const Eigen::Matrix4d& t_world_cam =
        problem.poses[static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)])];
    const Eigen::Vector3d point =
        problem.points.row(at(problem.pt_idx[static_cast<std::size_t>(m)])).transpose();
    const double depth =
        t_world_cam.topLeftCorner<3, 3>().col(2).dot(point - t_world_cam.topRightCorner<3, 1>());
    if (depth > camera::kMinDepth) {
      kept.push_back(m);
    } else {
      ++counts.dropped_observations;
    }
  }
  return kept;
}

/// @brief Points that cannot be placed: free points with fewer than two views or too little
///        parallax, and anchored points whose anchor was dropped.
/// @param problem The problem.
/// @param kept The kept observations.
/// @param min_parallax_deg The parallax threshold (degrees).
/// @param views Each point's number of kept observations.
/// @return Which points are degenerate.
std::vector<bool> degenerate_points(const Problem& problem, const IndexList& kept,
                                    double min_parallax_deg, const std::vector<Index>& views) {
  const Eigen::VectorXd parallax = ray_parallax(problem, kept);
  const double min_parallax = min_parallax_deg * std::numbers::pi / 180.0;
  std::vector<bool> is_kept(static_cast<std::size_t>(problem.observations.rows()), false);
  for (const Index m : kept) {
    is_kept[static_cast<std::size_t>(m)] = true;
  }
  std::vector<bool> degenerate(views.size(), false);
  for (std::size_t p = 0; p < views.size(); ++p) {
    degenerate[p] = !problem.fixed_points[p] && views[p] > 0 &&
                    (views[p] < 2 || parallax(static_cast<Index>(p)) < min_parallax);
    // An anchored point whose anchor was dropped has no ray to live on.
    const std::int64_t anchor = problem.point_anchor[p];
    degenerate[p] = degenerate[p] ||
                    (anchor >= 0 && views[p] > 0 && !is_kept[static_cast<std::size_t>(anchor)]);
  }
  return degenerate;
}

/// @brief Attach the depth and pose priors, free the observed affines, and resolve the gauge.
/// @param problem The problem.
/// @param options The settings.
/// @param layout The layout to update (its kept observations already chosen).
void attach_priors(const Problem& problem, const Options& options, Layout& layout) {
  const std::size_t n_cams = problem.poses.size();
  Mask affine_observed(n_cams, false);
  bool metric_depth = false;
  for (std::size_t i = 0; i < layout.obs.size(); ++i) {
    const Index m = layout.obs[i];
    if (std::isnan(problem.depth_prior(m)) || problem.depth_prior_weights(m) <= 0.0) {
      continue;
    }
    const auto c = static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)]);
    layout.depth.push_back(static_cast<Index>(i));
    affine_observed[c] = true;
    metric_depth = metric_depth || problem.fixed_depth_affine[c];
  }
  layout.affine_free.assign(n_cams, false);
  for (std::size_t c = 0; c < n_cams; ++c) {
    layout.affine_free[c] = !problem.fixed_depth_affine[c] && affine_observed[c];
    layout.counts.unobserved_depth_affines +=
        (!problem.fixed_depth_affine[c] && !affine_observed[c]) ? 1 : 0;
    if ((problem.pose_prior_weights.row(static_cast<Index>(c)).array() > 0.0).any()) {
      layout.prior_cams.push_back(static_cast<Index>(c));
    }
  }
  layout.counts.gauge = options.gauge;
  if (options.gauge == Gauge::kAuto) {
    if (!layout.prior_cams.empty()) {
      layout.counts.gauge = Gauge::kNone;
    } else if (metric_depth) {
      layout.counts.gauge = Gauge::kFirstPose;
    } else {
      layout.counts.gauge = Gauge::kSimilarity;
    }
  }
}

}  // namespace

Layout make_layout(const Problem& problem, const Options& options) {
  const std::size_t n_cams = problem.poses.size();
  const auto n_points = static_cast<std::size_t>(problem.points.rows());
  const auto n_groups = static_cast<std::size_t>(problem.intrinsics.rows());
  Layout layout;
  const IndexList kept = in_front(problem, layout.counts);
  std::vector<Index> views(n_points, 0);
  for (const Index m : kept) {
    ++views[static_cast<std::size_t>(problem.pt_idx[static_cast<std::size_t>(m)])];
  }
  const std::vector<bool> degenerate =
      degenerate_points(problem, kept, options.min_parallax_deg, views);
  layout.counts.dropped_points = std::count(degenerate.begin(), degenerate.end(), true);
  Mask observed(n_cams, false);
  Mask group_observed(n_groups, false);
  for (const Index m : kept) {
    const auto p = static_cast<std::size_t>(problem.pt_idx[static_cast<std::size_t>(m)]);
    if (degenerate[p]) {
      ++layout.counts.dropped_observations;
      continue;
    }
    const auto c = static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)]);
    layout.obs.push_back(m);
    observed[c] = true;
    group_observed[static_cast<std::size_t>(problem.intrinsics_idx[c])] = true;
  }
  layout.pose_free.assign(n_cams, false);
  for (std::size_t c = 0; c < n_cams; ++c) {
    layout.pose_free[c] = !problem.fixed_poses[c] && observed[c];
    layout.counts.unobserved_poses += (!problem.fixed_poses[c] && !observed[c]) ? 1 : 0;
  }
  layout.intrinsics_free.assign(n_groups, false);
  for (std::size_t g = 0; g < n_groups; ++g) {
    layout.intrinsics_free[g] = !problem.fixed_intrinsics[g] && group_observed[g];
    layout.counts.unobserved_intrinsics +=
        (!problem.fixed_intrinsics[g] && !group_observed[g]) ? 1 : 0;
  }
  attach_priors(problem, options, layout);
  fix_gauge(problem, observed, layout.counts.gauge, layout);
  layout.point_slot.assign(n_points, -1);
  layout.anchored.assign(n_points, false);
  std::vector<bool> is_anchor(static_cast<std::size_t>(problem.observations.rows()), false);
  for (std::size_t p = 0; p < n_points; ++p) {
    const bool used = !degenerate[p] && views[p] > 0;
    if (used && !problem.fixed_points[p]) {
      layout.point_slot[p] = layout.free_points++;
    }
    if (used && problem.point_anchor[p] >= 0) {
      layout.anchored[p] = true;
      is_anchor[static_cast<std::size_t>(problem.point_anchor[p])] = true;
    }
  }
  for (std::size_t i = 0; i < layout.obs.size(); ++i) {
    if (!is_anchor[static_cast<std::size_t>(layout.obs[i])]) {
      layout.reprojection.push_back(static_cast<Index>(i));
    }
  }
  return layout;
}

}  // namespace slamlab::foundations::ba::detail
