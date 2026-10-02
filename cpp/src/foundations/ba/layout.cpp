/// @file
/// @brief Bundle adjustment internals: the layout -- degeneracy, unobserved variables, gauge.
#include "layout.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>
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

/// @brief Known positions closer than this (relative to their magnitude) count as one.
constexpr double kAnchorTolerance = 1e-9;

/// @brief What the fixed variables and priors already pin, before any gauge is applied.
struct Anchoring {
  /// @brief Rotation and translation: a fixed pose, a full pose prior, or three non-collinear
  ///        known positions.
  bool rigid = false;
  /// @brief Scale: a metric depth prior, or two distinct known positions.
  bool scale = false;
  /// @brief The camera the rigid motion is pinned to (fixed or fully prior-constrained), or -1.
  std::int64_t reference = -1;
};

/// @brief Count what fixed poses, fixed points, pose priors and metric depth already pin.
/// @param problem The problem.
/// @param observed Which poses are observed.
/// @param metric_depth Whether a metric depth prior (a fixed affine) is present.
/// @param fixed_points_used Which fixed points are observed and kept.
/// @return The anchoring.
Anchoring anchoring(const Problem& problem, const Mask& observed, bool metric_depth,
                    const std::vector<bool>& fixed_points_used) {
  std::vector<Eigen::Vector3d> positions;
  Anchoring out;
  std::int64_t first_full_prior = -1;
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    const auto row = static_cast<Index>(c);
    const bool translation = problem.pose_prior_weights(row, 0) > 0.0;
    if (observed[c] && problem.fixed_poses[c]) {
      positions.emplace_back(problem.poses[c].topRightCorner<3, 1>());
      out.reference = out.reference < 0 ? static_cast<std::int64_t>(c) : out.reference;
    }
    if (translation) {
      positions.emplace_back(problem.pose_prior[c].topRightCorner<3, 1>());
    }
    if (translation && problem.pose_prior_weights(row, 1) > 0.0 && first_full_prior < 0) {
      first_full_prior = static_cast<std::int64_t>(c);
    }
  }
  for (std::size_t p = 0; p < fixed_points_used.size(); ++p) {
    if (fixed_points_used[p]) {
      positions.emplace_back(problem.points.row(static_cast<Index>(p)).transpose());
    }
  }
  out.reference = out.reference >= 0 ? out.reference : first_full_prior;
  bool distinct = false;
  bool non_collinear = false;
  if (positions.size() >= 2) {
    Eigen::MatrixX3d centred(static_cast<Index>(positions.size()), 3);
    for (std::size_t i = 0; i < positions.size(); ++i) {
      centred.row(static_cast<Index>(i)) = positions[i].transpose();
    }
    const double tolerance = kAnchorTolerance * (1.0 + centred.cwiseAbs().maxCoeff());
    centred.rowwise() -= centred.colwise().mean();
    const Eigen::JacobiSVD<Eigen::MatrixX3d> svd(centred);
    distinct = svd.singularValues()(0) > tolerance;
    non_collinear = svd.singularValues()(1) > tolerance;
  }
  out.rigid = out.reference >= 0 || non_collinear;
  out.scale = metric_depth || distinct;
  return out;
}

/// @brief Resolve `kAuto` to the minimal gauge the anchoring leaves (Triggs et al. 1999,
///        section 9); other choices pass through.
/// @param gauge The requested gauge.
/// @param anchors What is already pinned.
/// @return The gauge to apply.
Gauge resolve_gauge(Gauge gauge, const Anchoring& anchors) {
  if (gauge != Gauge::kAuto) {
    return gauge;
  }
  if (anchors.rigid) {
    return anchors.scale ? Gauge::kNone : Gauge::kScale;
  }
  return anchors.scale ? Gauge::kFirstPose : Gauge::kSimilarity;
}

/// @brief Confine the free pose farthest from `reference` to keep its baseline.
/// @param problem The problem.
/// @param reference The pose the baseline is measured from.
/// @param layout The layout to update.
void confine_scale(const Problem& problem, std::int64_t reference, Layout& layout) {
  const Eigen::Vector3d origin =
      problem.poses[static_cast<std::size_t>(reference)].topRightCorner<3, 1>();
  double farthest = 1e-12 * (1.0 + origin.norm());
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    const double distance = (problem.poses[c].topRightCorner<3, 1>() - origin).norm();
    if (layout.pose_free[c] && distance > farthest) {
      farthest = distance;
      layout.scale_pose = static_cast<std::int64_t>(c);
    }
  }
  if (layout.scale_pose >= 0) {
    layout.scale_reference = reference;
    layout.counts.scale_pose = layout.scale_pose;
    layout.pose_free[static_cast<std::size_t>(layout.scale_pose)] = false;
  }
}

/// @brief Fix the gauge: update the free poses and record the gauge and scale poses.
/// @param problem The problem.
/// @param observed Which poses are observed.
/// @param anchors What is already pinned.
/// @param layout The layout to update (its gauge already resolved).
/// @return Nothing, or `InvalidInput` for a scale gauge with nothing to measure scale from.
Result<void> fix_gauge(const Problem& problem, const Mask& observed, const Anchoring& anchors,
                       Layout& layout) {
  const Gauge gauge = layout.counts.gauge;
  if (gauge == Gauge::kNone) {
    return {};
  }
  if (gauge == Gauge::kScale) {
    if (anchors.reference < 0) {
      return fail(ErrorKind::kInvalidInput,
                  "gauge: 'scale' needs a fixed pose or a full pose prior to measure scale from");
    }
    confine_scale(problem, anchors.reference, layout);
    return {};
  }
  IndexList fixed_observed;
  std::int64_t first_free = -1;
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    if (observed[c] && problem.fixed_poses[c]) {
      fixed_observed.push_back(static_cast<Index>(c));
    } else if (observed[c] && first_free < 0) {
      first_free = static_cast<std::int64_t>(c);
    }
  }
  if (fixed_observed.empty()) {
    if (first_free < 0) {
      return {};
    }
    layout.counts.gauge_pose = first_free;
    layout.pose_free[static_cast<std::size_t>(first_free)] = false;
    fixed_observed.push_back(at(first_free));
  }
  if (gauge == Gauge::kFirstPose) {
    return {};
  }
  const Eigen::Vector3d origin =
      problem.poses[static_cast<std::size_t>(fixed_observed.front())].topRightCorner<3, 1>();
  for (const Index c : fixed_observed) {
    const Eigen::Vector3d centre =
        problem.poses[static_cast<std::size_t>(c)].topRightCorner<3, 1>();
    if ((centre - origin).norm() > 1e-12 * (1.0 + origin.norm())) {
      return {};  // two fixed poses apart already pin the scale
    }
  }
  confine_scale(problem, fixed_observed.front(), layout);
  return {};
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
    // So has one whose anchor pixel the starting intrinsics cannot undistort.
    if (anchor >= 0 && views[p] > 0 && !degenerate[p]) {
      const auto a = static_cast<std::size_t>(anchor);
      const auto group = at(problem.intrinsics_idx[static_cast<std::size_t>(problem.cam_idx[a])]);
      degenerate[p] = !camera::unproject(problem.model, problem.intrinsics.row(group).transpose(),
                                         problem.observations.row(at(anchor)).transpose())
                           .has_value();
    }
  }
  return degenerate;
}

/// @brief Attach the depth and pose priors and free the observed affines.
/// @param problem The problem.
/// @param layout The layout to update (its kept observations already chosen).
/// @return Whether a metric depth prior (a fixed affine) is present.
bool attach_priors(const Problem& problem, Layout& layout) {
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
  return metric_depth;
}

/// @brief Give each used free point a slot, mark the anchored points, and list the
///        observations that get a reprojection block (all but the anchors).
/// @param problem The problem.
/// @param degenerate Which points were dropped.
/// @param views Each point's number of kept observations.
/// @param layout The layout to update (its kept observations already chosen).
void assign_points(const Problem& problem, const std::vector<bool>& degenerate,
                   const std::vector<Index>& views, Layout& layout) {
  const std::size_t n_points = views.size();
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
}

}  // namespace

Result<Layout> make_layout(const Problem& problem, const Options& options) {
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
  const auto n_params = static_cast<std::size_t>(problem.intrinsics.cols());
  layout.intrinsics_free.assign(n_groups * n_params, false);
  for (std::size_t g = 0; g < n_groups; ++g) {
    bool any_free = false;
    for (std::size_t k = 0; k < n_params; ++k) {
      const bool free = !problem.fixed_intrinsics[(g * n_params) + k];
      layout.intrinsics_free[(g * n_params) + k] = free && group_observed[g];
      any_free = any_free || free;
    }
    layout.counts.unobserved_intrinsics += (any_free && !group_observed[g]) ? 1 : 0;
  }
  const bool metric_depth = attach_priors(problem, layout);
  std::vector<bool> fixed_points_used(n_points, false);
  for (std::size_t p = 0; p < n_points; ++p) {
    fixed_points_used[p] = problem.fixed_points[p] && !degenerate[p] && views[p] > 0;
  }
  const Anchoring anchors = anchoring(problem, observed, metric_depth, fixed_points_used);
  layout.counts.gauge = resolve_gauge(options.gauge, anchors);
  if (auto fixed = fix_gauge(problem, observed, anchors, layout); !fixed) {
    return std::unexpected(fixed.error());
  }
  assign_points(problem, degenerate, views, layout);
  return layout;
}

}  // namespace slamlab::foundations::ba::detail
