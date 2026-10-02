/// @file
/// @brief Bundle adjustment internals: the layout -- degeneracy, unobserved variables, gauge.
#include "layout.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <utility>
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

/// @brief The observed poses sorted by what pins them.
struct PinnedPoses {
  /// @brief Fixed observed poses.
  IndexList fixed;
  /// @brief Free observed poses with a translation prior.
  IndexList translation;
  /// @brief Free observed poses with a rotation prior.
  IndexList rotation;
  /// @brief Free observed poses with both.
  IndexList full;
  /// @brief Every observed pose's centre.
  Positions centres;
};

/// @brief Sort the observed poses by what pins them.
/// @param problem The problem.
/// @param observed Which poses are observed.
/// @return The poses.
PinnedPoses pinned_poses(const Problem& problem, const Mask& observed) {
  PinnedPoses out;
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    if (!observed[c]) {
      continue;
    }
    const auto row = static_cast<Index>(c);
    out.centres.emplace_back(problem.poses[c].topRightCorner<3, 1>());
    if (problem.fixed_poses[c]) {
      out.fixed.push_back(row);
      continue;
    }
    const bool by_translation = problem.pose_prior_weights(row, 0) > 0.0;
    const bool by_rotation = problem.pose_prior_weights(row, 1) > 0.0;
    if (by_translation) {
      out.translation.push_back(row);
    }
    if (by_rotation) {
      out.rotation.push_back(row);
    }
    if (by_translation && by_rotation) {
      out.full.push_back(row);
    }
  }
  return out;
}

/// @brief What a gauge choice counts as pinned: `kAuto` everything (fixed observed poses,
///        priors on observed poses, fixed observed points, metric depth), `kSimilarity` the
///        fixed observed poses, `kScale` its reference pose. Priors on unobserved poses pin
///        nothing the scene can feel.
/// @param problem The problem.
/// @param observed Which poses are observed.
/// @param metric_depth Whether a metric depth prior (a fixed affine) is present.
/// @param fixed_points_used Which fixed points are observed and kept.
/// @param gauge The requested gauge.
/// @return The anchors, or `InvalidInput` for a scale gauge with nothing to measure from.
Result<Anchors> pinned(const Problem& problem, const Mask& observed, bool metric_depth,
                       const std::vector<bool>& fixed_points_used, Gauge gauge) {
  const PinnedPoses poses = pinned_poses(problem, observed);
  Positions points;
  for (std::size_t p = 0; p < fixed_points_used.size(); ++p) {
    if (fixed_points_used[p]) {
      points.emplace_back(problem.points.row(static_cast<Index>(p)).transpose());
    }
  }
  Positions scene = poses.centres;  // the frame's positions: observed centres, then points
  scene.insert(scene.end(), points.begin(), points.end());
  Anchors out;
  out.frame = frame_of(scene);
  if (gauge == Gauge::kScale) {
    if (poses.fixed.empty() && poses.full.empty()) {
      return fail(ErrorKind::kInvalidInput,
                  "gauge: 'scale' needs a fixed pose or a full pose prior to measure scale from");
    }
    const Index reference = poses.fixed.empty() ? poses.full.front() : poses.fixed.front();
    out.centre_cams = {reference};
    out.orientation_cams = {reference};
    return out;
  }
  out.centre_cams = poses.fixed;
  out.orientation_cams = poses.fixed;
  if (gauge == Gauge::kSimilarity) {
    return out;
  }
  out.centre_cams.insert(out.centre_cams.end(), poses.translation.begin(), poses.translation.end());
  out.orientation_cams.insert(out.orientation_cams.end(), poses.rotation.begin(),
                              poses.rotation.end());
  out.points = std::move(points);
  out.metric = metric_depth;
  return out;
}

/// @brief The gauge as applied: its name, the poses that remove it, what pins the rest.
struct GaugeChoice {
  /// @brief The gauge applied: the request, or for `kAuto` what it found.
  Gauge name = Gauge::kNone;
  /// @brief Poses with all six directions free once the gauge is removed.
  Mask pose_free;
  /// @brief What pins the similarity, the plan's fixed poses included.
  Anchors anchors;
  /// @brief The poses that remove the gauge.
  Plan plan;
};

/// @brief Resolve the gauge choice and plan its removal (see `gauge.hpp`).
/// @param problem The problem.
/// @param observed Which poses are observed.
/// @param metric_depth Whether a metric depth prior is present.
/// @param fixed_points_used Which fixed points are observed and kept.
/// @param gauge The requested gauge.
/// @return The choice, or `InvalidInput` for a scale gauge with nothing to measure from.
Result<GaugeChoice> fix_gauge(const Problem& problem, const Mask& observed, bool metric_depth,
                              const std::vector<bool>& fixed_points_used, Gauge gauge) {
  auto anchors = pinned(problem, observed, metric_depth, fixed_points_used, gauge);
  if (!anchors) {
    return std::unexpected(anchors.error());
  }
  GaugeChoice out;
  out.name = gauge;
  out.pose_free.assign(problem.poses.size(), false);
  IndexList candidates;
  bool any_fixed = false;
  Positions centres;
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    out.pose_free[c] = !problem.fixed_poses[c] && observed[c];
    if (out.pose_free[c]) {
      candidates.push_back(static_cast<Index>(c));
    }
    any_fixed = any_fixed || (observed[c] && problem.fixed_poses[c]);
    centres.emplace_back(problem.poses[c].topRightCorner<3, 1>());
  }
  if (gauge == Gauge::kFirstPose) {
    if (!any_fixed && !candidates.empty()) {
      out.plan.fixed.push_back(candidates.front());
    }
  } else if (gauge != Gauge::kNone) {
    if (gauge == Gauge::kAuto) {
      out.name = classify(remaining(*anchors, centres));
    }
    out.plan = plan(*anchors, centres, candidates);
  }
  for (const Index c : out.plan.fixed) {
    anchors->centre_cams.push_back(c);
    anchors->orientation_cams.push_back(c);
    out.pose_free[static_cast<std::size_t>(c)] = false;
  }
  for (const Index c : out.plan.confined) {
    out.pose_free[static_cast<std::size_t>(c)] = false;
  }
  out.anchors = std::move(*anchors);
  return out;
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

/// @brief Attach the depth priors and free the observed affines.
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

/// @brief Keep the observations of points that are not degenerate, and mark what they see.
/// @param problem The problem.
/// @param kept The observations in front of their camera.
/// @param degenerate Which points were dropped.
/// @param layout The layout to update: its observations and dropped-observation count.
/// @return Which poses and which intrinsics groups are observed.
std::pair<Mask, Mask> observe(const Problem& problem, const IndexList& kept,
                              const std::vector<bool>& degenerate, Layout& layout) {
  Mask observed(problem.poses.size(), false);
  Mask group_observed(static_cast<std::size_t>(problem.intrinsics.rows()), false);
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
  return {observed, group_observed};
}

/// @brief Free the observed intrinsics parameters, and count the unobserved free groups.
/// @param problem The problem.
/// @param group_observed Which intrinsics groups are observed.
/// @param layout The layout to update.
void free_intrinsics(const Problem& problem, const Mask& group_observed, Layout& layout) {
  const auto n_groups = static_cast<std::size_t>(problem.intrinsics.rows());
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
}

/// @brief Attach the pose priors on poses that move. A prior on a pose that never moves
///        (fixed, unobserved, or fixed by the gauge) is a constant: it informs nothing and is
///        left out, like the views of a dropped point.
/// @param problem The problem.
/// @param layout The layout to update (its gauge already applied).
void attach_pose_priors(const Problem& problem, Layout& layout) {
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    const IndexList& confined = layout.gauge_plan.confined;
    const bool moves =
        layout.pose_free[c] || std::ranges::find(confined, static_cast<Index>(c)) != confined.end();
    if (moves && (problem.pose_prior_weights.row(static_cast<Index>(c)).array() > 0.0).any()) {
      layout.prior_cams.push_back(static_cast<Index>(c));
    }
  }
}

}  // namespace

Result<Layout> make_layout(const Problem& problem, const Options& options) {
  const auto n_points = static_cast<std::size_t>(problem.points.rows());
  Layout layout;
  const IndexList kept = in_front(problem, layout.counts);
  std::vector<Index> views(n_points, 0);
  for (const Index m : kept) {
    ++views[static_cast<std::size_t>(problem.pt_idx[static_cast<std::size_t>(m)])];
  }
  const std::vector<bool> degenerate =
      degenerate_points(problem, kept, options.min_parallax_deg, views);
  layout.counts.dropped_points = std::count(degenerate.begin(), degenerate.end(), true);
  const auto [observed, group_observed] = observe(problem, kept, degenerate, layout);
  for (std::size_t c = 0; c < problem.poses.size(); ++c) {
    layout.counts.unobserved_poses += (!problem.fixed_poses[c] && !observed[c]) ? 1 : 0;
  }
  free_intrinsics(problem, group_observed, layout);
  const bool metric_depth = attach_priors(problem, layout);
  std::vector<bool> fixed_points_used(n_points, false);
  for (std::size_t p = 0; p < n_points; ++p) {
    fixed_points_used[p] = problem.fixed_points[p] && !degenerate[p] && views[p] > 0;
  }
  auto gauge = fix_gauge(problem, observed, metric_depth, fixed_points_used, options.gauge);
  if (!gauge) {
    return std::unexpected(gauge.error());
  }
  layout.counts.gauge = gauge->name;
  layout.counts.gauge_pose = gauge->plan.fixed.empty() ? -1 : gauge->plan.fixed.front();
  layout.counts.confined_pose = gauge->plan.confined.empty() ? -1 : gauge->plan.confined.front();
  layout.counts.free_gauge = gauge->plan.free;
  layout.pose_free = std::move(gauge->pose_free);
  layout.gauge_anchors = std::move(gauge->anchors);
  layout.gauge_plan = std::move(gauge->plan);
  attach_pose_priors(problem, layout);
  assign_points(problem, degenerate, views, layout);
  return layout;
}

}  // namespace slamlab::foundations::ba::detail
