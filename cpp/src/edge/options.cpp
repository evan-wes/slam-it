/// @file
/// @brief Implementation of the settings declared in slam_it/edge/options.hpp.
#include "slam_it/edge/options.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include "slam_it/foundations/matching.hpp"

namespace slam_it::edge {

namespace {

/// @brief The largest an integer setting may be (Python's `1 << 30`).
constexpr std::int64_t kMaxSetting = 1'073'741'824;

/// @brief An integer setting, by name.
using IntSetting = std::pair<const char*, std::int64_t>;

/// @brief A real setting, by name.
using RealSetting = std::pair<const char*, double>;

/// @brief A group's own check's error, its setting named by the group too.
/// @param group The group's name.
/// @param error The error, naming the setting by its field.
/// @return The error, naming it `group.field`.
foundations::Result<void> in_group(const char* group, const foundations::Error& error) {
  return std::unexpected(
      foundations::Error{.kind = error.kind, .message = std::string(group) + "." + error.message});
}

}  // namespace

const char* name(Mode mode) {
  switch (mode) {
    case Mode::kMonocular:
      return "monocular";
    case Mode::kStereo:
      return "stereo";
    case Mode::kRgbd:
      return "rgbd";
    case Mode::kCount:
      break;
  }
  return "unknown";
}

foundations::orb::Options extraction(const Options& options, bool initializing) {
  const std::int64_t factor = initializing ? options.features.init_factor : 1;
  return {
      .n_features = std::min(options.features.n_features * factor, foundations::orb::kMaxFeatures),
      .levels = options.features.levels,
      .scale_factor = options.features.scale_factor,
      .fast_threshold = options.features.fast_threshold,
      .min_fast_threshold = options.features.min_fast_threshold};
}

foundations::stereo::MatchOptions stereo_matching(const Options& options) {
  return {.max_distance = options.stereo.max_distance,
          .band = options.stereo.band,
          .window = options.stereo.window,
          .search = options.stereo.search,
          .outlier_factor = options.stereo.outlier_factor};
}

foundations::Result<void> check_options(const Options& options) {
  if (auto ok = foundations::orb::check_options(extraction(options)); !ok) {
    return in_group("features", ok.error());
  }
  if (auto ok = foundations::stereo::check_options(stereo_matching(options)); !ok) {
    return in_group("stereo", ok.error());
  }
  for (const auto& [setting, value] :
       {std::pair{"matching.th_low", options.matching.th_low},
        std::pair{"matching.th_high", options.matching.th_high},
        std::pair{"relocalization.narrow_distance", options.relocalization.narrow_distance}}) {
    if (auto ok = foundations::check_range(setting, value, 1, foundations::matching::kMaxDistance);
        !ok) {
      return ok;
    }
  }
  // Settings that may be 0, then those that must be at least 1.
  const std::array<IntSetting, 4> may_be_zero{
      {{"seed", options.seed},
       {"initialization.min_features", options.initialization.min_features},
       {"initialization.min_features_depth", options.initialization.min_features_depth},
       {"loss.reset_keyframes", options.loss.reset_keyframes}}};
  for (const auto& [setting, value] : may_be_zero) {
    if (auto ok = foundations::check_range(setting, value, 0, kMaxSetting); !ok) {
      return ok;
    }
  }
  const std::array<IntSetting, 45> counts{{
      {"features.n_features", options.features.n_features},
      {"features.levels", options.features.levels},
      {"features.fast_threshold", options.features.fast_threshold},
      {"features.min_fast_threshold", options.features.min_fast_threshold},
      {"features.init_factor", options.features.init_factor},
      {"initialization.min_matches", options.initialization.min_matches},
      {"tracking.min_motion_matches", options.tracking.min_motion_matches},
      {"tracking.min_tracked", options.tracking.min_tracked},
      {"adjustment.pose_rounds", options.adjustment.pose_rounds},
      {"tracking.max_frames", options.tracking.max_frames},
      {"keyframes.min_inliers", options.keyframes.min_inliers},
      {"keyframes.close_tracked", options.keyframes.close_tracked},
      {"keyframes.close_untracked", options.keyframes.close_untracked},
      {"mapping.min_new_points", options.mapping.min_new_points},
      {"adjustment.iterations", options.adjustment.iterations},
      {"adjustment.refine_iterations", options.adjustment.refine_iterations},
      {"adjustment.min_keyframes", options.adjustment.min_keyframes},
      {"mapping.min_point_observations", options.mapping.min_point_observations},
      {"mapping.min_point_observations_depth", options.mapping.min_point_observations_depth},
      {"mapping.redundant_observers", options.mapping.redundant_observers},
      {"tracking.pnp_min_inliers", options.tracking.pnp_min_inliers},
      {"loss.lost_frames", options.loss.lost_frames},
      {"tracking.min_motion_inliers", options.tracking.min_motion_inliers},
      {"tracking.recovery_window_frames", options.tracking.recovery_window_frames},
      {"tracking.recovery_tracked", options.tracking.recovery_tracked},
      {"keyframes.ref_min_observations", options.keyframes.ref_min_observations},
      {"keyframes.ref_min_observations_early", options.keyframes.ref_min_observations_early},
      {"mapping.triangulation_neighbours", options.mapping.triangulation_neighbours},
      {"mapping.mono_triangulation_neighbours", options.mapping.mono_triangulation_neighbours},
      {"mapping.recent_point_keyframes", options.mapping.recent_point_keyframes},
      {"mapping.fuse_neighbours", options.mapping.fuse_neighbours},
      {"mapping.mono_fuse_neighbours", options.mapping.mono_fuse_neighbours},
      {"mapping.fuse_second_neighbours", options.mapping.fuse_second_neighbours},
      {"mapping.point_cull_age", options.mapping.point_cull_age},
      {"initialization.ba_iterations", options.initialization.ba_iterations},
      {"initialization.ransac_iterations", options.initialization.ransac_iterations},
      {"covisibility.min_points", options.covisibility.min_points},
      {"covisibility.local_keyframes", options.covisibility.local_keyframes},
      {"covisibility.local_neighbours", options.covisibility.local_neighbours},
      {"covisibility.strong_points", options.covisibility.strong_points},
      {"relocalization.neighbours", options.relocalization.neighbours},
      {"relocalization.min_matches", options.relocalization.min_matches},
      {"relocalization.min_inliers", options.relocalization.min_inliers},
      {"relocalization.ransac_iterations", options.relocalization.ransac_iterations},
      {"relocalization.narrow_inliers", options.relocalization.narrow_inliers},
  }};
  for (const auto& [setting, value] : counts) {
    if (auto ok = foundations::check_range(setting, value, 1, kMaxSetting); !ok) {
      return ok;
    }
  }
  const std::array<RealSetting, 21> positive{{
      {"features.scale_factor", options.features.scale_factor},
      {"stereo.close_depth", options.stereo.close_depth},
      {"initialization.window", options.initialization.window},
      {"tracking.motion_window", options.tracking.motion_window},
      {"tracking.stereo_motion_window", options.tracking.stereo_motion_window},
      {"tracking.local_window", options.tracking.local_window},
      {"tracking.rgbd_local_window", options.tracking.rgbd_local_window},
      {"tracking.recovery_local_window", options.tracking.recovery_local_window},
      {"adjustment.chi2_mono", options.adjustment.chi2_mono},
      {"adjustment.chi2_stereo", options.adjustment.chi2_stereo},
      {"mapping.scale_ratio", options.mapping.scale_ratio},
      {"mapping.fuse_window", options.mapping.fuse_window},
      {"tracking.pnp_threshold", options.tracking.pnp_threshold},
      {"tracking.aligned_view_radius", options.tracking.aligned_view_radius},
      {"tracking.oblique_view_radius", options.tracking.oblique_view_radius},
      {"tracking.max_distance_margin", options.tracking.max_distance_margin},
      {"mapping.epipolar_chi2", options.mapping.epipolar_chi2},
      {"mapping.min_epipole_distance2", options.mapping.min_epipole_distance2},
      {"relocalization.window", options.relocalization.window},
      {"relocalization.narrow_window", options.relocalization.narrow_window},
      {"loss.relocalize_seconds", options.loss.relocalize_seconds},
  }};
  for (const auto& [setting, value] : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      return foundations::fail_setting(setting, "must be finite and positive", value);
    }
  }
  const std::array<RealSetting, 18> fractions{{
      {"initialization.ratio", options.initialization.ratio},
      {"initialization.parallax_cos", options.initialization.parallax_cos},
      {"keyframes.ref_ratio", options.keyframes.ref_ratio},
      {"keyframes.mono_ref_ratio", options.keyframes.mono_ref_ratio},
      {"mapping.found_ratio", options.mapping.found_ratio},
      {"mapping.redundant_fraction", options.mapping.redundant_fraction},
      {"tracking.view_cos_limit", options.tracking.view_cos_limit},
      {"mapping.max_parallax_cos", options.mapping.max_parallax_cos},
      {"tracking.aligned_view_cos", options.tracking.aligned_view_cos},
      {"tracking.local_map_ratio", options.tracking.local_map_ratio},
      {"tracking.min_distance_margin", options.tracking.min_distance_margin},
      {"keyframes.first_ref_ratio", options.keyframes.first_ref_ratio},
      {"mapping.min_baseline_depth", options.mapping.min_baseline_depth},
      {"relocalization.ratio", options.relocalization.ratio},
      {"relocalization.common_words", options.relocalization.common_words},
      {"relocalization.group_score", options.relocalization.group_score},
      {"relocalization.inlier_ratio", options.relocalization.inlier_ratio},
      {"relocalization.confidence", options.relocalization.confidence},
  }};
  for (const auto& [setting, value] : fractions) {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
      return foundations::fail_setting(setting, "must be in (0, 1]", value);
    }
  }
  if (options.relocalization.confidence >= 1.0) {
    return foundations::fail_setting("relocalization.confidence", "must be in (0, 1)",
                                     options.relocalization.confidence);
  }
  return {};
}

}  // namespace slam_it::edge
