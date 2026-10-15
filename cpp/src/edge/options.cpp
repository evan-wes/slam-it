/// @file
/// @brief Implementation of the settings declared in slam_it/edge/options.hpp.
#include "slam_it/edge/options.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
  const std::int64_t factor = initializing ? options.init_feature_factor : 1;
  return {.n_features = std::min(options.n_features * factor, foundations::orb::kMaxFeatures),
          .levels = options.levels,
          .scale_factor = options.scale_factor,
          .fast_threshold = options.fast_threshold,
          .min_fast_threshold = options.min_fast_threshold};
}

foundations::stereo::MatchOptions stereo_matching(const Options& options) {
  return {.max_distance = options.stereo_max_distance,
          .band = options.stereo_band,
          .window = options.stereo_window,
          .search = options.stereo_search,
          .outlier_factor = options.stereo_outlier_factor};
}

foundations::Result<void> check_options(const Options& options) {
  if (auto ok = foundations::orb::check_options(extraction(options)); !ok) {
    return ok;
  }
  if (auto ok = foundations::stereo::check_options(stereo_matching(options)); !ok) {
    return std::unexpected(
        foundations::Error{.kind = ok.error().kind, .message = "stereo_" + ok.error().message});
  }
  for (const auto& [setting, value] :
       {std::pair{"th_low", options.th_low}, std::pair{"th_high", options.th_high},
        std::pair{"reloc_narrow_distance", options.reloc_narrow_distance}}) {
    if (auto ok = foundations::check_range(setting, value, 1, foundations::matching::kMaxDistance);
        !ok) {
      return ok;
    }
  }
  // Settings that may be 0, then those that must be at least 1.
  const std::array<IntSetting, 4> may_be_zero{
      {{"seed", options.seed},
       {"min_init_features", options.min_init_features},
       {"min_init_features_depth", options.min_init_features_depth},
       {"reset_keyframes", options.reset_keyframes}}};
  for (const auto& [setting, value] : may_be_zero) {
    if (auto ok = foundations::check_range(setting, value, 0, kMaxSetting); !ok) {
      return ok;
    }
  }
  const std::array<IntSetting, 40> counts{{
      {"n_features", options.n_features},
      {"levels", options.levels},
      {"fast_threshold", options.fast_threshold},
      {"min_fast_threshold", options.min_fast_threshold},
      {"init_feature_factor", options.init_feature_factor},
      {"init_min_matches", options.init_min_matches},
      {"min_motion_matches", options.min_motion_matches},
      {"min_tracked", options.min_tracked},
      {"pose_rounds", options.pose_rounds},
      {"max_frames", options.max_frames},
      {"min_keyframe_inliers", options.min_keyframe_inliers},
      {"close_tracked", options.close_tracked},
      {"close_untracked", options.close_untracked},
      {"window", options.window},
      {"min_new_points", options.min_new_points},
      {"ba_iterations", options.ba_iterations},
      {"ba_refine_iterations", options.ba_refine_iterations},
      {"ba_min_keyframes", options.ba_min_keyframes},
      {"min_point_observations", options.min_point_observations},
      {"min_point_observations_depth", options.min_point_observations_depth},
      {"redundant_observers", options.redundant_observers},
      {"pnp_min_inliers", options.pnp_min_inliers},
      {"lost_frames", options.lost_frames},
      {"min_motion_inliers", options.min_motion_inliers},
      {"recovery_window_frames", options.recovery_window_frames},
      {"recovery_tracked", options.recovery_tracked},
      {"ref_min_observations", options.ref_min_observations},
      {"ref_min_observations_early", options.ref_min_observations_early},
      {"triangulation_neighbours", options.triangulation_neighbours},
      {"mono_triangulation_neighbours", options.mono_triangulation_neighbours},
      {"recent_point_keyframes", options.recent_point_keyframes},
      {"point_cull_age", options.point_cull_age},
      {"init_ba_iterations", options.init_ba_iterations},
      {"init_ransac_iterations", options.init_ransac_iterations},
      {"covisible_points", options.covisible_points},
      {"reloc_neighbours", options.reloc_neighbours},
      {"reloc_min_matches", options.reloc_min_matches},
      {"reloc_min_inliers", options.reloc_min_inliers},
      {"reloc_ransac_iterations", options.reloc_ransac_iterations},
      {"reloc_narrow_inliers", options.reloc_narrow_inliers},
  }};
  for (const auto& [setting, value] : counts) {
    if (auto ok = foundations::check_range(setting, value, 1, kMaxSetting); !ok) {
      return ok;
    }
  }
  const std::array<RealSetting, 20> positive{{
      {"scale_factor", options.scale_factor},
      {"close_depth", options.close_depth},
      {"init_window", options.init_window},
      {"motion_window", options.motion_window},
      {"stereo_motion_window", options.stereo_motion_window},
      {"local_window", options.local_window},
      {"rgbd_local_window", options.rgbd_local_window},
      {"recovery_local_window", options.recovery_local_window},
      {"chi2_mono", options.chi2_mono},
      {"chi2_stereo", options.chi2_stereo},
      {"scale_ratio", options.scale_ratio},
      {"fuse_window", options.fuse_window},
      {"pnp_threshold", options.pnp_threshold},
      {"aligned_view_radius", options.aligned_view_radius},
      {"oblique_view_radius", options.oblique_view_radius},
      {"max_distance_margin", options.max_distance_margin},
      {"epipolar_chi2", options.epipolar_chi2},
      {"min_epipole_distance2", options.min_epipole_distance2},
      {"reloc_window", options.reloc_window},
      {"reloc_narrow_window", options.reloc_narrow_window},
  }};
  for (const auto& [setting, value] : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      return foundations::fail_setting(setting, "must be finite and positive", value);
    }
  }
  const std::array<RealSetting, 18> fractions{{
      {"init_ratio", options.init_ratio},
      {"init_parallax_cos", options.init_parallax_cos},
      {"ref_ratio", options.ref_ratio},
      {"mono_ref_ratio", options.mono_ref_ratio},
      {"found_ratio", options.found_ratio},
      {"redundant_fraction", options.redundant_fraction},
      {"view_cos_limit", options.view_cos_limit},
      {"max_parallax_cos", options.max_parallax_cos},
      {"aligned_view_cos", options.aligned_view_cos},
      {"local_map_ratio", options.local_map_ratio},
      {"min_distance_margin", options.min_distance_margin},
      {"first_ref_ratio", options.first_ref_ratio},
      {"min_baseline_depth", options.min_baseline_depth},
      {"relocalize_ratio", options.relocalize_ratio},
      {"reloc_common_words", options.reloc_common_words},
      {"reloc_group_score", options.reloc_group_score},
      {"reloc_inlier_ratio", options.reloc_inlier_ratio},
      {"reloc_confidence", options.reloc_confidence},
  }};
  for (const auto& [setting, value] : fractions) {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
      return foundations::fail_setting(setting, "must be in (0, 1]", value);
    }
  }
  if (options.reloc_confidence >= 1.0) {
    return foundations::fail_setting("reloc_confidence", "must be in (0, 1)",
                                     options.reloc_confidence);
  }
  return {};
}

}  // namespace slam_it::edge
