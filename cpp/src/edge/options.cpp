/// @file
/// @brief Implementation of the settings declared in slamlab/edge/options.hpp.
#include "slamlab/edge/options.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace slamlab::edge {

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

foundations::Result<void> check_options(const Options& options) {
  if (auto ok = foundations::orb::check_options(extraction(options)); !ok) {
    return ok;
  }
  // Settings that may be 0, then those that must be at least 1.
  const std::array<IntSetting, 3> may_be_zero{
      {{"seed", options.seed},
       {"min_init_features", options.min_init_features},
       {"min_init_features_depth", options.min_init_features_depth}}};
  for (const auto& [setting, value] : may_be_zero) {
    if (auto ok = foundations::check_range(setting, value, 0, kMaxSetting); !ok) {
      return ok;
    }
  }
  const std::array<IntSetting, 18> counts{{
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
  }};
  for (const auto& [setting, value] : counts) {
    if (auto ok = foundations::check_range(setting, value, 1, kMaxSetting); !ok) {
      return ok;
    }
  }
  const std::array<IntSetting, 5> more{{
      {"min_point_observations", options.min_point_observations},
      {"min_point_observations_depth", options.min_point_observations_depth},
      {"redundant_observers", options.redundant_observers},
      {"pnp_min_inliers", options.pnp_min_inliers},
      {"lost_frames", options.lost_frames},
  }};
  for (const auto& [setting, value] : more) {
    if (auto ok = foundations::check_range(setting, value, 1, kMaxSetting); !ok) {
      return ok;
    }
  }
  const std::array<RealSetting, 13> positive{{
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
  }};
  for (const auto& [setting, value] : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      return foundations::fail_setting(setting, "must be finite and positive", value);
    }
  }
  const std::array<RealSetting, 8> fractions{{
      {"init_ratio", options.init_ratio},
      {"init_parallax_cos", options.init_parallax_cos},
      {"ref_ratio", options.ref_ratio},
      {"mono_ref_ratio", options.mono_ref_ratio},
      {"found_ratio", options.found_ratio},
      {"redundant_fraction", options.redundant_fraction},
      {"view_cos_limit", options.view_cos_limit},
      {"max_parallax_cos", options.max_parallax_cos},
  }};
  for (const auto& [setting, value] : fractions) {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
      return foundations::fail_setting(setting, "must be in (0, 1]", value);
    }
  }
  return {};
}

}  // namespace slamlab::edge
