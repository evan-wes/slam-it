/// @file
/// @brief The edge pipeline's settings: the C++ core's mirror of `slamlab.edge.options`.
///
/// Every threshold named and defaulted to ORB-SLAM2's; the Python class's docstring says which
/// ORB-SLAM quantity each stands for.
#pragma once

#include <cstdint>

#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/orb.hpp"

/// @brief The edge rung: sparse visual odometry (ORB-SLAM-lite).
namespace slamlab::edge {

/// @brief The pipeline's sensor modes, with the codes they cross the bindings as.
enum class Mode : std::uint8_t {
  /// @brief One camera.
  kMonocular = 0,
  /// @brief A rectified stereo pair.
  kStereo = 1,
  /// @brief A camera and a registered depth map.
  kRgbd = 2,
  /// @brief The number of modes (a sentinel, not a mode).
  kCount = 3,
};

/// @brief A mode's name, as `slamlab.edge.options.MODES` spells it.
/// @param mode The mode.
/// @return A static string.
[[nodiscard]] const char* name(Mode mode);

/// @brief ORB-SLAM's stricter descriptor distance (initialisation, triangulation, fusion).
inline constexpr std::int64_t kThLow = 50;
/// @brief Its looser one (tracking).
inline constexpr std::int64_t kThHigh = 100;

/// @brief The edge pipeline's settings (`slamlab.edge.options.Options`).
struct Options {
  /// @brief ORB features per frame.
  std::int64_t n_features{1000};
  /// @brief Pyramid levels.
  std::int64_t levels{8};
  /// @brief The pyramid's scale factor.
  double scale_factor{1.2};
  /// @brief The FAST threshold a cell tries first.
  std::int64_t fast_threshold{20};
  /// @brief The FAST threshold for cells with no corner at the first.
  std::int64_t min_fast_threshold{7};
  /// @brief Keypoints nearer than this many baselines are close.
  double close_depth{40.0};
  /// @brief A monocular frame's features are this many times `n_features` before the map.
  std::int64_t init_feature_factor{2};
  /// @brief Keypoints a monocular frame needs to take part in initialisation.
  std::int64_t min_init_features{100};
  /// @brief Keypoints with depth the first stereo or RGB-D keyframe needs.
  std::int64_t min_depth_features{500};
  /// @brief The monocular initialisation's search window, in pixels.
  double init_window{100.0};
  /// @brief Its ratio test.
  double init_ratio{0.9};
  /// @brief Matches it needs (fewer: the reference frame is replaced).
  std::int64_t init_min_matches{100};
  /// @brief The motion model's search window, in pixels at level 0.
  double motion_window{15.0};
  /// @brief The same for stereo.
  double stereo_motion_window{7.0};
  /// @brief Matches the motion model needs before refining.
  std::int64_t min_motion_matches{20};
  /// @brief The local map's search window factor.
  double local_window{1.0};
  /// @brief The same for RGB-D.
  double rgbd_local_window{3.0};
  /// @brief Inliers the local map must give for the frame to be tracked.
  std::int64_t min_tracked{30};
  /// @brief The outlier test for a 2D observation (chi-square, 2 dof, 95%).
  double chi2_mono{5.991};
  /// @brief The same for a 3D (left and right) one.
  double chi2_stereo{7.815};
  /// @brief Rounds of pose refinement and outlier classification.
  std::int64_t pose_rounds{4};
  /// @brief A point is in view only within this cosine of its mean viewing direction.
  double view_cos_limit{0.5};
  /// @brief A keyframe is due at the latest this many frames after the last.
  std::int64_t max_frames{30};
  /// @brief Stereo and RGB-D keyframe ratio of the reference keyframe's points.
  double ref_ratio{0.75};
  /// @brief The same for monocular.
  double mono_ref_ratio{0.9};
  /// @brief A keyframe needs more inliers than this.
  std::int64_t min_keyframe_inliers{15};
  /// @brief Close points tracked below which stereo and RGB-D may need a keyframe.
  std::int64_t close_tracked{100};
  /// @brief Untracked close points above which they do.
  std::int64_t close_untracked{70};
  /// @brief The sliding window's keyframes.
  std::int64_t window{10};
  /// @brief Points a stereo or RGB-D keyframe creates from depth, at least.
  std::int64_t min_new_points{100};
  /// @brief Rays whose cosine is above this are too parallel to triangulate.
  double max_parallax_cos{0.9998};
  /// @brief The scale-consistency factor of a triangulated point.
  double scale_ratio{1.5};
  /// @brief The fusion search window, in pixels at level 0.
  double fuse_window{3.0};
  /// @brief Local BA's iterations before dropping outliers.
  std::int64_t ba_iterations{5};
  /// @brief And after.
  std::int64_t ba_refine_iterations{10};
  /// @brief A new point found in less than this fraction of its views is culled.
  double found_ratio{0.25};
  /// @brief Monocular points this weakly observed two keyframes on are culled.
  std::int64_t min_point_observations{2};
  /// @brief The same for stereo and RGB-D.
  std::int64_t min_point_observations_depth{3};
  /// @brief A keyframe is redundant when this fraction of its points are seen elsewhere.
  double redundant_fraction{0.9};
  /// @brief By at least this many other keyframes.
  std::int64_t redundant_observers{3};
  /// @brief The PnP fallback's RANSAC threshold, in pixels.
  double pnp_threshold{4.0};
  /// @brief Inliers the PnP fallback needs.
  std::int64_t pnp_min_inliers{30};
  /// @brief Failed frames in a row before the track is lost.
  std::int64_t lost_frames{5};
  /// @brief Every RANSAC's seed.
  std::int64_t seed{0};
};

/// @brief The ORB extraction settings.
/// @param options The settings.
/// @param initializing For a monocular frame before the map exists: `init_feature_factor`
///        times the features.
/// @return The extraction settings.
[[nodiscard]] foundations::orb::Options extraction(const Options& options,
                                                   bool initializing = false);

/// @brief Check the settings (`Options.__post_init__`).
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the first one out of range.
[[nodiscard]] foundations::Result<void> check_options(const Options& options);

}  // namespace slamlab::edge
