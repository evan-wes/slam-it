/// @file
/// @brief The edge pipeline's settings: the C++ core's mirror of `slam_it.edge.options`.
///
/// Every threshold named and defaulted to ORB-SLAM2's; the Python class's docstring says which
/// ORB-SLAM quantity each stands for.
#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/orb.hpp"
#include "slam_it/foundations/stereo.hpp"

/// @brief The edge rung: sparse visual odometry (ORB-SLAM-lite).
namespace slam_it::edge {

/// @brief The pipeline's sensor modes (they cross the bindings by name, as `run.json` records
///        them).
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

/// @brief A mode's name, as `slam_it.edge.options.MODES` spells it.
/// @param mode The mode.
/// @return A static string.
[[nodiscard]] const char* name(Mode mode);

/// @brief The edge pipeline's settings (`slam_it.edge.options.Options`).
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
  /// @brief The stricter descriptor distance: initialisation, triangulation, fusion (`TH_LOW`).
  std::int64_t th_low{50};
  /// @brief The looser one: tracking (`TH_HIGH`).
  std::int64_t th_high{100};
  /// @brief The stereo row matching's Hamming distance must be below this.
  std::int64_t stereo_max_distance{75};
  /// @brief Its row band, in multiples of the right keypoint's level scale.
  double stereo_band{2.0};
  /// @brief Its SAD patch's half-width.
  std::int64_t stereo_window{5};
  /// @brief Its SAD search's half-range.
  std::int64_t stereo_search{5};
  /// @brief Its matches whose best sum exceeds this times the median are dropped.
  double stereo_outlier_factor{2.1};
  /// @brief Keypoints nearer than this many baselines are close.
  double close_depth{40.0};
  /// @brief A monocular frame's features are this many times `n_features` before the map.
  std::int64_t init_feature_factor{2};
  /// @brief A monocular frame takes part in initialisation with more than this many keypoints.
  std::int64_t min_init_features{100};
  /// @brief A stereo or RGB-D frame starts the map with more than this many keypoints.
  std::int64_t min_init_features_depth{500};
  /// @brief The monocular initialisation's search window, in pixels.
  double init_window{100.0};
  /// @brief Its ratio test.
  double init_ratio{0.9};
  /// @brief Matches it needs (fewer: the reference frame is replaced).
  std::int64_t init_min_matches{100};
  /// @brief A monocular start keeps the points whose rays' cosine is below this.
  double init_parallax_cos{0.99998};
  /// @brief The motion model's search window, in pixels at level 0.
  double motion_window{15.0};
  /// @brief The same for stereo.
  double stereo_motion_window{7.0};
  /// @brief Matches the motion model needs before refining.
  std::int64_t min_motion_matches{20};
  /// @brief Inliers the motion model must keep after refinement.
  std::int64_t min_motion_inliers{10};
  /// @brief The local map's search window factor.
  double local_window{1.0};
  /// @brief The same for RGB-D.
  double rgbd_local_window{3.0};
  /// @brief The same on a recovery's frame and the next, in every mode.
  double recovery_local_window{5.0};
  /// @brief The frames, the recovery's included, searched in the recovery window.
  std::int64_t recovery_window_frames{2};
  /// @brief Points seen at a cosine above this to their viewing direction get the narrower
  ///        window.
  double aligned_view_cos{0.998};
  /// @brief The narrower window's factor.
  double aligned_view_radius{2.5};
  /// @brief The wider window's factor.
  double oblique_view_radius{4.0};
  /// @brief The local map search's ratio test.
  double local_map_ratio{0.8};
  /// @brief Inliers the local map must give for the frame to be tracked.
  std::int64_t min_tracked{30};
  /// @brief Inliers a frame needs just after a recovery.
  std::int64_t recovery_tracked{50};
  /// @brief The outlier test for a 2D observation (chi-square, 2 dof, 95%).
  double chi2_mono{5.991};
  /// @brief The same for a 3D (left and right) one.
  double chi2_stereo{7.815};
  /// @brief Rounds of pose refinement and outlier classification.
  std::int64_t pose_rounds{4};
  /// @brief A point is in view only within this cosine of its mean viewing direction.
  double view_cos_limit{0.5};
  /// @brief A point is in view from this fraction of its minimum distance...
  double min_distance_margin{0.8};
  /// @brief ...to this multiple of its maximum.
  double max_distance_margin{1.2};
  /// @brief Frames after a recovery that need more inliers, and that make no keyframe once the
  ///        map holds more than this many (the frame rate).
  std::int64_t max_frames{30};
  /// @brief Stereo and RGB-D keyframe ratio of the reference keyframe's points.
  double ref_ratio{0.75};
  /// @brief The same for monocular.
  double mono_ref_ratio{0.9};
  /// @brief The keyframe ratio while the map has a single keyframe.
  double first_ref_ratio{0.4};
  /// @brief The reference keyframe's points counted have this many observations...
  std::int64_t ref_min_observations{3};
  /// @brief ...this many while the map holds two keyframes or fewer.
  std::int64_t ref_min_observations_early{2};
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
  /// @brief Keyframes a new stereo or RGB-D keyframe triangulates against.
  std::int64_t triangulation_neighbours{10};
  /// @brief The same for monocular.
  std::int64_t mono_triangulation_neighbours{20};
  /// @brief A triangulation match's chi-square bound to its epipolar line (1 dof).
  double epipolar_chi2{3.84};
  /// @brief Without depth, its least squared distance from the epipole over the level scale s^l.
  double min_epipole_distance2{100.0};
  /// @brief A monocular keyframe's least baseline to a neighbour, over its median depth.
  double min_baseline_depth{0.01};
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
  /// @brief Local BA runs once the map holds this many keyframes.
  std::int64_t ba_min_keyframes{3};
  /// @brief The monocular map's first bundle adjustment's iterations.
  std::int64_t init_ba_iterations{20};
  /// @brief A new point found in less than this fraction of its views is culled.
  double found_ratio{0.25};
  /// @brief A point is culled on evidence until this many keyframes after its creation.
  std::int64_t recent_point_keyframes{3};
  /// @brief A recent point this many keyframes old is culled on too few observations.
  std::int64_t point_cull_age{2};
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
  /// @brief The fallback's descriptor ratio test.
  double relocalize_ratio{0.75};
  /// @brief Failed frames in a row before the track is lost.
  std::int64_t lost_frames{5};
  /// @brief A map lost with this many keyframes or fewer is reset rather than kept.
  std::int64_t reset_keyframes{10};
  /// @brief Every RANSAC's seed.
  std::int64_t seed{0};
};

/// @brief An integer setting's name and member.
using IntegerOption = std::pair<const char*, std::int64_t Options::*>;

/// @brief A real setting's name and member.
using RealOption = std::pair<const char*, double Options::*>;

/// @brief `Options`' integer fields, by their Python names.
inline constexpr std::array<IntegerOption, 42> kIntegerOptions{{
    {"n_features", &Options::n_features},
    {"levels", &Options::levels},
    {"fast_threshold", &Options::fast_threshold},
    {"min_fast_threshold", &Options::min_fast_threshold},
    {"th_low", &Options::th_low},
    {"th_high", &Options::th_high},
    {"stereo_max_distance", &Options::stereo_max_distance},
    {"stereo_window", &Options::stereo_window},
    {"stereo_search", &Options::stereo_search},
    {"init_feature_factor", &Options::init_feature_factor},
    {"min_init_features", &Options::min_init_features},
    {"min_init_features_depth", &Options::min_init_features_depth},
    {"init_min_matches", &Options::init_min_matches},
    {"min_motion_matches", &Options::min_motion_matches},
    {"min_tracked", &Options::min_tracked},
    {"pose_rounds", &Options::pose_rounds},
    {"max_frames", &Options::max_frames},
    {"min_keyframe_inliers", &Options::min_keyframe_inliers},
    {"close_tracked", &Options::close_tracked},
    {"close_untracked", &Options::close_untracked},
    {"window", &Options::window},
    {"min_new_points", &Options::min_new_points},
    {"ba_iterations", &Options::ba_iterations},
    {"ba_refine_iterations", &Options::ba_refine_iterations},
    {"ba_min_keyframes", &Options::ba_min_keyframes},
    {"min_point_observations", &Options::min_point_observations},
    {"min_point_observations_depth", &Options::min_point_observations_depth},
    {"redundant_observers", &Options::redundant_observers},
    {"pnp_min_inliers", &Options::pnp_min_inliers},
    {"lost_frames", &Options::lost_frames},
    {"reset_keyframes", &Options::reset_keyframes},
    {"seed", &Options::seed},
    {"min_motion_inliers", &Options::min_motion_inliers},
    {"recovery_window_frames", &Options::recovery_window_frames},
    {"recovery_tracked", &Options::recovery_tracked},
    {"ref_min_observations", &Options::ref_min_observations},
    {"ref_min_observations_early", &Options::ref_min_observations_early},
    {"triangulation_neighbours", &Options::triangulation_neighbours},
    {"mono_triangulation_neighbours", &Options::mono_triangulation_neighbours},
    {"recent_point_keyframes", &Options::recent_point_keyframes},
    {"point_cull_age", &Options::point_cull_age},
    {"init_ba_iterations", &Options::init_ba_iterations},
}};

/// @brief `Options`' real fields, by their Python names.
inline constexpr std::array<RealOption, 34> kRealOptions{{
    {"scale_factor", &Options::scale_factor},
    {"stereo_band", &Options::stereo_band},
    {"stereo_outlier_factor", &Options::stereo_outlier_factor},
    {"close_depth", &Options::close_depth},
    {"init_window", &Options::init_window},
    {"init_ratio", &Options::init_ratio},
    {"init_parallax_cos", &Options::init_parallax_cos},
    {"motion_window", &Options::motion_window},
    {"stereo_motion_window", &Options::stereo_motion_window},
    {"local_window", &Options::local_window},
    {"rgbd_local_window", &Options::rgbd_local_window},
    {"recovery_local_window", &Options::recovery_local_window},
    {"chi2_mono", &Options::chi2_mono},
    {"chi2_stereo", &Options::chi2_stereo},
    {"view_cos_limit", &Options::view_cos_limit},
    {"ref_ratio", &Options::ref_ratio},
    {"mono_ref_ratio", &Options::mono_ref_ratio},
    {"max_parallax_cos", &Options::max_parallax_cos},
    {"scale_ratio", &Options::scale_ratio},
    {"fuse_window", &Options::fuse_window},
    {"found_ratio", &Options::found_ratio},
    {"redundant_fraction", &Options::redundant_fraction},
    {"pnp_threshold", &Options::pnp_threshold},
    {"aligned_view_cos", &Options::aligned_view_cos},
    {"aligned_view_radius", &Options::aligned_view_radius},
    {"oblique_view_radius", &Options::oblique_view_radius},
    {"local_map_ratio", &Options::local_map_ratio},
    {"min_distance_margin", &Options::min_distance_margin},
    {"max_distance_margin", &Options::max_distance_margin},
    {"first_ref_ratio", &Options::first_ref_ratio},
    {"epipolar_chi2", &Options::epipolar_chi2},
    {"min_epipole_distance2", &Options::min_epipole_distance2},
    {"min_baseline_depth", &Options::min_baseline_depth},
    {"relocalize_ratio", &Options::relocalize_ratio},
}};

/// @brief The ORB extraction settings.
/// @param options The settings.
/// @param initializing For a monocular frame before the map exists: `init_feature_factor`
///        times the features.
/// @return The extraction settings.
[[nodiscard]] foundations::orb::Options extraction(const Options& options,
                                                   bool initializing = false);

/// @brief The stereo pair's row-matching settings (`Options.stereo_matching`).
/// @param options The settings.
/// @return The `stereo_*` settings.
[[nodiscard]] foundations::stereo::MatchOptions stereo_matching(const Options& options);

/// @brief Check the settings (`Options.__post_init__`).
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the first one out of range.
[[nodiscard]] foundations::Result<void> check_options(const Options& options);

}  // namespace slam_it::edge
