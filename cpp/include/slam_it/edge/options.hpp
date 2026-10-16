/// @file
/// @brief The edge pipeline's settings: the C++ core's mirror of `slam_it.edge.options`.
///
/// Every threshold named and defaulted to ORB-SLAM2's; the Python class's docstring says which
/// ORB-SLAM quantity each stands for.
#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>
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

/// @brief ORB extraction's settings (`Options.features`).
struct FeatureOptions {
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
  /// @brief A monocular frame's features are this many times `features.n_features` before the map.
  std::int64_t init_factor{2};
};

/// @brief The descriptor distances matching takes in every stage (`Options.matching`).
struct MatchingOptions {
  /// @brief The stricter descriptor distance: initialisation, triangulation, fusion (`TH_LOW`).
  std::int64_t th_low{50};
  /// @brief The looser one: tracking (`TH_HIGH`).
  std::int64_t th_high{100};
};

/// @brief A stereo pair's row matching, and the close depth of stereo and RGB-D (`Options.stereo`).
struct StereoOptions {
  /// @brief The stereo row matching's Hamming distance must be below this.
  std::int64_t max_distance{75};
  /// @brief Its row band, in multiples of the right keypoint's level scale.
  double band{2.0};
  /// @brief Its SAD patch's half-width.
  std::int64_t window{5};
  /// @brief Its SAD search's half-range.
  std::int64_t search{5};
  /// @brief Its matches whose best sum exceeds this times the median are dropped.
  double outlier_factor{2.1};
  /// @brief Keypoints nearer than this many baselines are close.
  double close_depth{40.0};
};

/// @brief Starting the map (`Options.initialization`).
struct InitializationOptions {
  /// @brief A monocular frame takes part in initialisation with more than this many keypoints.
  std::int64_t min_features{100};
  /// @brief A stereo or RGB-D frame starts the map with more than this many keypoints.
  std::int64_t min_features_depth{500};
  /// @brief The monocular initialisation's search window, in pixels.
  double window{100.0};
  /// @brief Its ratio test.
  double ratio{0.9};
  /// @brief Matches it needs (fewer: the reference frame is replaced).
  std::int64_t min_matches{100};
  /// @brief A monocular start keeps the points whose rays' cosine is below this.
  double parallax_cos{0.99998};
  /// @brief The monocular map's first bundle adjustment's iterations.
  std::int64_t ba_iterations{20};
  /// @brief The most samples a monocular start's two-view RANSAC draws, per model.
  std::int64_t ransac_iterations{200};
};

/// @brief Placing each frame: the motion model, the local map, the PnP fallback
///        (`Options.tracking`).
struct TrackingOptions {
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
  /// @brief Frames after a recovery that need more inliers, and that make no keyframe once the
  ///        map holds more than this many (the frame rate).
  std::int64_t max_frames{30};
  /// @brief A point is in view only within this cosine of its mean viewing direction.
  double view_cos_limit{0.5};
  /// @brief A point is in view from this fraction of its minimum distance...
  double min_distance_margin{0.8};
  /// @brief ...to this multiple of its maximum.
  double max_distance_margin{1.2};
  /// @brief The PnP fallback's RANSAC threshold, in pixels.
  double pnp_threshold{4.0};
  /// @brief Inliers the PnP fallback needs.
  std::int64_t pnp_min_inliers{30};
};

/// @brief Pose optimisation and local BA (`Options.adjustment`).
struct AdjustmentOptions {
  /// @brief The outlier test for a 2D observation (chi-square, 2 dof, 95%).
  double chi2_mono{5.991};
  /// @brief The same for a 3D (left and right) one.
  double chi2_stereo{7.815};
  /// @brief Rounds of pose refinement and outlier classification.
  std::int64_t pose_rounds{4};
  /// @brief Local BA's iterations before dropping outliers.
  std::int64_t iterations{5};
  /// @brief And after.
  std::int64_t refine_iterations{10};
  /// @brief Local BA runs once the map holds this many keyframes.
  std::int64_t min_keyframes{3};
};

/// @brief The keyframe decision (`Options.keyframes`).
struct KeyframeOptions {
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
  std::int64_t min_inliers{15};
  /// @brief Close points tracked below which stereo and RGB-D may need a keyframe.
  std::int64_t close_tracked{100};
  /// @brief Untracked close points above which they do.
  std::int64_t close_untracked{70};
};

/// @brief Local mapping: points from depth, triangulation, fusion and culling (`Options.mapping`).
struct MappingOptions {
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
};

/// @brief The covisibility graph (`Options.covisibility`).
struct CovisibilityOptions {
  /// @brief Keyframes observing at least this many of the same points are covisible.
  std::int64_t min_points{15};
};

/// @brief Relocalising a failed frame against the whole map (`Options.relocalization`).
struct RelocalizationOptions {
  /// @brief Relocalise a failed frame through the keyframe database (needs a vocabulary).
  bool enabled{true};
  /// @brief The descriptor ratio test of the fallback and of relocalisation's matching by word.
  double ratio{0.75};
  /// @brief Relocalisation scores keyframes sharing more than this fraction of the most words.
  double common_words{0.8};
  /// @brief A scored keyframe's score is accumulated over this many best covisible keyframes.
  std::int64_t neighbours{10};
  /// @brief Groups above this fraction of the best accumulated score give a candidate.
  double group_score{0.75};
  /// @brief A candidate needs this many matches by word.
  std::int64_t min_matches{15};
  /// @brief A candidate's PnP RANSAC needs this fraction of its matches as inliers.
  double inlier_ratio{0.5};
  /// @brief ...and at least this many, before and after the pose is optimised.
  std::int64_t min_inliers{10};
  /// @brief The confidence its samples are drawn for, in (0, 1).
  double confidence{0.99};
  /// @brief The most samples it draws.
  std::int64_t ransac_iterations{300};
  /// @brief The window, in pixels at the predicted level, of the search by projection.
  double window{10.0};
  /// @brief With more inliers than this but too few still, the search is repeated narrower.
  std::int64_t narrow_inliers{30};
  /// @brief That window.
  double narrow_window{3.0};
  /// @brief Its descriptor distance.
  std::int64_t narrow_distance{64};
};

/// @brief When a lost track's map is closed (`Options.loss`).
struct LossOptions {
  /// @brief A recently lost track is lost once more than this has passed since its first
  ///        failed frame, in seconds.
  double relocalize_seconds{3.0};
  /// @brief A map of `loss.reset_keyframes` keyframes or fewer is lost after this many failed
  ///        frames in a row instead.
  std::int64_t lost_frames{5};
  /// @brief A map lost with this many keyframes or fewer is reset rather than kept.
  std::int64_t reset_keyframes{10};
};

/// @brief The edge pipeline's settings (`slam_it.edge.options.Options`), grouped by the stage
///        that reads them.
struct Options {
  /// @brief ORB extraction's settings.
  FeatureOptions features{};
  /// @brief The descriptor distances matching takes in every stage.
  MatchingOptions matching{};
  /// @brief A stereo pair's row matching, and the close depth of stereo and RGB-D.
  StereoOptions stereo{};
  /// @brief Starting the map.
  InitializationOptions initialization{};
  /// @brief Placing each frame: the motion model, the local map, the PnP fallback.
  TrackingOptions tracking{};
  /// @brief Pose optimisation and local BA.
  AdjustmentOptions adjustment{};
  /// @brief The keyframe decision.
  KeyframeOptions keyframes{};
  /// @brief Local mapping: points from depth, triangulation, fusion and culling.
  MappingOptions mapping{};
  /// @brief The covisibility graph.
  CovisibilityOptions covisibility{};
  /// @brief Relocalising a failed frame against the whole map.
  RelocalizationOptions relocalization{};
  /// @brief When a lost track's map is closed.
  LossOptions loss{};
  /// @brief Every RANSAC's seed.
  std::int64_t seed{0};
};

/// @brief Calls `visit(name, value)` for every setting, in `Options.settings()`' order: `name`
///        the setting's, `group.field` (or `seed`), and `value` a reference to its field, `const`
///        when `options` is. The one list of the settings: the bindings, a run's record, the
///        command line and the tests all go through it.
/// @tparam Settings `Options` or `const Options`.
/// @tparam Visit Called with a `const char*` and a reference to an `std::int64_t`, a `double`
///         or a `bool`.
/// @param options The settings.
/// @param visit The visitor (copied: a lambda that captures by reference).
template <class Settings, class Visit>
  requires std::same_as<std::remove_const_t<Settings>, Options>
constexpr void for_each_setting(Settings& options, Visit visit) {
  visit("features.n_features", options.features.n_features);
  visit("features.levels", options.features.levels);
  visit("features.scale_factor", options.features.scale_factor);
  visit("features.fast_threshold", options.features.fast_threshold);
  visit("features.min_fast_threshold", options.features.min_fast_threshold);
  visit("features.init_factor", options.features.init_factor);
  visit("matching.th_low", options.matching.th_low);
  visit("matching.th_high", options.matching.th_high);
  visit("stereo.max_distance", options.stereo.max_distance);
  visit("stereo.band", options.stereo.band);
  visit("stereo.window", options.stereo.window);
  visit("stereo.search", options.stereo.search);
  visit("stereo.outlier_factor", options.stereo.outlier_factor);
  visit("stereo.close_depth", options.stereo.close_depth);
  visit("initialization.min_features", options.initialization.min_features);
  visit("initialization.min_features_depth", options.initialization.min_features_depth);
  visit("initialization.window", options.initialization.window);
  visit("initialization.ratio", options.initialization.ratio);
  visit("initialization.min_matches", options.initialization.min_matches);
  visit("initialization.parallax_cos", options.initialization.parallax_cos);
  visit("initialization.ba_iterations", options.initialization.ba_iterations);
  visit("initialization.ransac_iterations", options.initialization.ransac_iterations);
  visit("tracking.motion_window", options.tracking.motion_window);
  visit("tracking.stereo_motion_window", options.tracking.stereo_motion_window);
  visit("tracking.min_motion_matches", options.tracking.min_motion_matches);
  visit("tracking.min_motion_inliers", options.tracking.min_motion_inliers);
  visit("tracking.local_window", options.tracking.local_window);
  visit("tracking.rgbd_local_window", options.tracking.rgbd_local_window);
  visit("tracking.recovery_local_window", options.tracking.recovery_local_window);
  visit("tracking.recovery_window_frames", options.tracking.recovery_window_frames);
  visit("tracking.aligned_view_cos", options.tracking.aligned_view_cos);
  visit("tracking.aligned_view_radius", options.tracking.aligned_view_radius);
  visit("tracking.oblique_view_radius", options.tracking.oblique_view_radius);
  visit("tracking.local_map_ratio", options.tracking.local_map_ratio);
  visit("tracking.min_tracked", options.tracking.min_tracked);
  visit("tracking.recovery_tracked", options.tracking.recovery_tracked);
  visit("tracking.max_frames", options.tracking.max_frames);
  visit("tracking.view_cos_limit", options.tracking.view_cos_limit);
  visit("tracking.min_distance_margin", options.tracking.min_distance_margin);
  visit("tracking.max_distance_margin", options.tracking.max_distance_margin);
  visit("tracking.pnp_threshold", options.tracking.pnp_threshold);
  visit("tracking.pnp_min_inliers", options.tracking.pnp_min_inliers);
  visit("adjustment.chi2_mono", options.adjustment.chi2_mono);
  visit("adjustment.chi2_stereo", options.adjustment.chi2_stereo);
  visit("adjustment.pose_rounds", options.adjustment.pose_rounds);
  visit("adjustment.iterations", options.adjustment.iterations);
  visit("adjustment.refine_iterations", options.adjustment.refine_iterations);
  visit("adjustment.min_keyframes", options.adjustment.min_keyframes);
  visit("keyframes.ref_ratio", options.keyframes.ref_ratio);
  visit("keyframes.mono_ref_ratio", options.keyframes.mono_ref_ratio);
  visit("keyframes.first_ref_ratio", options.keyframes.first_ref_ratio);
  visit("keyframes.ref_min_observations", options.keyframes.ref_min_observations);
  visit("keyframes.ref_min_observations_early", options.keyframes.ref_min_observations_early);
  visit("keyframes.min_inliers", options.keyframes.min_inliers);
  visit("keyframes.close_tracked", options.keyframes.close_tracked);
  visit("keyframes.close_untracked", options.keyframes.close_untracked);
  visit("mapping.window", options.mapping.window);
  visit("mapping.min_new_points", options.mapping.min_new_points);
  visit("mapping.triangulation_neighbours", options.mapping.triangulation_neighbours);
  visit("mapping.mono_triangulation_neighbours", options.mapping.mono_triangulation_neighbours);
  visit("mapping.epipolar_chi2", options.mapping.epipolar_chi2);
  visit("mapping.min_epipole_distance2", options.mapping.min_epipole_distance2);
  visit("mapping.min_baseline_depth", options.mapping.min_baseline_depth);
  visit("mapping.max_parallax_cos", options.mapping.max_parallax_cos);
  visit("mapping.scale_ratio", options.mapping.scale_ratio);
  visit("mapping.fuse_window", options.mapping.fuse_window);
  visit("mapping.found_ratio", options.mapping.found_ratio);
  visit("mapping.recent_point_keyframes", options.mapping.recent_point_keyframes);
  visit("mapping.point_cull_age", options.mapping.point_cull_age);
  visit("mapping.min_point_observations", options.mapping.min_point_observations);
  visit("mapping.min_point_observations_depth", options.mapping.min_point_observations_depth);
  visit("mapping.redundant_fraction", options.mapping.redundant_fraction);
  visit("mapping.redundant_observers", options.mapping.redundant_observers);
  visit("covisibility.min_points", options.covisibility.min_points);
  visit("relocalization.enabled", options.relocalization.enabled);
  visit("relocalization.ratio", options.relocalization.ratio);
  visit("relocalization.common_words", options.relocalization.common_words);
  visit("relocalization.neighbours", options.relocalization.neighbours);
  visit("relocalization.group_score", options.relocalization.group_score);
  visit("relocalization.min_matches", options.relocalization.min_matches);
  visit("relocalization.inlier_ratio", options.relocalization.inlier_ratio);
  visit("relocalization.min_inliers", options.relocalization.min_inliers);
  visit("relocalization.confidence", options.relocalization.confidence);
  visit("relocalization.ransac_iterations", options.relocalization.ransac_iterations);
  visit("relocalization.window", options.relocalization.window);
  visit("relocalization.narrow_inliers", options.relocalization.narrow_inliers);
  visit("relocalization.narrow_window", options.relocalization.narrow_window);
  visit("relocalization.narrow_distance", options.relocalization.narrow_distance);
  visit("loss.relocalize_seconds", options.loss.relocalize_seconds);
  visit("loss.lost_frames", options.loss.lost_frames);
  visit("loss.reset_keyframes", options.loss.reset_keyframes);
  visit("seed", options.seed);
}

/// @brief A setting's name split into its group and its field: an empty group for `seed`.
/// @param name The setting's name, `group.field` or `seed`.
/// @return The group and the field.
[[nodiscard]] constexpr std::pair<std::string_view, std::string_view> split_setting(
    std::string_view name) {
  const std::size_t dot = name.find('.');
  if (dot == std::string_view::npos) {
    return {std::string_view{}, name};
  }
  return {name.substr(0, dot), name.substr(dot + 1)};
}

/// @brief The ORB extraction settings.
/// @param options The settings.
/// @param initializing For a monocular frame before the map exists: `features.init_factor`
///        times the features.
/// @return The extraction settings.
[[nodiscard]] foundations::orb::Options extraction(const Options& options,
                                                   bool initializing = false);

/// @brief The stereo pair's row-matching settings (`Options.stereo_matching`).
/// @param options The settings.
/// @return The `stereo` group's settings, but the close depth.
[[nodiscard]] foundations::stereo::MatchOptions stereo_matching(const Options& options);

/// @brief Check the settings (`Options.__post_init__`).
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the first one out of range.
[[nodiscard]] foundations::Result<void> check_options(const Options& options);

}  // namespace slam_it::edge
