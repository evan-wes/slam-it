/// @file
/// @brief Loop closing: the C++ core's mirror of `slam_it.edge.loops` (ORB-SLAM3's
///        `LoopClosing`), whose docstring states the steps and where they depart from ORB-SLAM3's
///        source: detection (a loop being confirmed carried to the new keyframe, else the
///        database's candidates tried with their groups), and correction (the keyframe and its
///        links moved by the loop's similarity, the loop's points fused, the essential graph
///        optimised, the loop edge added, then global BA); and merging (another map's place
///        found by detection's stages, each search taking the map searched as well as the
///        keyframe's own, and the active map brought into it, welded, and the weld spread:
///        ORB-SLAM3's `MergeLocal`, its correction as its paper gives it).
///
/// Similarities are ORB-SLAM's `S_cw`, `[[s R, t], [0, 1]]`; a search projects through the rigid
/// `(R, t / s)`. Stereo and RGB-D measure their scale, so their similarities are rigid.
#pragma once

#include <Eigen/Core>
#include <atomic>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "slam_it/edge/database.hpp"
#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/graphs.hpp"

namespace slam_it::edge::loops {

/// @brief A similarity's scale: the length of a column of `s R`.
/// @param similarity `[[s R, t], [0, 1]]`.
/// @return `s`.
[[nodiscard]] double scale_of(const Eigen::Matrix4d& similarity);

/// @brief The rigid pose a similarity projects through: `[[R, t / s], [0, 1]]`.
/// @param similarity `S_cw`.
/// @return `T_cw`.
[[nodiscard]] Eigen::Matrix4d rigid_of(const Eigen::Matrix4d& similarity);

/// @brief The settings of one search through a similarity.
struct Window {
  /// @brief The window's half-width, in pixels at level 0.
  double radius{0.0};
  /// @brief The largest Hamming distance a match may have.
  std::int64_t distance{0};
};

/// @brief Match points to a keyframe's keypoints through a similarity
///        (`slam_it.edge.loops.search_by_similarity`, `SearchByProjection`).
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param similarity Its `S_cw`.
/// @param points The points' rows, in the order searched.
/// @param window The window and the distance.
/// @param options `tracking.view_cos_limit`.
/// @return The id of the point each of the keyframe's keypoints matched, -1 where none;
///         `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> search_by_similarity(
    const Map& m, const Camera& camera, std::int64_t k, const Eigen::Matrix4d& similarity,
    std::span<const std::int64_t> points, Window window, const Options& options);

/// @brief Match two keyframes' points by word (`slam_it.edge.loops.match_by_word`).
/// @param m The map.
/// @param db Its keyframe database, holding both keyframes.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @param options `matching.th_low` and `loops.ratio`.
/// @return For each keypoint of `k1`, the id of the point of `k2` it matched, -1 where none;
///         `InvalidInput` for a row out of range or a keyframe the database does not hold.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k1, k2), as Python's
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> match_by_word(
    const Map& m, const Database& db, std::int64_t k1, std::int64_t k2, const Options& options);

/// @brief The points a matched keyframe's group sees (`slam_it.edge.loops.group_points`,
///        `FindMatchesByProjection`'s).
/// @param m The map.
/// @param matched The matched keyframe's row.
/// @param linked The rows of the searching keyframe's links, never added by widening.
/// @param options `loops.neighbours` and `covisibility.min_points`.
/// @return The points' rows, each once, in the group's order; `InvalidInput` for a row out of
///         range.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> group_points(
    const Map& m, std::int64_t matched, const std::set<std::int64_t>& linked,
    const Options& options);

/// @brief The most samples the loop's RANSAC draws (`slam_it.edge.loops.ransac_budget`,
///        `SetRansacParameters`).
/// @param matches The matches.
/// @param options `loops.min_inliers`, `loops.confidence` and `loops.ransac_iterations`.
/// @return The samples, at least 1.
[[nodiscard]] std::int64_t ransac_budget(std::int64_t matches, const Options& options);

/// @brief A similarity refined against a keyframe's matches (`slam_it.edge.loops.Refined`).
struct Refined {
  /// @brief The refined `S_12`.
  Eigen::Matrix4d similarity;
  /// @brief The matches whose errors stayed within `loops.huber_chi2` in both images.
  std::int64_t inliers{0};
};

/// @brief Refine the similarity between two keyframes against their matches
///        (`slam_it.edge.loops.refine`, `OptimizeSim3`).
/// @param m The map.
/// @param camera The camera.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @param similarity The starting `S_12`.
/// @param matched For each keypoint of `k1`, the id of the point it matched, or -1.
/// @param fixed_scale Refine a rigid motion.
/// @param options The `loops` refinement's settings.
/// @return The similarity and its inliers, or none when too few matches survive the first
///         round or none can be used; `InvalidInput` for rows out of range or matches not one
///         per keypoint; `NotConverged` for GTSAM's failure.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k1, k2), as Python's
[[nodiscard]] foundations::Result<std::optional<Refined>> refine(
    const Map& m, const Camera& camera, std::int64_t k1, std::int64_t k2,
    const Eigen::Matrix4d& similarity, std::span<const std::int64_t> matched, bool fixed_scale,
    const Options& options);

/// @brief What detection made of a keyframe (`slam_it.edge.loops.Outcome`).
enum class Outcome : std::uint8_t {
  /// @brief The map held too few keyframes to search.
  kSkipped = 0,
  /// @brief No loop: none was being confirmed, and no candidate passed.
  kNone = 1,
  /// @brief A candidate passed, with too few confirmations to close.
  kStarted = 2,
  /// @brief The loop being confirmed was confirmed again, not yet enough to close.
  kConfirmed = 3,
  /// @brief A loop closed on this keyframe.
  kClosed = 4,
  /// @brief The loop being confirmed went unconfirmed too many keyframes in a row.
  kDropped = 5,
  /// @brief A loop closed on the keyframe was dropped for a merge found on it (`merge_first`).
  kMerged = 6,
  /// @brief The number of outcomes (a sentinel).
  kCount = 7,
};

/// @brief An outcome's name, as Python's enum value.
/// @param outcome The outcome.
/// @return Its name.
[[nodiscard]] const char* name(Outcome outcome);

/// @brief A loop being confirmed, carried from keyframe to keyframe (`slam_it.edge.loops.Pending`).
struct Pending {
  /// @brief The id of the keyframe it matched.
  std::int64_t matched{0};
  /// @brief The id of the last keyframe that confirmed it.
  std::int64_t last{0};
  /// @brief That keyframe's `S_cw` as the loop places it.
  Eigen::Matrix4d similarity{Eigen::Matrix4d::Identity()};
  /// @brief Its confirmations so far.
  std::int64_t confirmations{0};
  /// @brief The keyframes in a row that failed to confirm it since.
  std::int64_t misses{0};
  /// @brief The ids of the matched keyframe's group's points.
  std::vector<std::int64_t> points;
  /// @brief For each keypoint of the last keyframe, the id of the group's point it matched, or
  ///        -1.
  std::vector<std::int64_t> matched_points;
};

/// @brief The keyframes a loop being confirmed holds, which culling spares: the matched one and
///        the last, ascending, each once (`Pending.held`).
/// @param pending The loop.
/// @return Their ids.
[[nodiscard]] std::vector<std::int64_t> held(const Pending& pending);

/// @brief A loop closed, ready to correct (`slam_it.edge.loops.Loop`).
struct Loop {
  /// @brief The id of the keyframe it closed on.
  std::int64_t current{0};
  /// @brief The id of the keyframe it matched.
  std::int64_t matched{0};
  /// @brief The current keyframe's `S_cw` as the loop places it.
  Eigen::Matrix4d similarity{Eigen::Matrix4d::Identity()};
  /// @brief The ids of the matched keyframe's group's points.
  std::vector<std::int64_t> points;
  /// @brief For each keypoint of the current keyframe, the id of the group's point it matched,
  ///        or -1.
  std::vector<std::int64_t> matched_points;
};

/// @brief How far one candidate got (`slam_it.edge.loops.CandidateSummary`); each count empty
///        where it stopped first.
struct CandidateSummary {
  /// @brief The candidate's id.
  std::int64_t keyframe{0};
  /// @brief Its group's accumulated score.
  double score{0.0};
  /// @brief Whether its group holds a keyframe linked to the new one, so it was not tried.
  bool near{false};
  /// @brief The new keyframe's points matched by word to the group's.
  std::optional<std::int64_t> matches;
  /// @brief The id of the group's keyframe that matched most.
  std::optional<std::int64_t> best;
  /// @brief The RANSAC's inliers.
  std::optional<std::int64_t> inliers;
  /// @brief The first search's matches.
  std::optional<std::int64_t> projected;
  /// @brief The refinement's inliers.
  std::optional<std::int64_t> refined;
  /// @brief The second search's matches.
  std::optional<std::int64_t> reprojected;
  /// @brief The new keyframe's links that confirmed it.
  std::optional<std::int64_t> confirmations;
};

/// @brief How a loop being confirmed fared on a later keyframe
///        (`slam_it.edge.loops.ConfirmationSummary`).
struct ConfirmationSummary {
  /// @brief The id of the keyframe the loop matched.
  std::int64_t matched{0};
  /// @brief The search's matches through the similarity carried to the keyframe.
  std::int64_t projected{0};
  /// @brief The refinement's inliers.
  std::optional<std::int64_t> refined;
  /// @brief The search's matches through the refined similarity.
  std::optional<std::int64_t> reprojected;
  /// @brief Whether it confirmed the loop.
  bool confirmed{false};
};

/// @brief How detection went on a keyframe (`slam_it.edge.loops.DetectionSummary`).
struct DetectionSummary {
  /// @brief What it made of the keyframe.
  Outcome outcome{Outcome::kNone};
  /// @brief The loop being confirmed, carried to it, if there was one.
  std::optional<ConfirmationSummary> confirmation;
  /// @brief The keyframes outside its links that share a word with it.
  std::int64_t sharing{0};
  /// @brief Those scored.
  std::int64_t scored{0};
  /// @brief The candidates tried, in order.
  std::vector<CandidateSummary> candidates;
  /// @brief The loop's confirmations after this keyframe (0 if none is being confirmed).
  std::int64_t confirmations{0};
};

/// @brief Detection's result (`slam_it.edge.loops.Detection`).
struct Detection {
  /// @brief The loop being confirmed after it, if any.
  std::optional<Pending> pending;
  /// @brief The loop closed on this keyframe, if any.
  std::optional<Loop> loop;
  /// @brief How it went.
  DetectionSummary summary;
};

/// @brief Look for a loop at a newly mapped keyframe (`slam_it.edge.loops.detect`,
///        `NewDetectCommonRegions`).
/// @param m The map, holding the keyframe.
/// @param db Its keyframe database, holding the keyframe.
/// @param camera The camera.
/// @param keyframe The new keyframe's id.
/// @param pending The loop being confirmed, if any.
/// @param mode The pipeline's mode: the similarity is rigid but in monocular.
/// @param options The settings.
/// @return The loop being confirmed after the keyframe, the loop closed on it, how it went;
///         `InvalidInput` for a keyframe the map or the database does not hold, an unknown mode
///         or invalid settings; `NotConverged` for GTSAM's failure.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the stage's state, as Python's
[[nodiscard]] foundations::Result<Detection> detect(const Map& m, const Database& db,
                                                    const Camera& camera, std::int64_t keyframe,
                                                    const std::optional<Pending>& pending,
                                                    const std::string& mode,
                                                    const Options& options);

/// @brief Fuse points into a keyframe through a similarity (`slam_it.edge.loops
///        .fuse_by_similarity`, `Fuse` with `Scw`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param similarity Its `S_cw`.
/// @param point_ids The points' ids, in the order fused; one no longer in the map is passed
///        over.
/// @param options `loops.fuse_window`, `matching.th_low` and `tracking.view_cos_limit`.
/// @return The points fused; `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<std::int64_t> fuse_by_similarity(
    Map& m, const Camera& camera, std::int64_t k, const Eigen::Matrix4d& similarity,
    std::span<const std::int64_t> point_ids, const Options& options);

/// @brief Bundle-adjust every keyframe and point (`slam_it.edge.loops.global_bundle_adjust`,
///        `GlobalBundleAdjustemnt` as loop closing runs it): no robust loss, the map's first
///        keyframe held fixed; every point refreshed after.
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param options `loops.global_iterations`.
/// @param stop A flag that stops it before its next iteration (a global BA in a thread of its
///        own, superseded: ORB-SLAM's `mbStopGBA`); not owned, null for never.
/// @return The solve's summary; `InvalidInput` for a map without its first keyframe;
///         `NotConverged` for GTSAM's failure.
[[nodiscard]] foundations::Result<foundations::graphs::Summary> global_bundle_adjust(
    Map& m, const Camera& camera, const Options& options, const std::atomic<bool>* stop = nullptr);

/// @brief How a global BA's result was carried onto the map as it has become
///        (`slam_it.edge.loops.Propagated`, the map aside: it changes in place).
struct Propagated {
  /// @brief Its keyframes the global BA solved, set to their result.
  std::int64_t adjusted{0};
  /// @brief Its keyframes made since, moved with their parents.
  std::int64_t propagated{0};
  /// @brief Its points the global BA solved, set to their result.
  std::int64_t points{0};
  /// @brief Its points made since, moved with their reference keyframes.
  std::int64_t carried{0};
};

/// @brief Carry a global BA's result, solved on an earlier copy of the map, onto the map as it
///        is now (`slam_it.edge.loops.propagate_global_ba`, ORB-SLAM's
///        `RunGlobalBundleAdjustment` after its solve): walking the spanning tree from its
///        roots, each keyframe the BA solved takes its solved pose, and each made since moves
///        with its parent (`T_child' = T_child T_parent^-1 T_parent'`); each point the BA solved
///        takes its solved position, and each made since moves with its reference keyframe
///        (`X' = T_ref'^-1 T_ref X`). A keyframe the walk does not reach, or a point whose
///        reference keyframe is gone, stays where it is. Every point is refreshed after.
/// @param now The map as it is now, changed in place.
/// @param solved The copy the global BA solved, moved to its result.
/// @return How many keyframes and points went each way; a refresh's error.
[[nodiscard]] foundations::Result<Propagated> propagate_global_ba(Map& now, const Map& solved);

/// @brief How a global BA in a thread of its own went (`slam_it.edge.loops.GlobalSummary`), as
///        a run records it on the frame it ended on.
struct GlobalSummary {
  /// @brief Its generation: the loops corrected before it in the run.
  std::int64_t generation{0};
  /// @brief Whether its result was carried onto the map; not if a newer loop superseded it, or
  ///        the map was lost, first.
  bool applied{false};
  /// @brief Its solve.
  foundations::graphs::Summary solve;
  /// @brief How its result was carried, when it was.
  std::optional<Propagated> propagation;
};

/// @brief How a loop's correction went (`slam_it.edge.loops.CorrectionSummary`).
struct CorrectionSummary {
  /// @brief The id of the keyframe the loop closed on.
  std::int64_t current{0};
  /// @brief The id of the keyframe it matched.
  std::int64_t matched{0};
  /// @brief The current keyframe's `S_cw` as the loop placed it.
  Eigen::Matrix4d similarity{Eigen::Matrix4d::Identity()};
  /// @brief The keyframes moved by the loop's similarity.
  std::int64_t corrected{0};
  /// @brief The points moved with them.
  std::int64_t moved{0};
  /// @brief The current keyframe's points the loop's matched points replaced, or joined.
  std::int64_t replaced{0};
  /// @brief The loop's points fused into the moved keyframes.
  std::int64_t fused{0};
  /// @brief The links new across the loop.
  std::int64_t links{0};
  /// @brief The essential graph's edges.
  std::int64_t edges{0};
  /// @brief Its solve.
  foundations::graphs::Summary essential;
  /// @brief Global BA's solve, if it ran.
  std::optional<foundations::graphs::Summary> global_ba;
};

/// @brief Correct a loop closed at a keyframe (`slam_it.edge.loops.correct`, `CorrectLoop`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param loop The loop.
/// @param mode The pipeline's mode: the essential graph is Sim(3) but in stereo and RGB-D.
/// @param options The settings.
/// @return How it went; `InvalidInput` for a keyframe the map does not hold, matched points not
///         one per keypoint of the current keyframe, an unknown mode or invalid settings;
///         `NotConverged` for GTSAM's failure.
[[nodiscard]] foundations::Result<CorrectionSummary> correct(Map& m, const Camera& camera,
                                                             const Loop& loop,
                                                             const std::string& mode,
                                                             const Options& options);

/// @brief A merge being confirmed (`slam_it.edge.loops.Merging`).
struct Merging {
  /// @brief The number of the map it matched.
  std::int64_t map{0};
  /// @brief The region as a loop being confirmed holds it: its matched keyframe that map's, its
  ///        last the active map's.
  Pending pending;
};

/// @brief A merge found, ready to make (`slam_it.edge.loops.Merge`).
struct Merge {
  /// @brief The number of the map it matched.
  std::int64_t map{0};
  /// @brief The common region as a closed loop holds it: its similarity the current keyframe's
  ///        `S_cw` in the matched map's world.
  Loop loop;
};

/// @brief Merge detection's result (`slam_it.edge.loops.MergeDetection`).
struct MergeDetection {
  /// @brief The merge being confirmed after it, if any.
  std::optional<Merging> pending;
  /// @brief The merge found on this keyframe, if any.
  std::optional<Merge> merge;
  /// @brief How it went, as loop detection's says; each candidate's keyframe an id in its map.
  DetectionSummary summary;
  /// @brief Each candidate's map number, in the summary's order.
  std::vector<std::int64_t> maps;
};

/// @brief Look for another map at a newly mapped keyframe (`slam_it.edge.loops.detect_merge`,
///        `NewDetectCommonRegions`' merge branch): none below `loops.min_keyframes`; the merge
///        being confirmed carried through its matched map; else the other maps' candidates, each
///        tried against its own map. A confirmation resets the misses, as a loop's does.
/// @param m The active map, holding the keyframe.
/// @param db Its keyframe database, holding the keyframe.
/// @param camera The camera.
/// @param keyframe The new keyframe's id.
/// @param others The other maps, each with its database and number.
/// @param pending The merge being confirmed, if any.
/// @param mode The pipeline's mode: the similarity is rigid but in monocular.
/// @param options The settings.
/// @return The merge being confirmed after the keyframe, the merge found on it, how it went;
///         `InvalidInput` for a keyframe the map or a database does not hold, two maps with one
///         number, a merge being confirmed in a map not given, an unknown mode or invalid
///         settings; GTSAM's failure.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the stage's state, as Python's
[[nodiscard]] foundations::Result<MergeDetection> detect_merge(
    const Map& m, const Database& db, const Camera& camera, std::int64_t keyframe,
    std::span<const Indexed> others, const std::optional<Merging>& pending, const std::string& mode,
    const Options& options);

/// @brief Give a merge found on a keyframe priority over a loop closed on it
///        (`slam_it.edge.loops.merge_first`, ORB-SLAM3's `Run`): the loop forgotten, its outcome
///        `kMerged`; a loop still being confirmed kept.
/// @param loop Loop detection's result on the keyframe.
/// @param merging Merge detection's on the same keyframe.
/// @return The loop's detection, its loop and state dropped if a merge was found too.
[[nodiscard]] Detection merge_first(Detection loop, const MergeDetection& merging);

/// @brief How a merge's welding BA went (`slam_it.edge.loops.WeldSummary`).
struct WeldSummary {
  /// @brief The keyframes optimised.
  std::int64_t keyframes{0};
  /// @brief The keyframes held fixed.
  std::int64_t held{0};
  /// @brief The points optimised.
  std::int64_t points{0};
  /// @brief The observations weighed.
  std::int64_t observations{0};
  /// @brief Those failing the chi-square test, or behind their camera, after the first solve.
  std::int64_t outliers{0};
  /// @brief Those failing it after the second, forgotten.
  std::int64_t forgotten{0};
  /// @brief The first solve, with a Huber loss.
  foundations::graphs::Summary first;
  /// @brief The second, with none, if any observation was left for it.
  std::optional<foundations::graphs::Summary> second;
};

/// @brief How a merge went (`slam_it.edge.loops.MergeSummary`).
struct MergeSummary {
  /// @brief The number of the map the active one was merged into.
  std::int64_t map{0};
  /// @brief The id, in the merged map, of the keyframe the merge was found on.
  std::int64_t current{0};
  /// @brief The id of the keyframe it matched.
  std::int64_t matched{0};
  /// @brief The current keyframe's `S_cw` in the matched map's world, as the merge placed it.
  Eigen::Matrix4d similarity{Eigen::Matrix4d::Identity()};
  /// @brief The active side's welding window's keyframes.
  std::int64_t window{0};
  /// @brief The matched side's.
  std::int64_t matched_window{0};
  /// @brief The matched window's points fused into the active window's keyframes.
  std::int64_t fused{0};
  /// @brief The welding BA.
  WeldSummary weld;
  /// @brief The essential graph's edges.
  std::int64_t edges{0};
  /// @brief Its solve.
  foundations::graphs::Summary essential;
  /// @brief Global BA's solve, if it ran.
  std::optional<foundations::graphs::Summary> global_ba;
};

/// @brief Two maps merged (`slam_it.edge.loops.Merged`).
struct Merged {
  /// @brief The merged map: the matched map's, then the active map's, each id of the active
  ///        map's moved on by an offset.
  Map map;
  /// @brief What each of the active map's keyframe ids, live or archived, became: the id plus
  ///        this.
  std::int64_t keyframe_offset{0};
  /// @brief The same for its point ids.
  std::int64_t point_offset{0};
  /// @brief How it went.
  MergeSummary summary;
};

/// @brief Merge the active map into the one it matched (`slam_it.edge.loops.merge`,
///        `MergeLocal` as ORB-SLAM3's paper gives it; the Python module's docstring states the
///        steps).
/// @param active The active map, holding the merge's current keyframe.
/// @param matched The map it matched, holding the merge's matched keyframe.
/// @param camera The camera.
/// @param found The merge.
/// @param mode The pipeline's mode: the similarity's scale is used but in stereo and RGB-D.
/// @param options The settings.
/// @return The merged map, the offsets that carried the active map's ids, and how it went;
///         `InvalidInput` for a keyframe its map does not hold, a similarity that is not one,
///         maps of different pyramids, an unknown mode or invalid settings; GTSAM's failure.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (active, matched), as Python's
[[nodiscard]] foundations::Result<Merged> merge(const Map& active, const Map& matched,
                                                const Camera& camera, const Merge& found,
                                                const std::string& mode, const Options& options);

}  // namespace slam_it::edge::loops
