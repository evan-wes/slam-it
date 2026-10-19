/// @file
/// @brief Bundle adjustment over the map's keyframes: the C++ core's mirror of
///        `slam_it.edge.adjust`, whose docstring states the problem.
///
/// The free keyframes, the points they observe and every keyframe observing those points go
/// into one `foundations::ba::Problem`, observations weighted by their levels' information and
/// given an inverse-depth prior where they have depth. Two solves bracket an outlier pass (a
/// point left with fewer than two inlier views restarts the second from where it began); the
/// observations still failing the chi-square test are forgotten, and the surviving points
/// refreshed. The result records how each solve went, which a run records per keyframe.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/nls.hpp"

namespace slam_it::edge {

/// @brief How one least-squares solve of a bundle adjustment went
///        (`slam_it.edge.adjust.Solve`).
struct Solve {
  /// @brief The robust cost at the start.
  double initial_cost{0.0};
  /// @brief The robust cost at the returned state.
  double final_cost{0.0};
  /// @brief Steps tried, accepted or not.
  std::int64_t iterations{0};
  /// @brief Why it stopped.
  foundations::nls::Termination termination{foundations::nls::Termination::kMaxIterations};
};

/// @brief How a bundle adjustment went (`slam_it.edge.adjust.Adjustment`, whose map is
///        changed in place here).
struct Adjustment {
  /// @brief The first solve and the refining one; none when nothing was solved: no point was
  ///        observed, or no keyframe was held fixed.
  std::vector<Solve> solves;
  /// @brief The observations forgotten as outliers after the refining solve.
  std::int64_t outliers{0};
  /// @brief The keyframes adjusted: those given, but the ones held fixed among them.
  std::int64_t free{0};
  /// @brief The keyframes held fixed: those given that were, and every other observing their
  ///        points.
  std::int64_t fixed{0};
  /// @brief Whether an abort stopped it early: before its first solve, between iterations, or
  ///        before the refining solve (`slam_it.edge.adjust`'s "Stopped early").
  bool aborted{false};
};

/// @brief A solve's record, from its summary (`slam_it.edge.adjust.Solve.of`).
/// @param summary The solve's summary.
/// @return Its costs, iterations and termination.
[[nodiscard]] Solve solve_of(const foundations::nls::Summary& summary);

/// @brief What a local BA does around its solves: ORB-SLAM's locking, C++'s baseline (the
///        online-SLAM design's section 12, item 7), where `LocalBundleAdjustment` holds the
///        map's update lock only to write its result. `leave` runs once the problem is built,
///        before the first solve; `enter` before the result is written back, and before any
///        error is returned after leaving.
struct Around {
  /// @brief Let the map's lock go.
  std::function<void()> leave;
  /// @brief Take it back.
  std::function<void()> enter;
};

/// @brief Bundle-adjust keyframes and the points they observe
///        (`slam_it.edge.adjust.bundle_adjust`, ORB-SLAM's `LocalBundleAdjustment`).
/// @param m The map, changed in place: the free keyframes' poses and the points adjusted, the
///        outlier observations forgotten.
/// @param camera The camera.
/// @param keyframes The rows of the keyframes to adjust; every other keyframe observing their
///        points is held fixed.
/// @param fixed One flag per keyframe: nonzero to hold it fixed even among `keyframes`.
/// @param options The iterations and thresholds.
/// @param abort A flag local mapping's thread sets to stop it early: checked before its first
///        solve, between its iterations and before its refining solve (ORB-SLAM's
///        `pbStopFlag`); not owned, null for never.
/// @param around What it does around its solves (the baseline's lock); not owned, null for
///        nothing.
/// @return The two solves (none, the map as it was, if no point was observed or no keyframe
///         was held fixed, as ORB-SLAM3 skips it, or if aborted before the first; one if
///         aborted before the second) and the outliers dropped; `InvalidInput` for a keyframe
///         row out of range, flags that are not one per keyframe, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<Adjustment> bundle_adjust(
    Map& m, const Camera& camera, std::span<const std::int64_t> keyframes,
    std::span<const std::uint8_t> fixed, const Options& options,
    const std::atomic<bool>* abort = nullptr, const Around* around = nullptr);

}  // namespace slam_it::edge
