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

#include <cstdint>
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
  /// @brief The first solve and the refining one; none when no point was observed.
  std::vector<Solve> solves;
  /// @brief The observations forgotten as outliers after the refining solve.
  std::int64_t outliers{0};
};

/// @brief A solve's record, from its summary (`slam_it.edge.adjust.Solve.of`).
/// @param summary The solve's summary.
/// @return Its costs, iterations and termination.
[[nodiscard]] Solve solve_of(const foundations::nls::Summary& summary);

/// @brief Bundle-adjust keyframes and the points they observe
///        (`slam_it.edge.adjust.bundle_adjust`, ORB-SLAM's `LocalBundleAdjustment`).
/// @param m The map, changed in place: the free keyframes' poses and the points adjusted, the
///        outlier observations forgotten.
/// @param camera The camera.
/// @param keyframes The rows of the keyframes to adjust; every other keyframe observing their
///        points is held fixed.
/// @param fixed One flag per keyframe: nonzero to hold it fixed even among `keyframes`.
/// @param options The iterations and thresholds.
/// @return The two solves (none if no point was observed) and the outliers dropped;
///         `InvalidInput` for a keyframe row out of range, flags that are not one per
///         keyframe, or an invalid camera or settings.
[[nodiscard]] foundations::Result<Adjustment> bundle_adjust(Map& m, const Camera& camera,
                                                            std::span<const std::int64_t> keyframes,
                                                            std::span<const std::uint8_t> fixed,
                                                            const Options& options);

}  // namespace slam_it::edge
