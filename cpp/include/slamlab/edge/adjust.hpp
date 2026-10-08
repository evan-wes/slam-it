/// @file
/// @brief Bundle adjustment over the map's keyframes: the C++ core's mirror of
///        `slamlab.edge.adjust`, whose docstring states the problem.
///
/// The free keyframes, the points they observe and every keyframe observing those points go
/// into one `foundations::ba::Problem`, observations weighted by their levels' information and
/// given an inverse-depth prior where they have depth. Two solves bracket an outlier pass (a
/// point left with fewer than two inlier views restarts the second from where it began); the
/// observations still failing the chi-square test are forgotten, and the surviving points
/// refreshed.
#pragma once

#include <cstdint>
#include <span>

#include "slamlab/edge/frames.hpp"
#include "slamlab/edge/map.hpp"
#include "slamlab/edge/options.hpp"
#include "slamlab/foundations/error.hpp"

namespace slamlab::edge {

/// @brief Bundle-adjust keyframes and the points they observe
///        (`slamlab.edge.adjust.bundle_adjust`, ORB-SLAM's `LocalBundleAdjustment`).
/// @param m The map, changed in place: the free keyframes' poses and the points adjusted, the
///        outlier observations forgotten.
/// @param camera The camera.
/// @param keyframes The rows of the keyframes to adjust; every other keyframe observing their
///        points is held fixed.
/// @param fixed One flag per keyframe: nonzero to hold it fixed even among `keyframes`.
/// @param options The iterations and thresholds.
/// @return Nothing; `InvalidInput` for a keyframe row out of range, flags that are not one
///         per keyframe, or an invalid camera or settings.
[[nodiscard]] foundations::Result<void> bundle_adjust(Map& m, const Camera& camera,
                                                      std::span<const std::int64_t> keyframes,
                                                      std::span<const std::uint8_t> fixed,
                                                      const Options& options);

}  // namespace slamlab::edge
