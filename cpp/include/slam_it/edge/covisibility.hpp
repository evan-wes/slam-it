/// @file
/// @brief The covisibility graph: the C++ core's mirror of `slam_it.edge.covisibility`.
///
/// Two keyframes' weight is the number of points both observe, counted from the map's
/// observations each time it is asked for. A keyframe is linked to those it shares at least
/// `covisibility.min_points` with, or, if it shares that many with none, to the one it shares
/// most with; its links run most shared first, ties broken by the lower id. Only the spanning
/// tree is stored (`Map::kf_parent`). The Python module's docstring gives the rules and their
/// ORB-SLAM sources.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"

namespace slam_it::edge {

/// @brief A link: another keyframe's row, and the points the two share.
struct Link {
  /// @brief The other keyframe's row.
  std::int64_t row{0};
  /// @brief The points both observe.
  std::int64_t weight{0};
};

/// @brief Tracking's local map (`slam_it.edge.covisibility.LocalMap`).
struct LocalMap {
  /// @brief Its keyframes' rows: K1, the most shared first, then K2 in the order added.
  std::vector<std::int64_t> keyframes;
  /// @brief The reference keyframe's row (K1's first), or -1.
  std::int64_t reference{-1};
};

/// @brief The points a keyframe shares with each keyframe.
/// @param m The map.
/// @param k The keyframe's row, in range.
/// @return One count per keyframe row, 0 for the keyframe itself.
[[nodiscard]] std::vector<std::int64_t> shared_points(const Map& m, std::size_t k);

/// @brief A keyframe's links from its shared points: those sharing at least `min_points`, or the
///        one sharing the most.
/// @param m The map.
/// @param shared Its shared points (`shared_points`).
/// @param min_points The links' least weight.
/// @return The links, most shared first, ties by the lower id.
[[nodiscard]] std::vector<Link> links_of(const Map& m, std::span<const std::int64_t> shared,
                                         std::int64_t min_points);

/// @brief A keyframe's links (`slam_it.edge.covisibility.links`).
/// @param m The map.
/// @param k The keyframe's row.
/// @param options The settings: `covisibility.min_points`.
/// @return The links, or `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<std::vector<Link>> links(const Map& m, std::int64_t k,
                                                           const Options& options);

/// @brief Tracking's local map, ORB-SLAM's K1 and K2 (`slam_it.edge.covisibility.local_map`).
/// @param m The map.
/// @param points The point each of the frame's keypoints was matched to, -1 for none.
/// @param options The settings: the `covisibility` group's.
/// @return The local map, or `InvalidInput` for an id below -1.
[[nodiscard]] foundations::Result<LocalMap> local_map(const Map& m,
                                                      std::span<const std::int64_t> points,
                                                      const Options& options);

/// @brief The keyframes tracking's next frame may read, from where the last frame left it: what
///        the view a writer publishes holds (the online-SLAM design's section 12, item 3), so
///        that tracking on it decides as on the whole map. The last frame's local map
///        (`local_map`) gives the points tracking may match, by the motion model or the local
///        fallback; the keyframes observing them (the K1 of any frame matching them), and for
///        each of those, every keyframe sharing a point with it, its children and its parent
///        (what K2 and the keyframe decision read); and the last frame's reference keyframe.
/// @param m The map.
/// @param last The point each of the last frame's keypoints tracked, -1 for none.
/// @param reference Its reference keyframe's id, or -1.
/// @param options The settings: the `covisibility` group's.
/// @return The keyframes' rows, ascending; `InvalidInput` for an id below -1.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> view_keyframes(
    const Map& m, std::span<const std::int64_t> last, std::int64_t reference,
    const Options& options);

/// @brief Give a keyframe its parent, if it has none yet (`slam_it.edge.covisibility.adopt`).
///
/// For the keyframe being inserted, the map's newest, every other already in the tree: given an
/// older keyframe without a parent, it may take a newer one whose own chain leads back to it, a
/// cycle.
/// @param m The map, changed in place.
/// @param k The keyframe's row.
/// @return Nothing, or `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<void> adopt(Map& m, std::int64_t k);

/// @brief Give a keyframe's children new parents before it is culled
///        (`slam_it.edge.covisibility.reparent`).
/// @param m The map, changed in place.
/// @param k The keyframe's row.
/// @param options The settings: `covisibility.min_points`.
/// @return Nothing, or `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<void> reparent(Map& m, std::int64_t k, const Options& options);

/// @brief Cull a keyframe: its children given new parents, then it erased (ORB-SLAM's
///        `SetBadFlag`; `slam_it.edge.covisibility.cull`). The one way a keyframe leaves the
///        graph, as `erase_keyframe` refuses a keyframe that is still a parent. The map's first
///        keyframe (id 0), its tree's root, is refused, as `SetBadFlag` leaves it. A keyframe
///        with a loop edge, or one of `held`, is spared, the map left as it was (`SetNotErase`).
/// @param m The map, changed in place.
/// @param k The keyframe's row.
/// @param options The settings: `covisibility.min_points`.
/// @param held The ids of the keyframes loop detection holds.
/// @return Nothing, or `InvalidInput` for a row out of range or the map's first keyframe.
[[nodiscard]] foundations::Result<void> cull(Map& m, std::int64_t k, const Options& options,
                                             std::span<const std::int64_t> held = {});

/// @brief Every pair of keyframes sharing at least `covisibility.strong_points`
///        (`slam_it.edge.covisibility.strong_links`).
/// @param m The map.
/// @param options The settings.
/// @return Each pair's ids, the lower first, and the points they share, by the first id then the
///         second.
[[nodiscard]] std::vector<std::array<std::int64_t, 3>> strong_links(const Map& m,
                                                                    const Options& options);

}  // namespace slam_it::edge
