/// @file
/// @brief ORB keypoints as ORB-SLAM detects them: the C++ core's mirror of
///        `slamlab.foundations.orb`.
///
/// FAST-9 corners in 30-pixel cells, spread by a quadtree and oriented by their intensity
/// centroid, on every level of a pyramid; all in integer arithmetic, so the three languages
/// return identical keypoints (the Python module's docstring defines each step).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/image.hpp"

/// @brief ORB features.
namespace slamlab::foundations::orb {

/// @brief Keypoints lie at least this many pixels inside their level (ORB-SLAM's).
inline constexpr std::int64_t kEdgeThreshold = 19;
/// @brief The target side of a detection cell, in pixels (ORB-SLAM's).
inline constexpr std::int64_t kCellSize = 30;
/// @brief The 16 `(dx, dy)` offsets of FAST's radius-3 circle, in order around it.
inline constexpr std::array<std::array<std::int64_t, 2>, 16> kFastCircle{{{0, 3},
                                                                          {1, 3},
                                                                          {2, 2},
                                                                          {3, 1},
                                                                          {3, 0},
                                                                          {3, -1},
                                                                          {2, -2},
                                                                          {1, -3},
                                                                          {0, -3},
                                                                          {-1, -3},
                                                                          {-2, -2},
                                                                          {-3, -1},
                                                                          {-3, 0},
                                                                          {-3, 1},
                                                                          {-2, 2},
                                                                          {-1, 3}}};
/// @brief The contiguous circle pixels a FAST-9 corner needs.
inline constexpr std::size_t kFastArc = 9;
/// @brief The radius of the orientation patch.
inline constexpr std::int64_t kHalfPatchSize = 15;
/// @brief The orientation disc's half-width at each row offset `|v|` (OpenCV's `umax`).
inline constexpr std::array<std::int64_t, 16> kUmax{15, 15, 15, 15, 14, 14, 14, 13,
                                                    13, 12, 11, 10, 9,  8,  6,  3};
/// @brief Orientation bins, 12 degrees each.
inline constexpr std::int64_t kAngleBins = 30;
/// @brief `round(2^14 (cos, sin))` of `12 k` degrees for each bin `k`.
inline constexpr std::array<std::array<std::int64_t, 2>, 30> kRotations{
    {{16384, 0},      {16026, 3406},   {14968, 6664},   {13255, 9630},   {10963, 12176},
     {8192, 14189},   {5063, 15582},   {1713, 16294},   {-1713, 16294},  {-5063, 15582},
     {-8192, 14189},  {-10963, 12176}, {-13255, 9630},  {-14968, 6664},  {-16026, 3406},
     {-16384, 0},     {-16026, -3406}, {-14968, -6664}, {-13255, -9630}, {-10963, -12176},
     {-8192, -14189}, {-5063, -15582}, {-1713, -16294}, {1713, -16294},  {5063, -15582},
     {8192, -14189},  {10963, -12176}, {13255, -9630},  {14968, -6664},  {16026, -3406}}};
/// @brief The most pyramid levels `Options` accepts.
inline constexpr std::int64_t kMaxLevels = 32;

/// @brief ORB extraction settings (`slamlab.foundations.orb.Options`).
struct Options {
  /// @brief The feature budget over all levels, from 1 to `image::kMaxSide`.
  std::int64_t n_features{1000};
  /// @brief Pyramid levels, from 1 to `kMaxLevels`.
  std::int64_t levels{8};
  /// @brief Each level's downscale from the previous one, above 1.
  double scale_factor{1.2};
  /// @brief The FAST threshold a cell tries first, from 0 to 254.
  std::int64_t fast_threshold{20};
  /// @brief The threshold for cells with no corner at `fast_threshold`.
  std::int64_t min_fast_threshold{7};
};

/// @brief One oriented keypoint.
struct Keypoint {
  /// @brief Its pixel in the full image: the level's pixel times `scale_factor^level`.
  std::array<double, 2> xy{};
  /// @brief The pyramid level it was detected on.
  std::int64_t level{0};
  /// @brief Its integer pixel in that level.
  std::array<std::int64_t, 2> level_xy{};
  /// @brief Its FAST score.
  std::int64_t response{0};
  /// @brief Its orientation bin, in `[0, kAngleBins)`: `12 * angle` degrees.
  std::int64_t angle{0};
};

/// @brief A half-open box `[x_min, x_max) x [y_min, y_max)`, in pixels.
struct Bounds {
  /// @brief The first column inside.
  std::int64_t x_min{0};
  /// @brief One past the last column inside.
  std::int64_t x_max{0};
  /// @brief The first row inside.
  std::int64_t y_min{0};
  /// @brief One past the last row inside.
  std::int64_t y_max{0};
};

/// @brief Check extraction settings.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the first setting out of range.
[[nodiscard]] Result<void> check_options(const Options& options);

/// @brief Split the feature budget over the levels, falling geometrically (ORB-SLAM's split).
/// @param n_features The total budget.
/// @param levels The number of levels.
/// @param scale_factor The pyramid's scale factor, above 1.
/// @return One budget per level, summing to `n_features`.
[[nodiscard]] std::vector<std::int64_t> level_budgets(std::int64_t n_features, std::int64_t levels,
                                                      double scale_factor);

/// @brief Every pixel's FAST-9 score: the largest threshold at which it is a corner, or -1.
/// @param image The image.
/// @return The scores, row-major, or `InvalidInput` for a malformed image.
[[nodiscard]] Result<std::vector<std::int64_t>> fast_scores(const image::Image& image);

/// @brief Spread corners with ORB-SLAM's quadtree.
/// @param xy Integer pixel coordinates, each inside `bounds`.
/// @param response Each corner's strength.
/// @param bounds The box the corners lie in, non-empty.
/// @param target How many corners to keep.
/// @return The kept corners' indices, ascending; `InvalidInput` for mismatched sizes, an
///         empty box, a corner outside it, or a target out of range.
[[nodiscard]] Result<std::vector<std::int64_t>> distribute(
    std::span<const std::array<std::int64_t, 2>> xy, std::span<const std::int64_t> response,
    const Bounds& bounds, std::int64_t target);

/// @brief Detect oriented keypoints on every pyramid level.
/// @param image The image.
/// @param options The settings.
/// @return The keypoints by level, then row, then column; `InvalidInput` for a malformed
///         image or setting, or a pyramid level that would be empty.
[[nodiscard]] Result<std::vector<Keypoint>> detect(const image::Image& image,
                                                   const Options& options);

}  // namespace slamlab::foundations::orb
