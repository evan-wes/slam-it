/// @file
/// @brief Matching binary descriptors: the C++ core's mirror of `slam_it_all.foundations.matching`.
///
/// Hamming distances, and ORB-SLAM's filtered nearest neighbours: a distance cap, Lowe's ratio
/// test, one match per second-set descriptor, and the rotation-consistency histogram; and
/// guided matching, a search by projection over a feature grid (the Python module's docstring
/// defines each filter).
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

#include "slam_it_all/foundations/error.hpp"
#include "slam_it_all/foundations/orb.hpp"

/// @brief Descriptor matching.
namespace slam_it_all::foundations::matching {

/// @brief The most descriptors a set may hold.
inline constexpr std::size_t kMaxDescriptors = std::size_t{1} << 20U;
/// @brief The largest Hamming distance: every bit differs.
inline constexpr std::int64_t kMaxDistance = 8 * static_cast<std::int64_t>(orb::kDescriptorBytes);
/// @brief The rotation histogram's second and third bins are kept only if they hold at least
///        this fraction of the highest (ORB-SLAM's `ComputeThreeMaxima`).
inline constexpr double kSecondaryFraction = 0.1;
/// @brief The feature grid's cells across the keypoints' bounding box (ORB-SLAM's
///        `FRAME_GRID_COLS`).
inline constexpr std::int64_t kGridCols = 64;
/// @brief Its cells down the box (ORB-SLAM's `FRAME_GRID_ROWS`).
inline constexpr std::int64_t kGridRows = 48;

/// @brief Matching filters (`slam_it_all.foundations.matching.Options`).
struct Options {
  /// @brief The largest Hamming distance a match may have, from 0 to `kMaxDistance`.
  std::int64_t max_distance{50};
  /// @brief The ratio test's bound, in (0, 1].
  double ratio{0.8};
  /// @brief Keep only matches in the rotation histogram's three highest bins.
  bool check_orientation{true};
};

/// @brief One match.
struct Match {
  /// @brief The first-set index.
  std::int64_t first{0};
  /// @brief The second-set index.
  std::int64_t second{0};
  /// @brief The Hamming distance.
  std::int64_t distance{0};
};

/// @brief The filters of a search by projection (`slam_it_all.foundations.matching.SearchOptions`;
///        defaults: ORB-SLAM's search from the last frame).
struct SearchOptions {
  /// @brief The largest Hamming distance a match may have, from 0 to `kMaxDistance`.
  std::int64_t max_distance{100};
  /// @brief The one-level ratio test's bound, in (0, 1]; 1 disables it.
  double ratio{1.0};
  /// @brief Keep only matches in the rotation histogram's three highest bins.
  bool check_orientation{true};
};

/// @brief One map point projected into a frame (a row of
///        `slam_it_all.foundations.matching.Projections`).
struct Projection {
  /// @brief The projection, in the full image's pixels.
  std::array<double, 2> xy{};
  /// @brief The square search window's half-width, in pixels, positive.
  double radius{1.0};
  /// @brief The lowest pyramid level a match may lie on.
  std::int64_t min_level{0};
  /// @brief The highest (an empty range finds nothing).
  std::int64_t max_level{0};
  /// @brief The map point's descriptor.
  orb::Descriptor descriptor{};
  /// @brief The orientation bin of the observation the descriptor came from.
  std::int64_t angle{0};
  /// @brief The predicted right-image column (stereo, RGB-D); NaN for none.
  double right{std::numeric_limits<double>::quiet_NaN()};
};

/// @brief A frame's keypoints bucketed into `kGridCols` x `kGridRows` cells over their bounding
///        box (ORB-SLAM's grid), so that a window reads only the cells it covers.
///
/// A position's cell is `floor((x - x_min) * scale)` along each axis, clamped to the grid,
/// with `scale` the cells over the box's width (0 for a box of no width, or one too narrow
/// for a finite scale). The map is monotone in `x`, and IEEE rounding is monotone too, so the
/// cells of a window's two floating-point ends `u - r` and `u + r` bound the cells of every
/// keypoint that passes the window's test `|x - u| < r`: the grid finds exactly the
/// keypoints a brute force finds.
class FeatureGrid {
 public:
  /// @brief Bucket keypoints.
  /// @param keypoints The keypoints (positions and levels are read).
  /// @return The grid; `InvalidInput` for a position that is not finite, or more than
  ///         `kMaxDescriptors` keypoints.
  [[nodiscard]] static Result<FeatureGrid> build(std::span<const orb::Keypoint> keypoints);

  /// @brief The number of keypoints.
  /// @return It.
  [[nodiscard]] std::size_t size() const { return x_.size(); }

  /// @brief The keypoints in a window, checked.
  /// @param center The window's centre `(u, v)`.
  /// @param radius Its half-width `r`: a keypoint is in it when `|x - u| < r` and
  ///        `|y - v| < r`.
  /// @param min_level The lowest level it covers.
  /// @param max_level The highest.
  /// @return The keypoints' indices, ascending; `InvalidInput` for a centre or radius that is
  ///         not finite, or a radius that is not positive.
  [[nodiscard]] Result<std::vector<std::int64_t>> in_area(std::array<double, 2> center,
                                                          double radius, std::int64_t min_level,
                                                          std::int64_t max_level) const;

  /// @brief Append the keypoints in a window, unchecked and in cell order (not ascending).
  /// @param center The window's centre, finite.
  /// @param radius Its half-width, finite and positive.
  /// @param min_level The lowest level it covers.
  /// @param max_level The highest.
  /// @param out Where the indices go.
  void collect(std::array<double, 2> center, double radius, std::int64_t min_level,
               std::int64_t max_level, std::vector<std::uint32_t>& out) const;

 private:
  /// @brief One axis of the grid: a position `t` is in cell `floor((t - origin) * scale)`,
  ///        clamped to `[0, cells)`.
  struct Axis {
    /// @brief The box's lower edge.
    double origin{0.0};
    /// @brief Cells per pixel (0: every position in the first cell).
    double scale{0.0};
    /// @brief The cells along the axis.
    std::int64_t cells{1};
  };

  /// @brief One axis of the box around some positions.
  /// @param values The keypoints' coordinates along the axis.
  /// @param cells The cells along it.
  /// @return The axis: its origin the values' minimum, its scale the cells over their range (0
  ///         for no range, or one too narrow or too wide for a finite scale).
  static Axis axis(std::span<const double> values, std::int64_t cells);

  /// @brief A position's cell along an axis.
  /// @param t The position.
  /// @param axis The axis.
  /// @return The cell, in `[0, axis.cells)`.
  static std::int64_t cell(double t, const Axis& axis);

  /// @brief The keypoints' columns.
  std::vector<double> x_;
  /// @brief Their rows.
  std::vector<double> y_;
  /// @brief Their levels.
  std::vector<std::int64_t> level_;
  /// @brief The grid across.
  Axis x_axis_;
  /// @brief The grid down.
  Axis y_axis_;
  /// @brief Where each cell's keypoints start in `items_`, cell by cell, row-major, with one
  ///        past the end last.
  std::vector<std::uint32_t> start_;
  /// @brief The keypoints, cell by cell, ascending within each.
  std::vector<std::uint32_t> items_;
};

/// @brief Check matching filters.
/// @param options The filters.
/// @return Nothing, or `InvalidInput` naming the first one out of range.
[[nodiscard]] Result<void> check_options(const Options& options);

/// @brief The Hamming distance between two descriptors: the bits in which they differ.
/// @param a A descriptor.
/// @param b Another.
/// @return From 0 to 256.
[[nodiscard]] std::int64_t distance(const orb::Descriptor& a, const orb::Descriptor& b);

/// @brief Every pair's Hamming distance.
/// @param descriptors1 The first set.
/// @param descriptors2 The second set.
/// @return The distances, row-major (first set by second set); `InvalidInput` for a set of
///         more than `kMaxDescriptors`.
[[nodiscard]] Result<std::vector<std::int64_t>> distances(
    std::span<const orb::Descriptor> descriptors1, std::span<const orb::Descriptor> descriptors2);

/// @brief Match each descriptor of the first set to its nearest in the second, filtered.
/// @param descriptors1 The first set.
/// @param descriptors2 The second set.
/// @param angles1 The first set's orientation bins.
/// @param angles2 The second set's.
/// @param options The filters.
/// @return The kept matches, by first-set index; `InvalidInput` for a bad filter, a set too
///         large, or angles that are not one bin in `[0, orb::kAngleBins)` per descriptor.
[[nodiscard]] Result<std::vector<Match>> match(std::span<const orb::Descriptor> descriptors1,
                                               std::span<const orb::Descriptor> descriptors2,
                                               std::span<const std::int64_t> angles1,
                                               std::span<const std::int64_t> angles2,
                                               const Options& options);

/// @brief Check a search's filters.
/// @param options The filters.
/// @return Nothing, or `InvalidInput` naming the first one out of range.
[[nodiscard]] Result<void> check_search_options(const SearchOptions& options);

/// @brief The keypoints in a square window, on a range of levels
///        (`slam_it_all.foundations.matching.features_in_area`, ORB-SLAM's `GetFeaturesInArea`).
/// @param keypoints The keypoints (positions and levels are read).
/// @param center The window's centre `(u, v)`.
/// @param radius Its half-width `r`, positive.
/// @param min_level The lowest level it covers.
/// @param max_level The highest.
/// @return The keypoints' indices, ascending; `InvalidInput` for a position that is not finite,
///         a negative level, too many keypoints, or a bad window.
[[nodiscard]] Result<std::vector<std::int64_t>> features_in_area(
    std::span<const orb::Keypoint> keypoints, std::array<double, 2> center, double radius,
    std::int64_t min_level, std::int64_t max_level);

/// @brief Match map points projected into a frame to its keypoints
///        (`slam_it_all.foundations.matching.search_by_projection`, ORB-SLAM's
///        `SearchByProjection`).
/// @param projections The projections.
/// @param features The frame's features (positions, levels, angles and descriptors are read).
/// @param grid The frame's grid, built from `features.keypoints`.
/// @param right Each keypoint's right-image column; NaN for none.
/// @param taken Each keypoint's flag: nonzero if already matched, never a candidate.
/// @param options The filters.
/// @return `(projection, keypoint)` matches, by projection; `InvalidInput` for a bad filter, a
///         malformed projection, features whose arrays, grid, `right` or `taken` disagree in
///         size, a level outside the pyramid, an angle outside `[0, orb::kAngleBins)`, or an
///         infinite right column.
[[nodiscard]] Result<std::vector<Match>> search_by_projection(
    std::span<const Projection> projections, const orb::Features& features, const FeatureGrid& grid,
    std::span<const double> right, std::span<const std::uint8_t> taken,
    const SearchOptions& options);

}  // namespace slam_it_all::foundations::matching
