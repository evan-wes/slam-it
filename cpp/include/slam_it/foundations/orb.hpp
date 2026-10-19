/// @file
/// @brief ORB features as ORB-SLAM extracts them: the C++ core's mirror of
///        `slam_it.foundations.orb`.
///
/// FAST-9 corners in 30-pixel cells, spread by a quadtree and oriented by their intensity
/// centroid, on every level of a pyramid, with rBRIEF descriptors from OpenCV's learned
/// pattern; all in integer arithmetic, so the three languages return identical features (the
/// Python module's docstring defines each step).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/image.hpp"

/// @brief ORB features.
namespace slam_it::foundations::orb {

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
/// @brief The fixed-point bits of `kRotations`.
inline constexpr int kRotationBits = 14;
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
/// @brief The largest feature budget `Options` and `distribute` accept: a bound on the work
///        one call may ask for, far above any real budget (ORB-SLAM extracts 1000 to 2000).
inline constexpr std::int64_t kMaxFeatures = image::pow2(20);
/// @brief The largest FAST threshold: a margin of 255, the most an 8-bit difference can be,
///        less one (a corner needs every margin strictly above the threshold).
inline constexpr std::int64_t kMaxFastThreshold = 254;
/// @brief The descriptor's smoothing: a 7-tap Gaussian, sigma 2, summing to `2^12`.
inline constexpr std::array<std::uint32_t, 7> kBlurKernel{287, 537, 781, 886, 781, 537, 287};
/// @brief A descriptor's length: 256 bits.
inline constexpr std::size_t kDescriptorBytes = 32;
/// @brief This descriptor's version (`slam_it.foundations.orb.DESCRIPTOR_VERSION`), raised when
///        what a descriptor holds changes; a vocabulary records the one it was trained on.
inline constexpr std::int64_t kDescriptorVersion = 1;

// rBRIEF's learned test pairs (x1, y1, x2, y2): OpenCV's bit_pattern_31_
// (modules/features2d/src/orb.cpp), learned by Ethan Rublee, Vincent Rabaud and Gary Bradski
// for the ORB paper. Reproduced under OpenCV's licence for that file:
//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2009, Willow Garage, Inc.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of the Willow Garage nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// clang-format off
/// @brief rBRIEF's 256 test pairs `(x1, y1, x2, y2)`, in pixels from the keypoint (OpenCV's).
inline constexpr std::array<std::array<std::int64_t, 4>, 256> kPattern{{
    {8, -3, 9, 5}, {4, 2, 7, -12}, {-11, 9, -8, 2}, {7, -12, 12, -13}, {2, -13, 2, 12},
    {1, -7, 1, 6}, {-2, -10, -2, -4}, {-13, -13, -11, -8}, {-13, -3, -12, -9}, {10, 4, 11, 9},
    {-13, -8, -8, -9}, {-11, 7, -9, 12}, {7, 7, 12, 6}, {-4, -5, -3, 0}, {-13, 2, -12, -3},
    {-9, 0, -7, 5}, {12, -6, 12, -1}, {-3, 6, -2, 12}, {-6, -13, -4, -8}, {11, -13, 12, -8},
    {4, 7, 5, 1}, {5, -3, 10, -3}, {3, -7, 6, 12}, {-8, -7, -6, -2}, {-2, 11, -1, -10},
    {-13, 12, -8, 10}, {-7, 3, -5, -3}, {-4, 2, -3, 7}, {-10, -12, -6, 11}, {5, -12, 6, -7},
    {5, -6, 7, -1}, {1, 0, 4, -5}, {9, 11, 11, -13}, {4, 7, 4, 12}, {2, -1, 4, 4},
    {-4, -12, -2, 7}, {-8, -5, -7, -10}, {4, 11, 9, 12}, {0, -8, 1, -13}, {-13, -2, -8, 2},
    {-3, -2, -2, 3}, {-6, 9, -4, -9}, {8, 12, 10, 7}, {0, 9, 1, 3}, {7, -5, 11, -10},
    {-13, -6, -11, 0}, {10, 7, 12, 1}, {-6, -3, -6, 12}, {10, -9, 12, -4}, {-13, 8, -8, -12},
    {-13, 0, -8, -4}, {3, 3, 7, 8}, {5, 7, 10, -7}, {-1, 7, 1, -12}, {3, -10, 5, 6},
    {2, -4, 3, -10}, {-13, 0, -13, 5}, {-13, -7, -12, 12}, {-13, 3, -11, 8}, {-7, 12, -4, 7},
    {6, -10, 12, 8}, {-9, -1, -7, -6}, {-2, -5, 0, 12}, {-12, 5, -7, 5}, {3, -10, 8, -13},
    {-7, -7, -4, 5}, {-3, -2, -1, -7}, {2, 9, 5, -11}, {-11, -13, -5, -13}, {-1, 6, 0, -1},
    {5, -3, 5, 2}, {-4, -13, -4, 12}, {-9, -6, -9, 6}, {-12, -10, -8, -4}, {10, 2, 12, -3},
    {7, 12, 12, 12}, {-7, -13, -6, 5}, {-4, 9, -3, 4}, {7, -1, 12, 2}, {-7, 6, -5, 1},
    {-13, 11, -12, 5}, {-3, 7, -2, -6}, {7, -8, 12, -7}, {-13, -7, -11, -12}, {1, -3, 12, 12},
    {2, -6, 3, 0}, {-4, 3, -2, -13}, {-1, -13, 1, 9}, {7, 1, 8, -6}, {1, -1, 3, 12},
    {9, 1, 12, 6}, {-1, -9, -1, 3}, {-13, -13, -10, 5}, {7, 7, 10, 12}, {12, -5, 12, 9},
    {6, 3, 7, 11}, {5, -13, 6, 10}, {2, -12, 2, 3}, {3, 8, 4, -6}, {2, 6, 12, -13},
    {9, -12, 10, 3}, {-8, 4, -7, 9}, {-11, 12, -4, -6}, {1, 12, 2, -8}, {6, -9, 7, -4},
    {2, 3, 3, -2}, {6, 3, 11, 0}, {3, -3, 8, -8}, {7, 8, 9, 3}, {-11, -5, -6, -4},
    {-10, 11, -5, 10}, {-5, -8, -3, 12}, {-10, 5, -9, 0}, {8, -1, 12, -6}, {4, -6, 6, -11},
    {-10, 12, -8, 7}, {4, -2, 6, 7}, {-2, 0, -2, 12}, {-5, -8, -5, 2}, {7, -6, 10, 12},
    {-9, -13, -8, -8}, {-5, -13, -5, -2}, {8, -8, 9, -13}, {-9, -11, -9, 0}, {1, -8, 1, -2},
    {7, -4, 9, 1}, {-2, 1, -1, -4}, {11, -6, 12, -11}, {-12, -9, -6, 4}, {3, 7, 7, 12},
    {5, 5, 10, 8}, {0, -4, 2, 8}, {-9, 12, -5, -13}, {0, 7, 2, 12}, {-1, 2, 1, 7},
    {5, 11, 7, -9}, {3, 5, 6, -8}, {-13, -4, -8, 9}, {-5, 9, -3, -3}, {-4, -7, -3, -12},
    {6, 5, 8, 0}, {-7, 6, -6, 12}, {-13, 6, -5, -2}, {1, -10, 3, 10}, {4, 1, 8, -4},
    {-2, -2, 2, -13}, {2, -12, 12, 12}, {-2, -13, 0, -6}, {4, 1, 9, 3}, {-6, -10, -3, -5},
    {-3, -13, -1, 1}, {7, 5, 12, -11}, {4, -2, 5, -7}, {-13, 9, -9, -5}, {7, 1, 8, 6},
    {7, -8, 7, 6}, {-7, -4, -7, 1}, {-8, 11, -7, -8}, {-13, 6, -12, -8}, {2, 4, 3, 9},
    {10, -5, 12, 3}, {-6, -5, -6, 7}, {8, -3, 9, -8}, {2, -12, 2, 8}, {-11, -2, -10, 3},
    {-12, -13, -7, -9}, {-11, 0, -10, -5}, {5, -3, 11, 8}, {-2, -13, -1, 12}, {-1, -8, 0, 9},
    {-13, -11, -12, -5}, {-10, -2, -10, 11}, {-3, 9, -2, -13}, {2, -3, 3, 2}, {-9, -13, -4, 0},
    {-4, 6, -3, -10}, {-4, 12, -2, -7}, {-6, -11, -4, 9}, {6, -3, 6, 11}, {-13, 11, -5, 5},
    {11, 11, 12, 6}, {7, -5, 12, -2}, {-1, 12, 0, 7}, {-4, -8, -3, -2}, {-7, 1, -6, 7},
    {-13, -12, -8, -13}, {-7, -2, -6, -8}, {-8, 5, -6, -9}, {-5, -1, -4, 5}, {-13, 7, -8, 10},
    {1, 5, 5, -13}, {1, 0, 10, -13}, {9, 12, 10, -1}, {5, -8, 10, -9}, {-1, 11, 1, -13},
    {-9, -3, -6, 2}, {-1, -10, 1, 12}, {-13, 1, -8, -10}, {8, -11, 10, -6}, {2, -13, 3, -6},
    {7, -13, 12, -9}, {-10, -10, -5, -7}, {-10, -8, -8, -13}, {4, -6, 8, 5}, {3, 12, 8, -13},
    {-4, 2, -3, -3}, {5, -13, 10, -12}, {4, -13, 5, -1}, {-9, 9, -4, 3}, {0, 3, 3, -9},
    {-12, 1, -6, 1}, {3, 2, 4, -8}, {-10, -10, -10, 9}, {8, -13, 12, 12}, {-8, -12, -6, -5},
    {2, 2, 3, 7}, {10, 6, 11, -8}, {6, 8, 8, -12}, {-7, 10, -6, 5}, {-3, -9, -3, 9},
    {-1, -13, -1, 5}, {-3, -7, -3, 4}, {-8, -2, -8, 3}, {4, 2, 12, 12}, {2, -5, 3, 11},
    {6, -9, 11, -13}, {3, -1, 7, 12}, {11, -1, 12, 4}, {-3, 0, -3, 6}, {4, -11, 4, 12},
    {2, -4, 2, 1}, {-10, -6, -8, 1}, {-13, 7, -11, 1}, {-13, 12, -11, -13}, {6, 0, 11, -13},
    {0, -1, 1, 4}, {-13, 3, -9, -2}, {-9, 8, -6, -3}, {-13, -6, -8, -2}, {5, -9, 8, 10},
    {2, 7, 3, -9}, {-1, -6, -1, -1}, {9, 5, 11, -2}, {11, -3, 12, -8}, {3, 0, 3, 5},
    {-1, 4, 0, 10}, {3, -6, 4, 5}, {-13, 0, -10, 5}, {5, 8, 12, 11}, {8, 9, 9, -6},
    {7, -4, 8, -12}, {-10, 4, -10, 9}, {7, 3, 12, 4}, {9, -7, 10, -2}, {7, 0, 12, -2},
    {-1, -6, 0, -11},
}};
// clang-format on

/// @brief ORB extraction settings (`slam_it.foundations.orb.Options`).
struct Options {
  /// @brief The feature budget over all levels, from 1 to `kMaxFeatures`.
  std::int64_t n_features{1000};
  /// @brief Pyramid levels, from 1 to `kMaxLevels`.
  std::int64_t levels{8};
  /// @brief Each level's downscale from the previous one, above 1.
  double scale_factor{1.2};
  /// @brief The FAST threshold a cell tries first, from 1 to `kMaxFastThreshold`.
  std::int64_t fast_threshold{20};
  /// @brief The threshold for cells with no corner at `fast_threshold`, from 1 to it.
  std::int64_t min_fast_threshold{7};
};

/// @brief One oriented keypoint.
struct Keypoint {
  /// @brief Its coordinates in the full image: the level pixel mapped exactly
  ///        (`image::full_image_coordinate`); on level 0, the pixel itself.
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

/// @brief A 256-bit rBRIEF descriptor, least significant bit first.
using Descriptor = std::array<std::uint8_t, kDescriptorBytes>;

/// @brief The Hamming distance between two descriptors: the bits in which they differ
///        (`slam_it.foundations.orb.distance`), the one that matching, stereo, the vocabulary
///        and the edge rung compare descriptors by.
/// @param a A descriptor.
/// @param b Another.
/// @return From 0 to 256.
[[nodiscard]] std::int64_t distance(const Descriptor& a, const Descriptor& b);

/// @brief ORB features: keypoints and their descriptors, element for element, and the
///        pyramid they span (with the image's shape, every level's shape).
struct Features {
  /// @brief The keypoints.
  std::vector<Keypoint> keypoints;
  /// @brief Each keypoint's descriptor.
  std::vector<Descriptor> descriptors;
  /// @brief The pyramid's scale factor (`Options::scale_factor`).
  double scale_factor{0.0};
  /// @brief Its number of levels (`Options::levels`).
  std::int64_t levels{0};
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

/// @brief Extract features: `detect`, then each keypoint's rBRIEF descriptor on its level
///        blurred by `kBlurKernel`.
/// @param image The image.
/// @param options The settings.
/// @return The features; `InvalidInput` as `detect`.
[[nodiscard]] Result<Features> extract(const image::Image& image, const Options& options);

/// @brief Extract features from a pyramid already built (`image::pyramid` with the settings'
///        levels and scale factor), as `extract` does from its image: for a consumer that
///        needs the pyramid too, such as stereo matching (ORB-SLAM's `ComputeStereoMatches`
///        reads its extractor's pyramid) (`slam_it.foundations.orb.extract_levels`).
/// @param levels The pyramid.
/// @param options The settings.
/// @return The features; `InvalidInput` for a malformed setting or a pyramid that is not of
///         the settings' levels and scale factor (`image::check_pyramid`).
[[nodiscard]] Result<Features> extract_levels(std::span<const image::Image> levels,
                                              const Options& options);

}  // namespace slam_it::foundations::orb
