/// @file
/// @brief Shared by every binding that returns or takes ORB features (`orb`, `stereo`): they
///        cross as a dict of `slam_it.foundations.orb.Keypoints`' fields, one array each, with
///        `descriptors`, `scale_factor` and `levels` for features.
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "common.hpp"
#include "slam_it/foundations/orb.hpp"

namespace slam_it::bindings {

/// @brief A read-only, contiguous array of the given element type and shape.
/// @tparam T The element type.
/// @tparam Shape The nanobind shape.
template <typename T, typename Shape>
using OrbIn = nanobind::ndarray<const T, Shape, nanobind::c_contig, nanobind::device::cpu>;

/// @brief Keypoints as a dict of `Keypoints`' fields, one array each.
/// @param keypoints The keypoints.
/// @return The dict: `xy`, `level`, `level_xy`, `response` and `angle`.
inline nanobind::dict keypoints_dict(const std::vector<foundations::orb::Keypoint>& keypoints) {
  const std::size_t n = keypoints.size();
  auto [xy, xy_data] = new_array({n, 2});
  auto [level, level_data] = new_array<std::int64_t>({n});
  auto [level_xy, level_xy_data] = new_array<std::int64_t>({n, 2});
  auto [response, response_data] = new_array<std::int64_t>({n});
  auto [angle, angle_data] = new_array<std::int64_t>({n});
  for (std::size_t i = 0; i < n; ++i) {
    const foundations::orb::Keypoint& k = keypoints[i];
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    xy_data[2 * i] = k.xy[0];
    xy_data[(2 * i) + 1] = k.xy[1];
    level_data[i] = k.level;
    level_xy_data[2 * i] = k.level_xy[0];
    level_xy_data[(2 * i) + 1] = k.level_xy[1];
    response_data[i] = k.response;
    angle_data[i] = k.angle;
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  nanobind::dict out;
  out["xy"] = xy;
  out["level"] = level;
  out["level_xy"] = level_xy;
  out["response"] = response;
  out["angle"] = angle;
  return out;
}

/// @brief Features as a dict: the keypoints' fields, `descriptors` (N, 32) uint8,
///        `scale_factor` and `levels`.
/// @param features The features.
/// @return The dict.
inline nanobind::dict features_dict(const foundations::orb::Features& features) {
  nanobind::dict out = keypoints_dict(features.keypoints);
  auto [descriptors, data] =
      new_array<std::uint8_t>({features.descriptors.size(), foundations::orb::kDescriptorBytes});
  for (std::size_t i = 0; i < features.descriptors.size(); ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): row offset
    std::ranges::copy(features.descriptors[i], data + (i * foundations::orb::kDescriptorBytes));
  }
  out["descriptors"] = descriptors;
  out["scale_factor"] = features.scale_factor;
  out["levels"] = features.levels;
  return out;
}

/// @brief Read features from a dict, as a matcher needs them.
/// @param features The dict: `xy` (N, 2) float64, `level` (N,) and `level_xy` (N, 2) int64,
///        `descriptors` (N, 32) uint8, `scale_factor` and `levels`.
/// @return The features (responses and angles, unused by matching, are zero).
/// @throws std::invalid_argument for a missing or unknown key or a malformed array.
inline foundations::orb::Features features_in(const nanobind::dict& features) {
  check_keys(features,
             std::array<const char*, 6>{"xy", "level", "level_xy", "descriptors", "scale_factor",
                                        "levels"},
             "the features");
  namespace nb = nanobind;
  const auto xy = nb::cast<OrbIn<double, nb::shape<-1, 2>>>(features["xy"]);
  const auto level = nb::cast<OrbIn<std::int64_t, nb::shape<-1>>>(features["level"]);
  const auto level_xy = nb::cast<OrbIn<std::int64_t, nb::shape<-1, 2>>>(features["level_xy"]);
  const auto descriptors =
      nb::cast<OrbIn<std::uint8_t, nb::shape<-1, foundations::orb::kDescriptorBytes>>>(
          features["descriptors"]);
  const std::size_t n = xy.shape(0);
  if (level.shape(0) != n || level_xy.shape(0) != n || descriptors.shape(0) != n) {
    throw std::invalid_argument("features: expected one row per keypoint in every array");
  }
  foundations::orb::Features out{.keypoints = std::vector<foundations::orb::Keypoint>(n),
                                 .descriptors = std::vector<foundations::orb::Descriptor>(n),
                                 .scale_factor = nb::cast<double>(features["scale_factor"]),
                                 .levels = nb::cast<std::int64_t>(features["levels"])};
  for (std::size_t i = 0; i < n; ++i) {
    out.keypoints[i] = {.xy = {xy(i, 0), xy(i, 1)},
                        .level = level(i),
                        .level_xy = {level_xy(i, 0), level_xy(i, 1)},
                        .response = 0,
                        .angle = 0};
    for (std::size_t b = 0; b < foundations::orb::kDescriptorBytes; ++b) {
      out.descriptors[i].at(b) = descriptors(i, b);
    }
  }
  return out;
}

}  // namespace slam_it::bindings
