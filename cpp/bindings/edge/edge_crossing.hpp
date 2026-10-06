/// @file
/// @brief Shared by the edge rung's bindings: the map and a frame cross as dicts of arrays.
///
/// A map is a dict of `slamlab.edge.map.Map`'s fields; a frame is the dict
/// `slamlab.edge.map.frame_dict` builds. Arrays are C-contiguous, of the dtype and shape the
/// Python dataclasses hold; a malformed dict raises `std::invalid_argument` (a `ValueError`),
/// and the map is then checked by the core (`check_map`, an `InvalidInputError`).
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "common.hpp"
#include "slamlab/edge/frames.hpp"
#include "slamlab/edge/map.hpp"

namespace slamlab::bindings::edge_crossing {

/// @brief A read-only, contiguous array of the given element type and shape.
/// @tparam T The element type.
/// @tparam Shape The nanobind shape.
template <typename T, typename Shape>
using In = nanobind::ndarray<const T, Shape, nanobind::c_contig, nanobind::device::cpu>;

/// @brief Read one item of a dict as an array of the given type and shape.
/// @tparam T The element type.
/// @tparam Shape The nanobind shape.
/// @param values The dict.
/// @param key The item's key.
/// @return The array.
/// @throws std::invalid_argument if the key is missing; nanobind's cast error if the item is
///         not such an array.
template <typename T, typename Shape>
In<T, Shape> item(const nanobind::dict& values, const char* key) {
  if (!values.contains(key)) {
    throw std::invalid_argument(std::string("missing field `") + key + "`");
  }
  return nanobind::cast<In<T, Shape>>(values[key]);
}

/// @brief Read a 1D array item into a vector.
/// @tparam T The element type.
/// @param values The dict.
/// @param key The item's key.
/// @return The values.
template <typename T>
std::vector<T> vector_item(const nanobind::dict& values, const char* key) {
  const auto in = item<T, nanobind::shape<-1>>(values, key);
  std::vector<T> out(in.shape(0));
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = in(i);
  }
  return out;
}

/// @brief Read an (N, 2) float64 item into pixels.
/// @param values The dict.
/// @param key The item's key.
/// @return The pixels.
inline std::vector<std::array<double, 2>> pixels_item(const nanobind::dict& values,
                                                      const char* key) {
  const auto in = item<double, nanobind::shape<-1, 2>>(values, key);
  std::vector<std::array<double, 2>> out(in.shape(0));
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = {in(i, 0), in(i, 1)};
  }
  return out;
}

/// @brief Read an (N, 3) float64 item into vectors.
/// @param values The dict.
/// @param key The item's key.
/// @return The vectors.
inline std::vector<Eigen::Vector3d> vectors_item(const nanobind::dict& values, const char* key) {
  const auto in = item<double, nanobind::shape<-1, 3>>(values, key);
  std::vector<Eigen::Vector3d> out(in.shape(0));
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = {in(i, 0), in(i, 1), in(i, 2)};
  }
  return out;
}

/// @brief Read an (N, 32) uint8 item into descriptors.
/// @param values The dict.
/// @param key The item's key.
/// @return The descriptors.
inline std::vector<foundations::orb::Descriptor> descriptors_item(const nanobind::dict& values,
                                                                  const char* key) {
  const auto in =
      item<std::uint8_t, nanobind::shape<-1, foundations::orb::kDescriptorBytes>>(values, key);
  std::vector<foundations::orb::Descriptor> out(in.shape(0));
  for (std::size_t i = 0; i < out.size(); ++i) {
    for (std::size_t b = 0; b < foundations::orb::kDescriptorBytes; ++b) {
      out[i].at(b) = in(i, b);
    }
  }
  return out;
}

/// @brief Read a (K, 4, 4) float64 item into poses.
/// @param values The dict.
/// @param key The item's key.
/// @return The poses.
inline std::vector<Eigen::Matrix4d> poses_item(const nanobind::dict& values, const char* key) {
  const auto in = item<double, nanobind::shape<-1, 4, 4>>(values, key);
  std::vector<Eigen::Matrix4d> out(in.shape(0));
  for (std::size_t k = 0; k < out.size(); ++k) {
    for (std::size_t r = 0; r < 4; ++r) {
      for (std::size_t c = 0; c < 4; ++c) {
        out[k](static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = in(k, r, c);
      }
    }
  }
  return out;
}

/// @brief A vector as a new 1D array.
/// @tparam T The element type.
/// @param values The values.
/// @return The array.
template <typename T>
nanobind::ndarray<nanobind::numpy, T> vector_array(const std::vector<T>& values) {
  auto [out, data] = new_array<T>({values.size()});
  std::ranges::copy(values, data);
  return out;
}

/// @brief Pixels as a new (N, 2) array.
/// @param values The pixels.
/// @return The array.
inline nanobind::ndarray<nanobind::numpy, double> pixels_array(
    const std::vector<std::array<double, 2>>& values) {
  auto [out, data] = new_array({values.size(), 2});
  for (std::size_t i = 0; i < values.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    data[2 * i] = values[i][0];
    data[(2 * i) + 1] = values[i][1];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  return out;
}

/// @brief Vectors as a new (N, 3) array.
/// @param values The vectors.
/// @return The array.
inline nanobind::ndarray<nanobind::numpy, double> vectors_array(
    const std::vector<Eigen::Vector3d>& values) {
  auto [out, data] = new_array({values.size(), 3});
  for (std::size_t i = 0; i < values.size(); ++i) {
    for (std::size_t c = 0; c < 3; ++c) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
      data[(3 * i) + c] = values[i](static_cast<Eigen::Index>(c));
    }
  }
  return out;
}

/// @brief Descriptors as a new (N, 32) uint8 array.
/// @param values The descriptors.
/// @return The array.
inline nanobind::ndarray<nanobind::numpy, std::uint8_t> descriptors_array(
    const std::vector<foundations::orb::Descriptor>& values) {
  constexpr std::size_t kBytes = foundations::orb::kDescriptorBytes;
  auto [out, data] = new_array<std::uint8_t>({values.size(), kBytes});
  for (std::size_t i = 0; i < values.size(); ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): row offset
    std::ranges::copy(values[i], data + (i * kBytes));
  }
  return out;
}

/// @brief The map's fields, as the dict's keys.
inline constexpr std::array<const char*, 26> kMapKeys = {
    "levels",     "scale_factor",  "kf_id",       "kf_time",         "kf_pose",
    "kf_start",   "kp_xy",         "kp_level",    "kp_angle",        "kp_descriptor",
    "kp_bearing", "kp_depth",      "kp_right",    "kp_point",        "pt_id",
    "pt_xyz",     "pt_descriptor", "pt_normal",   "pt_min_distance", "pt_max_distance",
    "pt_visible", "pt_found",      "pt_first_kf", "pt_reference_kf", "next_kf_id",
    "next_pt_id"};

/// @brief Read a map from a dict of its fields, checked.
/// @param values The dict.
/// @return The map.
/// @throws std::invalid_argument for a missing or unknown key or a malformed array;
///         CoreError for a map that fails `check_map`.
inline edge::Map map_in(const nanobind::dict& values) {
  check_keys(values, kMapKeys, "edge.map.Map");
  edge::Map m{.levels = nanobind::cast<std::int64_t>(values["levels"]),
              .scale_factor = nanobind::cast<double>(values["scale_factor"]),
              .kf_id = vector_item<std::int64_t>(values, "kf_id"),
              .kf_time = vector_item<double>(values, "kf_time"),
              .kf_pose = poses_item(values, "kf_pose"),
              .kf_start = vector_item<std::int64_t>(values, "kf_start"),
              .kp_xy = pixels_item(values, "kp_xy"),
              .kp_level = vector_item<std::int64_t>(values, "kp_level"),
              .kp_angle = vector_item<std::int64_t>(values, "kp_angle"),
              .kp_descriptor = descriptors_item(values, "kp_descriptor"),
              .kp_bearing = vectors_item(values, "kp_bearing"),
              .kp_depth = vector_item<double>(values, "kp_depth"),
              .kp_right = vector_item<double>(values, "kp_right"),
              .kp_point = vector_item<std::int64_t>(values, "kp_point"),
              .pt_id = vector_item<std::int64_t>(values, "pt_id"),
              .pt_xyz = vectors_item(values, "pt_xyz"),
              .pt_descriptor = descriptors_item(values, "pt_descriptor"),
              .pt_normal = vectors_item(values, "pt_normal"),
              .pt_min_distance = vector_item<double>(values, "pt_min_distance"),
              .pt_max_distance = vector_item<double>(values, "pt_max_distance"),
              .pt_visible = vector_item<std::int64_t>(values, "pt_visible"),
              .pt_found = vector_item<std::int64_t>(values, "pt_found"),
              .pt_first_kf = vector_item<std::int64_t>(values, "pt_first_kf"),
              .pt_reference_kf = vector_item<std::int64_t>(values, "pt_reference_kf"),
              .next_kf_id = nanobind::cast<std::int64_t>(values["next_kf_id"]),
              .next_pt_id = nanobind::cast<std::int64_t>(values["next_pt_id"])};
  unwrap(edge::check_map(m));
  return m;
}

/// @brief A map as a dict of its fields.
/// @param m The map.
/// @return The dict.
inline nanobind::dict map_dict(const edge::Map& m) {
  nanobind::dict out;
  out["levels"] = m.levels;
  out["scale_factor"] = m.scale_factor;
  out["kf_id"] = vector_array(m.kf_id);
  out["kf_time"] = vector_array(m.kf_time);
  out["kf_pose"] = poses_array(m.kf_pose);
  out["kf_start"] = vector_array(m.kf_start);
  out["kp_xy"] = pixels_array(m.kp_xy);
  out["kp_level"] = vector_array(m.kp_level);
  out["kp_angle"] = vector_array(m.kp_angle);
  out["kp_descriptor"] = descriptors_array(m.kp_descriptor);
  out["kp_bearing"] = vectors_array(m.kp_bearing);
  out["kp_depth"] = vector_array(m.kp_depth);
  out["kp_right"] = vector_array(m.kp_right);
  out["kp_point"] = vector_array(m.kp_point);
  out["pt_id"] = vector_array(m.pt_id);
  out["pt_xyz"] = vectors_array(m.pt_xyz);
  out["pt_descriptor"] = descriptors_array(m.pt_descriptor);
  out["pt_normal"] = vectors_array(m.pt_normal);
  out["pt_min_distance"] = vector_array(m.pt_min_distance);
  out["pt_max_distance"] = vector_array(m.pt_max_distance);
  out["pt_visible"] = vector_array(m.pt_visible);
  out["pt_found"] = vector_array(m.pt_found);
  out["pt_first_kf"] = vector_array(m.pt_first_kf);
  out["pt_reference_kf"] = vector_array(m.pt_reference_kf);
  out["next_kf_id"] = m.next_kf_id;
  out["next_pt_id"] = m.next_pt_id;
  return out;
}

/// @brief The frame's fields, as the dict's keys.
inline constexpr std::array<const char*, 12> kFrameKeys = {
    "timestamp",   "xy",           "level",  "level_xy", "response", "angle",
    "descriptors", "scale_factor", "levels", "bearings", "depth",    "right"};

/// @brief Read a frame from its dict.
/// @param values The dict.
/// @return The frame.
/// @throws std::invalid_argument for a missing or unknown key, or arrays that are malformed
///         or not one row per keypoint.
inline edge::Frame frame_in(const nanobind::dict& values) {
  check_keys(values, kFrameKeys, "the frame");
  const auto xy = pixels_item(values, "xy");
  const auto level = vector_item<std::int64_t>(values, "level");
  const auto level_xy = item<std::int64_t, nanobind::shape<-1, 2>>(values, "level_xy");
  const auto response = vector_item<std::int64_t>(values, "response");
  const auto angle = vector_item<std::int64_t>(values, "angle");
  edge::Frame frame{.timestamp = nanobind::cast<double>(values["timestamp"]),
                    .features = {.keypoints = {},
                                 .descriptors = descriptors_item(values, "descriptors"),
                                 .scale_factor = nanobind::cast<double>(values["scale_factor"]),
                                 .levels = nanobind::cast<std::int64_t>(values["levels"])},
                    .bearings = vectors_item(values, "bearings"),
                    .depth = vector_item<double>(values, "depth"),
                    .right = vector_item<double>(values, "right")};
  const std::size_t n = xy.size();
  if (level.size() != n || level_xy.shape(0) != n || response.size() != n || angle.size() != n ||
      frame.features.descriptors.size() != n || frame.bearings.size() != n ||
      frame.depth.size() != n || frame.right.size() != n) {
    throw std::invalid_argument("frame: expected one row per keypoint in every array");
  }
  frame.features.keypoints.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    frame.features.keypoints[i] = {.xy = xy[i],
                                   .level = level[i],
                                   .level_xy = {level_xy(i, 0), level_xy(i, 1)},
                                   .response = response[i],
                                   .angle = angle[i]};
  }
  return frame;
}

}  // namespace slamlab::bindings::edge_crossing
