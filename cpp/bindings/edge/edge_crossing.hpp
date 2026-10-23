/// @file
/// @brief Shared by the edge rung's bindings: the map and a frame cross as dicts of arrays.
///
/// A map is a dict of `slam_it.edge.map.Map`'s fields; a frame is the dict
/// `slam_it.edge.frames.Frame.as_dict` builds; the camera and the settings are the dicts of
/// scalars `Camera.as_dict` and `Options.as_dict` give. Arrays are C-contiguous, of the dtype and
/// shape the Python dataclasses hold; a malformed dict raises `std::invalid_argument` (a
/// `ValueError`), and the map is then checked by the core (`check_map`, an `InvalidInputError`).
/// A stage's summary crosses back as a dict of its fields, an empty count as None and an enum as
/// its code in the module's `codes()`.
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "common.hpp"
#include "foundations/nls_crossing.hpp"
#include "slam_it/edge/adjust.hpp"
#include "slam_it/edge/database.hpp"
#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/initialization.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/matching.hpp"

namespace slam_it::bindings::edge_crossing {

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

/// @brief A read-only, contiguous int64 vector (rows, ids).
using Int64In = VectorIn<std::int64_t>;

/// @brief Copy an int64 vector in.
/// @param in The array.
/// @return The values.
inline std::vector<std::int64_t> int64_in(const Int64In& in) {
  return copy_vector<std::vector<std::int64_t>>(in);
}

/// @brief Copy a boolean vector in as the core's flags.
/// @param in The array.
/// @return One flag per element: 1 for true, 0 for false.
inline std::vector<std::uint8_t> flags_in(const VectorIn<bool>& in) {
  std::vector<std::uint8_t> out(in.shape(0));
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = in(i) ? 1 : 0;
  }
  return out;
}

/// @brief Read an (N, 2) item into pairs: pixels (float64), or keyframe ids (int64).
/// @tparam T The element type.
/// @param values The dict.
/// @param key The item's key.
/// @return The pairs.
template <typename T>
std::vector<std::array<T, 2>> pairs_item(const nanobind::dict& values, const char* key) {
  const auto in = item<T, nanobind::shape<-1, 2>>(values, key);
  std::vector<std::array<T, 2>> out(in.shape(0));
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

/// @brief Pairs as a new (N, 2) array: pixels (float64), or keyframe ids (int64).
/// @tparam T The element type.
/// @param values The pairs.
/// @return The array.
template <typename T>
nanobind::ndarray<nanobind::numpy, T> pairs_array(const std::vector<std::array<T, 2>>& values) {
  auto [out, data] = new_array<T>({values.size(), 2});
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
inline constexpr std::array<const char*, 37> kMapKeys = {
    "levels",          "scale_factor", "kf_id",         "kf_time",        "kf_pose",
    "kf_parent",       "kf_start",     "kp_xy",         "kp_level",       "kp_angle",
    "kp_descriptor",   "kp_bearing",   "kp_depth",      "kp_right",       "kp_point",
    "pt_id",           "pt_xyz",       "pt_descriptor", "pt_normal",      "pt_min_distance",
    "pt_max_distance", "pt_visible",   "pt_found",      "pt_first_kf",    "pt_reference_kf",
    "next_kf_id",      "next_pt_id",   "gone_pt_id",    "gone_pt_xyz",    "gone_pt_how",
    "gone_kf_id",      "gone_kf_time", "gone_kf_pose",  "gone_kf_parent", "gone_kf_relative",
    "loop_edges",      "merge_edges"};

/// @brief Read a map from a dict of its fields, its observations indexed, checked.
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
              .kf_parent = vector_item<std::int64_t>(values, "kf_parent"),
              .kf_start = vector_item<std::int64_t>(values, "kf_start"),
              .kp_xy = pairs_item<double>(values, "kp_xy"),
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
              .next_pt_id = nanobind::cast<std::int64_t>(values["next_pt_id"]),
              .gone_pt_id = vector_item<std::int64_t>(values, "gone_pt_id"),
              .gone_pt_xyz = vectors_item(values, "gone_pt_xyz"),
              .gone_pt_how = vector_item<std::int64_t>(values, "gone_pt_how"),
              .gone_kf_id = vector_item<std::int64_t>(values, "gone_kf_id"),
              .gone_kf_time = vector_item<double>(values, "gone_kf_time"),
              .gone_kf_pose = poses_item(values, "gone_kf_pose"),
              .gone_kf_parent = vector_item<std::int64_t>(values, "gone_kf_parent"),
              .gone_kf_relative = poses_item(values, "gone_kf_relative"),
              .loop_edges = pairs_item<std::int64_t>(values, "loop_edges"),
              .merge_edges = pairs_item<std::int64_t>(values, "merge_edges"),
              .pt_observations = {}};
  edge::index_observations(m);
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
  out["kf_parent"] = vector_array(m.kf_parent);
  out["kf_start"] = vector_array(m.kf_start);
  out["kp_xy"] = pairs_array(m.kp_xy);
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
  out["gone_pt_id"] = vector_array(m.gone_pt_id);
  out["gone_pt_xyz"] = vectors_array(m.gone_pt_xyz);
  out["gone_pt_how"] = vector_array(m.gone_pt_how);
  out["gone_kf_id"] = vector_array(m.gone_kf_id);
  out["gone_kf_time"] = vector_array(m.gone_kf_time);
  out["gone_kf_pose"] = poses_array(m.gone_kf_pose);
  out["gone_kf_parent"] = vector_array(m.gone_kf_parent);
  out["gone_kf_relative"] = poses_array(m.gone_kf_relative);
  out["loop_edges"] = pairs_array(m.loop_edges);
  out["merge_edges"] = pairs_array(m.merge_edges);
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
///         or not one row per keypoint; CoreError for a keypoint position that is not finite.
inline edge::Frame frame_in(const nanobind::dict& values) {
  check_keys(values, kFrameKeys, "the frame");
  const auto xy = pairs_item<double>(values, "xy");
  const auto level = vector_item<std::int64_t>(values, "level");
  const auto level_xy = item<std::int64_t, nanobind::shape<-1, 2>>(values, "level_xy");
  const auto response = vector_item<std::int64_t>(values, "response");
  const auto angle = vector_item<std::int64_t>(values, "angle");
  foundations::orb::Features features{
      .keypoints = {},
      .descriptors = descriptors_item(values, "descriptors"),
      .scale_factor = nanobind::cast<double>(values["scale_factor"]),
      .levels = nanobind::cast<std::int64_t>(values["levels"])};
  std::vector<Eigen::Vector3d> bearings = vectors_item(values, "bearings");
  std::vector<double> depth = vector_item<double>(values, "depth");
  std::vector<double> right = vector_item<double>(values, "right");
  const std::size_t n = xy.size();
  if (level.size() != n || level_xy.shape(0) != n || response.size() != n || angle.size() != n ||
      features.descriptors.size() != n || bearings.size() != n || depth.size() != n ||
      right.size() != n) {
    throw std::invalid_argument("frame: expected one row per keypoint in every array");
  }
  features.keypoints.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    features.keypoints[i] = {.xy = xy[i],
                             .level = level[i],
                             .level_xy = {level_xy(i, 0), level_xy(i, 1)},
                             .response = response[i],
                             .angle = angle[i]};
  }
  return unwrap(edge::make_frame(nanobind::cast<double>(values["timestamp"]), std::move(features),
                                 std::move(bearings), std::move(depth), std::move(right)));
}

/// @brief Read the settings from a dict of groups (`Options.as_dict`).
/// @param values The dict: exactly `Options`' groups, each a dict of exactly its fields, and
///        `seed`.
/// @return The settings (checked by the core).
/// @throws std::invalid_argument for a missing or unknown key.
inline edge::Options options_in(const nanobind::dict& values) {
  edge::Options out;
  std::vector<std::string> top;  // the groups, then seed
  std::vector<std::pair<std::string, std::vector<std::string>>> groups;
  edge::for_each_setting(out, [&](const char* name, const auto&) {
    const auto [group, field] = edge::split_setting(name);
    if (group.empty()) {
      top.emplace_back(field);
      return;
    }
    if (groups.empty() || groups.back().first != group) {
      top.emplace_back(group);
      groups.emplace_back(group, std::vector<std::string>{});
    }
    groups.back().second.emplace_back(field);
  });
  const auto pointers = [](const std::vector<std::string>& names) {
    std::vector<const char*> out;
    for (const std::string& name : names) {
      out.push_back(name.c_str());
    }
    return out;
  };
  check_keys(values, pointers(top), "edge.Options");
  for (const auto& [group, fields] : groups) {
    const std::string what = "edge.Options." + group;
    check_keys(nanobind::cast<nanobind::dict>(values[group.c_str()]), pointers(fields),
               what.c_str());
  }
  edge::for_each_setting(out, [&](const char* name, auto& field) {
    const auto [group, item] = edge::split_setting(name);
    const std::string key(item);
    const nanobind::dict owner =
        group.empty() ? values : nanobind::cast<nanobind::dict>(values[std::string(group).c_str()]);
    field = nanobind::cast<std::remove_reference_t<decltype(field)>>(owner[key.c_str()]);
  });
  return out;
}

/// @brief The camera's fields, as the dict's keys.
inline constexpr std::array<const char*, 5> kCameraKeys = {"model", "intrinsics", "width", "height",
                                                           "baseline"};

/// @brief Read the camera from its dict.
/// @param values The dict, with exactly `Camera`'s fields.
/// @return The camera (checked by the core).
/// @throws std::invalid_argument for a missing or unknown key, or an unknown model.
inline edge::Camera camera_in(const nanobind::dict& values) {
  check_keys(values, kCameraKeys, "edge.frames.Camera");
  const std::vector<double> intrinsics = vector_item<double>(values, "intrinsics");
  return {.model = enum_by_name<foundations::camera::Model>(
              nanobind::cast<std::string>(values["model"]),
              [](foundations::camera::Model m) { return foundations::camera::name(m); }, "model"),
          .intrinsics = Eigen::Map<const Eigen::VectorXd>(
              intrinsics.data(), static_cast<Eigen::Index>(intrinsics.size())),
          .width = nanobind::cast<std::int64_t>(values["width"]),
          .height = nanobind::cast<std::int64_t>(values["height"]),
          .baseline = nanobind::cast<double>(values["baseline"])};
}

/// @brief A tracked frame's fields, as the dict's keys.
inline constexpr std::array<const char*, 2> kTrackedKeys = {"pose", "points"};

/// @brief Read a tracked frame from its dict.
/// @param values The dict: `pose` (4, 4) and `points` (N,).
/// @return The tracked frame.
/// @throws std::invalid_argument for a missing or unknown key, or a malformed array.
inline edge::Tracked tracked_in(const nanobind::dict& values) {
  check_keys(values, kTrackedKeys, "edge.frames.Tracked");
  const auto pose = item<double, nanobind::shape<4, 4>>(values, "pose");
  edge::Tracked out{.pose = Eigen::Matrix4d::Zero(),
                    .points = vector_item<std::int64_t>(values, "points")};
  for (std::size_t r = 0; r < 4; ++r) {
    for (std::size_t c = 0; c < 4; ++c) {
      out.pose(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = pose(r, c);
    }
  }
  return out;
}

/// @brief A tracked frame as a dict.
/// @param tracked The tracked frame.
/// @return `pose` (4, 4) and `points` (N,).
inline nanobind::dict tracked_dict(const edge::Tracked& tracked) {
  nanobind::dict out;
  out["pose"] = matrix_array(tracked.pose);
  out["points"] = vector_array(tracked.points);
  return out;
}

/// @brief A tracked frame as a dict, or None.
/// @param tracked The tracked frame, if any.
/// @return The dict (see `tracked_dict`), or None.
inline nanobind::object maybe_tracked(const std::optional<edge::Tracked>& tracked) {
  return tracked ? nanobind::object(tracked_dict(*tracked)) : nanobind::none();
}

/// @brief Read a mode by its name (`slam_it.edge.options.MODES`).
/// @param name The name.
/// @return The mode.
/// @throws std::invalid_argument for an unknown name.
inline edge::Mode mode_in(const std::string& name) {
  return enum_by_name<edge::Mode>(name, [](edge::Mode m) { return edge::name(m); }, "mode");
}

/// @brief A count, or None.
/// @param count The count, if any.
/// @return It as an int, or None.
inline nanobind::object count_or_none(const std::optional<std::int64_t>& count) {
  return count ? nanobind::object(nanobind::int_(*count)) : nanobind::none();
}

/// @brief Bundle-adjustment solves as they cross (`slam_it.edge.adjust.decode_solves`).
/// @param solves The solves.
/// @return A list of dicts of `initial_cost`, `final_cost`, `iterations` and `termination`
///         (its code in `nls_termination`).
inline nanobind::list solves_list(const std::vector<edge::Solve>& solves) {
  nanobind::list out;
  for (const edge::Solve& solve : solves) {
    nanobind::dict item;
    item["initial_cost"] = solve.initial_cost;
    item["final_cost"] = solve.final_cost;
    item["iterations"] = solve.iterations;
    item["termination"] = static_cast<int>(solve.termination);
    out.append(item);
  }
  return out;
}

/// @brief `nls_termination`'s table, for a module whose results carry solves.
/// @return `{"nls_termination": [names in code order]}`.
inline nanobind::dict solve_codes() {
  nanobind::dict out;
  out["nls_termination"] = nls_terminations();
  return out;
}

/// @brief An attempt to start the map as a dict.
/// @param attempt The attempt.
/// @return `started` (a dict of `map`, as `map_dict`, `pose` (4, 4) and `points` (N,); or
///         None), `outcome` (its code), `matches` (or None) and `solves` (as `solves_list`).
inline nanobind::dict initialization_dict(const edge::Initialization& attempt) {
  nanobind::dict out;
  if (attempt.started) {
    nanobind::dict started = tracked_dict(attempt.started->tracked);
    started["map"] = map_dict(attempt.started->map);
    out["started"] = started;
  } else {
    out["started"] = nanobind::none();
  }
  out["outcome"] = static_cast<int>(attempt.summary.outcome);
  out["matches"] = count_or_none(attempt.summary.matches);
  out["solves"] = solves_list(attempt.summary.solves);
  return out;
}

/// @brief `slam_it.edge.database.Database.as_dict`'s keys.
inline constexpr std::array<const char*, 6> kDatabaseKeys = {"kf_id",     "bow_start",  "bow_word",
                                                             "bow_value", "node_start", "node"};

/// @brief A keyframe database from `slam_it.edge.database.Database.as_dict()`'s dict: each
///        keyframe's vector and nodes, laid end to end by id; the core builds its own index.
/// @param values The dict: `kf_id`, `bow_start`, `bow_word`, `bow_value`, `node_start` and
///        `node`.
/// @return The database.
/// @throws std::invalid_argument for a missing or unknown key, or offsets that do not lay the
///         arrays end to end; `CoreError` for a keyframe given twice.
inline edge::Database database_in(const nanobind::dict& values) {
  check_keys(values, kDatabaseKeys, "edge.database.Database.as_dict");
  const auto ids = vector_item<std::int64_t>(values, "kf_id");
  const auto bow_start = vector_item<std::int64_t>(values, "bow_start");
  const auto bow_word = vector_item<std::int64_t>(values, "bow_word");
  const auto bow_value = vector_item<double>(values, "bow_value");
  const auto node_start = vector_item<std::int64_t>(values, "node_start");
  const auto node = vector_item<std::int64_t>(values, "node");
  const auto laid = [&ids](const std::vector<std::int64_t>& start, std::size_t total) {
    return start.size() == ids.size() + 1 && start.front() == 0 &&
           std::cmp_equal(start.back(), total) && std::ranges::is_sorted(start);
  };
  if (!laid(bow_start, bow_word.size()) || bow_value.size() != bow_word.size() ||
      !laid(node_start, node.size())) {
    throw std::invalid_argument("database: offsets that do not lay its arrays end to end");
  }
  edge::Database out;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const auto words =
        std::span(bow_word).subspan(static_cast<std::size_t>(bow_start[i]),
                                    static_cast<std::size_t>(bow_start[i + 1] - bow_start[i]));
    const auto weights =
        std::span(bow_value).subspan(static_cast<std::size_t>(bow_start[i]), words.size());
    const auto nodes =
        std::span(node).subspan(static_cast<std::size_t>(node_start[i]),
                                static_cast<std::size_t>(node_start[i + 1] - node_start[i]));
    unwrap(edge::insert(out, ids[i],
                        {.vector = {.ids = {words.begin(), words.end()},
                                    .values = {weights.begin(), weights.end()}},
                         .nodes = {nodes.begin(), nodes.end()}}));
  }
  return out;
}

/// @brief The keys of a map crossing with its database and number (`Indexed.as_dict`).
inline constexpr std::array<const char*, 3> kIndexedKeys = {"number", "m", "database"};

/// @brief Read maps with their databases and numbers (`slam_it.edge.database.Indexed`).
/// @param values A list of dicts, each `number`, `m` and `database`.
/// @return The maps.
/// @throws std::invalid_argument for a missing or unknown key; what `map_in` and `database_in`
///         throw.
inline std::vector<edge::Indexed> indexed_in(const nanobind::list& values) {
  std::vector<edge::Indexed> out;
  out.reserve(values.size());
  for (const nanobind::handle item : values) {
    const auto dict = nanobind::cast<nanobind::dict>(item);
    check_keys(dict, kIndexedKeys, "edge.database.Indexed.as_dict");
    out.push_back({.number = nanobind::cast<std::int64_t>(dict["number"]),
                   .map = map_in(nanobind::cast<nanobind::dict>(dict["m"])),
                   .database = database_in(nanobind::cast<nanobind::dict>(dict["database"]))});
  }
  return out;
}

}  // namespace slam_it::bindings::edge_crossing
