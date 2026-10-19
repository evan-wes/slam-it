/// @file
/// @brief Implementation of the TUM-format reader declared in slam_it/eval/tum.hpp.
#include "slam_it/eval/tum.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <string>
#include <utility>

#include "slam_it/eval/png.hpp"
#include "slam_it/eval/text.hpp"
#include "slam_it/foundations/image.hpp"

namespace slam_it::eval::tum {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
namespace fs = std::filesystem;

/// @brief The benchmark's sensors' published `radtan` intrinsics, by the name a folder holds.
constexpr std::array<std::pair<const char*, std::array<double, 9>>, 3> kSensors{{
    {"freiburg1", {517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633}},
    {"freiburg2", {520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172}},
    {"freiburg3", {535.4, 539.2, 320.1, 247.6, 0.0, 0.0, 0.0, 0.0, 0.0}},
}};

/// @brief `camera.txt`'s keys.
constexpr std::array<const char*, 4> kCameraKeys = {"model", "intrinsics", "size", "depth_scale"};

/// @brief The keys it may hold too.
constexpr std::array<const char*, 1> kOptionalCameraKeys = {"bf"};

/// @brief A `timestamp filename` file's rows.
struct Stamped {
  /// @brief The stamps, non-decreasing.
  std::vector<double> stamps;
  /// @brief The file names.
  std::vector<std::string> names;
};

/// @brief Read a `timestamp filename` file.
/// @param path The file.
/// @return Its rows; `InvalidInput` for a short row, a non-numeric or non-finite stamp, or
///         stamps that descend.
Result<Stamped> stamped(const std::string& path) {
  auto lines = text::read_lines(path);
  if (!lines) {
    return std::unexpected(std::move(lines.error()));
  }
  Stamped out;
  for (std::size_t k = 0; k < lines->size(); ++k) {
    const std::string& line = (*lines)[k];
    if (!text::is_data(line)) {
      continue;
    }
    const auto parts = text::fields(line);
    if (parts.size() < 2) {
      return fail(ErrorKind::kInvalidInput, text::where(path, k + 1) + "expected 2 fields, got " +
                                                std::to_string(parts.size()));
    }
    const auto stamp = text::number(parts[0]);
    if (!stamp) {
      return fail(ErrorKind::kInvalidInput, text::where(path, k + 1) +
                                                "expected a numeric timestamp, got '" +
                                                std::string(parts[0]) + "'");
    }
    out.stamps.push_back(*stamp);
    out.names.emplace_back(parts[1]);
  }
  for (std::size_t k = 0; k < out.stamps.size(); ++k) {
    if (!std::isfinite(out.stamps[k]) || (k > 0 && out.stamps[k] < out.stamps[k - 1])) {
      return fail(ErrorKind::kInvalidInput, path + ": must ascend (sort them first)");
    }
  }
  return out;
}

/// @brief The index of the nearest of ascending times to a stamp (ties: the earlier),
///        `slam_it.eval.timestamps.nearest`.
/// @param times The times, ascending, not empty.
/// @param stamp The stamp.
/// @return The index.
std::size_t nearest(const std::vector<double>& times, double stamp) {
  if (times.size() == 1) {
    return 0;
  }
  const auto first =
      static_cast<std::size_t>(std::ranges::lower_bound(times, stamp) - times.begin());
  const std::size_t after = std::clamp<std::size_t>(first, 1, times.size() - 1);
  const std::size_t before = after - 1;
  return std::abs(times[before] - stamp) <= std::abs(times[after] - stamp) ? before : after;
}

/// @brief Find a camera model by its name.
/// @param name The name.
/// @return The model, if one has it.
std::optional<foundations::camera::Model> model_named(std::string_view name) {
  for (std::uint8_t code = 0; code < static_cast<std::uint8_t>(foundations::camera::Model::kCount);
       ++code) {
    const auto model = static_cast<foundations::camera::Model>(code);
    if (name == foundations::camera::name(model)) {
      return model;
    }
  }
  return std::nullopt;
}

/// @brief Read a `camera.txt`'s keys, each once.
/// @param path The file.
/// @return Each key's values; `InvalidInput` for an unknown, repeated or missing key.
Result<std::map<std::string, std::vector<std::string>>> camera_values(const std::string& path) {
  auto lines = text::read_uncommented(path);
  if (!lines) {
    return std::unexpected(std::move(lines.error()));
  }
  std::map<std::string, std::vector<std::string>> out;
  for (std::size_t k = 0; k < lines->size(); ++k) {
    const auto parts = text::fields((*lines)[k]);
    if (parts.empty()) {
      continue;
    }
    const std::string key(parts[0]);
    const bool known = std::ranges::find(kCameraKeys, key) != kCameraKeys.end() ||
                       std::ranges::find(kOptionalCameraKeys, key) != kOptionalCameraKeys.end();
    if (!known || out.contains(key)) {
      return fail(ErrorKind::kInvalidInput,
                  text::where(path, k + 1) + "unknown or repeated key '" + key + "'");
    }
    out[key] = std::vector<std::string>(parts.begin() + 1, parts.end());
  }
  for (const char* key : kCameraKeys) {
    if (!out.contains(key)) {
      return fail(ErrorKind::kInvalidInput, path + ": missing " + key);
    }
  }
  return out;
}

/// @brief The error for a `camera.txt` value that is not a number, naming it.
/// @param path The file.
/// @param key Its key.
/// @param value The value.
/// @return `InvalidInput`.
foundations::Error not_a_number(const std::string& path, const std::string& key,
                                const std::string& value) {
  return foundations::Error{.kind = ErrorKind::kInvalidInput,
                            .message = path + ": expected numbers (" + key + ": '" + value + "')"};
}

/// @brief Read a `camera.txt` (`slam_it.eval.tum._camera_file`).
/// @param path The file.
/// @return The camera; `InvalidInput` for a malformed one.
Result<Camera> camera_file(const std::string& path) {
  auto values = camera_values(path);
  if (!values) {
    return std::unexpected(std::move(values.error()));
  }
  auto& v = *values;
  if (v["model"].size() != 1 || v["size"].size() != 2 || v["depth_scale"].size() != 1 ||
      (v.contains("bf") && v["bf"].size() != 1)) {
    return fail(ErrorKind::kInvalidInput,
                path + ": expected one model, two sizes, one depth scale, at most one bf");
  }
  const auto model = model_named(v["model"][0]);
  if (!model) {
    return fail(ErrorKind::kInvalidInput, path + ": unknown camera model '" + v["model"][0] + "'");
  }
  Camera out{.model = *model,
             .intrinsics = Eigen::VectorXd(static_cast<Eigen::Index>(v["intrinsics"].size())),
             .width = 0,
             .height = 0,
             .depth_scale = 0.0,
             .bf = std::nullopt};
  for (std::size_t k = 0; k < v["intrinsics"].size(); ++k) {
    const auto value = text::number(v["intrinsics"][k]);
    if (!value) {
      return std::unexpected(not_a_number(path, "intrinsics", v["intrinsics"][k]));
    }
    out.intrinsics(static_cast<Eigen::Index>(k)) = *value;
  }
  const auto width = text::integer(v["size"][0]);
  const auto height = text::integer(v["size"][1]);
  if (!width || !height) {
    return std::unexpected(not_a_number(path, "size", v["size"][0] + " " + v["size"][1]));
  }
  const auto scale = text::number(v["depth_scale"][0]);
  if (!scale) {
    return std::unexpected(not_a_number(path, "depth_scale", v["depth_scale"][0]));
  }
  if (auto ok = foundations::camera::check_intrinsics(out.model, out.intrinsics); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (*width < 1 || *width > foundations::image::kMaxSide || *height < 1 ||
      *height > foundations::image::kMaxSide) {
    return fail(ErrorKind::kInvalidInput, path + ": size must lie in [1, " +
                                              std::to_string(foundations::image::kMaxSide) + "]");
  }
  if (!std::isfinite(*scale) || !(*scale > 0.0)) {
    return fail(ErrorKind::kInvalidInput, path + ": depth_scale must be finite and positive");
  }
  if (v.contains("bf")) {
    const auto bf = text::number(v["bf"][0]);
    if (!bf) {
      return std::unexpected(not_a_number(path, "bf", v["bf"][0]));
    }
    if (!std::isfinite(*bf) || !(*bf > 0.0)) {
      return fail(ErrorKind::kInvalidInput, path + ": bf must be finite and positive");
    }
    out.bf = bf;
  }
  out.width = *width;
  out.height = *height;
  out.depth_scale = *scale;
  return out;
}

}  // namespace

Result<Camera> read_camera(const std::string& root) {
  const fs::path folder(root);
  if (fs::exists(folder / kCameraFile)) {
    return camera_file((folder / kCameraFile).string());
  }
  const std::string name = folder.filename().string();
  for (const auto& [sensor, values] : kSensors) {
    if (name.contains(sensor)) {
      return Camera{.model = foundations::camera::Model::kRadTan,
                    .intrinsics = Eigen::Map<const Eigen::VectorXd>(values.data(), 9),
                    .width = kWidth,
                    .height = kHeight,
                    .depth_scale = kDepthScale,
                    .bf = kSensorBf};
    }
  }
  return fail(ErrorKind::kInvalidInput, name + ": expected a " + kCameraFile +
                                            ", or 'freiburg1', '2' or '3' in the folder's name");
}

Result<Frames> read_frames(const std::string& root, double max_difference) {
  if (!std::isfinite(max_difference) || max_difference < 0.0) {
    return fail(ErrorKind::kInvalidInput, "max_difference: must be non-negative");
  }
  const fs::path folder(root);
  auto camera = read_camera(root);
  if (!camera) {
    return std::unexpected(std::move(camera.error()));
  }
  auto colour = stamped((folder / "rgb.txt").string());
  if (!colour) {
    return std::unexpected(std::move(colour.error()));
  }
  Frames out{.root = root,
             .timestamps = colour->stamps,
             .images = {},
             .depths = {},
             .camera = std::move(*camera),
             .mask = std::nullopt};
  if (const fs::path mask_file = folder / kMaskFile; fs::exists(mask_file)) {
    auto mask = png::read_gray(mask_file.string());
    if (!mask) {
      return std::unexpected(std::move(mask.error()));
    }
    if (mask->rows != out.camera.height || mask->cols != out.camera.width) {
      return fail(ErrorKind::kInvalidInput, mask_file.string() + ": expected the camera's size " +
                                                std::to_string(out.camera.width) + "x" +
                                                std::to_string(out.camera.height));
    }
    out.mask = std::move(*mask);
  }
  for (const std::string& name : colour->names) {
    out.images.push_back((folder / name).string());
  }
  out.depths.assign(out.timestamps.size(), std::nullopt);
  if (fs::exists(folder / "depth.txt")) {
    auto depth = stamped((folder / "depth.txt").string());
    if (!depth) {
      return std::unexpected(std::move(depth.error()));
    }
    for (std::size_t k = 0; !depth->stamps.empty() && k < out.timestamps.size(); ++k) {
      const std::size_t d = nearest(depth->stamps, out.timestamps[k]);
      if (std::abs(depth->stamps[d] - out.timestamps[k]) <= max_difference) {
        out.depths[k] = (folder / depth->names[d]).string();
      }
    }
  }
  return out;
}

Result<Depth> read_depth(const std::string& path, double depth_scale) {
  if (!std::isfinite(depth_scale) || !(depth_scale > 0.0)) {
    return fail(ErrorKind::kInvalidInput, "depth_scale: must be finite and positive");
  }
  auto raw = png::read_depth(path);
  if (!raw) {
    return std::unexpected(std::move(raw.error()));
  }
  Depth out{.rows = raw->rows, .cols = raw->cols, .values = {}};
  out.values.reserve(raw->values.size());
  for (const std::uint16_t value : raw->values) {
    out.values.push_back(value == 0 ? std::numeric_limits<double>::quiet_NaN()
                                    : static_cast<double>(value) / depth_scale);
  }
  return out;
}

}  // namespace slam_it::eval::tum
