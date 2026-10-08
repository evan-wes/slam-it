/// @file
/// @brief Implementation of the run writer declared in slamlab/eval/runs.hpp.
#include "slamlab/eval/runs.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string_view>
#include <system_error>

#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::eval::runs {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
namespace fs = std::filesystem;

/// @brief The modes a run may have.
constexpr std::array<const char*, 3> kModes = {"monocular", "stereo", "rgbd"};

/// @brief The backends a run may come from.
constexpr std::array<const char*, 3> kBackends = {"python", "cpp", "rust"};

/// @brief numpy leaves room for the first dimension to grow to this many digits.
constexpr std::size_t kGrowthDigits = 21;

/// @brief numpy aligns the data to this many bytes.
constexpr std::size_t kAlign = 64;

/// @brief A string as JSON: quoted, escaped.
/// @param value The string.
/// @return Its text.
std::string json_string(const std::string& value) {
  std::string out = "\"";
  for (const char c : value) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\t':
        out += "\\t";
        break;
      case '\r':
        out += "\\r";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          constexpr std::string_view kHex = "0123456789abcdef";
          const auto code = static_cast<unsigned char>(c);
          out += "\\u00";
          out += kHex[code >> 4U];
          out += kHex[code & 0xFU];
        } else {
          out += c;
        }
    }
  }
  return out + "\"";
}

/// @brief A JSON value's text.
/// @param value The value.
/// @return Its text.
std::string json_value(const JsonValue& value) {
  if (const auto* integer = std::get_if<std::int64_t>(&value)) {
    return std::to_string(*integer);
  }
  if (const auto* number = std::get_if<double>(&value)) {
    return json_number(*number);
  }
  if (const auto* flag = std::get_if<bool>(&value)) {
    return *flag ? "true" : "false";
  }
  return json_string(std::get<std::string>(value));
}

/// @brief A flat object as JSON.
/// @param object The object.
/// @return Its text.
std::string json_object(const JsonObject& object) {
  std::string out = "{";
  for (std::size_t k = 0; k < object.size(); ++k) {
    out += (k > 0 ? ", " : "") + json_string(object[k].first) + ": " + json_value(object[k].second);
  }
  return out + "}";
}

/// @brief Whether an object's numbers are all finite.
/// @param object The object.
/// @return Whether they are.
bool finite(const JsonObject& object) {
  return std::ranges::all_of(object, [](const auto& item) {
    const auto* number = std::get_if<double>(&item.second);
    return number == nullptr || std::isfinite(*number);
  });
}

/// @brief Check stamped poses: finite, strictly ascending times, one rigid transform each.
/// @param times The times.
/// @param poses The poses.
/// @param name Their name, for messages.
/// @return Nothing, or `InvalidInput`.
Result<void> check_stamped(const std::vector<double>& times,
                           const std::vector<Eigen::Matrix4d>& poses, const char* name) {
  if (times.size() != poses.size()) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": expected (F,) timestamps and (F, 4, 4) poses");
  }
  for (std::size_t k = 0; k < times.size(); ++k) {
    if (!std::isfinite(times[k]) || (k > 0 && !(times[k] > times[k - 1]))) {
      return fail(ErrorKind::kInvalidInput,
                  std::string(name) + " timestamps: must be finite and ascend strictly");
    }
    if (auto ok = foundations::se3::check_transform(poses[k]); !ok) {
      return ok;
    }
  }
  return {};
}

/// @brief Check a run before anything is written.
/// @param run The run.
/// @return Nothing, or `InvalidInput`.
Result<void> check(const Run& run) {
  if (std::ranges::find(kModes, run.mode) == kModes.end()) {
    return fail(ErrorKind::kInvalidInput, "mode: unknown '" + run.mode + "'");
  }
  if (std::ranges::find(kBackends, run.backend) == kBackends.end()) {
    return fail(ErrorKind::kInvalidInput, "backend: unknown '" + run.backend + "'");
  }
  if (auto ok = check_stamped(run.timestamps, run.poses, "trajectory"); !ok) {
    return ok;
  }
  if (auto ok = check_stamped(run.keyframe_timestamps, run.keyframe_poses, "keyframes"); !ok) {
    return ok;
  }
  if (!std::ranges::all_of(run.points, [](const Eigen::Vector3d& p) { return p.allFinite(); })) {
    return fail(ErrorKind::kInvalidInput, "points: expected finite values");
  }
  if (!finite(run.settings)) {
    return fail(ErrorKind::kInvalidInput, "settings: not JSON (a number is not finite)");
  }
  if (run.peak_memory.value_or(0) < 0) {
    return fail(ErrorKind::kInvalidInput, "peak_memory: expected a whole number of bytes");
  }
  if (run.cpus &&
      (run.cpus->empty() || run.cpus->front() < 0 ||
       std::ranges::adjacent_find(*run.cpus, std::greater_equal<>()) != run.cpus->end())) {
    return fail(ErrorKind::kInvalidInput, "cpus: expected CPU numbers, ascending");
  }
  if (run.memory_cap.value_or(1) < 1) {
    return fail(ErrorKind::kInvalidInput, "memory_cap: expected a positive number of bytes");
  }
  for (std::size_t k = 0; k < run.diagnostics.size(); ++k) {
    const auto& entry = run.diagnostics[k];
    const bool stamped =
        std::ranges::any_of(entry, [](const auto& item) { return item.first == "timestamp"; });
    if (!stamped || !finite(entry)) {
      return fail(ErrorKind::kInvalidInput, "diagnostics[" + std::to_string(k) +
                                                "]: expected an object with a timestamp, its "
                                                "numbers finite");
    }
  }
  return {};
}

/// @brief Stamped poses as TUM-format lines, each number round-tripping.
/// @param times The times.
/// @param poses The poses.
/// @return The text, or the quaternion conversion's error.
Result<std::string> trajectory_text(const std::vector<double>& times,
                                    const std::vector<Eigen::Matrix4d>& poses) {
  std::string out = "# timestamp tx ty tz qx qy qz qw\n";
  for (std::size_t k = 0; k < times.size(); ++k) {
    auto q = foundations::so3::to_quaternion(poses[k].topLeftCorner<3, 3>());
    if (!q) {
      return std::unexpected(std::move(q.error()));
    }
    const std::array<double, 8> values = {times[k], poses[k](0, 3), poses[k](1, 3), poses[k](2, 3),
                                          (*q)(0),  (*q)(1),        (*q)(2),        (*q)(3)};
    for (std::size_t i = 0; i < values.size(); ++i) {
      out += (i > 0 ? " " : "") + json_number(values[i]);
    }
    out += '\n';
  }
  return out;
}

/// @brief Poses' values, row-major.
/// @param poses The poses.
/// @return 16 values each.
std::vector<double> flatten(const std::vector<Eigen::Matrix4d>& poses) {
  std::vector<double> out;
  out.reserve(16 * poses.size());
  for (const Eigen::Matrix4d& pose : poses) {
    for (Eigen::Index r = 0; r < 4; ++r) {
      for (Eigen::Index c = 0; c < 4; ++c) {
        out.push_back(pose(r, c));
      }
    }
  }
  return out;
}

/// @brief Write a file whole: to a temporary sibling, then renamed over it.
/// @param path The file.
/// @param data Its bytes.
/// @return Nothing, or `InvalidInput` naming the file.
Result<void> replace(const fs::path& path, const std::string& data) {
  const fs::path temporary = path.parent_path() / ("." + path.filename().string() + ".tmp");
  {
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    file.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!file) {
      return fail(ErrorKind::kInvalidInput, temporary.string() + ": could not be written");
    }
  }
  std::error_code error;
  fs::rename(temporary, path, error);
  if (error) {
    return fail(ErrorKind::kInvalidInput, path.string() + ": " + error.message());
  }
  return {};
}

}  // namespace

std::string json_number(double value) {
  std::array<char, 32> buffer{};
  const auto [end, error] = std::to_chars(buffer.data(), std::to_address(buffer.end()), value);
  std::string out(buffer.data(), end);
  if (out.find_first_of(".en") == std::string::npos) {
    out += ".0";
  }
  return out;
}

std::string npy(const std::vector<std::size_t>& shape, const std::vector<double>& values) {
  std::string dims;
  for (std::size_t k = 0; k < shape.size(); ++k) {
    dims += (k > 0 ? ", " : "") + std::to_string(shape[k]);
  }
  if (shape.size() == 1) {
    dims += ',';
  }
  std::string header = "{'descr': '<f8', 'fortran_order': False, 'shape': (" + dims + "), }";
  if (!shape.empty()) {
    header.append(kGrowthDigits - std::to_string(shape[0]).size(), ' ');
  }
  constexpr std::size_t kPrefix = 10;  // magic, version, header length
  header.append(kAlign - ((kPrefix + header.size() + 1) % kAlign), ' ');
  header += '\n';
  std::string out = "\x93NUMPY";
  out += '\x01';
  out += '\x00';
  out += static_cast<char>(header.size() & 0xFFU);
  out += static_cast<char>((header.size() >> 8U) & 0xFFU);
  out += header;
  static_assert(std::endian::native == std::endian::little, "npy is written little-endian");
  out.reserve(out.size() + (values.size() * sizeof(double)));
  for (const double value : values) {
    const auto raw = std::bit_cast<std::array<char, sizeof(double)>>(value);
    out.append(raw.data(), raw.size());
  }
  return out;
}

Result<void> write_run(const std::string& folder, const Run& run) {
  if (auto ok = check(run); !ok) {
    return ok;
  }
  auto trajectory = trajectory_text(run.timestamps, run.poses);
  if (!trajectory) {
    return std::unexpected(std::move(trajectory.error()));
  }
  const JsonObject info = {
      {"format", std::string(kFormat)}, {"version", kVersion}, {"dataset", run.dataset},
      {"sequence", run.sequence},       {"mode", run.mode},    {"backend", run.backend}};
  std::string info_text = json_object(info);
  info_text.pop_back();
  info_text += ", \"settings\": " + json_object(run.settings);
  const auto integer_or_null = [](const std::optional<std::int64_t>& value) {
    return value ? std::to_string(*value) : std::string("null");
  };
  info_text += ", \"peak_memory\": " + integer_or_null(run.peak_memory);
  info_text += ", \"cpus\": ";
  if (run.cpus) {
    info_text += '[';
    for (std::size_t k = 0; k < run.cpus->size(); ++k) {
      info_text += (k == 0 ? "" : ", ") + std::to_string((*run.cpus)[k]);
    }
    info_text += ']';
  } else {
    info_text += "null";
  }
  info_text += ", \"memory_cap\": " + integer_or_null(run.memory_cap) + "}\n";
  std::string diagnostics;
  for (const JsonObject& entry : run.diagnostics) {
    diagnostics += json_object(entry) + "\n";
  }
  std::vector<double> points;
  points.reserve(3 * run.points.size());
  for (const Eigen::Vector3d& p : run.points) {
    points.insert(points.end(), {p.x(), p.y(), p.z()});
  }
  const fs::path out(folder);
  std::error_code error;
  fs::create_directories(out, error);
  if (error) {
    return fail(ErrorKind::kInvalidInput, folder + ": " + error.message());
  }
  const std::array<std::pair<const char*, std::string>, 6> files = {{
      {"run.json", info_text},
      {"trajectory.txt", *trajectory},
      {"keyframes.npy", npy({run.keyframe_timestamps.size()}, run.keyframe_timestamps)},
      {"keyframe_poses.npy", npy({run.keyframe_poses.size(), 4, 4}, flatten(run.keyframe_poses))},
      {"points.npy", npy({run.points.size(), 3}, points)},
      {"diagnostics.jsonl", diagnostics},
  }};
  for (const auto& [name, data] : files) {
    if (auto ok = replace(out / name, data); !ok) {
      return ok;
    }
  }
  return {};
}

}  // namespace slamlab::eval::runs
