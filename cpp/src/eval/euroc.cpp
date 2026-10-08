/// @file
/// @brief Implementation of the EuRoC reader declared in slamlab/eval/euroc.hpp.
///
/// `sensor.yaml` is read as the Python reader's regular expressions read it, by hand: comments
/// (`#` to the line's end) dropped, a setting as the first token after `key:` at a line's
/// start, a list as the comma-separated numbers between `key: [` and the next `]`, and
/// `T_BS` as the `data:` list of the text from `T_BS:` to its first `]`.
#include "slamlab/eval/euroc.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <utility>

#include "slamlab/eval/text.hpp"
#include "slamlab/foundations/image.hpp"
#include "slamlab/foundations/se3.hpp"

namespace slamlab::eval::euroc {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
namespace fs = std::filesystem;

/// @brief Where `key:` starts a line (after its indentation), if anywhere.
/// @param text The text.
/// @param key The key.
/// @return The offset just past the colon.
std::optional<std::size_t> after_key(std::string_view text, std::string_view key) {
  std::size_t line = 0;
  while (line <= text.size()) {
    std::size_t at = line;
    while (at < text.size() && (text[at] == ' ' || text[at] == '\t')) {
      ++at;
    }
    if (text.substr(at).starts_with(key) && text.substr(at + key.size()).starts_with(':')) {
      return at + key.size() + 1;
    }
    const std::size_t next = text.find('\n', line);
    if (next == std::string_view::npos) {
      break;
    }
    line = next + 1;
  }
  return std::nullopt;
}

/// @brief A `key: value` setting's value.
/// @param text The text.
/// @param key The key.
/// @return The first token after the colon, if the key is there.
std::optional<std::string> setting(std::string_view text, std::string_view key) {
  const auto at = after_key(text, key);
  if (!at) {
    return std::nullopt;
  }
  std::istringstream rest{std::string(text.substr(*at))};
  std::string token;
  rest >> token;
  return token.empty() ? std::nullopt : std::optional<std::string>(token);
}

/// @brief A `key: [a, b, ...]` list's numbers.
/// @param text The text.
/// @param key The key.
/// @return The numbers; nothing if the key, the brackets or a number are missing.
std::optional<std::vector<double>> numbers(std::string_view text, std::string_view key) {
  const auto at = after_key(text, key);
  if (!at) {
    return std::nullopt;
  }
  const std::string_view rest = text.substr(*at);
  const std::size_t open = rest.find_first_not_of(" \t\n");
  const std::size_t close = rest.find(']');
  if (open == std::string_view::npos || rest[open] != '[' || close == std::string_view::npos) {
    return std::nullopt;
  }
  std::vector<double> out;
  std::string_view items = rest.substr(open + 1, close - open - 1);
  while (!items.empty()) {
    const std::size_t comma = items.find(',');
    const std::string_view item = text::trim(items.substr(0, comma));
    if (!item.empty()) {
      const auto value = text::number(item);
      if (!value) {
        return std::nullopt;
      }
      out.push_back(*value);
    }
    items = comma == std::string_view::npos ? std::string_view() : items.substr(comma + 1);
  }
  return out;
}

/// @brief `T_BS`'s 16 numbers: the `data:` list of the text from `T_BS:` to its first `]`.
/// @param text The text.
/// @return The numbers, if there.
std::optional<std::vector<double>> body_pose(std::string_view text) {
  const std::size_t start = text.find("T_BS:");
  if (start == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t close = text.find(']', start);
  if (close == std::string_view::npos) {
    return std::nullopt;
  }
  return numbers(text.substr(start + 5, close - start - 4), "data");
}

/// @brief Check a camera's fields and build it.
/// @param path The file, for messages.
/// @param text Its uncommented text.
/// @return The camera, or `InvalidInput`.
Result<Camera> camera_from(const std::string& path, std::string_view text) {
  if (setting(text, "camera_model") != "pinhole") {
    return fail(ErrorKind::kInvalidInput, path + ": expected camera_model: pinhole");
  }
  if (setting(text, "distortion_model") != "radial-tangential") {
    return fail(ErrorKind::kInvalidInput, path + ": expected distortion_model: radial-tangential");
  }
  const auto pinhole = numbers(text, "intrinsics");
  const auto distortion = numbers(text, "distortion_coefficients");
  const auto resolution = numbers(text, "resolution");
  const auto pose = body_pose(text);
  if (!pinhole || !distortion || !resolution || !pose || pinhole->size() != 4 ||
      distortion->size() != 4 || resolution->size() != 2 || pose->size() != 16) {
    return fail(ErrorKind::kInvalidInput,
                path +
                    ": expected 4 intrinsics, 4 distortion coefficients, a 2-value "
                    "resolution and a 16-value T_BS");
  }
  bool finite = true;
  for (const auto* values : {&*pinhole, &*distortion, &*resolution, &*pose}) {
    for (const double v : *values) {
      finite = finite && std::isfinite(v);
    }
  }
  if (!finite) {
    return fail(ErrorKind::kInvalidInput, path + ": expected finite numbers");
  }
  const auto side = [](double v) {
    return v == std::floor(v) && v >= 1.0 && v <= static_cast<double>(foundations::image::kMaxSide);
  };
  if (!side((*resolution)[0]) || !side((*resolution)[1])) {
    return fail(ErrorKind::kInvalidInput, path +
                                              ": expected a resolution of two integers from 1 to " +
                                              std::to_string(foundations::image::kMaxSide));
  }
  Camera out{
      .intrinsics = Eigen::VectorXd::Zero(9),
      .T_body_cam = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(pose->data()),
      .rows = static_cast<std::int64_t>((*resolution)[1]),
      .cols = static_cast<std::int64_t>((*resolution)[0])};
  for (std::size_t k = 0; k < 4; ++k) {
    out.intrinsics(static_cast<Eigen::Index>(k)) = (*pinhole)[k];
    out.intrinsics(static_cast<Eigen::Index>(k + 4)) = (*distortion)[k];
  }
  if (auto ok = foundations::se3::check_transform(out.T_body_cam); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return out;
}

/// @brief A camera folder's frames, by timestamp in nanoseconds (`_frames`).
/// @param folder The camera's folder.
/// @return Each stamp's image; `InvalidInput` for a malformed row or a repeated stamp.
Result<std::map<std::int64_t, std::string>> camera_frames(const fs::path& folder) {
  const std::string path = (folder / "data.csv").string();
  auto lines = text::read_lines(path);
  if (!lines) {
    return std::unexpected(std::move(lines.error()));
  }
  std::map<std::int64_t, std::string> out;
  for (std::size_t k = 0; k < lines->size(); ++k) {
    const std::string& line = (*lines)[k];
    if (line.empty() || text::trim(line).starts_with('#')) {
      continue;
    }
    const std::size_t comma = line.find(',');
    const std::string_view first = text::trim(std::string_view(line).substr(0, comma));
    const bool digits =
        !first.empty() && std::ranges::all_of(first, [](char c) { return c >= '0' && c <= '9'; });
    const auto stamp = digits ? text::integer(first) : std::nullopt;
    if (comma == std::string::npos || !stamp) {
      return fail(ErrorKind::kInvalidInput,
                  text::where(path, k + 1) + "expected `timestamp,filename`");
    }
    const std::string_view rest = std::string_view(line).substr(comma + 1);
    const std::string name(text::trim(rest.substr(0, rest.find(','))));
    if (!out.emplace(*stamp, (folder / "data" / name).string()).second) {
      return fail(ErrorKind::kInvalidInput,
                  text::where(path, k + 1) + "timestamp " + std::to_string(*stamp) + " repeats");
    }
  }
  return out;
}

}  // namespace

Result<Camera> read_camera(const std::string& path) {
  return text::read_uncommented_text(path).and_then(
      [&path](const std::string& body) { return camera_from(path, body); });
}

Result<Frames> read_frames(const std::string& root) {
  const fs::path mav = fs::path(root) / "mav0";
  auto left_camera = read_camera((mav / "cam0" / "sensor.yaml").string());
  if (!left_camera) {
    return std::unexpected(std::move(left_camera.error()));
  }
  auto right_camera = read_camera((mav / "cam1" / "sensor.yaml").string());
  if (!right_camera) {
    return std::unexpected(std::move(right_camera.error()));
  }
  auto left = camera_frames(mav / "cam0");
  if (!left) {
    return std::unexpected(std::move(left.error()));
  }
  auto right = camera_frames(mav / "cam1");
  if (!right) {
    return std::unexpected(std::move(right.error()));
  }
  Frames out{.root = root,
             .timestamps = {},
             .left = {},
             .right = {},
             .left_camera = std::move(*left_camera),
             .right_camera = std::move(*right_camera)};
  for (const auto& [stamp, image] : *left) {
    if (const auto it = right->find(stamp); it != right->end()) {
      out.timestamps.push_back(static_cast<double>(stamp) / kNanoseconds);
      out.left.push_back(image);
      out.right.push_back(it->second);
    }
  }
  if (out.timestamps.empty()) {
    return fail(ErrorKind::kInvalidInput, root + ": no frame has both a left and a right image");
  }
  return out;
}

Result<Eigen::Matrix4d> right_from_left(const Camera& left, const Camera& right) {
  auto inverse = foundations::se3::inverse(right.T_body_cam);
  if (!inverse) {
    return std::unexpected(std::move(inverse.error()));
  }
  return Eigen::Matrix4d(*inverse * left.T_body_cam);
}

}  // namespace slamlab::eval::euroc
