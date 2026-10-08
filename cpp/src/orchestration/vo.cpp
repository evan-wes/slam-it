/// @file
/// @brief A whole sequence through the edge pipeline (slamlab/orchestration/vo.hpp).
#include "slamlab/orchestration/vo.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "slamlab/edge/frames.hpp"
#include "slamlab/edge/map.hpp"
#include "slamlab/edge/options.hpp"
#include "slamlab/edge/pipeline.hpp"
#include "slamlab/eval/euroc.hpp"
#include "slamlab/eval/png.hpp"
#include "slamlab/eval/runs.hpp"
#include "slamlab/eval/text.hpp"
#include "slamlab/eval/tum.hpp"
#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/image.hpp"
#include "slamlab/foundations/orb.hpp"
#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/stereo.hpp"

namespace slamlab::orchestration::vo {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
using foundations::camera::Model;

using foundations::image::Image;

/// @brief A stereo pair, rectified.
struct StereoImages {
  /// @brief The left image.
  Image left;
  /// @brief The right image.
  Image right;
};

/// @brief An RGB-D frame's image and its depth map.
struct RgbdImages {
  /// @brief The image.
  Image image;
  /// @brief The depth map, in metres.
  edge::DepthMap depth;
};

/// @brief A frame's images as a camera delivers them: decoded, and a stereo pair rectified
///        (`slamlab.orchestration.vo._Delivered`).
struct Delivered {
  /// @brief The frame's time.
  double timestamp{0.0};
  /// @brief A monocular frame's image, a stereo pair, or an RGB-D frame's image and depth.
  std::variant<Image, StereoImages, RgbdImages> images;
};

/// @brief Delivered images, or nothing for a frame the run skips.
using Read = Result<std::optional<Delivered>>;

/// @brief Delivers the images at an index.
using Reader = std::function<Read(std::size_t)>;

/// @brief A sequence opened for a run.
struct Source {
  /// @brief The camera the pipeline runs with.
  edge::Camera camera;
  /// @brief The sequence's frames.
  std::size_t frames{0};
  /// @brief Reads one.
  Reader read;
};

/// @brief Build a frame from its images, for the mode they were delivered for.
/// @param delivered The images.
/// @param camera The camera.
/// @param extraction The extraction settings the pipeline asks for.
/// @param matching The stereo matcher's settings.
/// @return The frame; the frame builder's error.
Result<edge::Frame> build(const Delivered& delivered, const edge::Camera& camera,
                          const foundations::orb::Options& extraction,
                          const foundations::stereo::MatchOptions& matching) {
  return std::visit(
      [&](const auto& images) -> Result<edge::Frame> {
        using Images = std::decay_t<decltype(images)>;
        if constexpr (std::is_same_v<Images, StereoImages>) {
          return edge::stereo_pair(images.left, images.right, delivered.timestamp, camera,
                                   extraction, matching);
        } else if constexpr (std::is_same_v<Images, RgbdImages>) {
          return edge::rgbd(images.image, images.depth, delivered.timestamp, camera, extraction);
        } else {
          return edge::monocular(images, delivered.timestamp, camera, extraction);
        }
      },
      delivered.images);
}

/// @brief Open a TUM-format folder: its camera, an RGB-D one with the virtual baseline.
/// @param root The folder.
/// @param mode Monocular or RGB-D.
/// @return The source; the reader's error.
Result<Source> tum_source(const std::string& root, edge::Mode mode) {
  auto read = eval::tum::read_frames(root);
  if (!read) {
    return std::unexpected(read.error());
  }
  const eval::tum::Camera& sensor = read->camera;
  double baseline = 0.0;
  if (mode == edge::Mode::kRgbd) {
    if (!sensor.bf) {
      return fail(ErrorKind::kInvalidInput,
                  root + ": an RGB-D run needs the depth sensor's bf: add a 'bf' line to " +
                      eval::tum::kCameraFile);
    }
    baseline = *sensor.bf / sensor.intrinsics(0);
  }
  const edge::Camera camera{.model = sensor.model,
                            .intrinsics = sensor.intrinsics,
                            .width = sensor.width,
                            .height = sensor.height,
                            .baseline = baseline};
  const std::size_t count = read->images.size();
  Reader deliver = [frames = std::move(*read), mode](std::size_t i) -> Read {
    auto image = eval::png::read_gray(frames.images.at(i));
    if (!image) {
      return std::unexpected(image.error());
    }
    const double timestamp = frames.timestamps.at(i);
    if (mode == edge::Mode::kMonocular) {
      return Delivered{.timestamp = timestamp, .images = std::move(*image)};
    }
    const std::optional<std::string>& depth_file = frames.depths.at(i);
    if (!depth_file.has_value()) {
      return std::optional<Delivered>();
    }
    auto depth = eval::tum::read_depth(*depth_file, frames.camera.depth_scale);
    if (!depth) {
      return std::unexpected(depth.error());
    }
    edge::DepthMap depth_map{
        .rows = depth->rows, .cols = depth->cols, .values = std::move(depth->values)};
    return Delivered{
        .timestamp = timestamp,
        .images = RgbdImages{.image = std::move(*image), .depth = std::move(depth_map)}};
  };
  return Source{.camera = camera, .frames = count, .read = std::move(deliver)};
}

/// @brief Read an image and rectify it.
/// @param path The file.
/// @param coordinates Its rectification map.
/// @param camera The rectified camera.
/// @return The rectified image; the reader's or the remap's error.
Result<foundations::image::Image> rectified(const std::string& path,
                                            const foundations::image::Coordinates& coordinates,
                                            const edge::Camera& camera) {
  const auto image = eval::png::read_gray(path);
  if (!image) {
    return std::unexpected(image.error());
  }
  return foundations::image::remap(*image, coordinates, camera.height, camera.width);
}

/// @brief Open a EuRoC sequence's stereo pairs, rectified: the rectified left camera
///        (pinhole) with the pair's baseline.
/// @param frames The sequence's frames.
/// @return The source; the rectification's error.
Result<Source> stereo_source(eval::euroc::Frames frames) {
  const eval::euroc::Camera& left = frames.left_camera;
  const eval::euroc::Camera& right = frames.right_camera;
  const auto pose = eval::euroc::right_from_left(left, right);
  if (!pose) {
    return std::unexpected(pose.error());
  }
  const auto rectification =
      foundations::stereo::rectify(Model::kRadTan, left.intrinsics, Model::kRadTan,
                                   right.intrinsics, *pose, left.rows, left.cols);
  if (!rectification) {
    return std::unexpected(rectification.error());
  }
  auto map_left = foundations::stereo::rectification_map(
      Model::kRadTan, left.intrinsics, rectification->rotation_left, rectification->intrinsics,
      left.rows, left.cols);
  auto map_right = foundations::stereo::rectification_map(
      Model::kRadTan, right.intrinsics, rectification->rotation_right, rectification->intrinsics,
      left.rows, left.cols);
  if (!map_left || !map_right) {
    return std::unexpected(map_left ? map_right.error() : map_left.error());
  }
  const edge::Camera camera{.model = Model::kPinhole,
                            .intrinsics = rectification->intrinsics,
                            .width = left.cols,
                            .height = left.rows,
                            .baseline = rectification->baseline};
  const std::size_t count = frames.left.size();
  Reader deliver = [frames = std::move(frames), maps = std::array{*map_left, *map_right},
                    camera](std::size_t i) -> Read {
    auto image_left = rectified(frames.left.at(i), maps[0], camera);
    if (!image_left) {
      return std::unexpected(image_left.error());
    }
    auto image_right = rectified(frames.right.at(i), maps[1], camera);
    if (!image_right) {
      return std::unexpected(image_right.error());
    }
    return Delivered{
        .timestamp = frames.timestamps.at(i),
        .images = StereoImages{.left = std::move(*image_left), .right = std::move(*image_right)}};
  };
  return Source{.camera = camera, .frames = count, .read = std::move(deliver)};
}

/// @brief Open a EuRoC sequence: its left camera for monocular, the pairs rectified for
///        stereo.
/// @param root The sequence folder.
/// @param mode Monocular or stereo.
/// @return The source; the reader's error.
Result<Source> euroc_source(const std::string& root, edge::Mode mode) {
  auto read = eval::euroc::read_frames(root);
  if (!read) {
    return std::unexpected(read.error());
  }
  if (mode == edge::Mode::kStereo) {
    return stereo_source(std::move(*read));
  }
  const eval::euroc::Camera& left = read->left_camera;
  const edge::Camera camera{.model = Model::kRadTan,
                            .intrinsics = left.intrinsics,
                            .width = left.cols,
                            .height = left.rows,
                            .baseline = 0.0};
  const std::size_t count = read->left.size();
  Reader deliver = [frames = std::move(*read)](std::size_t i) -> Read {
    auto image = eval::png::read_gray(frames.left.at(i));
    if (!image) {
      return std::unexpected(image.error());
    }
    return Delivered{.timestamp = frames.timestamps.at(i), .images = std::move(*image)};
  };
  return Source{.camera = camera, .frames = count, .read = std::move(deliver)};
}

/// @brief The settings as a run records them, by their Python names.
/// @param options The settings.
/// @return Each one's name and value.
eval::runs::JsonObject settings(const edge::Options& options) {
  eval::runs::JsonObject out;
  for (const auto& [name, member] : edge::kIntegerOptions) {
    out.emplace_back(name, options.*member);
  }
  for (const auto& [name, member] : edge::kRealOptions) {
    out.emplace_back(name, options.*member);
  }
  return out;
}

/// @brief A sequence's name: its folder's, the path resolved.
/// @param sequence The folder.
/// @return The name.
std::string sequence_name(const std::string& sequence) {
  std::error_code error;
  const std::filesystem::path resolved = std::filesystem::weakly_canonical(sequence, error);
  return (error ? std::filesystem::path(sequence) : resolved).filename().string();
}

/// @brief Record a frame's result: its diagnostics, and its pose if it was tracked.
/// @param out The run.
/// @param result The result.
/// @param read The time delivering its images took, in seconds.
/// @param build The time building the frame took, in seconds.
/// @return Nothing, or the pose's inversion's error.
Result<void> record(eval::runs::Run& out, const edge::FrameResult& result, double read,
                    double build) {
  out.diagnostics.push_back({{"timestamp", result.timestamp},
                             {"state", std::string(edge::name(result.state))},
                             {"tracked", result.tracked},
                             {"keyframe", result.keyframe},
                             {"read_seconds", read},
                             {"build_seconds", build},
                             {"track_seconds", result.track_seconds},
                             {"map_seconds", result.map_seconds}});
  if (!result.pose.has_value()) {
    return {};
  }
  const auto pose = foundations::se3::inverse(*result.pose);
  if (!pose) {
    return std::unexpected(pose.error());
  }
  out.timestamps.push_back(result.timestamp);
  out.poses.push_back(*pose);
  return {};
}

/// @brief Record the map: its keyframes' times and poses, and its points.
/// @param out The run.
/// @param m The map.
/// @return Nothing, or a pose's inversion's error.
Result<void> record_map(eval::runs::Run& out, const edge::Map& m) {
  out.keyframe_timestamps = m.kf_time;
  for (const Eigen::Matrix4d& T_cam_world : m.kf_pose) {  // NOLINT(readability-identifier-naming)
    const auto pose = foundations::se3::inverse(T_cam_world);
    if (!pose) {
      return std::unexpected(pose.error());
    }
    out.keyframe_poses.push_back(*pose);
  }
  out.points = m.pt_xyz;
  return {};
}

/// @brief Track a sequence's frames, from the first asked for, until the end or the track
///        is lost.
/// @param source The sequence.
/// @param pipeline The pipeline.
/// @param request The request.
/// @param out The run, each frame recorded.
/// @return Nothing; a reader's, a frame builder's or the pipeline's error.
Result<void> track(const Source& source, edge::Pipeline& pipeline, const Request& request,
                   eval::runs::Run& out) {
  using Clock = std::chrono::steady_clock;
  using Seconds = std::chrono::duration<double>;
  const auto first = static_cast<std::size_t>(request.first);
  const std::size_t end =
      request.count.has_value()
          ? std::min(source.frames, first + static_cast<std::size_t>(*request.count))
          : source.frames;
  const foundations::stereo::MatchOptions matching = edge::stereo_matching(request.options);
  for (std::size_t i = first; i < end; ++i) {
    const auto start = Clock::now();
    const Read delivered = source.read(i);
    const auto read = Clock::now();
    if (!delivered) {
      return std::unexpected(delivered.error());
    }
    if (!delivered->has_value()) {
      continue;
    }
    const auto frame = build(**delivered, source.camera, pipeline.extraction(), matching);
    const auto built = Clock::now();
    if (!frame) {
      return std::unexpected(frame.error());
    }
    const auto result = pipeline.track(*frame);
    if (!result) {
      return std::unexpected(result.error());
    }
    if (const auto recorded =
            record(out, *result, Seconds(read - start).count(), Seconds(built - read).count());
        !recorded) {
      return recorded;
    }
    if (result->state == edge::State::kLost) {
      break;
    }
  }
  return {};
}

/// @brief Read a mode's name.
/// @param text The name, as `slamlab.edge.options.MODES` spells it.
/// @return The mode, or `InvalidInput`.
Result<edge::Mode> mode_named(std::string_view text) {
  for (std::uint8_t code = 0; code < static_cast<std::uint8_t>(edge::Mode::kCount); ++code) {
    const auto mode = static_cast<edge::Mode>(code);
    if (text == edge::name(mode)) {
      return mode;
    }
  }
  return fail(ErrorKind::kInvalidInput,
              "--mode: expected monocular, stereo or rgbd, got '" + std::string(text) + "'");
}

/// @brief Read a whole integer.
/// @param flag The flag it was given with, for the message.
/// @param text The text.
/// @return The integer, or `InvalidInput`.
Result<std::int64_t> integer_of(std::string_view flag, std::string_view text) {
  if (const auto value = eval::text::integer(text); value.has_value()) {
    return *value;
  }
  return fail(ErrorKind::kInvalidInput,
              std::string(flag) + ": expected an integer, got '" + std::string(text) + "'");
}

/// @brief Read one `--option KEY=VALUE` into the settings.
/// @param text `KEY=VALUE`.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` for an unknown key or a value of the wrong kind.
Result<void> read_option(std::string_view text, edge::Options& options) {
  const std::size_t equals = text.find('=');
  if (equals == std::string_view::npos) {
    return fail(ErrorKind::kInvalidInput,
                "--option: expected KEY=VALUE, got '" + std::string(text) + "'");
  }
  const std::string_view key = text.substr(0, equals);
  const std::string_view value = text.substr(equals + 1);
  for (const auto& [name, member] : edge::kIntegerOptions) {
    if (key == name) {
      const auto integer = integer_of(key, value);
      if (!integer) {
        return std::unexpected(integer.error());
      }
      options.*member = *integer;
      return {};
    }
  }
  for (const auto& [name, member] : edge::kRealOptions) {
    if (key == name) {
      const auto number = eval::text::number(value);
      if (!number.has_value() || !std::isfinite(*number)) {
        return fail(
            ErrorKind::kInvalidInput,
            std::string(key) + ": expected a finite number, got '" + std::string(value) + "'");
      }
      options.*member = *number;
      return {};
    }
  }
  return fail(ErrorKind::kInvalidInput, "--option: no setting '" + std::string(key) + "'");
}

/// @brief Read one flag's value into the request.
/// @param out The request.
/// @param flag The flag.
/// @param value Its value.
/// @return Nothing, or `InvalidInput` for an unknown flag or a bad value.
Result<void> read_flag(Request& out, std::string_view flag, const std::string& value) {
  if (flag == "--dataset") {
    out.dataset = value;
  } else if (flag == "--sequence") {
    out.sequence = value;
  } else if (flag == "--out") {
    out.out = value;
  } else if (flag == "--mode") {
    const auto mode = mode_named(value);
    if (!mode) {
      return std::unexpected(mode.error());
    }
    out.mode = *mode;
  } else if (flag == "--first" || flag == "--count") {
    const auto integer = integer_of(flag, value);
    if (!integer) {
      return std::unexpected(integer.error());
    }
    if (flag == "--first") {
      out.first = *integer;
    } else {
      out.count = *integer;
    }
  } else if (flag == "--option") {
    return read_option(value, out.options);
  } else {
    return fail(ErrorKind::kInvalidInput, "unknown argument '" + std::string(flag) + "'");
  }
  return {};
}

}  // namespace

Result<Request> parse(std::span<const std::string> arguments) {
  Request out;
  std::vector<std::string_view> seen;
  for (std::size_t i = 0; i < arguments.size(); i += 2) {
    const std::string& flag = arguments[i];
    if (i + 1 == arguments.size()) {
      return fail(ErrorKind::kInvalidInput, flag + ": expected a value");
    }
    if (flag != "--option" && std::ranges::find(seen, flag) != seen.end()) {
      return fail(ErrorKind::kInvalidInput, flag + ": given twice");
    }
    seen.emplace_back(flag);
    if (const auto read = read_flag(out, flag, arguments[i + 1]); !read) {
      return std::unexpected(read.error());
    }
  }
  for (const std::string_view required : {"--dataset", "--sequence", "--mode", "--out"}) {
    if (std::ranges::find(seen, required) == seen.end()) {
      return fail(ErrorKind::kInvalidInput, std::string(required) + ": missing");
    }
  }
  return out;
}

Result<void> check(const Request& request) {
  if (std::ranges::find(kDatasets, request.dataset) == kDatasets.end()) {
    return fail(ErrorKind::kInvalidInput,
                "dataset: expected tum or euroc, got '" + request.dataset + "'");
  }
  if (request.dataset == "tum" && request.mode == edge::Mode::kStereo) {
    return fail(ErrorKind::kInvalidInput, "mode: a tum sequence has no stereo pairs");
  }
  if (request.dataset == "euroc" && request.mode == edge::Mode::kRgbd) {
    return fail(ErrorKind::kInvalidInput, "mode: a euroc sequence has no depth maps");
  }
  if (request.first < 0 || request.count.value_or(0) < 0) {
    return fail(ErrorKind::kInvalidInput, "first, count: must be at least 0");
  }
  return edge::check_options(request.options);
}

Result<eval::runs::Run> run(const Request& request) {
  if (const auto checked = check(request); !checked) {
    return std::unexpected(checked.error());
  }
  auto source = request.dataset == "tum" ? tum_source(request.sequence, request.mode)
                                         : euroc_source(request.sequence, request.mode);
  if (!source) {
    return std::unexpected(source.error());
  }
  auto pipeline = edge::Pipeline::create(request.mode, source->camera, request.options);
  if (!pipeline) {
    return std::unexpected(pipeline.error());
  }
  eval::runs::Run out;
  out.dataset = request.dataset;
  out.sequence = sequence_name(request.sequence);
  out.mode = edge::name(request.mode);
  out.backend = "cpp";
  out.settings = settings(request.options);
  if (const auto tracked = track(*source, *pipeline, request, out); !tracked) {
    return std::unexpected(tracked.error());
  }
  if (const std::optional<edge::Map>& m = pipeline->map(); m.has_value()) {
    if (const auto recorded = record_map(out, *m); !recorded) {
      return std::unexpected(recorded.error());
    }
  }
  if (const auto written = eval::runs::write_run(request.out, out); !written) {
    return std::unexpected(written.error());
  }
  return out;
}

}  // namespace slamlab::orchestration::vo
