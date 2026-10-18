/// @file
/// @brief A whole sequence through the edge pipeline (slam_it/orchestration/vo.hpp).
#include "slam_it/orchestration/vo.hpp"

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
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/loops.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/edge/pipeline.hpp"
#include "slam_it/eval/euroc.hpp"
#include "slam_it/eval/png.hpp"
#include "slam_it/eval/runs.hpp"
#include "slam_it/eval/text.hpp"
#include "slam_it/eval/tum.hpp"
#include "slam_it/eval/vocabularies.hpp"
#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/graphs.hpp"
#include "slam_it/foundations/image.hpp"
#include "slam_it/foundations/nls.hpp"
#include "slam_it/foundations/orb.hpp"
#include "slam_it/foundations/ransac.hpp"
#include "slam_it/foundations/se3.hpp"
#include "slam_it/foundations/stereo.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace slam_it::orchestration::vo {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
using foundations::camera::Model;

using foundations::image::Image;

/// @brief The one flag without a value: record each frame's keypoints, with their fates.
constexpr std::string_view kRecordFeatures = "--record-features";

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
///        (`slam_it.orchestration.vo._Delivered`).
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
  /// @brief The features every frame drops, those on a 0 (a TUM-format folder's `mask.png`);
  ///        none keeps them all.
  std::optional<Image> mask;
};

/// @brief Build a frame from its images, for the mode they were delivered for.
/// @param delivered The images.
/// @param camera The camera.
/// @param extraction The extraction settings the pipeline asks for.
/// @param matching The stereo matcher's settings.
/// @param mask The features to drop, those on a 0; null keeps them all.
/// @return The frame; the frame builder's error.
Result<edge::Frame> build(const Delivered& delivered, const edge::Camera& camera,
                          const foundations::orb::Options& extraction,
                          const foundations::stereo::MatchOptions& matching, const Image* mask) {
  return std::visit(
      [&](const auto& images) -> Result<edge::Frame> {
        using Images = std::decay_t<decltype(images)>;
        if constexpr (std::is_same_v<Images, StereoImages>) {
          return edge::stereo_pair(images.left, images.right, delivered.timestamp, camera,
                                   extraction, matching, mask);
        } else if constexpr (std::is_same_v<Images, RgbdImages>) {
          return edge::rgbd(images.image, images.depth, delivered.timestamp, camera, extraction,
                            mask);
        } else {
          return edge::monocular(images, delivered.timestamp, camera, extraction, mask);
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
  std::optional<Image> mask = std::move(read->mask);
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
  return Source{
      .camera = camera, .frames = count, .read = std::move(deliver), .mask = std::move(mask)};
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
  return Source{
      .camera = camera, .frames = count, .read = std::move(deliver), .mask = std::nullopt};
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
  return Source{
      .camera = camera, .frames = count, .read = std::move(deliver), .mask = std::nullopt};
}

/// @brief The settings as a run records them (`Options.as_dict`): each group an object of its
///        fields' values, in order, then `seed`.
/// @param options The settings.
/// @return Each group's name and object, and the seed; or the error writing a group.
Result<eval::runs::JsonObject> settings(const edge::Options& options) {
  std::vector<std::pair<std::string, eval::runs::JsonObject>> groups;
  eval::runs::JsonObject top;
  edge::for_each_setting(options, [&](const char* name, const auto& value) {
    const auto [group, field] = edge::split_setting(name);
    if (group.empty()) {
      top.emplace_back(field, value);
      return;
    }
    if (groups.empty() || groups.back().first != group) {
      groups.emplace_back(group, eval::runs::JsonObject{});
    }
    groups.back().second.emplace_back(field, value);
  });
  eval::runs::JsonObject out;
  for (const auto& [group, fields] : groups) {
    auto text = eval::runs::json_object(fields);
    if (!text) {
      return std::unexpected(text.error());
    }
    out.emplace_back(group, *std::move(text));
  }
  out.insert(out.end(), top.begin(), top.end());
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

/// @brief A count as JSON, or null if there is none.
/// @param count The count.
/// @return Its value.
eval::runs::JsonValue count_or_null(const std::optional<std::int64_t>& count) {
  return count ? eval::runs::JsonValue(*count) : eval::runs::JsonValue(nullptr);
}

/// @brief A name as JSON.
/// @param name The name, static.
/// @return Its value.
eval::runs::JsonValue name_json(const char* name) { return std::string(name); }

/// @brief An object written as nested JSON.
/// @param object The object.
/// @return Its value; `InvalidInput` for a number that is not finite.
Result<eval::runs::JsonValue> nested(const eval::runs::JsonObject& object) {
  auto text = eval::runs::json_object(object);
  if (!text) {
    return std::unexpected(std::move(text.error()));
  }
  return eval::runs::JsonValue(std::move(*text));
}

/// @brief Items written as a JSON array of objects.
/// @tparam Item An item's type.
/// @tparam Write Its writer, giving its object or an error.
/// @param items The items.
/// @param write The writer.
/// @return The array; the writer's error, or `InvalidInput` for a number that is not finite.
template <typename Item, typename Write>
Result<eval::runs::JsonValue> array_of(const std::vector<Item>& items, Write write) {
  std::vector<eval::runs::JsonValue> values;
  values.reserve(items.size());
  for (const Item& item : items) {
    auto value = write(item);
    if (!value) {
      return std::unexpected(std::move(value.error()));
    }
    values.push_back(std::move(*value));
  }
  auto text = eval::runs::json_array(values);
  if (!text) {
    return std::unexpected(std::move(text.error()));
  }
  return eval::runs::JsonValue(std::move(*text));
}

/// @brief Bundle-adjustment solves as JSON (`slam_it.edge.adjust.Solve.as_dict`, each).
/// @param solves The solves.
/// @return An array of one object per solve; `InvalidInput` for a cost that is not finite.
Result<eval::runs::JsonValue> solves_json(const std::vector<edge::Solve>& solves) {
  return array_of(solves, [](const edge::Solve& solve) {
    return nested({{"initial_cost", solve.initial_cost},
                   {"final_cost", solve.final_cost},
                   {"iterations", solve.iterations},
                   {"termination", name_json(foundations::nls::name(solve.termination))}});
  });
}

/// @brief How the frame's initialisation went, as JSON.
/// @param s The summary.
/// @return Its object; a cost's error.
Result<eval::runs::JsonValue> init_json(const edge::InitSummary& s) {
  auto ba = solves_json(s.solves);
  if (!ba) {
    return std::unexpected(std::move(ba.error()));
  }
  return nested({{"matches", count_or_null(s.matches)},
                 {"outcome", name_json(edge::name(s.outcome))},
                 {"ba", std::move(*ba)}});
}

/// @brief A RANSAC run's termination as a stage records it: its name, `degenerate` when it
///        found no pose, or null when it did not run.
/// @param degenerate Whether it found no pose.
/// @param termination Why it stopped, if it found one.
/// @return The value.
eval::runs::JsonValue termination_json(
    bool degenerate, const std::optional<foundations::ransac::Termination>& termination) {
  if (degenerate) {
    return name_json("degenerate");
  }
  return termination ? name_json(foundations::ransac::name(*termination))
                     : eval::runs::JsonValue(nullptr);
}

/// @brief How the PnP fallback went, as JSON.
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> pnp_json(const edge::PnpSummary& s) {
  return nested({{"candidates", s.candidates},
                 {"matches", count_or_null(s.matches)},
                 {"iterations", count_or_null(s.iterations)},
                 {"termination", termination_json(s.degenerate, s.termination)},
                 {"inliers", count_or_null(s.inliers)},
                 {"refined", count_or_null(s.refined)}});
}

/// @brief How relocalising the frame went, as JSON
///        (`slam_it.edge.relocalization.RelocalizationSummary.as_dict`).
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> relocalization_json(const edge::RelocalizationSummary& s) {
  auto candidates = array_of(s.candidates, [](const edge::CandidateSummary& c) {
    return nested({{"keyframe", c.keyframe},
                   {"score", c.score},
                   {"matches", c.matches},
                   {"iterations", count_or_null(c.iterations)},
                   {"termination", termination_json(c.degenerate, c.termination)},
                   {"inliers", count_or_null(c.inliers)},
                   {"refined", count_or_null(c.refined)},
                   {"found", count_or_null(c.found)}});
  });
  if (!candidates) {
    return std::unexpected(std::move(candidates.error()));
  }
  return nested({{"words", s.words},
                 {"sharing", s.sharing},
                 {"scored", s.scored},
                 {"candidates", std::move(*candidates)},
                 {"placed", s.placed}});
}

/// @brief How mapping the keyframe went, as JSON.
/// @param s The summary.
/// @return Its object; a cost's error.
Result<eval::runs::JsonValue> mapping_json(const edge::MappingSummary& s) {
  auto ba = solves_json(s.solves);
  if (!ba) {
    return std::unexpected(std::move(ba.error()));
  }
  return nested({{"from_depth", s.from_depth},
                 {"culled_points", s.culled_points},
                 {"triangulated", s.triangulated},
                 {"fused", s.fused},
                 {"ba", std::move(*ba)},
                 {"free", s.free},
                 {"fixed", s.fixed},
                 {"outliers", s.outliers},
                 {"weak_points", s.weak_points},
                 {"culled_keyframes", s.culled_keyframes}});
}

/// @brief A graph solve's summary as JSON (`slam_it.edge.loops._solve_record`).
/// @param s The summary.
/// @return Its object; `InvalidInput` for an error that is not finite.
Result<eval::runs::JsonValue> graph_solve_json(const foundations::graphs::Summary& s) {
  return nested({{"initial_error", s.initial_error},
                 {"final_error", s.final_error},
                 {"iterations", static_cast<std::int64_t>(s.iterations)},
                 {"termination", name_json(foundations::graphs::name(s.termination))}});
}

/// @brief A loop search's summary as JSON (`slam_it.edge.loops.DetectionSummary.as_dict`).
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> detection_json(const edge::loops::DetectionSummary& s) {
  auto candidates = array_of(s.candidates, [](const edge::loops::CandidateSummary& c) {
    return nested({{"keyframe", c.keyframe},
                   {"score", c.score},
                   {"near", c.near},
                   {"matches", count_or_null(c.matches)},
                   {"best", count_or_null(c.best)},
                   {"inliers", count_or_null(c.inliers)},
                   {"projected", count_or_null(c.projected)},
                   {"refined", count_or_null(c.refined)},
                   {"reprojected", count_or_null(c.reprojected)},
                   {"confirmations", count_or_null(c.confirmations)}});
  });
  if (!candidates) {
    return std::unexpected(std::move(candidates.error()));
  }
  eval::runs::JsonValue confirmation(nullptr);
  if (s.confirmation) {
    auto carried = nested({{"matched", s.confirmation->matched},
                           {"projected", s.confirmation->projected},
                           {"refined", count_or_null(s.confirmation->refined)},
                           {"reprojected", count_or_null(s.confirmation->reprojected)},
                           {"confirmed", s.confirmation->confirmed}});
    if (!carried) {
      return std::unexpected(std::move(carried.error()));
    }
    confirmation = std::move(*carried);
  }
  return nested({{"outcome", name_json(edge::loops::name(s.outcome))},
                 {"confirmation", std::move(confirmation)},
                 {"sharing", s.sharing},
                 {"scored", s.scored},
                 {"candidates", std::move(*candidates)},
                 {"confirmations", s.confirmations}});
}

/// @brief A loop's correction as JSON (`slam_it.edge.loops.CorrectionSummary.as_dict`).
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> correction_json(const edge::loops::CorrectionSummary& s) {
  std::vector<eval::runs::JsonValue> rows;
  rows.reserve(4);
  for (Eigen::Index r = 0; r < 4; ++r) {
    std::vector<eval::runs::JsonValue> row;
    row.reserve(4);
    for (Eigen::Index c = 0; c < 4; ++c) {
      row.emplace_back(s.similarity(r, c));
    }
    auto text = eval::runs::json_array(row);
    if (!text) {
      return std::unexpected(std::move(text.error()));
    }
    rows.emplace_back(std::move(*text));
  }
  auto similarity = eval::runs::json_array(rows);
  if (!similarity) {
    return std::unexpected(std::move(similarity.error()));
  }
  auto essential = graph_solve_json(s.essential);
  if (!essential) {
    return std::unexpected(std::move(essential.error()));
  }
  eval::runs::JsonValue ba(nullptr);
  if (s.global_ba) {
    auto solved = graph_solve_json(*s.global_ba);
    if (!solved) {
      return std::unexpected(std::move(solved.error()));
    }
    ba = std::move(*solved);
  }
  return nested({{"current", s.current},
                 {"matched", s.matched},
                 {"similarity", std::move(*similarity)},
                 {"corrected", s.corrected},
                 {"moved", s.moved},
                 {"replaced", s.replaced},
                 {"fused", s.fused},
                 {"links", s.links},
                 {"edges", s.edges},
                 {"essential", std::move(*essential)},
                 {"global_ba", std::move(ba)}});
}

/// @brief How loop closing went on a keyframe, as JSON
///        (`slam_it.edge.pipeline.LoopSummary.as_dict`).
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> loop_json(const edge::LoopSummary& s) {
  auto detection = detection_json(s.detection);
  if (!detection) {
    return std::unexpected(std::move(detection.error()));
  }
  eval::runs::JsonValue correction(nullptr);
  if (s.correction) {
    auto corrected = correction_json(*s.correction);
    if (!corrected) {
      return std::unexpected(std::move(corrected.error()));
    }
    correction = std::move(*corrected);
  }
  return nested({{"detection", std::move(*detection)}, {"correction", std::move(correction)}});
}

/// @brief A stage's summary as JSON, or null if the stage did not run.
/// @tparam Summary The stage's summary type.
/// @tparam Write Its writer.
/// @param summary The summary, if the stage ran.
/// @param write The writer.
/// @return The value; the writer's error.
template <typename Summary, typename Write>
Result<eval::runs::JsonValue> or_null(const std::optional<Summary>& summary, Write write) {
  if (!summary) {
    return eval::runs::JsonValue(nullptr);
  }
  return write(*summary);
}

/// @brief How a frame's stages went, as a run records it
///        (`slam_it.edge.pipeline.FrameSummary.as_dict`): each stage an object, or null.
/// @param summary The summary.
/// @return Its keys and values, in order; `InvalidInput` for a cost that is not finite.
Result<eval::runs::JsonObject> summary_json(const edge::FrameSummary& summary) {
  using eval::runs::JsonValue;
  const auto motion = [](const edge::MotionSummary& s) {
    return nested(
        {{"matches", s.matches}, {"widened", s.widened}, {"inliers", count_or_null(s.inliers)}});
  };
  const auto local_map = [](const edge::LocalMapSummary& s) {
    return nested({{"in_view", s.in_view},
                   {"matches", s.matches},
                   {"inliers", s.inliers},
                   {"keyframes", s.keyframes},
                   {"points", s.points},
                   {"reference", s.reference}});
  };
  const auto lost = [](const edge::LostSummary& s) {
    return nested({{"keyframes", s.keyframes}, {"points", s.points}, {"reset", s.reset}});
  };
  const std::array<std::pair<const char*, Result<JsonValue>>, 8> stages = {{
      {"init", or_null(summary.init, init_json)},
      {"motion", or_null(summary.motion, motion)},
      {"pnp", or_null(summary.pnp, pnp_json)},
      {"relocalization", or_null(summary.relocalization, relocalization_json)},
      {"local_map", or_null(summary.local_map, local_map)},
      {"mapping", or_null(summary.mapping, mapping_json)},
      {"loop", or_null(summary.loop, loop_json)},
      {"lost", or_null(summary.lost, lost)},
  }};
  eval::runs::JsonObject out;
  out.emplace_back("placed_by", summary.placed_by ? name_json(edge::name(*summary.placed_by))
                                                  : JsonValue(nullptr));
  for (const auto& [key, value] : stages) {
    if (!value) {
      return std::unexpected(value.error());
    }
    out.emplace_back(key, *value);
  }
  return out;
}

/// @brief Record a frame's result: its diagnostics (the trajectory is the pipeline's, at the
///        end: `record_trajectory`).
/// @param out The run.
/// @param result The result.
/// @param read_seconds The time delivering its images took, in seconds.
/// @param build_seconds The time building the frame took, in seconds.
/// @return Nothing, or a summary's error.
Result<void> record(eval::runs::Run& out, const edge::FrameResult& result, double read_seconds,
                    double build_seconds) {
  out.diagnostics.push_back({{"timestamp", result.timestamp},
                             {"state", std::string(edge::name(result.state))},
                             {"map", count_or_null(result.map)},
                             {"tracked", result.tracked},
                             {"keyframe", result.keyframe},
                             {"read_seconds", read_seconds},
                             {"build_seconds", build_seconds},
                             {"track_seconds", result.track_seconds},
                             {"map_seconds", result.map_seconds},
                             {"loop_seconds", result.loop_seconds}});
  auto summary = summary_json(result.summary);
  if (!summary) {
    return std::unexpected(std::move(summary.error()));
  }
  for (auto& item : *summary) {
    out.diagnostics.back().push_back(std::move(item));
  }
  return {};
}

/// @brief Record the run's trajectory: each tracked frame's final pose and its pose as tracked,
///        as `T_world_cam`, and its map; a reset map's frames are left out
///        (`edge::Pipeline::trajectory`).
/// @param out The run.
/// @param trajectory The pipeline's trajectory.
/// @return Nothing, or a pose's inversion's error.
Result<void> record_trajectory(eval::runs::Run& out, const edge::Trajectory& trajectory) {
  for (std::size_t i = 0; i < trajectory.timestamps.size(); ++i) {
    const auto final_pose = foundations::se3::inverse(trajectory.poses[i]);
    if (!final_pose) {
      return std::unexpected(final_pose.error());
    }
    const auto tracked_pose = foundations::se3::inverse(trajectory.tracked[i]);
    if (!tracked_pose) {
      return std::unexpected(tracked_pose.error());
    }
    out.timestamps.push_back(trajectory.timestamps[i]);
    out.poses.push_back(*final_pose);
    out.tracked_poses.push_back(*tracked_pose);
    out.pose_maps.push_back(trajectory.maps[i]);
  }
  return {};
}

/// @brief Record every keyframe and point the run made, with their lives and maps.
/// @param out The run.
/// @param lives The pipeline's lifetimes.
/// @return Nothing, or a pose's inversion's error.
Result<void> record_lives(eval::runs::Run& out, const edge::Lifetimes& lives) {
  out.keyframe_timestamps = lives.keyframe_times;
  for (const Eigen::Matrix4d& T_cam_world :  // NOLINT(readability-identifier-naming)
       lives.keyframe_poses) {
    const auto pose = foundations::se3::inverse(T_cam_world);
    if (!pose) {
      return std::unexpected(pose.error());
    }
    out.keyframe_poses.push_back(*pose);
  }
  out.points = lives.points;
  out.point_lives = lives.point_lives;
  out.keyframe_lives = lives.keyframe_lives;
  out.keyframe_parents = lives.keyframe_parents;
  out.keyframe_links = lives.keyframe_links;
  out.keyframe_loops = lives.keyframe_loops;
  out.point_maps = lives.point_maps;
  out.keyframe_maps = lives.keyframe_maps;
  return {};
}

/// @brief Append a frame's keypoints, with their fates, to the run's feature files.
/// @param features The writer.
/// @param frame The frame.
/// @param fates What became of each of its keypoints.
/// @return Nothing; the writer's error.
Result<void> write_features(eval::runs::FeatureWriter& features, const edge::Frame& frame,
                            const edge::Fates& fates) {
  std::vector<std::array<double, 2>> xy;
  std::vector<std::int64_t> level;
  for (const foundations::orb::Keypoint& keypoint : frame.features.keypoints) {
    xy.push_back(keypoint.xy);
    level.push_back(keypoint.level);
  }
  std::vector<std::int64_t> fate;
  std::ranges::transform(fates.fate, std::back_inserter(fate),
                         [](edge::Fate f) { return static_cast<std::int64_t>(f); });
  return features.write(xy, level, fate, fates.point);
}

/// @brief Track a sequence's frames, from the first asked for, to the end: a lost track
///        closes its map, and the next frame starts a new one.
/// @param source The sequence.
/// @param pipeline The pipeline.
/// @param request The request.
/// @param out The run, each frame recorded.
/// @param features The writer of each frame's keypoints, if the run records them.
/// @return Nothing; a reader's, a frame builder's, the pipeline's or the feature writer's error.
Result<void> track(const Source& source, edge::Pipeline& pipeline, const Request& request,
                   eval::runs::Run& out, eval::runs::FeatureWriter* features) {
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
    const auto frame = build(**delivered, source.camera, pipeline.extraction(), matching,
                             source.mask ? &*source.mask : nullptr);
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
    if (features != nullptr) {
      if (const auto written = write_features(*features, *frame, result->fates); !written) {
        return written;
      }
    }
  }
  return {};
}

/// @brief Read a mode's name.
/// @param text The name, as `slam_it.edge.options.MODES` spells it.
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

/// @brief Read a flag setting's value: `true` or `false`.
/// @param key The setting.
/// @param value Its text.
/// @param field The setting's field.
/// @return Nothing, or `InvalidInput` for another word.
Result<void> read_value(std::string_view key, std::string_view value, bool& field) {
  const std::string_view word = eval::text::trim(value);
  if (word != "true" && word != "false") {
    return fail(ErrorKind::kInvalidInput,
                std::string(key) + ": expected true or false, got '" + std::string(value) + "'");
  }
  field = word == "true";
  return {};
}

/// @brief Read an integer setting's value: a whole integer.
/// @param key The setting.
/// @param value Its text.
/// @param field The setting's field.
/// @return Nothing, or `InvalidInput` for anything else.
Result<void> read_value(std::string_view key, std::string_view value, std::int64_t& field) {
  const auto integer = integer_of(key, value);
  if (!integer) {
    return std::unexpected(integer.error());
  }
  field = *integer;
  return {};
}

/// @brief Read a real setting's value: a finite decimal number.
/// @param key The setting.
/// @param value Its text.
/// @param field The setting's field.
/// @return Nothing, or `InvalidInput` for anything else.
Result<void> read_value(std::string_view key, std::string_view value, double& field) {
  const auto number = eval::text::number(value);
  if (!number.has_value() || !std::isfinite(*number)) {
    return fail(ErrorKind::kInvalidInput,
                std::string(key) + ": expected a finite number, got '" + std::string(value) + "'");
  }
  field = *number;
  return {};
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
  std::optional<Result<void>> read;
  edge::for_each_setting(options, [&](const char* name, auto& field) {
    if (!read && key == name) {
      read = read_value(key, value, field);
    }
  });
  if (!read) {
    return fail(ErrorKind::kInvalidInput, "--option: no setting '" + std::string(key) + "'");
  }
  return *read;
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
  } else if (flag == "--vocabulary") {
    out.vocabulary = value;
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
  std::size_t i = 0;
  while (i < arguments.size()) {
    const std::string& flag = arguments[i];
    if (flag != "--option" && std::ranges::find(seen, flag) != seen.end()) {
      return fail(ErrorKind::kInvalidInput, flag + ": given twice");
    }
    if (flag == kRecordFeatures) {
      seen.emplace_back(flag);
      out.record_features = true;
      ++i;
      continue;
    }
    if (i + 1 == arguments.size()) {
      return fail(ErrorKind::kInvalidInput, flag + ": expected a value");
    }
    seen.emplace_back(flag);
    if (const auto read = read_flag(out, flag, arguments[i + 1]); !read) {
      return std::unexpected(read.error());
    }
    i += 2;
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
  const bool needed = edge::keeps_keyframe_database(request.options);
  if (needed && !request.vocabulary) {
    return fail(ErrorKind::kInvalidInput,
                "--vocabulary: relocalization.enabled or loops.enabled needs one");
  }
  if (!needed && request.vocabulary) {
    return fail(ErrorKind::kInvalidInput,
                "--vocabulary: given, but relocalization.enabled and loops.enabled are both off");
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
  std::optional<foundations::vocabulary::Vocabulary> words;
  std::optional<eval::vocabularies::Header> record;
  if (request.vocabulary) {
    auto read = eval::vocabularies::read(*request.vocabulary);
    if (!read) {
      return std::unexpected(read.error());
    }
    words = std::move(read->vocabulary);
    record = read->header;
  }
  auto pipeline =
      edge::Pipeline::create(request.mode, source->camera, request.options, std::move(words));
  if (!pipeline) {
    return std::unexpected(pipeline.error());
  }
  eval::runs::Run out;
  out.dataset = request.dataset;
  out.sequence = sequence_name(request.sequence);
  out.mode = edge::name(request.mode);
  out.backend = "cpp";
  auto recorded = settings(request.options);
  if (!recorded) {
    return std::unexpected(recorded.error());
  }
  out.settings = *std::move(recorded);
  out.vocabulary = record;
  out.record_features = request.record_features;
  std::optional<eval::runs::FeatureWriter> features;
  if (request.record_features) {
    auto opened = eval::runs::FeatureWriter::open(request.out);
    if (!opened) {
      return std::unexpected(opened.error());
    }
    features = std::move(*opened);
  }
  if (const auto tracked = track(*source, *pipeline, request, out, features ? &*features : nullptr);
      !tracked) {
    return std::unexpected(tracked.error());
  }
  if (const auto recorded = record_trajectory(out, pipeline->trajectory()); !recorded) {
    return std::unexpected(recorded.error());
  }
  if (const auto recorded = record_lives(out, pipeline->lifetimes()); !recorded) {
    return std::unexpected(recorded.error());
  }
  return out;
}

}  // namespace slam_it::orchestration::vo
