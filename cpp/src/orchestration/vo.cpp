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
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
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

/// @brief A flag without a value: record each frame's keypoints, with their fates.
constexpr std::string_view kRecordFeatures = "--record-features";

/// @brief A flag without a value: run the steps in threads of their own.
constexpr std::string_view kThreaded = "--threaded";

/// @brief A flag without a value: with `kThreaded`, share the map by ORB-SLAM's locks.
constexpr std::string_view kLocks = "--locks";

/// @brief A flag without a value: in turn, track each frame on the view a writer would publish.
constexpr std::string_view kViews = "--views";

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
  /// @brief Each frame's time, in seconds, in the sequence's order.
  std::vector<double> times;
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
  std::vector<double> times = read->timestamps;
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
  return Source{.camera = camera,
                .times = std::move(times),
                .read = std::move(deliver),
                .mask = std::move(mask)};
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
  std::vector<double> times = frames.timestamps;
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
  return Source{.camera = camera,
                .times = std::move(times),
                .read = std::move(deliver),
                .mask = std::nullopt};
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
  std::vector<double> times = read->timestamps;
  Reader deliver = [frames = std::move(*read)](std::size_t i) -> Read {
    auto image = eval::png::read_gray(frames.left.at(i));
    if (!image) {
      return std::unexpected(image.error());
    }
    return Delivered{.timestamp = frames.timestamps.at(i), .images = std::move(*image)};
  };
  return Source{.camera = camera,
                .times = std::move(times),
                .read = std::move(deliver),
                .mask = std::nullopt};
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

/// @brief Whether two sequences' cameras are the same: model, intrinsics, size, baseline.
/// @param first One's camera.
/// @param other The other's.
/// @return Whether they are.
bool same_camera(const edge::Camera& first, const edge::Camera& other) {
  return first.model == other.model && first.intrinsics.size() == other.intrinsics.size() &&
         first.intrinsics == other.intrinsics && first.width == other.width &&
         first.height == other.height && first.baseline == other.baseline;
}

/// @brief A session's median frame interval, as numpy's `median` of its times' differences.
/// @param times Its frames' times, at least two, ascending.
/// @return The median interval, in seconds.
double median_interval(const std::vector<double>& times) {
  std::vector<double> intervals;
  intervals.reserve(times.size() - 1);
  for (std::size_t i = 1; i < times.size(); ++i) {
    intervals.push_back(times[i] - times[i - 1]);
  }
  const std::size_t half = intervals.size() / 2;
  std::ranges::nth_element(intervals, intervals.begin() + static_cast<std::ptrdiff_t>(half));
  const double upper = intervals[half];
  if (intervals.size() % 2 == 1) {
    return upper;
  }
  const double lower = *std::ranges::max_element(std::span(intervals).first(half));
  return (lower + upper) / 2.0;
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

/// @brief A similarity as JSON: its rows.
/// @param similarity The similarity.
/// @return Its four rows of four numbers; `InvalidInput` for a number that is not finite.
Result<eval::runs::JsonValue> similarity_json(const Eigen::Matrix4d& similarity) {
  std::vector<eval::runs::JsonValue> rows;
  rows.reserve(4);
  for (Eigen::Index r = 0; r < 4; ++r) {
    std::vector<eval::runs::JsonValue> row;
    row.reserve(4);
    for (Eigen::Index c = 0; c < 4; ++c) {
      row.emplace_back(similarity(r, c));
    }
    auto text = eval::runs::json_array(row);
    if (!text) {
      return std::unexpected(std::move(text.error()));
    }
    rows.emplace_back(std::move(*text));
  }
  return eval::runs::json_array(rows).transform(
      [](eval::runs::JsonText text) { return eval::runs::JsonValue(std::move(text)); });
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

/// @brief How a map was closed as JSON (`slam_it.edge.pipeline.LostSummary.as_dict`).
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> lost_json(const edge::LostSummary& s) {
  return nested({{"keyframes", s.keyframes}, {"points", s.points}, {"reset", s.reset}});
}

/// @brief A correction's similarity, essential graph and global BA as JSON: a loop's or a merge's.
struct Spread {
  /// @brief The similarity.
  eval::runs::JsonValue similarity;
  /// @brief The essential graph's solve.
  eval::runs::JsonValue essential;
  /// @brief Global BA's solve, or null.
  eval::runs::JsonValue global_ba;
};

/// @brief A correction's similarity, essential graph and global BA as JSON.
/// @param similarity The similarity.
/// @param essential The essential graph's solve.
/// @param global_ba Global BA's, if it ran.
/// @return Their values; `InvalidInput` for a number that is not finite.
Result<Spread> spread_json(const Eigen::Matrix4d& similarity,
                           const foundations::graphs::Summary& essential,
                           const std::optional<foundations::graphs::Summary>& global_ba) {
  auto placed = similarity_json(similarity);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  auto graph = graph_solve_json(essential);
  if (!graph) {
    return std::unexpected(std::move(graph.error()));
  }
  auto ba = or_null(global_ba, graph_solve_json);
  if (!ba) {
    return std::unexpected(std::move(ba.error()));
  }
  return Spread{.similarity = std::move(*placed),
                .essential = std::move(*graph),
                .global_ba = std::move(*ba)};
}

/// @brief A loop's correction as JSON (`slam_it.edge.pipeline.Corrected.as_dict`): the
///        summary's (`slam_it.edge.loops.CorrectionSummary.as_dict`), its keyframes by the run's
///        ids too (`matched_run`, `current_run`).
/// @param corrected The correction.
/// @return Its object; a graph solve's error.
Result<eval::runs::JsonValue> correction_json(const edge::Corrected& corrected) {
  const edge::loops::CorrectionSummary& s = corrected.summary;
  auto spread = spread_json(s.similarity, s.essential, s.global_ba);
  if (!spread) {
    return std::unexpected(std::move(spread.error()));
  }
  return nested({{"current", s.current},
                 {"matched", s.matched},
                 {"similarity", std::move(spread->similarity)},
                 {"corrected", s.corrected},
                 {"moved", s.moved},
                 {"replaced", s.replaced},
                 {"fused", s.fused},
                 {"links", s.links},
                 {"edges", s.edges},
                 {"essential", std::move(spread->essential)},
                 {"global_ba", std::move(spread->global_ba)},
                 {"matched_run", corrected.run_ids[0]},
                 {"current_run", corrected.run_ids[1]}});
}

/// @brief A merge's welding BA as JSON (`slam_it.edge.loops.WeldSummary.as_dict`).
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> weld_json(const edge::loops::WeldSummary& s) {
  auto first = graph_solve_json(s.first);
  if (!first) {
    return std::unexpected(std::move(first.error()));
  }
  auto second = or_null(s.second, graph_solve_json);
  if (!second) {
    return std::unexpected(std::move(second.error()));
  }
  return nested({{"keyframes", s.keyframes},
                 {"held", s.held},
                 {"points", s.points},
                 {"observations", s.observations},
                 {"outliers", s.outliers},
                 {"forgotten", s.forgotten},
                 {"first", std::move(*first)},
                 {"second", std::move(*second)}});
}

/// @brief A merge as JSON (`slam_it.edge.loops.MergeSummary.as_dict`).
/// @param s The summary.
/// @return Its object.
Result<eval::runs::JsonValue> merge_json(const edge::loops::MergeSummary& s) {
  auto spread = spread_json(s.similarity, s.essential, s.global_ba);
  if (!spread) {
    return std::unexpected(std::move(spread.error()));
  }
  auto weld = weld_json(s.weld);
  if (!weld) {
    return std::unexpected(std::move(weld.error()));
  }
  return nested({{"map", s.map},
                 {"current", s.current},
                 {"matched", s.matched},
                 {"similarity", std::move(spread->similarity)},
                 {"window", s.window},
                 {"matched_window", s.matched_window},
                 {"fused", s.fused},
                 {"weld", std::move(*weld)},
                 {"edges", s.edges},
                 {"essential", std::move(spread->essential)},
                 {"global_ba", std::move(spread->global_ba)}});
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
  auto correction = or_null(s.correction, correction_json);
  if (!correction) {
    return std::unexpected(std::move(correction.error()));
  }
  auto merging = or_null(s.merging, detection_json);
  if (!merging) {
    return std::unexpected(std::move(merging.error()));
  }
  auto merge = or_null(s.merge, merge_json);
  if (!merge) {
    return std::unexpected(std::move(merge.error()));
  }
  return nested({{"detection", std::move(*detection)},
                 {"correction", std::move(*correction)},
                 {"merging", std::move(*merging)},
                 {"merge", std::move(*merge)}});
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
  const std::array<std::pair<const char*, Result<JsonValue>>, 9> stages = {{
      {"init", or_null(summary.init, init_json)},
      {"motion", or_null(summary.motion, motion)},
      {"pnp", or_null(summary.pnp, pnp_json)},
      {"relocalization", or_null(summary.relocalization, relocalization_json)},
      {"local_map", or_null(summary.local_map, local_map)},
      {"mapping", or_null(summary.mapping, mapping_json)},
      {"loop", or_null(summary.loop, loop_json)},
      {"lost", or_null(summary.lost, lost_json)},
      {"left", or_null(summary.left, lost_json)},
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

/// @brief How local mapping scheduled a keyframe's steps, as JSON
///        (`slam_it.edge.mapping.Schedule.as_dict`).
/// @param schedule The schedule.
/// @return Its object.
Result<eval::runs::JsonValue> schedule_json(const edge::Schedule& schedule) {
  return nested({{"fusion", name_json(edge::name(schedule.fusion))},
                 {"adjustment", name_json(edge::name(schedule.adjustment))},
                 {"culling", name_json(edge::name(schedule.culling))}});
}

/// @brief How a global BA in a thread of its own went, as JSON
///        (`slam_it.edge.loops.GlobalSummary.as_dict`).
/// @param s The summary.
/// @return Its object; `InvalidInput` for an error that is not finite.
Result<eval::runs::JsonValue> global_json(const edge::loops::GlobalSummary& s) {
  auto solve = graph_solve_json(s.solve);
  if (!solve) {
    return std::unexpected(std::move(solve.error()));
  }
  const edge::loops::Propagated moved = s.propagation.value_or(edge::loops::Propagated{});
  return nested({{"generation", s.generation},
                 {"applied", s.applied},
                 {"solve", *std::move(solve)},
                 {"adjusted", moved.adjusted},
                 {"propagated", moved.propagated},
                 {"points", moved.points},
                 {"carried", moved.carried}});
}

/// @brief A session's start as JSON (`slam_it.orchestration.vo`'s docstring): its index, its
///        sequence's name, its time offset, and how the map tracked before it was closed.
/// @param index The session's index.
/// @param sequence Its folder.
/// @param offset Its time offset, in seconds.
/// @param closed How the map tracked was closed; none if none was.
/// @return Its object.
Result<eval::runs::JsonValue> session_json(std::size_t index, const std::string& sequence,
                                           double offset,
                                           const std::optional<edge::LostSummary>& closed) {
  auto map = or_null(closed, lost_json);
  if (!map) {
    return std::unexpected(std::move(map.error()));
  }
  return nested({{"index", static_cast<std::int64_t>(index)},
                 {"sequence", sequence_name(sequence)},
                 {"offset", offset},
                 {"closed", *std::move(map)}});
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
  auto schedule =
      result.schedule ? schedule_json(*result.schedule) : Result<eval::runs::JsonValue>(nullptr);
  auto none = eval::runs::json_array({});
  if (!schedule || !none) {
    return std::unexpected(!schedule ? schedule.error() : none.error());
  }
  eval::runs::JsonObject& line = out.diagnostics.back();
  line.emplace_back("wait_seconds", result.wait_seconds);
  line.emplace_back("asked", result.asked);
  line.emplace_back("refused", result.refused);
  line.emplace_back("queue", result.queue);
  line.emplace_back("schedule", *std::move(schedule));
  line.emplace_back("global_ba", *none);
  line.emplace_back("global_ba_seconds", 0.0);
  line.emplace_back("views", *std::move(none));
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
    out.tracked_maps.push_back(trajectory.tracked_maps[i]);
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
  out.keyframe_merges = lives.keyframe_merges;
  out.point_maps = lives.point_maps;
  out.keyframe_maps = lives.keyframe_maps;
  return {};
}

/// @brief A frame's keypoints as a run records them: pixels, levels, and their fates
///        (`slam_it.orchestration.vo._Keypoints`).
struct Keypoints {
  /// @brief Each keypoint's pixel.
  std::vector<std::array<double, 2>> xy;
  /// @brief Its pyramid level.
  std::vector<std::int64_t> level;
  /// @brief What became of it.
  edge::Fates fates;
};

/// @brief A frame's keypoints, with their fates.
/// @param frame The frame.
/// @param fates What became of each of its keypoints.
/// @return The keypoints.
Keypoints keypoints_of(const edge::Frame& frame, edge::Fates fates) {
  Keypoints out{.xy = {}, .level = {}, .fates = std::move(fates)};
  for (const foundations::orb::Keypoint& keypoint : frame.features.keypoints) {
    out.xy.push_back(keypoint.xy);
    out.level.push_back(keypoint.level);
  }
  return out;
}

/// @brief Append a frame's keypoints, with their fates, to the run's feature files.
/// @param features The writer.
/// @param keypoints The keypoints.
/// @return Nothing; the writer's error.
Result<void> write_features(eval::runs::FeatureWriter& features, const Keypoints& keypoints) {
  std::vector<std::int64_t> fate;
  std::ranges::transform(keypoints.fates.fate, std::back_inserter(fate),
                         [](edge::Fate f) { return static_cast<std::int64_t>(f); });
  return features.write(keypoints.xy, keypoints.level, fate, keypoints.fates.point);
}

/// @brief Set a key of a frame's diagnostics, in its place, or last if it is new.
/// @param line The frame's diagnostics.
/// @param key The key.
/// @param value Its value.
void set_key(eval::runs::JsonObject& line, std::string_view key, eval::runs::JsonValue value) {
  for (auto& [name, held] : line) {
    if (name == key) {
      held = std::move(value);
      return;
    }
  }
  line.emplace_back(std::string(key), std::move(value));
}

/// @brief Give the keypoints mapping made a point of the fate `Fate::kMade` (threaded).
/// @param fates The keypoints' fates, changed.
/// @param made The point made of each keypoint, by the run's ids, -1 where none.
void mark_made(edge::Fates& fates, const std::vector<std::int64_t>& made) {
  for (std::size_t j = 0; j < std::min(made.size(), fates.fate.size()); ++j) {
    if (made[j] >= 0) {
      fates.fate[j] = edge::Fate::kMade;
      fates.point[j] = made[j];
    }
  }
}

/// @brief A frame's global BAs and views published, as the completions bring them.
struct Globals {
  /// @brief Each global BA's record.
  std::vector<eval::runs::JsonValue> listed;
  /// @brief Their time, in seconds.
  double seconds{0.0};
  /// @brief Each view's size.
  std::vector<eval::runs::JsonValue> views;
};

/// @brief Add one completion to its frame's diagnostics: a keyframe's mapping, its schedule and
///        loop closing set, a global BA and a view published kept for the frame's lists.
/// @param line The frame's diagnostics.
/// @param completion The completion.
/// @param globals The frame's global BAs so far.
/// @return Nothing; a summary's error.
Result<void> merge_one(eval::runs::JsonObject& line, const edge::Completion& completion,
                       Globals& globals) {
  if (completion.map_seconds) {
    set_key(line, "map_seconds", *completion.map_seconds);
  }
  if (completion.loop_seconds) {
    set_key(line, "loop_seconds", *completion.loop_seconds);
  }
  const auto value = [&](std::string_view key, Result<eval::runs::JsonValue> json) -> Result<void> {
    if (!json) {
      return std::unexpected(std::move(json.error()));
    }
    set_key(line, key, *std::move(json));
    return {};
  };
  if (completion.mapping) {
    if (auto ok = value("mapping", mapping_json(*completion.mapping)); !ok) {
      return ok;
    }
  }
  if (completion.schedule) {
    if (auto ok = value("schedule", schedule_json(*completion.schedule)); !ok) {
      return ok;
    }
  }
  if (completion.loop) {
    if (auto ok = value("loop", loop_json(*completion.loop)); !ok) {
      return ok;
    }
  }
  if (completion.global_ba) {
    auto global = global_json(*completion.global_ba);
    if (!global) {
      return std::unexpected(std::move(global.error()));
    }
    globals.listed.push_back(*std::move(global));
    globals.seconds += completion.global_ba_seconds;
  }
  if (completion.view) {
    auto view = nested({{"keyframes", completion.view->keyframes},
                        {"points", completion.view->points},
                        {"bytes", completion.view->bytes}});
    if (!view) {
      return std::unexpected(std::move(view.error()));
    }
    globals.views.push_back(*std::move(view));
  }
  return {};
}

/// @brief Add what the threads finished to the frames' diagnostics (a keyframe's mapping and
///        loop closing on its frame, a global BA on the frame it ended on, listed, its time
///        summed), and give the keypoints mapping made a point of the fate `Fate::kMade`
///        (`slam_it.orchestration.vo._merge`).
/// @param out The run, each frame recorded.
/// @param held Each frame's keypoints, when the run records them; else empty.
/// @param done The completions.
/// @return Nothing; a summary's error.
Result<void> merge(eval::runs::Run& out, std::vector<Keypoints>& held,
                   const std::vector<edge::Completion>& done) {
  if (out.diagnostics.empty()) {
    return {};
  }
  std::map<std::size_t, Globals> globals;
  for (const edge::Completion& completion : done) {
    const auto frame = static_cast<std::size_t>(completion.frame);
    const std::size_t at = std::min(frame, out.diagnostics.size() - 1);
    if (auto ok = merge_one(out.diagnostics[at], completion, globals[at]); !ok) {
      return ok;
    }
    if (frame < held.size()) {
      mark_made(held[frame].fates, completion.made);
    }
  }
  for (auto& [at, global] : globals) {
    if (!global.listed.empty()) {
      auto listed = eval::runs::json_array(global.listed);
      if (!listed) {
        return std::unexpected(std::move(listed.error()));
      }
      set_key(out.diagnostics[at], "global_ba", *std::move(listed));
      set_key(out.diagnostics[at], "global_ba_seconds", global.seconds);
    }
    if (!global.views.empty()) {
      auto views = eval::runs::json_array(global.views);
      if (!views) {
        return std::unexpected(std::move(views.error()));
      }
      set_key(out.diagnostics[at], "views", *std::move(views));
    }
  }
  return {};
}

/// @brief The clock a run is timed and paced by.
using Clock = std::chrono::steady_clock;

/// @brief Seconds, as a run records them.
using Seconds = std::chrono::duration<double>;

/// @brief Feeds frames at their recorded rate (threaded): each is given its interval since
///        the last, as ORB-SLAM3's examples sleep out the rest of theirs, a late frame delaying
///        the next.
class Pacer {
 public:
  /// @brief Wait until a frame is due: the last frame's interval after the last was given.
  /// @param timestamp The frame's time.
  void wait(double timestamp) {
    if (last_) {
      const auto interval = std::chrono::duration_cast<Clock::duration>(
          Seconds(std::max(0.0, timestamp - last_->second)));
      std::this_thread::sleep_until(last_->first + interval);
    }
    last_ = {Clock::now(), timestamp};
  }

 private:
  /// @brief When the last frame was given, and its time.
  std::optional<std::pair<Clock::time_point, double>> last_;
};

/// @brief End a run's tracking: wait for the threads (threaded), add what they finished to the
///        frames' diagnostics, and write the keypoints held for it.
/// @param pipeline The pipeline.
/// @param out The run, each frame recorded.
/// @param held Each frame's keypoints, held while their keyframes were mapped; else empty.
/// @param features The writer of each frame's keypoints, if the run records them.
/// @return Nothing; a thread's, a summary's or the feature writer's error.
Result<void> finish(edge::Pipeline& pipeline, eval::runs::Run& out, std::vector<Keypoints>& held,
                    eval::runs::FeatureWriter* features) {
  if (const auto finished = pipeline.finish(); !finished) {
    return finished;
  }
  if (const auto merged = merge(out, held, pipeline.completed()); !merged) {
    return merged;
  }
  if (features == nullptr) {
    return {};
  }
  for (const Keypoints& keypoints : held) {
    if (const auto written = write_features(*features, keypoints); !written) {
      return written;
    }
  }
  return {};
}

/// @brief The frames of a run's sequences, in turn.
struct Turns {
  /// @brief Each frame's session and its index in it.
  std::vector<std::pair<std::size_t, std::size_t>> order;
  /// @brief Each session's time offset, in seconds (`session_offsets`).
  std::vector<double> offsets;
};

/// @brief Put a run's sequences' frames in turn.
/// @param sources The sequences.
/// @return Their frames, in turn, and the sessions' time offsets.
Turns in_turn(std::span<const Source> sources) {
  Turns out;
  std::vector<std::vector<double>> times;
  for (std::size_t k = 0; k < sources.size(); ++k) {
    times.push_back(sources[k].times);
    for (std::size_t i = 0; i < sources[k].times.size(); ++i) {
      out.order.emplace_back(k, i);
    }
  }
  out.offsets = session_offsets(times);
  return out;
}

/// @brief Before a frame of session `index` runs: start the session if it follows another
///        (`slam_it.orchestration.vo._start_session`), closing the map tracked
///        (`edge::Pipeline::start_session`), its record kept for its first frame
///        (`session_json`).
/// @param pipeline The pipeline.
/// @param request The request.
/// @param turns The run's frames in turn, and the sessions' offsets.
/// @param index The frame's session.
/// @param current The session of the frame run before, if any; set to `index`.
/// @param session The record of a session started, set when one starts.
/// @return Whether a session started; the threads' or a summary's error.
Result<bool> enter_session(edge::Pipeline& pipeline, const Request& request, const Turns& turns,
                           std::size_t index, std::optional<std::size_t>& current,
                           std::optional<eval::runs::JsonValue>& session) {
  const bool starts = current.has_value() && index != *current;
  current = index;
  if (!starts) {
    return false;
  }
  auto closed = pipeline.start_session();
  if (!closed) {
    return std::unexpected(std::move(closed.error()));
  }
  auto started = session_json(index, request.sequences[index], turns.offsets[index], *closed);
  if (!started) {
    return std::unexpected(std::move(started.error()));
  }
  session = *std::move(started);
  return true;
}

/// @brief Add the record of the session a frame started to the frame's diagnostics, the frame
///        recorded last; nothing if none started on it.
/// @param out The run.
/// @param session The record, if a session started; emptied.
void note_session(eval::runs::Run& out, std::optional<eval::runs::JsonValue>& session) {
  if (session && !out.diagnostics.empty()) {
    out.diagnostics.back().emplace_back("session", *std::move(session));
  }
  session.reset();
}

/// @brief Keep a frame's keypoints for the run: written at once, or, threaded, held until
///        their keyframe's points are made (`finish` writes them).
/// @param keypoints The frame's keypoints, with their fates.
/// @param threaded Whether the run is threaded.
/// @param held The keypoints held, given these when threaded.
/// @param features The writer of each frame's keypoints.
/// @return Nothing; the feature writer's error.
Result<void> keep_keypoints(Keypoints keypoints, bool threaded, std::vector<Keypoints>& held,
                            eval::runs::FeatureWriter& features) {
  if (threaded) {
    held.push_back(std::move(keypoints));
    return {};
  }
  return write_features(features, keypoints);
}

/// @brief Track the sequences' frames in turn, from the first asked for, to the end: a lost
///        track closes its map, and the next frame starts a new one; so does each sequence
///        after the first (`edge::Pipeline::start_session`), its first frame run recording the
///        session (`session_json`). Threaded, the frames come at their recorded rate, each
///        session's first at once (as ORB-SLAM3's examples sleep only within a sequence), and
///        the run's end waits for the threads.
/// @param sources The sequences.
/// @param pipeline The pipeline.
/// @param request The request.
/// @param out The run, each frame recorded.
/// @param features The writer of each frame's keypoints, if the run records them.
/// @return Nothing; a reader's, a frame builder's, the pipeline's or the feature writer's error.
Result<void> track(std::span<const Source> sources, edge::Pipeline& pipeline,
                   const Request& request, eval::runs::Run& out,
                   eval::runs::FeatureWriter* features) {
  const Turns turns = in_turn(sources);
  const auto first = static_cast<std::size_t>(request.first);
  const std::size_t end =
      request.count.has_value()
          ? std::min(turns.order.size(), first + static_cast<std::size_t>(*request.count))
          : turns.order.size();
  const foundations::stereo::MatchOptions matching = edge::stereo_matching(request.options);
  Pacer pacer;
  std::vector<Keypoints> held;  // threaded: written once their keyframes are mapped
  std::optional<std::size_t> current;
  std::optional<eval::runs::JsonValue> session;  // a session started, for its first frame run
  for (std::size_t n = first; n < end; ++n) {
    const auto [k, i] = turns.order[n];
    const Source& source = sources[k];
    const auto start = Clock::now();
    Read delivered = source.read(i);
    const auto read = Clock::now();
    if (!delivered) {
      return std::unexpected(delivered.error());
    }
    if (!delivered->has_value()) {
      continue;
    }
    const auto entered = enter_session(pipeline, request, turns, k, current, session);
    if (!entered) {
      return std::unexpected(entered.error());
    }
    if (*entered) {
      pacer = Pacer{};  // a session's first frame comes at once
    }
    (*delivered)->timestamp += turns.offsets[k];
    if (request.threaded) {
      pacer.wait((*delivered)->timestamp);
    }
    const auto given = Clock::now();
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
            record(out, *result, Seconds(read - start).count(), Seconds(built - given).count());
        !recorded) {
      return recorded;
    }
    note_session(out, session);
    if (features != nullptr) {
      if (const auto kept = keep_keypoints(keypoints_of(*frame, result->fates), request.threaded,
                                           held, *features);
          !kept) {
        return kept;
      }
    }
  }
  return finish(pipeline, out, held, features);
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
    out.sequences.push_back(value);
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

/// @brief The request's switch a flag without a value sets.
/// @param out The request.
/// @param flag The flag.
/// @return The switch, or null for a flag with a value.
bool* switched(Request& out, std::string_view flag) {
  if (flag == kRecordFeatures) {
    return &out.record_features;
  }
  if (flag == kThreaded) {
    return &out.threaded;
  }
  if (flag == kLocks) {
    return &out.locks;
  }
  if (flag == kViews) {
    return &out.views;
  }
  return nullptr;
}

/// @brief How a request's steps run.
/// @param request The request.
/// @return Inline, or in threads of their own by our design or by ORB-SLAM's locks.
edge::Threading threading_of(const Request& request) {
  if (!request.threaded) {
    return request.views ? edge::Threading::kInlineView : edge::Threading::kInline;
  }
  return request.locks ? edge::Threading::kLocks : edge::Threading::kView;
}

}  // namespace

Result<Request> parse(std::span<const std::string> arguments) {
  Request out;
  std::vector<std::string_view> seen;
  std::size_t i = 0;
  while (i < arguments.size()) {
    const std::string& flag = arguments[i];
    if (flag != "--option" && flag != "--sequence" && std::ranges::find(seen, flag) != seen.end()) {
      return fail(ErrorKind::kInvalidInput, flag + ": given twice");
    }
    if (bool* const set = switched(out, flag)) {
      seen.emplace_back(flag);
      *set = true;
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
  if (request.sequences.empty()) {
    return fail(ErrorKind::kInvalidInput, "--sequence: missing");
  }
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
  if (request.locks && !request.threaded) {
    return fail(ErrorKind::kInvalidInput, "--locks: needs --threaded");
  }
  if (request.views && request.threaded) {
    return fail(ErrorKind::kInvalidInput, "--views: tracks in turn, not with --threaded");
  }
  return edge::check_options(request.options);
}

std::vector<double> session_offsets(std::span<const std::vector<double>> times) {
  std::vector<double> offsets;
  std::optional<double> end;
  double period = 1.0;
  for (const std::vector<double>& session : times) {
    double offset = 0.0;
    if (end.has_value() && !session.empty() && session.front() <= *end) {
      offset = *end + period - session.front();
    }
    offsets.push_back(offset);
    if (!session.empty()) {
      end = session.back() + offset;
      period = session.size() >= 2 ? median_interval(session) : 1.0;
    }
  }
  return offsets;
}

Result<eval::runs::Run> run(const Request& request) {
  if (const auto checked = check(request); !checked) {
    return std::unexpected(checked.error());
  }
  std::vector<Source> sources;
  for (const std::string& sequence : request.sequences) {
    auto source = request.dataset == "tum" ? tum_source(sequence, request.mode)
                                           : euroc_source(sequence, request.mode);
    if (!source) {
      return std::unexpected(source.error());
    }
    if (!sources.empty() && !same_camera(sources.front().camera, source->camera)) {
      return fail(ErrorKind::kInvalidInput, "--sequence: the sequences' cameras differ");
    }
    sources.push_back(*std::move(source));
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
  auto pipeline = edge::Pipeline::create(request.mode, sources.front().camera, request.options,
                                         std::move(words), threading_of(request));
  if (!pipeline) {
    return std::unexpected(pipeline.error());
  }
  eval::runs::Run out;
  out.dataset = request.dataset;
  for (const std::string& sequence : request.sequences) {
    out.sequence += (out.sequence.empty() ? "" : "+") + sequence_name(sequence);
  }
  out.mode = edge::name(request.mode);
  out.backend = "cpp";
  auto recorded = settings(request.options);
  if (!recorded) {
    return std::unexpected(recorded.error());
  }
  out.settings = *std::move(recorded);
  out.vocabulary = record;
  out.record_features = request.record_features;
  out.threaded = request.threaded;
  out.locks = request.locks;
  std::optional<eval::runs::FeatureWriter> features;
  if (request.record_features) {
    auto opened = eval::runs::FeatureWriter::open(request.out);
    if (!opened) {
      return std::unexpected(opened.error());
    }
    features = std::move(*opened);
  }
  if (const auto tracked = track(sources, *pipeline, request, out, features ? &*features : nullptr);
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
