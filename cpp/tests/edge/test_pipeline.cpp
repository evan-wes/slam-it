/// @file
/// @brief Native tests for the pipeline (slam_it/edge/pipeline.hpp) on the exact scene seen
///        along a sideways path: RGB-D and stereo track every frame exactly, monocular starts
///        its map and keeps the path's direction, and empty frames lose the track, the next
///        frame starting a new map.
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it/edge/pipeline.hpp"

namespace {

namespace edge = slam_it::edge;
namespace scene = slam_it::testing::scene;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief The path's frames.
constexpr std::int64_t kFrames = 16;

/// @brief The path: the camera moves 2 cm right each frame.
/// @param i The frame.
/// @return Its `T_cam_world`.
Eigen::Matrix4d along(std::int64_t i) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out(0, 3) = -0.02 * static_cast<double>(i);
  return out;
}

/// @brief The settings for a 60-point scene: the counts ORB-SLAM sizes for hundreds lowered.
/// @return The settings.
edge::Options small_scene() {
  edge::Options out{.min_init_features = 20, .min_init_features_depth = 20, .init_min_matches = 20};
  out.relocalize = false;  // the scene's words would need a vocabulary; A2's loss rule holds
  return out;
}

/// @brief Run the path through a pipeline.
/// @param mode The mode.
/// @param results Where each frame's result goes.
/// @return The pipeline, or an error.
slam_it::foundations::Result<edge::Pipeline> run(edge::Mode mode,
                                                 std::vector<edge::FrameResult>& results) {
  auto pipeline = edge::Pipeline::create(mode, scene::camera(), small_scene(), std::nullopt);
  if (!pipeline) {
    return pipeline;
  }
  for (std::int64_t i = 0; i < kFrames; ++i) {
    edge::Frame frame = scene::frame_at(along(i), mode != edge::Mode::kMonocular);
    frame.timestamp = static_cast<double>(i);
    auto result = pipeline->track(frame);
    if (!result) {
      return std::unexpected(std::move(result.error()));
    }
    results.push_back(*result);
  }
  return pipeline;
}

/// @brief Check RGB-D and stereo: every frame tracked, on the true path.
/// @param failures Running failure count.
void check_depth_modes(int& failures) {
  for (const edge::Mode mode : {edge::Mode::kRgbd, edge::Mode::kStereo}) {
    std::vector<edge::FrameResult> results;
    const auto pipeline = run(mode, results);
    double worst = 0.0;
    bool every = pipeline.has_value() && std::cmp_equal(results.size(), kFrames);
    for (std::size_t i = 0; every && i < results.size(); ++i) {
      if (const auto& pose = results[i].pose) {
        worst =
            std::max(worst, (*pose - along(static_cast<std::int64_t>(i))).cwiseAbs().maxCoeff());
      } else {
        every = false;
      }
    }
    expect(
        every && worst < 1e-6 && pipeline->state() == edge::State::kTracking && results[0].keyframe,
        "with depth, every frame is tracked on the true path", failures);
  }
}

/// @brief Check monocular: the map starts once there is parallax, and every later frame is
///        tracked along the path's direction.
/// @param failures Running failure count.
void check_monocular(int& failures) {
  std::vector<edge::FrameResult> results;
  const auto pipeline = run(edge::Mode::kMonocular, results);
  std::size_t first = 0;
  while (first < results.size() && !results[first].pose) {
    ++first;
  }
  bool along_path = pipeline.has_value() && first < results.size() - 4;
  const Eigen::Vector3d last =
      -results.back().pose.value_or(Eigen::Matrix4d::Identity()).topRightCorner<3, 1>();
  for (std::size_t i = first; along_path && i < results.size(); ++i) {
    if (const auto& pose = results[i].pose) {
      const Eigen::Vector3d centre = -pose->topRightCorner<3, 1>();
      const double ratio = static_cast<double>(i) / static_cast<double>(results.size() - 1);
      along_path = (centre - (ratio * last)).norm() < 1e-6 * last.norm();
    } else {
      along_path = false;
    }
  }
  expect(along_path, "monocular starts its map and tracks the path's direction", failures);
}

/// @brief A frame with no keypoints.
/// @return The frame.
edge::Frame empty_frame() {
  return edge::make_frame(0.0,
                          {.keypoints = {}, .descriptors = {}, .scale_factor = 1.2, .levels = 8},
                          {}, {}, {})
      .value();
}

/// @brief Whether the lives after a loss and a new map are right: map 0's keyframe and points
///        made on frame 0, ended as reset on the lost frame if `reset`, map 1's alive.
/// @param lives The pipeline's lifetimes.
/// @param first The points map 0 made.
/// @param reset Whether map 0 was reset.
/// @param lost_at The frame the track was lost on.
/// @return Whether they are.
bool lived(const edge::Lifetimes& lives, std::int64_t first, bool reset, std::int64_t lost_at) {
  const std::int64_t code = reset ? static_cast<std::int64_t>(edge::Removal::kReset) : -1;
  if (lives.keyframe_maps != std::vector<std::int64_t>{0, 1} ||
      lives.keyframe_lives[0] != edge::Life{0, reset ? lost_at : -1, code}) {
    return false;
  }
  for (std::size_t id = 0; id < lives.point_maps.size(); ++id) {
    const bool in_first = std::cmp_less(id, first);
    if (lives.point_maps[id] != (in_first ? 0 : 1) ||
        lives.point_lives[id][2] != (in_first ? code : -1)) {
      return false;
    }
  }
  return true;
}

/// @brief Check losing the track: the map of one keyframe is reset with `reset_keyframes` 1, the
///        rule's boundary, or kept with 0, and the next frame starts map 1, its ids following
///        map 0's.
/// @param failures Running failure count.
void check_lost(int& failures) {
  for (const std::int64_t reset_keyframes : {std::int64_t{1}, std::int64_t{0}}) {
    edge::Options options = small_scene();
    options.reset_keyframes = reset_keyframes;
    auto pipeline =
        edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), options, std::nullopt);
    const auto started =
        pipeline
            ? pipeline->track(scene::frame_at(along(0), true))
            : slam_it::foundations::Result<edge::FrameResult>(std::unexpected(pipeline.error()));
    if (!started || started->map != 0) {
      expect(false, "the pipeline starts map 0", failures);
      return;
    }
    std::optional<edge::FrameResult> lost;
    for (std::int64_t i = 0; i < options.lost_frames; ++i) {
      lost = pipeline->track(empty_frame()).value();
    }
    const bool reset = reset_keyframes > 0;
    expect(lost && lost->state == edge::State::kLost && !lost->pose && lost->map == 0 &&
               lost->summary.lost && lost->summary.lost->reset == reset &&
               lost->summary.lost->keyframes == 1 &&
               pipeline->state() == edge::State::kNotInitialized && !pipeline->map(),
           "lost_frames frames in a row untracked lose the track and close the map", failures);
    const auto restarted = pipeline->track(scene::frame_at(along(1), true));
    bool made_after = restarted.has_value() && restarted->map == 1 && restarted->keyframe;
    for (std::size_t j = 0; made_after && j < restarted->fates.point.size(); ++j) {
      made_after = restarted->fates.fate[j] != edge::Fate::kMade ||
                   restarted->fates.point[j] >= started->tracked;
    }
    expect(made_after && lived(pipeline->lifetimes(), started->tracked, reset, options.lost_frames),
           "the next frame starts map 1, the run's ids following map 0's", failures);
  }
  edge::Camera monocular = scene::camera();
  monocular.baseline = 0.0;
  expect(is_invalid_input(
             edge::Pipeline::create(edge::Mode::kRgbd, monocular, edge::Options{}, std::nullopt)),
         "RGB-D without a baseline is refused", failures);
}

/// @brief The test body.
/// @return The exit code.
int run_tests() {
  int failures = 0;
  check_depth_modes(failures);
  check_monocular(failures);
  check_lost(failures);
  return slam_it::testing::finish("edge/pipeline", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/pipeline", run_tests); }
