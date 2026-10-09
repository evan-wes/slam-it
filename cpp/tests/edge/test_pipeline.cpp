/// @file
/// @brief Native tests for the pipeline (slam_it_all/edge/pipeline.hpp) on the exact scene seen
///        along a sideways path: RGB-D and stereo track every frame exactly, monocular starts
///        its map and keeps the path's direction, and empty frames lose the track.
#include <cmath>
#include <cstdint>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it_all/edge/pipeline.hpp"

namespace {

namespace edge = slam_it_all::edge;
namespace scene = slam_it_all::testing::scene;
using slam_it_all::testing::expect;
using slam_it_all::testing::is_invalid_input;

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
  return {.min_init_features = 20,
          .min_init_features_depth = 20,
          .min_init_depth_points = 20,
          .init_min_matches = 20};
}

/// @brief Run the path through a pipeline.
/// @param mode The mode.
/// @param results Where each frame's result goes.
/// @return The pipeline, or an error.
slam_it_all::foundations::Result<edge::Pipeline> run(edge::Mode mode,
                                                     std::vector<edge::FrameResult>& results) {
  auto pipeline = edge::Pipeline::create(mode, scene::camera(), small_scene());
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

/// @brief Check losing the track, and the checks.
/// @param failures Running failure count.
void check_lost(int& failures) {
  auto pipeline = edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), small_scene());
  if (!pipeline || !pipeline->track(scene::frame_at(along(0), true))) {
    expect(false, "the pipeline starts", failures);
    return;
  }
  const edge::Frame empty =
      edge::make_frame(0.0, {.keypoints = {}, .descriptors = {}, .scale_factor = 1.2, .levels = 8},
                       {}, {}, {})
          .value();
  bool ok = true;
  for (std::int64_t i = 0; ok && i < edge::Options{}.lost_frames; ++i) {
    const auto result = pipeline->track(empty);
    ok = result.has_value() && !result->pose;
  }
  expect(ok && pipeline->state() == edge::State::kLost,
         "lost_frames frames in a row untracked lose the track", failures);
  edge::Camera monocular = scene::camera();
  monocular.baseline = 0.0;
  expect(is_invalid_input(edge::Pipeline::create(edge::Mode::kRgbd, monocular, edge::Options{})),
         "RGB-D without a baseline is refused", failures);
}

/// @brief The test body.
/// @return The exit code.
int run_tests() {
  int failures = 0;
  check_depth_modes(failures);
  check_monocular(failures);
  check_lost(failures);
  return slam_it_all::testing::finish("edge/pipeline", failures);
}

}  // namespace

int main() { return slam_it_all::testing::guarded("edge/pipeline", run_tests); }
