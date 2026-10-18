/// @file
/// @brief Native tests for the pipeline (slam_it/edge/pipeline.hpp) on the exact scene seen
///        along a sideways path: RGB-D and stereo track every frame exactly, monocular starts
///        its map and keeps the path's direction, and empty frames lose the track, the next
///        frame starting a new map, as a new session does; relocalising, a kept map is lost in
///        time and a small one in frames, and a frame the fallback cannot reach is relocalised.
///        Tracking on the view a writer would publish decides as on the map, and threaded, the
///        views published are a part of the map, their sizes kept; after a correction the next
///        frame follows its keyframe, and a keyframe queued across it is placed by its
///        reference.
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <numbers>
#include <optional>
#include <string>
#include <tuple>
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
  edge::Options out{
      .initialization = {.min_features = 20, .min_features_depth = 20, .min_matches = 20}};
  out.relocalization.enabled =
      false;                  // A2's loss rule; relocalizing() turns it on, with a vocabulary
  out.loops.enabled = false;  // the keyframe database's other user; relocalizing() turns it on
  return out;
}

/// @brief Run the path through a pipeline.
/// @param mode The mode.
/// @param results Where each frame's result goes.
/// @param threading How the steps run, in turn: inline, or each frame on a view.
/// @return The pipeline, or an error.
slam_it::foundations::Result<edge::Pipeline> run(
    edge::Mode mode, std::vector<edge::FrameResult>& results,
    edge::Threading threading = edge::Threading::kInline) {
  auto pipeline =
      edge::Pipeline::create(mode, scene::camera(), small_scene(), std::nullopt, threading);
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

/// @brief Whether two frames' results decided alike: state, map, pose, matches, keyframe, how
///        they were placed and what became of each keypoint.
/// @param a One.
/// @param b The other.
/// @return Whether they did.
bool decided_alike(const edge::FrameResult& a, const edge::FrameResult& b) {
  return a.state == b.state && a.map == b.map && a.pose == b.pose && a.tracked == b.tracked &&
         a.keyframe == b.keyframe && a.asked == b.asked &&
         a.summary.placed_by == b.summary.placed_by && a.fates.fate == b.fates.fate &&
         a.fates.point == b.fates.point;
}

/// @brief Whether two maps hold the same keyframes, points, observations and counts.
/// @param a One, if any.
/// @param b The other, if any.
/// @return Whether both exist and do.
bool same_map(const std::optional<edge::Map>& a, const std::optional<edge::Map>& b) {
  if (!a || !b) {
    return false;
  }
  return a->kf_id == b->kf_id && a->kf_pose == b->kf_pose && a->kf_parent == b->kf_parent &&
         a->kp_point == b->kp_point && a->pt_id == b->pt_id && a->pt_xyz == b->pt_xyz &&
         a->pt_visible == b->pt_visible && a->pt_found == b->pt_found;
}

/// @brief Check that tracking each frame on the view a writer would publish then decides as on
///        the whole map (`Threading::kInlineView`): the same frames and the same map, and
///        nothing published in turn.
/// @param failures Running failure count.
void check_views_decide_as_the_map(int& failures) {
  for (const edge::Mode mode : {edge::Mode::kRgbd, edge::Mode::kStereo, edge::Mode::kMonocular}) {
    std::vector<edge::FrameResult> on_map;
    std::vector<edge::FrameResult> on_view;
    auto whole = run(mode, on_map);
    auto viewed = run(mode, on_view, edge::Threading::kInlineView);
    bool alike = whole && viewed && on_map.size() == on_view.size() &&
                 same_map(whole->map(), viewed->map()) && viewed->completed().empty();
    for (std::size_t i = 0; alike && i < on_map.size(); ++i) {
      alike = decided_alike(on_map[i], on_view[i]);
    }
    expect(alike, "each frame tracked on a view decides as on the map, and makes the same map",
           failures);
  }
}

/// @brief A frame with no keypoints.
/// @return The frame.
edge::Frame empty_frame() {
  return edge::make_frame(0.0,
                          {.keypoints = {}, .descriptors = {}, .scale_factor = 1.2, .levels = 8},
                          {}, {}, {})
      .value();
}

/// @brief Start a pipeline's map on the path's first frame, RGB-D.
/// @param pipeline The pipeline, or the error that made none.
/// @return The first frame's result, or the error.
slam_it::foundations::Result<edge::FrameResult> start(
    slam_it::foundations::Result<edge::Pipeline>& pipeline) {
  if (!pipeline) {
    return std::unexpected(pipeline.error());
  }
  return pipeline->track(scene::frame_at(along(0), true));
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

/// @brief An RGB-D pipeline at some settings, its map started on the path's first frame.
/// @param options The settings.
/// @param failures Running failure count, one more if the pipeline did not start map 0.
/// @return The pipeline and its first frame's result; none if it did not start map 0.
std::optional<std::pair<edge::Pipeline, edge::FrameResult>> started_rgbd(
    const edge::Options& options, int& failures) {
  auto pipeline = edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), options, std::nullopt);
  const auto started = start(pipeline);
  if (!started || started->map != 0) {
    expect(false, "the pipeline starts map 0", failures);
    return std::nullopt;
  }
  return std::pair{*std::move(pipeline), *started};
}

/// @brief Check losing the track: the map of one keyframe is reset with `loss.reset_keyframes` 1,
///        the rule's boundary, or kept with 0, and the next frame starts map 1, its ids following
///        map 0's.
/// @param failures Running failure count.
void check_lost(int& failures) {
  for (const std::int64_t reset_keyframes : {std::int64_t{1}, std::int64_t{0}}) {
    edge::Options options = small_scene();
    options.loss.reset_keyframes = reset_keyframes;
    auto run = started_rgbd(options, failures);
    if (!run) {
      return;
    }
    auto& [pipeline, started] = *run;
    std::optional<edge::FrameResult> lost;
    for (std::int64_t i = 0; i < options.loss.lost_frames; ++i) {
      lost = pipeline.track(empty_frame()).value();
    }
    const bool reset = reset_keyframes > 0;
    expect(lost && lost->state == edge::State::kLost && !lost->pose && lost->map == 0 &&
               lost->summary.lost && lost->summary.lost->reset == reset &&
               lost->summary.lost->keyframes == 1 &&
               pipeline.state() == edge::State::kNotInitialized && !pipeline.map(),
           "loss.lost_frames frames in a row untracked lose the track and close the map", failures);
    const auto restarted = pipeline.track(scene::frame_at(along(1), true));
    bool made_after = restarted.has_value() && restarted->map == 1 && restarted->keyframe;
    for (std::size_t j = 0; made_after && j < restarted->fates.point.size(); ++j) {
      made_after = restarted->fates.fate[j] != edge::Fate::kMade ||
                   restarted->fates.point[j] >= started.tracked;
    }
    expect(
        made_after && lived(pipeline.lifetimes(), started.tracked, reset, options.loss.lost_frames),
        "the next frame starts map 1, the run's ids following map 0's", failures);
  }
  edge::Camera monocular = scene::camera();
  monocular.baseline = 0.0;
  expect(is_invalid_input(
             edge::Pipeline::create(edge::Mode::kRgbd, monocular, edge::Options{}, std::nullopt)),
         "RGB-D without a baseline is refused", failures);
}

/// @brief Check starting a session: the map of one keyframe is reset with
///        `loss.session_keyframes` 2 (fewer than 2) or kept with 1, nothing is closed with no
///        map tracked, and the session's first frame starts map 1, a reset map's lives ending
///        on it and its frames leaving the trajectory.
/// @param failures Running failure count.
void check_sessions(int& failures) {
  for (const std::int64_t session_keyframes : {std::int64_t{2}, std::int64_t{1}}) {
    edge::Options options = small_scene();
    options.loss.session_keyframes = session_keyframes;
    auto run = started_rgbd(options, failures);
    if (!run) {
      return;
    }
    auto& [pipeline, started] = *run;
    const bool reset = session_keyframes > 1;
    const auto closed = pipeline.start_session();
    const auto again = pipeline.start_session();
    expect(closed && *closed && (*closed)->keyframes == 1 && (*closed)->points == started.tracked &&
               (*closed)->reset == reset && pipeline.state() == edge::State::kNotInitialized &&
               !pipeline.map() && again && !*again,
           "a session started closes the map tracked, and nothing with none", failures);
    const auto restarted = pipeline.track(scene::frame_at(along(1), true));
    const std::vector<std::int64_t> maps =
        reset ? std::vector<std::int64_t>{1} : std::vector<std::int64_t>{0, 1};
    expect(restarted && restarted->map == 1 && restarted->keyframe &&
               pipeline.trajectory().maps == maps &&
               lived(pipeline.lifetimes(), started.tracked, reset, 1),
           "the session's first frame starts map 1", failures);
  }
}

/// @brief The settings relocalising, a map above `loss.reset_keyframes` kept: the scene's, with
///        `relocalization.enabled` and `loops.enabled` on.
/// @param reset_keyframes The most keyframes a lost map is reset with.
/// @return The settings.
edge::Options relocalizing(std::int64_t reset_keyframes) {
  edge::Options out = small_scene();
  out.relocalization.enabled = true;
  out.loops.enabled = true;
  out.loss.reset_keyframes = reset_keyframes;
  return out;
}

/// @brief An empty frame at a time.
/// @param timestamp The time.
/// @return The frame.
edge::Frame empty_at(double timestamp) {
  edge::Frame out = empty_frame();
  out.timestamp = timestamp;
  return out;
}

/// @brief Start the scene's map with a vocabulary of one word, then track empty frames until
///        the track is lost (Python's `_failures`): each frame before the last keeps it, the
///        last loses it, and each tried relocalisation or not as asked.
/// @param options The settings.
/// @param step The empty frames' interval, in seconds.
/// @param lost_on The failure the track is lost on.
/// @param relocalizes Whether each failed frame tries relocalisation.
/// @return Whether every frame went so.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (step, lost_on), Python's order
bool loses_the_track(const edge::Options& options, double step, std::int64_t lost_on,
                     bool relocalizes) {
  auto pipeline =
      edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), options, scene::one_word());
  const auto started = start(pipeline);
  bool ok = started && started->keyframe;
  for (std::int64_t k = 0; ok && k < lost_on; ++k) {
    const auto failed = pipeline->track(empty_at(step * static_cast<double>(k + 1)));
    const edge::State expected = k + 1 < lost_on ? edge::State::kTracking : edge::State::kLost;
    ok = failed && failed->state == expected &&
         failed->summary.relocalization.has_value() == relocalizes;
  }
  return ok;
}

/// @brief Check the loss rules relocalising (Python's
///        `test_a_large_map_is_lost_in_time_and_a_small_one_in_frames`): a map above
///        `loss.reset_keyframes` is lost once more than `loss.relocalize_seconds` have passed since
///        the first failed frame (at 0.25 s a frame, on the fourteenth: 3.25 s, where the
///        thirteenth's 3 s is not more), one of `loss.reset_keyframes` or fewer on the
///        `loss.lost_frames`-th failure, whatever the time; with `loss.relocalize_seconds` 0, a
///        map above it on its first failed frame; each failed frame tried relocalisation.
/// @param failures Running failure count.
void check_lost_relocalizing(int& failures) {
  struct Case {
    std::int64_t reset_keyframes;
    double step;
    std::int64_t lost_on;
    double seconds;
  };
  for (const Case c : {Case{.reset_keyframes = 0, .step = 0.25, .lost_on = 14, .seconds = 3.0},
                       Case{.reset_keyframes = 1, .step = 0.25, .lost_on = 5, .seconds = 3.0},
                       Case{.reset_keyframes = 1, .step = 0.001, .lost_on = 5, .seconds = 3.0},
                       Case{.reset_keyframes = 0, .step = 0.25, .lost_on = 1, .seconds = 0.0}}) {
    edge::Options options = relocalizing(c.reset_keyframes);
    options.loss.relocalize_seconds = c.seconds;
    expect(loses_the_track(options, c.step, c.lost_on, true),
           c.reset_keyframes == 0
               ? "a kept map is lost past loss.relocalize_seconds from the first failed frame, "
                 "or on it with 0"
               : "a small map is lost on the loss.lost_frames-th failure, whatever the time",
           failures);
  }
}

/// @brief Check loop closing alone (Python's
///        `test_off_a_kept_map_is_lost_after_lost_frames_and_nothing_relocalizes`, its
///        loop-closing case): the keyframe database is kept, but no failed frame tries
///        relocalisation, and a kept map is lost on the `loss.lost_frames`-th failure (B3's
///        review panel, run 2, A20).
/// @param failures Running failure count.
void check_closing_loops_alone(int& failures) {
  edge::Options options = relocalizing(0);
  options.relocalization.enabled = false;
  expect(loses_the_track(options, 0.001, options.loss.lost_frames, false),
         "closing loops alone, no failed frame relocalises", failures);
}

/// @brief Check that a frame the local fallback cannot reach is placed by relocalisation in
///        the same map, at its pose: after the path, 30 cm to the right, and an empty frame, the
///        camera is back where the map began, 25 px away, past the motion model's narrowed
///        window.
/// @param failures Running failure count.
void check_relocalized(int& failures) {
  edge::Options options = relocalizing(0);
  options.tracking.pnp_min_inliers = 1'000'000;  // the local fallback out of reach
  options.tracking.motion_window = 5.0;  // 10 px widened: the 25 px back to the start out of reach
  auto pipeline =
      edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), options, scene::one_word());
  bool tracked = pipeline.has_value();
  for (std::int64_t i = 0; tracked && i < kFrames; ++i) {
    edge::Frame frame = scene::frame_at(along(i), true);
    frame.timestamp = static_cast<double>(i);
    const auto result = pipeline->track(frame);
    tracked = result && result->pose;
  }
  if (!tracked) {
    expect(false, "the path is tracked", failures);
    return;
  }
  const edge::FrameResult failed = pipeline->track(empty_at(static_cast<double>(kFrames))).value();
  edge::Frame back = scene::frame_at(along(0), true);
  back.timestamp = static_cast<double>(kFrames + 1);
  const std::optional<edge::FrameResult> found = pipeline->track(back).value();
  expect(!failed.pose, "the empty frame fails", failures);
  expect(found && found->map == 0 && found->state == edge::State::kTracking &&
             found->summary.placed_by == edge::Placement::kRelocalization && found->pose &&
             (*found->pose - along(0)).cwiseAbs().maxCoeff() < 1e-6,
         "a frame out of the fallback's reach is relocalised in its map, at its pose", failures);
}

/// @brief The scene's settings, a first keyframe's reference ratio raised to the others' (from
///        0.4), so that `thinning`'s frames ask for a keyframe.
/// @return The settings.
edge::Options asking() {
  edge::Options out = small_scene();
  out.keyframes.first_ref_ratio = out.keyframes.ref_ratio;
  return out;
}

/// @brief A frame of the path, RGB-D, at its time, that sees the scene's points from `2 i` on
///        (24 at most dropped): its tracking thins as the path goes, so that with `asking`'s
///        settings it asks for a keyframe once under `keyframes.ref_ratio` of its reference's
///        points (from the eighth).
/// @param i The frame.
/// @return The frame.
edge::Frame thinning(std::int64_t i) {
  edge::Frame full = scene::frame_at(along(i), true);
  const auto dropped = static_cast<std::ptrdiff_t>(std::min<std::int64_t>(2 * i, 24));
  auto& keypoints = full.features.keypoints;
  auto& descriptors = full.features.descriptors;
  keypoints.erase(keypoints.begin(), keypoints.begin() + dropped);
  descriptors.erase(descriptors.begin(), descriptors.begin() + dropped);
  full.bearings.erase(full.bearings.begin(), full.bearings.begin() + dropped);
  full.depth.erase(full.depth.begin(), full.depth.begin() + dropped);
  full.right.erase(full.right.begin(), full.right.begin() + dropped);
  return edge::make_frame(static_cast<double>(i), std::move(full.features),
                          std::move(full.bearings), std::move(full.depth), std::move(full.right))
      .value();
}

/// @brief Track the path's thinning frames through a pipeline.
/// @param pipeline The pipeline.
/// @param from The first frame.
/// @param to The frame after the last.
/// @param results Where each frame's result goes.
/// @return Whether every frame was tracked without an error.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (from, to), a range
bool track_path(edge::Pipeline& pipeline, std::int64_t from, std::int64_t to,
                std::vector<edge::FrameResult>& results) {
  for (std::int64_t i = from; i < to; ++i) {
    auto result = pipeline.track(thinning(i));
    if (!result) {
      return false;
    }
    results.push_back(*std::move(result));
  }
  return true;
}

/// @brief A check's message, the threading named.
/// @param what The check.
/// @param threading How the steps ran.
/// @return The message.
std::string threaded_check(const char* what, edge::Threading threading) {
  return std::string(what) + (threading == edge::Threading::kLocks ? " (locks)" : " (view)");
}

/// @brief Check the path threaded, RGB-D (`tests/edge/test_pipeline_threads.py`): every frame
///        tracked on the true path; each keyframe given mapped once, a completion on its frame
///        with its schedule; the lives count the first keyframe and each mapped one; the
///        trajectory places every frame, in order; a finished run tracks no more; and inline,
///        nothing completes later.
/// @param threading Our design or the baseline.
/// @param failures Running failure count.
void check_threaded(edge::Threading threading, int& failures) {
  auto pipeline =
      edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), asking(), std::nullopt, threading);
  std::vector<edge::FrameResult> results;
  if (!pipeline || !track_path(*pipeline, 0, kFrames, results) || !pipeline->finish()) {
    expect(false, "a threaded pipeline tracks the path and finishes", failures);
    return;
  }
  double worst = 0.0;
  bool every = std::cmp_equal(results.size(), kFrames);
  std::vector<std::int64_t> given;
  for (std::size_t i = 0; every && i < results.size(); ++i) {
    const std::optional<Eigen::Matrix4d>& pose = results[i].pose;
    every = pose.has_value() && results[i].map_seconds == 0.0 && results[i].loop_seconds == 0.0;
    if (pose) {
      worst = std::max(worst, (*pose - along(static_cast<std::int64_t>(i))).cwiseAbs().maxCoeff());
    }
    if (i > 0 && results[i].keyframe) {
      given.push_back(static_cast<std::int64_t>(i));
    }
  }
  expect(every && worst < 1e-6,
         threaded_check("threaded, every frame is tracked on the true path", threading).c_str(),
         failures);
  std::vector<std::int64_t> mapped;
  bool scheduled = true;
  const std::vector<edge::Completion> published = pipeline->completed();
  for (const edge::Completion& done : published) {
    if (done.mapping) {
      mapped.push_back(done.frame);
      scheduled = scheduled && done.schedule.has_value() && done.map_seconds.has_value();
    }
  }
  std::ranges::sort(mapped);
  expect(!given.empty() && mapped == given && scheduled,
         threaded_check("each keyframe given is mapped once, a completion on its frame", threading)
             .c_str(),
         failures);
  const edge::Lifetimes lives = pipeline->lifetimes();
  const edge::Trajectory trajectory = pipeline->trajectory();
  std::vector<edge::ViewSize> views;
  for (const edge::Completion& done : published) {
    if (done.view) {
      views.push_back(*done.view);
    }
  }
  const bool sized = std::ranges::all_of(views, [&](const edge::ViewSize& v) {
    return v.keyframes >= 1 && v.points >= 1 && v.bytes > 0 &&
           std::cmp_less_equal(v.keyframes, lives.keyframe_times.size()) &&
           std::cmp_less_equal(v.points, lives.points.size());
  });
  // Ours publishes a view at the start and as each keyframe is mapped; the baseline none.
  expect(
      threading == edge::Threading::kLocks ? views.empty() : views.size() > given.size() && sized,
      threaded_check("the views published are a part of the map, sized", threading).c_str(),
      failures);
  expect(
      lives.keyframe_times.size() == 1 + given.size() &&
          trajectory.timestamps.size() == results.size() &&
          std::ranges::adjacent_find(trajectory.timestamps, std::greater_equal<>()) ==
              trajectory.timestamps.end(),
      threaded_check("the records hold every keyframe mapped and every frame, in order", threading)
          .c_str(),
      failures);
  expect(is_invalid_input(pipeline->track(scene::frame_at(along(0), true))),
         threaded_check("a finished run tracks no more", threading).c_str(), failures);
  std::vector<edge::FrameResult> inline_results;
  auto in_order = run(edge::Mode::kRgbd, inline_results);
  expect(in_order && in_order->finish() && in_order->completed().empty(),
         "inline, nothing completes later", failures);
}

/// @brief Check a stop held: while mapping is stopped, every keyframe tracking asks for is
///        refused (`test_a_stop_refuses_every_keyframe_asked`).
/// @param threading Our design or the baseline.
/// @param failures Running failure count.
void check_a_stop_refuses_keyframes(edge::Threading threading, int& failures) {
  auto pipeline =
      edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), asking(), std::nullopt, threading);
  std::vector<edge::FrameResult> before;
  std::vector<edge::FrameResult> held;
  bool tracked = pipeline && track_path(*pipeline, 0, kFrames / 2, before);
  if (tracked) {
    const edge::threads::Writer::Stopped stop(*pipeline->writer());
    tracked = track_path(*pipeline, kFrames / 2, kFrames, held);
  }
  tracked = tracked && pipeline->finish();
  const bool asked = std::ranges::any_of(held, [](const auto& r) { return r.asked; });
  const bool refused = std::ranges::all_of(
      held, [](const auto& r) { return !r.asked || (r.refused && !r.keyframe); });
  expect(
      tracked && asked && refused,
      threaded_check("while mapping is stopped, each keyframe asked is refused", threading).c_str(),
      failures);
}

/// @brief A rigid move of the world, as a loop's correction moves a map: 20 degrees about the
///        vertical and 0.5 m along x, far past the motion model's search.
/// @return The move, `G`.
Eigen::Matrix4d correction() {
  Eigen::Matrix4d g = Eigen::Matrix4d::Identity();
  g.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(20.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  g(0, 3) = 0.5;
  return g;
}

/// @brief Move a map's world by `g`, as a loop's correction moves it rigidly: a keyframe's
///        `T_cam_world` becomes `T_cam_world G⁻¹`, a point `x` becomes `G x`, its viewing
///        direction turned with it.
/// @param m The map, changed in place.
/// @param g The move.
/// @return True: the map was corrected.
slam_it::foundations::Result<bool> move_world(edge::Map& m, const Eigen::Matrix4d& g) {
  const Eigen::Matrix4d inverse = g.inverse();
  const Eigen::Matrix3d rotation = g.topLeftCorner<3, 3>();
  const Eigen::Vector3d shift = g.topRightCorner<3, 1>();
  for (Eigen::Matrix4d& pose : m.kf_pose) {
    pose = Eigen::Matrix4d(pose * inverse);
  }
  for (Eigen::Vector3d& x : m.pt_xyz) {
    x = (rotation * x) + shift;
  }
  for (Eigen::Vector3d& n : m.pt_normal) {
    n = rotation * n;
  }
  return true;
}

/// @brief The view tracking would read now: the baseline's under the map's lock, which its
///        tracking holds, as mapping changes the map in place.
/// @param owner The map's owner.
/// @return The view.
edge::View view_now(const edge::MapOwner& owner) {
  const std::unique_lock<std::mutex> lock = owner.locked();
  return owner.view();
}

/// @brief Check a correction followed
/// (`test_after_a_correction_the_next_frame_follows_its_keyframe`,
///        B3b's review panel, A1): the map corrected under a stop, the view published with it
///        carries the new count; the next frame, tracked on the one view it reads, moves its
///        last frame with its keyframe first, so the motion model places it, on the path in
///        the corrected world.
/// @param threading Our design or the baseline.
/// @param failures Running failure count.
void check_a_correction_followed(edge::Threading threading, int& failures) {
  auto pipeline =
      edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), asking(), std::nullopt, threading);
  std::vector<edge::FrameResult> results;
  bool ok = pipeline && track_path(*pipeline, 0, kFrames / 2, results);
  const Eigen::Matrix4d g = correction();
  if (ok) {
    edge::MapOwner& owner = pipeline->owner();
    const std::int64_t before = view_now(owner).corrections;
    const auto corrected = owner.apply_correction([&g](edge::Map& m) { return move_world(m, g); });
    const edge::View after = view_now(owner);
    ok = corrected && after.corrections == before + 1 && after.corrections == owner.corrections();
  }
  ok = ok && track_path(*pipeline, kFrames / 2, (kFrames / 2) + 1, results) && pipeline->finish();
  const edge::FrameResult* next = ok ? &results.back() : nullptr;
  const Eigen::Matrix4d expected = along(kFrames / 2) * g.inverse();
  expect(
      next != nullptr && next->summary.placed_by == edge::Placement::kMotion && next->pose &&
          (*next->pose - expected).cwiseAbs().maxCoeff() < 1e-6,
      threaded_check("after a correction, the next frame follows its keyframe", threading).c_str(),
      failures);
}

/// @brief The scene's `asking` settings with local BA and keyframe culling off: a keyframe keeps
///        the pose it is inserted at, unless a correction moves the map.
/// @return The settings.
edge::Options still() {
  edge::Options out = asking();
  out.adjustment.min_keyframes = 1000;
  out.mapping.redundant_observers = 1000;
  return out;
}

/// @brief Check a keyframe queued across a correction
///        (`test_a_keyframe_queued_across_a_correction_is_placed_by_its_reference`, B3b's review
///        panel, A2): with mapping held off, the keyframes given wait while the map is
///        corrected; once mapped, each sits where its reference keyframe now is. With local BA
///        and culling off (`still`), every keyframe keeps the pose it was given: on the path, in
///        the corrected world.
/// @param threading Our design or the baseline.
/// @param failures Running failure count.
void check_a_queued_keyframe_placed(edge::Threading threading, int& failures) {
  auto pipeline =
      edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), still(), std::nullopt, threading);
  std::vector<edge::FrameResult> results;
  std::vector<edge::FrameResult> held;
  bool ok = pipeline && track_path(*pipeline, 0, kFrames / 2, results);
  const Eigen::Matrix4d g = correction();
  if (ok) {
    const edge::threads::Writer::Held hold(*pipeline->writer());  // the keyframes given wait
    ok = track_path(*pipeline, kFrames / 2, 3 * kFrames / 4, held) &&
         pipeline->owner()
             .apply_correction([&g](edge::Map& m) { return move_world(m, g); })
             .has_value();  // a stop inside the hold does not wait for itself
  }
  ok = ok && track_path(*pipeline, 3 * kFrames / 4, kFrames, results) && pipeline->finish();
  const bool waited = std::ranges::any_of(held, [](const auto& r) { return r.keyframe; });
  bool placed = false;
  if (const std::optional<edge::Map>& mapped = ok ? pipeline->map() : std::nullopt; mapped) {
    const edge::Map& m = *mapped;
    const Eigen::Matrix4d inverse = g.inverse();
    placed = true;
    for (std::size_t k = 0; placed && k < m.kf_id.size(); ++k) {
      const auto i = static_cast<std::int64_t>(m.kf_time[k]);  // `thinning(i)`'s time is i
      placed = (m.kf_pose[k] - (along(i) * inverse)).cwiseAbs().maxCoeff() < 1e-6;
    }
  }
  expect(
      waited && placed,
      threaded_check("a keyframe queued across a correction is placed by its reference", threading)
          .c_str(),
      failures);
}

/// @brief Check a lost map threaded (`test_a_lost_map_settles_the_threads_then_closes`): the
///        threads settle, then it closes as inline, kept, and the next frame starts map 1.
/// @param failures Running failure count.
void check_lost_threaded(int& failures) {
  edge::Options options = small_scene();
  options.loss.reset_keyframes = 0;
  std::vector<std::vector<std::tuple<edge::State, std::optional<std::int64_t>, bool, std::int64_t>>>
      outcomes;
  for (const edge::Threading threading :
       {edge::Threading::kInline, edge::Threading::kView, edge::Threading::kLocks}) {
    auto pipeline = edge::Pipeline::create(edge::Mode::kRgbd, scene::camera(), options,
                                           std::nullopt, threading);
    std::vector<edge::FrameResult> results;
    bool ok = pipeline && track_path(*pipeline, 0, 1, results);
    for (std::int64_t i = 1; ok && i <= options.loss.lost_frames; ++i) {
      auto lost = pipeline->track(empty_at(static_cast<double>(i)));
      ok = lost.has_value();
      if (ok) {
        results.push_back(*std::move(lost));
      }
    }
    ok = ok && track_path(*pipeline, 1, 2, results) && pipeline->finish();
    if (!ok) {
      expect(false, "a lost map's run goes on", failures);
      return;
    }
    auto& outcome = outcomes.emplace_back();
    for (const edge::FrameResult& r : results) {
      outcome.emplace_back(r.state, r.map, r.summary.lost.has_value(),
                           r.summary.lost ? r.summary.lost->keyframes : -1);
    }
  }
  expect(outcomes[0] == outcomes[1] && outcomes[0] == outcomes[2] &&
             std::get<0>(outcomes[1].back()) == edge::State::kTracking &&
             std::get<1>(outcomes[1].back()) == 1,
         "threaded, a lost map closes as inline, and the next frame starts map 1", failures);
}

/// @brief The test body.
/// @return The exit code.
int run_tests() {
  int failures = 0;
  check_depth_modes(failures);
  check_monocular(failures);
  check_lost(failures);
  check_sessions(failures);
  check_lost_relocalizing(failures);
  check_closing_loops_alone(failures);
  check_relocalized(failures);
  check_views_decide_as_the_map(failures);
  for (const edge::Threading threading : {edge::Threading::kView, edge::Threading::kLocks}) {
    check_threaded(threading, failures);
    check_a_stop_refuses_keyframes(threading, failures);
    check_a_correction_followed(threading, failures);
    check_a_queued_keyframe_placed(threading, failures);
  }
  check_lost_threaded(failures);
  return slam_it::testing::finish("edge/pipeline", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/pipeline", run_tests); }
