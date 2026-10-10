/// @file
/// @brief Implementation of the pipeline declared in slam_it_all/edge/pipeline.hpp.
#include "slam_it_all/edge/pipeline.hpp"

#include <chrono>
#include <vector>

#include "slam_it_all/edge/mapping.hpp"
#include "slam_it_all/edge/tracking.hpp"
#include "slam_it_all/foundations/se3.hpp"

namespace slam_it_all::edge {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

const char* name(State state) {
  switch (state) {
    case State::kNotInitialized:
      return "not_initialized";
    case State::kTracking:
      return "tracking";
    case State::kLost:
      return "lost";
    case State::kCount:
      break;
  }
  return "unknown";
}

const char* name(Placement placement) {
  switch (placement) {
    case Placement::kInitialization:
      return "initialization";
    case Placement::kMotion:
      return "motion";
    case Placement::kLocalPnp:
      return "local_pnp";
    case Placement::kCount:
      break;
  }
  return "unknown";
}

Result<Pipeline> Pipeline::create(Mode mode, Camera camera, Options options) {
  if (auto ok = check_setup(camera, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (mode != Mode::kMonocular && !(camera.baseline > 0.0)) {
    return fail(ErrorKind::kInvalidInput,
                std::string("camera.baseline: ") + name(mode) + " needs a positive baseline");
  }
  return Pipeline(mode, std::move(camera), options);
}

foundations::orb::Options Pipeline::extraction() const {
  return edge::extraction(options_, mode_ == Mode::kMonocular && state_ == State::kNotInitialized);
}

Result<FrameResult> Pipeline::track(const Frame& frame) {
  const auto start = std::chrono::steady_clock::now();
  Step step;
  if (state_ == State::kNotInitialized) {
    auto started = initialize(frame);
    if (!started) {
      return std::unexpected(std::move(started.error()));
    }
    step = std::move(*started);
  } else if (state_ == State::kTracking) {
    auto placed = track_frame(frame);
    if (!placed) {
      return std::unexpected(std::move(placed.error()));
    }
    step = std::move(*placed);
  }
  date();
  const std::chrono::duration<double> took = std::chrono::steady_clock::now() - start;
  const std::optional<Tracked>& tracked = step.tracked;
  return FrameResult{.timestamp = frame.timestamp,
                     .state = state_,
                     .pose = tracked ? std::optional<Eigen::Matrix4d>(tracked->pose) : std::nullopt,
                     .tracked = tracked ? inliers(*tracked) : 0,
                     .keyframe = step.keyframe,
                     .track_seconds = took.count() - step.map_seconds,
                     .map_seconds = step.map_seconds,
                     .summary = std::move(step.summary)};
}

Lifetimes Pipeline::lifetimes() const {
  Lifetimes out{
      .points = std::vector<Eigen::Vector3d>(point_lives_.size(), Eigen::Vector3d::Zero()),
      .point_lives = point_lives_,
      .keyframe_times = std::vector<double>(keyframe_lives_.size(), 0.0),
      .keyframe_poses =
          std::vector<Eigen::Matrix4d>(keyframe_lives_.size(), Eigen::Matrix4d::Identity()),
      .keyframe_lives = keyframe_lives_};
  for (std::size_t id = 0; id < point_lives_.size(); ++id) {
    if (point_lives_[id][1] >= 0) {
      out.points[id] = gone_points_[id];
    }
  }
  for (std::size_t id = 0; id < keyframe_lives_.size(); ++id) {
    if (keyframe_lives_[id][1] >= 0) {
      out.keyframe_times[id] = gone_times_[id];
      out.keyframe_poses[id] = gone_poses_[id];
    }
  }
  if (map_) {
    for (std::size_t p = 0; p < point_count(*map_); ++p) {
      out.points[static_cast<std::size_t>(map_->pt_id[p])] = map_->pt_xyz[p];
    }
    for (std::size_t k = 0; k < keyframe_count(*map_); ++k) {
      const auto id = static_cast<std::size_t>(map_->kf_id[k]);
      out.keyframe_times[id] = map_->kf_time[k];
      out.keyframe_poses[id] = map_->kf_pose[k];
    }
  }
  return out;
}

void Pipeline::date() {
  const std::int64_t frame = frames_++;
  if (!map_) {
    return;
  }
  Map& m = *map_;
  const auto points = static_cast<std::size_t>(m.next_pt_id);
  point_lives_.resize(points, Life{frame, -1, -1});
  gone_points_.resize(points, Eigen::Vector3d::Zero());
  const auto keyframes = static_cast<std::size_t>(m.next_kf_id);
  keyframe_lives_.resize(keyframes, Life{frame, -1, -1});
  gone_times_.resize(keyframes, 0.0);
  gone_poses_.resize(keyframes, Eigen::Matrix4d::Identity());
  for (std::size_t g = 0; g < m.gone_pt_id.size(); ++g) {
    const auto id = static_cast<std::size_t>(m.gone_pt_id[g]);
    point_lives_[id][1] = frame;
    point_lives_[id][2] = m.gone_pt_how[g];
    gone_points_[id] = m.gone_pt_xyz[g];
  }
  for (std::size_t g = 0; g < m.gone_kf_id.size(); ++g) {
    const auto id = static_cast<std::size_t>(m.gone_kf_id[g]);
    keyframe_lives_[id][1] = frame;
    keyframe_lives_[id][2] = static_cast<std::int64_t>(Removal::kCulled);
    gone_times_[id] = m.gone_kf_time[g];
    gone_poses_[id] = m.gone_kf_pose[g];
  }
  clear_gone(m);
}

Result<Pipeline::Step> Pipeline::initialize(const Frame& frame) {
  auto attempt = mode_ == Mode::kMonocular ? initialize_monocular(frame)
                                           : initialize_from_depth(frame, options_);
  if (!attempt) {
    return std::unexpected(std::move(attempt.error()));
  }
  Step step{.tracked = std::nullopt,
            .keyframe = false,
            .map_seconds = 0.0,
            .summary = {.placed_by = std::nullopt,
                        .init = std::move(attempt->summary),
                        .motion = std::nullopt,
                        .pnp = std::nullopt,
                        .local_map = std::nullopt,
                        .mapping = std::nullopt}};
  if (!attempt->started) {
    return step;
  }
  map_ = std::move(attempt->started->map);
  state_ = State::kTracking;
  Tracked tracked = attempt->started->tracked;
  if (auto ok = accept(frame, tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  step.tracked = std::move(tracked);
  step.keyframe = true;
  step.summary.placed_by = Placement::kInitialization;
  return step;
}

Result<Initialization> Pipeline::initialize_monocular(const Frame& frame) {
  const auto not_started = [](Outcome outcome, std::optional<std::int64_t> matches) {
    return Initialization{.started = std::nullopt,
                          .summary = {.outcome = outcome, .matches = matches, .solves = {}}};
  };
  if (std::cmp_less_equal(keypoint_count(frame), options_.min_init_features)) {
    reference_.reset();  // too few features: ORB-SLAM drops its initializer
    return not_started(Outcome::kTooFewFeatures, std::nullopt);
  }
  if (!reference_) {
    set_reference(frame);
    return not_started(Outcome::kReferenceSet, std::nullopt);
  }
  auto matched = match_for_initialization(*reference_, frame, previous_, options_);
  if (!matched) {
    return std::unexpected(std::move(matched.error()));
  }
  if (std::cmp_less(matched->size(), options_.init_min_matches)) {
    set_reference(frame);  // the views no longer overlap enough: start again
    return not_started(Outcome::kReferenceReplaced, static_cast<std::int64_t>(matched->size()));
  }
  // The search follows each matched keypoint (SearchForInitialization's vbPrevMatched).
  std::vector<std::array<std::int64_t, 2>> pairs;
  pairs.reserve(matched->size());
  for (const foundations::matching::Match& match : *matched) {
    previous_[static_cast<std::size_t>(match.first)] =
        frame.features.keypoints[static_cast<std::size_t>(match.second)].xy;
    pairs.push_back({match.first, match.second});
  }
  // Nothing keeps the reference: a later frame may have the parallax.
  return edge::initialize_monocular(camera_, *reference_, frame, pairs, options_);
}

void Pipeline::set_reference(const Frame& frame) {
  reference_ = frame;
  previous_.clear();
  for (const foundations::orb::Keypoint& k : frame.features.keypoints) {
    previous_.push_back(k.xy);
  }
}

Result<void> Pipeline::accept(const Frame& frame, const Tracked& tracked) {
  velocity_.reset();
  if (last_tracked_ && failures_ == 0) {
    auto inverse = foundations::se3::inverse(last_tracked_->pose);
    if (!inverse) {
      return std::unexpected(std::move(inverse.error()));
    }
    velocity_ = Eigen::Matrix4d(tracked.pose * *inverse);
  }
  last_frame_ = frame;
  last_tracked_ = tracked;
  failures_ = 0;
  return {};
}

void Pipeline::record_failure() {
  ++failures_;
  velocity_.reset();
  if (failures_ >= options_.lost_frames) {
    state_ = State::kLost;
  }
}

Result<std::pair<std::optional<Tracked>, FrameSummary>> Pipeline::place(const Map& m,
                                                                        const Frame& frame) const {
  if (!last_frame_ || !last_tracked_) {
    return fail(ErrorKind::kInvalidInput, "track: no last frame to track from");
  }
  // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM tracks its
  // reference keyframe by bag of words here; the lite pipeline has none.
  const Eigen::Matrix4d predicted = predict(last_tracked_->pose, velocity_);
  auto motion =
      track_last_frame(m, camera_, frame, *last_frame_, *last_tracked_, predicted, mode_, options_);
  if (!motion) {
    return std::unexpected(std::move(motion.error()));
  }
  FrameSummary summary;
  summary.motion = motion->summary;
  if (motion->tracked) {
    summary.placed_by = Placement::kMotion;
    return std::pair{std::move(motion->tracked), std::move(summary)};
  }
  auto recovery = relocalize_locally(m, camera_, frame, options_);
  if (!recovery) {
    return std::unexpected(std::move(recovery.error()));
  }
  summary.pnp = recovery->summary;
  if (recovery->tracked) {
    summary.placed_by = Placement::kLocalPnp;
  }
  return std::pair{std::move(recovery->tracked), std::move(summary)};
}

Result<std::pair<std::optional<Tracked>, LocalMapSummary>> Pipeline::refine(
    Map& m, const Frame& frame, const Tracked& placed, bool wide, bool recent) const {
  auto local = track_local_map(m, camera_, frame, placed, mode_, options_, wide);
  if (!local) {
    return std::unexpected(std::move(local.error()));
  }
  const std::vector<std::int64_t> seen(local->visible.size(), 1);
  const std::vector<std::int64_t> unseen(local->visible.size(), 0);
  if (auto ok = count(m, local->visible, seen, unseen); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::vector<std::int64_t> found(local->found.size(), 1);
  const std::vector<std::int64_t> unfound(local->found.size(), 0);
  if (auto ok = count(m, local->found, unfound, found); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const LocalMapSummary summary = edge::summary(*local);
  const std::int64_t needed = recent ? options_.recovery_tracked : options_.min_tracked;
  if (inliers(local->tracked) < needed) {
    return std::pair{std::optional<Tracked>(), summary};
  }
  return std::pair{std::optional<Tracked>(std::move(local->tracked)), summary};
}

Result<Pipeline::Step> Pipeline::track_frame(const Frame& frame) {
  if (!map_) {
    return fail(ErrorKind::kInvalidInput, "track: no map to track against");
  }
  Map& m = *map_;
  auto placed = place(m, frame);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  auto& [where, summary] = *placed;
  if (summary.placed_by == Placement::kLocalPnp) {
    since_recovery_ = 0;
  } else if (since_recovery_) {
    ++*since_recovery_;
  }
  const bool recent = since_recovery_ && *since_recovery_ < options_.max_frames;
  const bool wide = since_recovery_ && *since_recovery_ < options_.recovery_window_frames;
  std::optional<Tracked> tracked;
  if (where) {
    auto refined = refine(m, frame, *where, wide, recent);
    if (!refined) {
      return std::unexpected(std::move(refined.error()));
    }
    tracked = std::move(refined->first);
    summary.local_map = refined->second;
  }
  if (!tracked) {
    record_failure();
    return Step{.tracked = std::nullopt,
                .keyframe = false,
                .map_seconds = 0.0,
                .summary = std::move(summary)};
  }
  const bool held = recent && std::cmp_greater(keyframe_count(m), options_.max_frames);
  auto need = held ? Result<bool>(false)
                   : need_keyframe(m, camera_, frame.depth, *tracked, mode_, options_);
  if (!need) {
    return std::unexpected(std::move(need.error()));
  }
  std::chrono::duration<double> mapped{0.0};
  if (*need) {
    const auto start = std::chrono::steady_clock::now();
    auto mapping = process(m, camera_, frame, *tracked, mode_, options_);
    if (!mapping) {
      return std::unexpected(std::move(mapping.error()));
    }
    mapped = std::chrono::steady_clock::now() - start;
    summary.mapping = std::move(*mapping);
  }
  if (auto ok = accept(frame, *tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return Step{.tracked = std::move(tracked),
              .keyframe = *need,
              .map_seconds = mapped.count(),
              .summary = std::move(summary)};
}

}  // namespace slam_it_all::edge
