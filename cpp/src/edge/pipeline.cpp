/// @file
/// @brief Implementation of the pipeline declared in slam_it/edge/pipeline.hpp.
#include "slam_it/edge/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "slam_it/edge/covisibility.hpp"
#include "slam_it/edge/loops.hpp"
#include "slam_it/edge/mapping.hpp"
#include "slam_it/edge/tracking.hpp"
#include "slam_it/foundations/se3.hpp"
#include "slam_it/foundations/sim3.hpp"

namespace slam_it::edge {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

namespace {

/// @brief Give a fate to the keypoints a stage gave a point, over any fate before it
///        (`slam_it.edge.pipeline._Step.fates`, applied rejected, tracked, then made).
/// @param fates The frame's fates, changed.
/// @param fate The fate.
/// @param ids Each keypoint's point in the map, -1 where none; empty if no stage gave them.
/// @param base The run's id of the map's point 0.
void mark(Fates& fates, Fate fate, const std::vector<std::int64_t>& ids, std::int64_t base) {
  for (std::size_t j = 0; j < std::min(ids.size(), fates.fate.size()); ++j) {
    if (ids[j] >= 0) {
      fates.fate[j] = fate;
      fates.point[j] = base + ids[j];
    }
  }
}

}  // namespace

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
    case Placement::kRelocalization:
      return "relocalization";
    case Placement::kCount:
      break;
  }
  return "unknown";
}

Result<Pipeline> Pipeline::create(Mode mode, Camera camera, Options options,
                                  std::optional<foundations::vocabulary::Vocabulary> vocabulary) {
  if (auto ok = check_setup(camera, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (mode != Mode::kMonocular && !(camera.baseline > 0.0)) {
    return fail(ErrorKind::kInvalidInput,
                std::string("camera.baseline: ") + name(mode) + " needs a positive baseline");
  }
  const bool needed = keeps_keyframe_database(options);
  if (needed && !vocabulary) {
    return fail(ErrorKind::kInvalidInput,
                "vocabulary: relocalization.enabled or loops.enabled needs one");
  }
  if (!needed && vocabulary) {
    return fail(ErrorKind::kInvalidInput,
                "vocabulary: given, but relocalization.enabled and loops.enabled are both off");
  }
  return Pipeline(mode, std::move(camera), options, std::move(vocabulary));
}

foundations::orb::Options Pipeline::extraction() const {
  return edge::extraction(options_, mode_ == Mode::kMonocular && state_ == State::kNotInitialized);
}

Result<FrameResult> Pipeline::track(const Frame& frame) {
  const auto start = std::chrono::steady_clock::now();
  // Between frames the state is never kLost: a lost track closes its map on its frame.
  auto stepped = state_ == State::kTracking ? track_frame(frame) : initialize(frame);
  if (!stepped) {
    return std::unexpected(std::move(stepped.error()));
  }
  Step step = std::move(*stepped);
  date();
  const State state = step.lost ? State::kLost : state_;
  const std::optional<std::int64_t> map_id = map_id_;
  const std::optional<Tracked>& tracked = step.tracked;
  Fates fates{.fate = std::vector<Fate>(keypoint_count(frame), Fate::kUnmatched),
              .point = std::vector<std::int64_t>(keypoint_count(frame), -1)};
  mark(fates, Fate::kRejected, step.matched, pt_base_);
  if (tracked) {
    mark(fates, Fate::kTracked, tracked->points, pt_base_);
  }
  mark(fates, Fate::kMade, step.made, pt_base_);
  if (step.lost && map_) {  // a lost frame was tracking, so there is a map
    const Map lost = std::move(*map_);
    step.summary.lost = close(lost);
  }
  const std::chrono::duration<double> took = std::chrono::steady_clock::now() - start;
  return FrameResult{.timestamp = frame.timestamp,
                     .state = state,
                     .map = map_id,
                     .pose = tracked ? std::optional<Eigen::Matrix4d>(tracked->pose) : std::nullopt,
                     .tracked = tracked ? inliers(*tracked) : 0,
                     .keyframe = step.keyframe,
                     .track_seconds = took.count() - step.map_seconds - step.loop_seconds,
                     .map_seconds = step.map_seconds,
                     .loop_seconds = step.loop_seconds,
                     .summary = std::move(step.summary),
                     .fates = std::move(fates)};
}

Lifetimes Pipeline::lifetimes() const {
  Lifetimes out{.points = past_points_,
                .point_lives = point_lives_,
                .keyframe_times = past_times_,
                .keyframe_poses = past_poses_,
                .keyframe_lives = keyframe_lives_,
                .keyframe_parents = past_parents_,
                .keyframe_links = past_links_,
                .keyframe_loops = past_loops_,
                .point_maps = point_maps_,
                .keyframe_maps = keyframe_maps_};
  if (map_) {
    for (std::size_t p = 0; p < point_count(*map_); ++p) {
      out.points[static_cast<std::size_t>(pt_base_ + map_->pt_id[p])] = map_->pt_xyz[p];
    }
    for (std::size_t k = 0; k < keyframe_count(*map_); ++k) {
      const auto id = static_cast<std::size_t>(kf_base_ + map_->kf_id[k]);
      out.keyframe_times[id] = map_->kf_time[k];
      out.keyframe_poses[id] = map_->kf_pose[k];
      out.keyframe_parents[id] = run_id(map_->kf_parent[k]);
    }
    for (const auto& link : strong_links(*map_, options_)) {
      out.keyframe_links.push_back({kf_base_ + link[0], kf_base_ + link[1], link[2]});
    }
    for (const auto& [older, newer] : map_->loop_edges) {
      out.keyframe_loops.push_back({kf_base_ + older, kf_base_ + newer});
    }
  }
  return out;
}

Trajectory Pipeline::trajectory() const {
  std::map<std::int64_t, Eigen::Matrix4d> last;
  for (std::size_t id = 0; id < past_poses_.size(); ++id) {
    if (keyframe_lives_[id][1] >= 0) {
      last[static_cast<std::int64_t>(id)] = past_poses_[id];  // removed: where it last was
    }
  }
  for (std::size_t id = 0; id < keyframe_maps_.size(); ++id) {
    const auto at = static_cast<std::int64_t>(id);
    if (keyframe_lives_[id][1] < 0 && (!map_id_ || keyframe_maps_[id] != *map_id_)) {
      last[at] = past_poses_[id];  // a closed map's, kept
    }
  }
  if (map_) {
    for (std::size_t k = 0; k < keyframe_count(*map_); ++k) {
      last[kf_base_ + map_->kf_id[k]] = map_->kf_pose[k];
    }
  }
  Trajectory out;
  for (const FramePlace& placed : placements_) {
    Eigen::Matrix4d pose = placed.relative;
    std::int64_t keyframe = placed.reference;
    for (auto gone = culled_.find(keyframe); gone != culled_.end(); gone = culled_.find(keyframe)) {
      pose = pose * gone->second.second;
      keyframe = gone->second.first;
    }
    const auto anchor = last.find(keyframe);
    out.timestamps.push_back(placed.timestamp);
    out.poses.push_back(anchor == last.end() ? placed.tracked
                                             : Eigen::Matrix4d(pose * anchor->second));
    out.tracked.push_back(placed.tracked);
    out.maps.push_back(placed.map);
  }
  return out;
}

std::int64_t Pipeline::run_id(std::int64_t parent) const {
  return parent >= 0 ? kf_base_ + parent : -1;
}

void Pipeline::date() {
  const std::int64_t frame = frames_++;
  if (!map_ || !map_id_) {
    return;
  }
  Map& m = *map_;
  const auto points = static_cast<std::size_t>(pt_base_ + m.next_pt_id);
  point_lives_.resize(points, Life{frame, -1, -1});
  point_maps_.resize(points, *map_id_);
  past_points_.resize(points, Eigen::Vector3d::Zero());
  const auto keyframes = static_cast<std::size_t>(kf_base_ + m.next_kf_id);
  keyframe_lives_.resize(keyframes, Life{frame, -1, -1});
  keyframe_maps_.resize(keyframes, *map_id_);
  past_times_.resize(keyframes, 0.0);
  past_poses_.resize(keyframes, Eigen::Matrix4d::Identity());
  past_parents_.resize(keyframes, -1);
  for (std::size_t g = 0; g < m.gone_pt_id.size(); ++g) {
    const auto id = static_cast<std::size_t>(pt_base_ + m.gone_pt_id[g]);
    point_lives_[id][1] = frame;
    point_lives_[id][2] = m.gone_pt_how[g];
    past_points_[id] = m.gone_pt_xyz[g];
  }
  for (std::size_t g = 0; g < m.gone_kf_id.size(); ++g) {
    const auto id = static_cast<std::size_t>(kf_base_ + m.gone_kf_id[g]);
    keyframe_lives_[id][1] = frame;
    keyframe_lives_[id][2] = static_cast<std::int64_t>(Removal::kCulled);
    past_times_[id] = m.gone_kf_time[g];
    past_poses_[id] = m.gone_kf_pose[g];
    past_parents_[id] = run_id(m.gone_kf_parent[g]);
    if (m.gone_kf_parent[g] >= 0) {
      culled_[static_cast<std::int64_t>(id)] = {run_id(m.gone_kf_parent[g]), m.gone_kf_relative[g]};
    }
  }
  clear_gone(m);
}

LostSummary Pipeline::close(const Map& m) {
  const std::int64_t frame = frames_ - 1;  // dated already
  const bool reset = std::cmp_less_equal(keyframe_count(m), options_.loss.reset_keyframes);
  const auto code = static_cast<std::int64_t>(Removal::kReset);
  for (std::size_t p = 0; p < point_count(m); ++p) {
    const auto id = static_cast<std::size_t>(pt_base_ + m.pt_id[p]);
    past_points_[id] = m.pt_xyz[p];
    if (reset) {
      point_lives_[id][1] = frame;
      point_lives_[id][2] = code;
    }
  }
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    const auto id = static_cast<std::size_t>(kf_base_ + m.kf_id[k]);
    past_times_[id] = m.kf_time[k];
    past_poses_[id] = m.kf_pose[k];
    past_parents_[id] = run_id(m.kf_parent[k]);
    if (reset) {
      keyframe_lives_[id][1] = frame;
      keyframe_lives_[id][2] = code;
    }
  }
  for (const auto& link : strong_links(m, options_)) {
    past_links_.push_back({kf_base_ + link[0], kf_base_ + link[1], link[2]});
  }
  for (const auto& [older, newer] : m.loop_edges) {
    past_loops_.push_back({kf_base_ + older, kf_base_ + newer});
  }
  if (reset && map_id_) {  // its frames' poses go with it
    std::erase_if(placements_, [this](const FramePlace& p) { return p.map == *map_id_; });
  }
  const LostSummary lost{.keyframes = static_cast<std::int64_t>(keyframe_count(m)),
                         .points = static_cast<std::int64_t>(point_count(m)),
                         .reset = reset};
  map_.reset();
  map_id_.reset();
  database_.reset();
  pending_.reset();
  state_ = State::kNotInitialized;
  last_frame_.reset();
  last_tracked_.reset();
  velocity_.reset();
  reference_.reset();
  previous_.clear();
  failures_ = 0;
  since_recovery_.reset();
  lost_since_.reset();
  return lost;
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
            .loop_seconds = 0.0,
            .summary = {.placed_by = std::nullopt,
                        .init = std::move(attempt->summary),
                        .motion = std::nullopt,
                        .pnp = std::nullopt,
                        .relocalization = std::nullopt,
                        .local_map = std::nullopt,
                        .mapping = std::nullopt,
                        .loop = std::nullopt,
                        .lost = std::nullopt},
            .matched = {},
            .made = {},
            .lost = false};
  if (!attempt->started) {
    return step;
  }
  map_id_ = maps_++;
  kf_base_ = static_cast<std::int64_t>(keyframe_lives_.size());
  pt_base_ = static_cast<std::int64_t>(point_lives_.size());
  state_ = State::kTracking;
  Tracked tracked = attempt->started->tracked;
  const Map& started = attempt->started->map;
  place_frame(started, frame, tracked, started.kf_id.back());
  map_ = std::move(attempt->started->map);
  if (vocabulary_) {
    database_.emplace();
    if (auto ok = sync_database(); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  if (auto ok = accept(frame, tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  step.made = tracked.points;  // every point the frame observes was made of it
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
  if (std::cmp_less_equal(keypoint_count(frame), options_.initialization.min_features)) {
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
  if (std::cmp_less(matched->size(), options_.initialization.min_matches)) {
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
  lost_since_.reset();
  return {};
}

void Pipeline::place_frame(const Map& m, const Frame& frame, const Tracked& tracked,
                           std::int64_t reference) {
  const auto k = static_cast<std::size_t>(keyframe_row(m, reference));
  placements_.push_back({.timestamp = frame.timestamp,
                         .map = map_id_.value_or(0),
                         .reference = kf_base_ + reference,
                         .relative = tracked.pose * foundations::sim3::inverse(m.kf_pose[k]),
                         .tracked = tracked.pose});
}

Result<std::pair<std::optional<LoopSummary>, bool>> Pipeline::close_loops(Map& m) {
  const std::int64_t keyframe = m.kf_id.back();
  if (!database_ || !options_.loops.enabled || keyframe == 0) {
    return std::pair<std::optional<LoopSummary>, bool>{std::nullopt, false};
  }
  auto found = loops::detect(m, *database_, camera_, keyframe, pending_, name(mode_), options_);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  pending_ = std::move(found->pending);
  std::optional<loops::CorrectionSummary> correction;
  if (found->loop) {
    auto fixed = loops::correct(m, camera_, *found->loop, name(mode_), options_);
    if (!fixed) {
      return std::unexpected(std::move(fixed.error()));
    }
    correction = std::move(*fixed);
    if (auto ok = sync_database(); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  const bool corrected = correction.has_value();
  LoopSummary summary{.detection = std::move(found->summary), .correction = std::move(correction)};
  return std::pair<std::optional<LoopSummary>, bool>{std::move(summary), corrected};
}

Result<void> Pipeline::sync_database() {
  if (!database_ || !map_ || !vocabulary_) {
    return {};  // no keyframe database
  }
  const Map& m = *map_;
  std::vector<std::int64_t> culled;
  for (const auto& [keyframe, words] : database_->keyframes) {
    if (keyframe_row(m, keyframe) < 0) {
      culled.push_back(keyframe);
    }
  }
  for (const std::int64_t keyframe : culled) {
    if (auto ok = erase(*database_, keyframe); !ok) {
      return ok;
    }
  }
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    if (database_->keyframes.contains(m.kf_id[k])) {
      continue;
    }
    const auto begin = static_cast<std::size_t>(m.kf_start[k]);
    const auto end = static_cast<std::size_t>(m.kf_start[k + 1]);
    const auto descriptors = std::span(m.kp_descriptor).subspan(begin, end - begin);
    if (auto ok = add(*database_, *vocabulary_, m.kf_id[k], descriptors); !ok) {
      return ok;
    }
  }
  return {};
}

bool Pipeline::record_failure(const Frame& frame, std::size_t keyframes) {
  ++failures_;
  velocity_.reset();
  const double since = lost_since_.has_value() ? *lost_since_ : frame.timestamp;
  lost_since_ = since;
  if (!options_.relocalization.enabled ||
      std::cmp_less_equal(keyframes, options_.loss.reset_keyframes)) {
    return failures_ >= options_.loss.lost_frames;
  }
  return frame.timestamp - since > options_.loss.relocalize_seconds;
}

Result<Pipeline::Placed> Pipeline::place(const Map& m, const Frame& frame) const {
  if (!last_frame_ || !last_tracked_) {
    return fail(ErrorKind::kInvalidInput, "track: no last frame to track from");
  }
  // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM tracks its
  // reference keyframe by bag of words here; the lite pipeline's words only relocalise.
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
    return Placed{.tracked = std::move(motion->tracked),
                  .summary = std::move(summary),
                  .matched = std::move(motion->matched)};
  }
  auto recovery = relocalize_locally(m, camera_, frame, last_tracked_->points, options_);
  if (!recovery) {
    return std::unexpected(std::move(recovery.error()));
  }
  summary.pnp = recovery->summary;
  if (recovery->tracked || !database_ || !vocabulary_ ||
      !options_.relocalization.enabled) {  // placed, or not relocalising
    if (recovery->tracked) {
      summary.placed_by = Placement::kLocalPnp;
    }
    return Placed{.tracked = std::move(recovery->tracked),
                  .summary = std::move(summary),
                  .matched = std::move(recovery->matched)};
  }
  auto found = relocalize(m, *database_, *vocabulary_, camera_, frame, options_);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  summary.relocalization = found->summary;
  if (found->tracked) {
    summary.placed_by = Placement::kRelocalization;
  }
  return Placed{.tracked = std::move(found->tracked),
                .summary = std::move(summary),
                .matched = std::move(found->matched)};
}

Result<Pipeline::Refined> Pipeline::refine(Map& m, const Frame& frame, const Tracked& placed,
                                           bool wide, bool recent) const {
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
  Refined out{.tracked = std::nullopt,
              .summary = edge::summary(*local, kf_base_),
              .matched = std::move(local->matched),
              .reference = local->reference};
  const std::int64_t needed =
      recent ? options_.tracking.recovery_tracked : options_.tracking.min_tracked;
  if (inliers(local->tracked) >= needed) {
    out.tracked = std::move(local->tracked);
  }
  return out;
}

Result<Pipeline::Keyframed> Pipeline::make_keyframe(Map& m, const Frame& frame,
                                                    const Tracked& tracked, FrameSummary& summary) {
  const auto start = std::chrono::steady_clock::now();
  const std::vector<std::int64_t> holding =
      pending_ ? loops::held(*pending_) : std::vector<std::int64_t>{};
  auto mapping = process(m, camera_, frame, tracked, mode_, options_, holding);
  if (!mapping) {
    return std::unexpected(std::move(mapping.error()));
  }
  if (auto ok = sync_database(); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::chrono::duration<double> mapped = std::chrono::steady_clock::now() - start;
  summary.mapping = std::move(mapping->summary);
  // The frame's reference is its own keyframe, placed where the frame was.
  placements_.push_back({.timestamp = frame.timestamp,
                         .map = map_id_.value_or(0),
                         .reference = kf_base_ + m.kf_id.back(),
                         .relative = Eigen::Matrix4d::Identity(),
                         .tracked = tracked.pose});
  const auto searching = std::chrono::steady_clock::now();
  auto looped = close_loops(m);
  if (!looped) {
    return std::unexpected(std::move(looped.error()));
  }
  Keyframed out{.map_seconds = mapped.count(),
                .loop_seconds = 0.0,
                .corrected = looped->second,
                .made = std::move(mapping->made)};
  if (looped->first) {
    out.loop_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - searching).count();
    summary.loop = std::move(looped->first);
  }
  return out;
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
  auto& [where, summary, matched] = *placed;
  if (summary.placed_by == Placement::kLocalPnp ||
      summary.placed_by == Placement::kRelocalization) {
    since_recovery_ = 0;
  } else if (since_recovery_) {
    ++*since_recovery_;
  }
  const bool recent = since_recovery_ && *since_recovery_ < options_.tracking.max_frames;
  const bool wide = since_recovery_ && *since_recovery_ < options_.tracking.recovery_window_frames;
  std::optional<Tracked> tracked;
  std::int64_t reference = -1;
  if (where) {
    auto refined = refine(m, frame, *where, wide, recent);
    if (!refined) {
      return std::unexpected(std::move(refined.error()));
    }
    tracked = std::move(refined->tracked);
    reference = refined->reference;
    summary.local_map = refined->summary;
    matched = std::move(refined->matched);
  }
  if (!tracked) {
    const bool lost = record_failure(frame, keyframe_count(m));
    return Step{.tracked = std::nullopt,
                .keyframe = false,
                .map_seconds = 0.0,
                .loop_seconds = 0.0,
                .summary = std::move(summary),
                .matched = std::move(matched),
                .made = {},
                .lost = lost};
  }
  const bool held = recent && std::cmp_greater(keyframe_count(m), options_.tracking.max_frames);
  auto need = held ? Result<bool>(false)
                   : need_keyframe(m, camera_, frame.depth, *tracked, reference, mode_, options_);
  if (!need) {
    return std::unexpected(std::move(need.error()));
  }
  Keyframed keyframed;
  if (*need) {
    auto made = make_keyframe(m, frame, *tracked, summary);
    if (!made) {
      return std::unexpected(std::move(made.error()));
    }
    keyframed = std::move(*made);
  } else {
    const std::int64_t row =
        reference >= 0 ? reference : static_cast<std::int64_t>(keyframe_count(m)) - 1;
    place_frame(m, frame, *tracked, m.kf_id[static_cast<std::size_t>(row)]);
  }
  if (auto ok = accept(frame, *tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (keyframed.corrected && last_tracked_) {  // the last frame, the new keyframe's own, moved
    last_tracked_->pose = m.kf_pose.back();
  }
  return Step{.tracked = std::move(tracked),
              .keyframe = *need,
              .map_seconds = keyframed.map_seconds,
              .loop_seconds = keyframed.loop_seconds,
              .summary = std::move(summary),
              .matched = std::move(matched),
              .made = std::move(keyframed.made),
              .lost = false};
}

}  // namespace slam_it::edge
