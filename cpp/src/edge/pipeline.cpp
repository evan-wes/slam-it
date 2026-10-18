/// @file
/// @brief Implementation of the pipeline declared in slam_it/edge/pipeline.hpp.
#include "slam_it/edge/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "slam_it/edge/loops.hpp"
#include "slam_it/edge/mapping.hpp"
#include "slam_it/edge/tracking.hpp"
#include "slam_it/foundations/se3.hpp"

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

/// @brief A frame placed, as `place` gives it.
struct Placed {
  /// @brief The placed frame, if any.
  std::optional<Tracked> tracked;
  /// @brief How it was placed.
  FrameSummary summary;
  /// @brief The last stage's match for each keypoint before refinement.
  std::vector<std::int64_t> matched;
};

/// @brief A placed frame tracked against the local map, as `refine` gives it.
struct Refined {
  /// @brief The tracked frame, if it kept enough inliers.
  std::optional<Tracked> tracked;
  /// @brief How the stage went.
  LocalMapSummary summary;
  /// @brief Its match for each keypoint before refinement.
  std::vector<std::int64_t> matched;
  /// @brief The frame's reference keyframe's row.
  std::int64_t reference{-1};
};

/// @brief Tracking's result on a frame, as `track_step` gives it
///        (`slam_it.edge.pipeline._Tracking`).
struct Tracking {
  /// @brief The tracked frame, if it kept enough inliers.
  std::optional<Tracked> tracked;
  /// @brief How its stages went.
  FrameSummary summary;
  /// @brief The last stage's match for each keypoint before refinement.
  std::vector<std::int64_t> matched;
  /// @brief The frame's reference keyframe's row, -1 when not tracked.
  std::int64_t reference{-1};
  /// @brief Whether it asks for a keyframe.
  bool keyframe{false};
  /// @brief Whether the frame lost the track.
  bool lost{false};
};

/// @brief Pair a monocular frame with the reference, or make it the reference.
/// @param state Tracking's state: its reference and where its keypoints were last matched.
/// @param camera The camera.
/// @param frame The frame.
/// @param options The settings.
/// @return The started map, if any, and how the attempt went; a stage's error.
Result<Initialization> initialize_monocular(TrackingState& state, const Camera& camera,
                                            const Frame& frame, const Options& options) {
  const auto not_started = [](Outcome outcome, std::optional<std::int64_t> matches) {
    return Initialization{.started = std::nullopt,
                          .summary = {.outcome = outcome, .matches = matches, .solves = {}}};
  };
  if (std::cmp_less_equal(keypoint_count(frame), options.initialization.min_features)) {
    state.reference.reset();  // too few features: ORB-SLAM drops its initializer
    return not_started(Outcome::kTooFewFeatures, std::nullopt);
  }
  if (!state.reference) {
    set_reference(state, frame);
    return not_started(Outcome::kReferenceSet, std::nullopt);
  }
  auto matched = match_for_initialization(*state.reference, frame, state.previous, options);
  if (!matched) {
    return std::unexpected(std::move(matched.error()));
  }
  if (std::cmp_less(matched->size(), options.initialization.min_matches)) {
    set_reference(state, frame);  // the views no longer overlap enough: start again
    return not_started(Outcome::kReferenceReplaced, static_cast<std::int64_t>(matched->size()));
  }
  // The search follows each matched keypoint (SearchForInitialization's vbPrevMatched).
  std::vector<std::array<std::int64_t, 2>> pairs;
  pairs.reserve(matched->size());
  for (const foundations::matching::Match& match : *matched) {
    state.previous[static_cast<std::size_t>(match.first)] =
        frame.features.keypoints[static_cast<std::size_t>(match.second)].xy;
    pairs.push_back({match.first, match.second});
  }
  // Nothing keeps the reference: a later frame may have the parallax.
  return edge::initialize_monocular(camera, *state.reference, frame, pairs, options);
}

/// @brief Track the last frame, or recover by PnP, or relocalise.
/// @param state Tracking's state.
/// @param owner The map's owner, for its database.
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame.
/// @param mode The sensor mode.
/// @param options The settings.
/// @return The placed frame, if any, how it was placed, and the last stage's matches; a
///         stage's error.
Result<Placed> place(const TrackingState& state, const MapOwner& owner, const Map& m,
                     const Camera& camera, const Frame& frame, Mode mode, const Options& options) {
  if (!state.last_frame || !state.last_tracked) {
    return fail(ErrorKind::kInvalidInput, "track: no last frame to track from");
  }
  // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM tracks its
  // reference keyframe by bag of words here; the lite pipeline's words only relocalise.
  const Eigen::Matrix4d predicted = predict(state.last_tracked->pose, state.velocity);
  auto motion = track_last_frame(m, camera, frame, *state.last_frame, *state.last_tracked,
                                 predicted, mode, options);
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
  auto recovery = relocalize_locally(m, camera, frame, state.last_tracked->points, options);
  if (!recovery) {
    return std::unexpected(std::move(recovery.error()));
  }
  summary.pnp = recovery->summary;
  const std::optional<Database>& db = owner.database();
  const std::optional<foundations::vocabulary::Vocabulary>& words = owner.vocabulary();
  if (recovery->tracked || !db || !words ||
      !options.relocalization.enabled) {  // placed, or not relocalising
    if (recovery->tracked) {
      summary.placed_by = Placement::kLocalPnp;
    }
    return Placed{.tracked = std::move(recovery->tracked),
                  .summary = std::move(summary),
                  .matched = std::move(recovery->matched)};
  }
  auto found = relocalize(m, *db, *words, camera, frame, options);
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

/// @brief Track the local map from a placed frame, and count its points through the owner.
/// @param owner The map's owner, given the counts.
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame.
/// @param placed The placed frame.
/// @param wide Whether the frame is a recovery's or the next (the search widened).
/// @param recent Whether a recovery was recent (more inliers needed).
/// @param mode The sensor mode.
/// @param options The settings.
/// @param keyframe_base The run's id of the map's keyframe 0, for the summary.
/// @return The tracked frame, if it keeps enough inliers, how the stage went, and its
///         matches; a stage's error.
Result<Refined> refine(MapOwner& owner, const Map& m, const Camera& camera, const Frame& frame,
                       const Tracked& placed, bool wide, bool recent, Mode mode,
                       const Options& options, std::int64_t keyframe_base) {
  auto local = track_local_map(m, camera, frame, placed, mode, options, wide);
  if (!local) {
    return std::unexpected(std::move(local.error()));
  }
  if (auto ok = owner.count(local->visible, local->found); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  Refined out{.tracked = std::nullopt,
              .summary = edge::summary(*local, keyframe_base),
              .matched = std::move(local->matched),
              .reference = local->reference};
  const std::int64_t needed =
      recent ? options.tracking.recovery_tracked : options.tracking.min_tracked;
  if (inliers(local->tracked) >= needed) {
    out.tracked = std::move(local->tracked);
  }
  return out;
}

/// @brief Tracking's step (`slam_it.edge.pipeline._track_frame`): place a frame against the
///        map, track the local map, and ask for a keyframe; a frame not tracked is counted as a
///        failure.
/// @param state Tracking's state.
/// @param owner The map's owner, given the frame's counts.
/// @param camera The camera.
/// @param frame The frame.
/// @param mode The sensor mode.
/// @param options The settings.
/// @param keyframe_base The run's id of the map's keyframe 0, for the summary.
/// @return How tracking went; a stage's error, or `InvalidInput` with no map.
Result<Tracking> track_step(TrackingState& state, MapOwner& owner, const Camera& camera,
                            const Frame& frame, Mode mode, const Options& options,
                            std::int64_t keyframe_base) {
  const std::optional<Map>& current = owner.map();
  if (!current) {
    return fail(ErrorKind::kInvalidInput, "track: no map to track against");
  }
  const Map& m = *current;
  auto placed = place(state, owner, m, camera, frame, mode, options);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  auto& [where, summary, matched] = *placed;
  if (summary.placed_by == Placement::kLocalPnp ||
      summary.placed_by == Placement::kRelocalization) {
    state.since_recovery = 0;
  } else if (state.since_recovery) {
    ++*state.since_recovery;
  }
  const bool recent = state.since_recovery && *state.since_recovery < options.tracking.max_frames;
  const bool wide =
      state.since_recovery && *state.since_recovery < options.tracking.recovery_window_frames;
  Tracking out{.tracked = std::nullopt,
               .summary = {},
               .matched = {},
               .reference = -1,
               .keyframe = false,
               .lost = false};
  if (where) {
    auto refined =
        refine(owner, m, camera, frame, *where, wide, recent, mode, options, keyframe_base);
    if (!refined) {
      return std::unexpected(std::move(refined.error()));
    }
    out.tracked = std::move(refined->tracked);
    out.reference = refined->reference;
    summary.local_map = refined->summary;
    matched = std::move(refined->matched);
  }
  out.summary = std::move(summary);
  out.matched = std::move(matched);
  if (!out.tracked) {
    out.lost = record_failure(state, frame, keyframe_count(m), options);
    return out;
  }
  const bool held = recent && std::cmp_greater(keyframe_count(m), options.tracking.max_frames);
  auto need =
      held ? Result<bool>(false)
           : need_keyframe(m, camera, frame.depth, *out.tracked, out.reference, mode, options);
  if (!need) {
    return std::unexpected(std::move(need.error()));
  }
  out.keyframe = *need;
  return out;
}

/// @brief Loop closing's step (`slam_it.edge.pipeline._close_loops`): search the newest
///        keyframe for a loop, and correct one that closes, through the owner.
/// @param state Loop closing's state.
/// @param owner The map's owner.
/// @param camera The camera.
/// @param mode The sensor mode.
/// @param options The settings.
/// @return How it went (none when loop closing is off, there is no database, or the keyframe
///         is the map's first), and whether a loop was corrected; a stage's error.
Result<std::pair<std::optional<LoopSummary>, bool>> close_loops(LoopState& state, MapOwner& owner,
                                                                const Camera& camera, Mode mode,
                                                                const Options& options) {
  const std::optional<Map>& current = owner.map();
  const std::optional<Database>& db = owner.database();
  if (!current || !db || !options.loops.enabled || current->kf_id.back() == 0) {
    return std::pair<std::optional<LoopSummary>, bool>{std::nullopt, false};
  }
  const std::int64_t keyframe = current->kf_id.back();
  auto found = loops::detect(*current, *db, camera, keyframe, state.pending, name(mode), options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  state.pending = std::move(found->pending);
  std::optional<loops::CorrectionSummary> correction;
  if (found->loop) {
    const loops::Loop& loop = *found->loop;
    auto fixed = owner.apply_correction(
        [&](Map& m) { return loops::correct(m, camera, loop, name(mode), options); });
    if (!fixed) {
      return std::unexpected(std::move(fixed.error()));
    }
    correction = std::move(*fixed);
  }
  const bool corrected = correction.has_value();
  LoopSummary summary{.detection = std::move(found->summary), .correction = std::move(correction)};
  return std::pair<std::optional<LoopSummary>, bool>{std::move(summary), corrected};
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

Result<void> accept(TrackingState& state, const Frame& frame, const Tracked& tracked) {
  state.velocity.reset();
  if (state.last_tracked && state.failures == 0) {
    auto inverse = foundations::se3::inverse(state.last_tracked->pose);
    if (!inverse) {
      return std::unexpected(std::move(inverse.error()));
    }
    state.velocity = Eigen::Matrix4d(tracked.pose * *inverse);
  }
  state.last_frame = frame;
  state.last_tracked = tracked;
  state.failures = 0;
  state.lost_since.reset();
  return {};
}

bool record_failure(TrackingState& state, const Frame& frame, std::size_t keyframes,
                    const Options& options) {
  ++state.failures;
  state.velocity.reset();
  const double since = state.lost_since.has_value() ? *state.lost_since : frame.timestamp;
  state.lost_since = since;
  if (!options.relocalization.enabled ||
      std::cmp_less_equal(keyframes, options.loss.reset_keyframes)) {
    return state.failures >= options.loss.lost_frames;
  }
  return frame.timestamp - since > options.loss.relocalize_seconds;
}

void set_reference(TrackingState& state, const Frame& frame) {
  state.reference = frame;
  state.previous.clear();
  for (const foundations::orb::Keypoint& k : frame.features.keypoints) {
    state.previous.push_back(k.xy);
  }
}

void move_last(TrackingState& state, const Eigen::Matrix4d& pose) {
  if (state.last_tracked) {
    state.last_tracked->pose = pose;
  }
}

std::vector<std::int64_t> held(const LoopState& state) {
  return state.pending ? loops::held(*state.pending) : std::vector<std::int64_t>{};
}

Result<void> MapOwner::start(Map m) {
  map_ = std::move(m);
  if (vocabulary_) {
    database_.emplace();
    return sync_database();
  }
  return {};
}

Result<void> MapOwner::count(const std::vector<std::int64_t>& visible,
                             const std::vector<std::int64_t>& found) {
  if (!map_) {
    return fail(ErrorKind::kInvalidInput, "count: no map");
  }
  const std::vector<std::int64_t> seen(visible.size(), 1);
  const std::vector<std::int64_t> unseen(visible.size(), 0);
  if (auto ok = edge::count(*map_, visible, seen, unseen); !ok) {
    return ok;
  }
  const std::vector<std::int64_t> tracked(found.size(), 1);
  const std::vector<std::int64_t> untracked(found.size(), 0);
  return edge::count(*map_, found, untracked, tracked);
}

void MapOwner::clear_gone() {
  if (map_) {
    edge::clear_gone(*map_);
  }
}

std::optional<Map> MapOwner::close() {
  std::optional<Map> out = std::move(map_);
  map_.reset();
  database_.reset();
  return out;
}

Result<void> MapOwner::sync_database() {
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
  atlas_.date(owner_.map() ? &*owner_.map() : nullptr);
  owner_.clear_gone();
  const State state = step.lost ? State::kLost : state_;
  const std::optional<std::int64_t> map_id = atlas_.map_id();
  const std::optional<Tracked>& tracked = step.tracked;
  Fates fates{.fate = std::vector<Fate>(keypoint_count(frame), Fate::kUnmatched),
              .point = std::vector<std::int64_t>(keypoint_count(frame), -1)};
  mark(fates, Fate::kRejected, step.matched, atlas_.point_base());
  if (tracked) {
    mark(fates, Fate::kTracked, tracked->points, atlas_.point_base());
  }
  mark(fates, Fate::kMade, step.made, atlas_.point_base());
  if (step.lost) {  // a lost frame was tracking, so there is a map
    if (const std::optional<Map> lost = owner_.close()) {
      step.summary.lost = close(*lost);
    }
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
  return atlas_.lifetimes(owner_.map() ? &*owner_.map() : nullptr);
}

Trajectory Pipeline::trajectory() const {
  return atlas_.trajectory(owner_.map() ? &*owner_.map() : nullptr);
}

LostSummary Pipeline::close(const Map& m) {
  const bool reset = std::cmp_less_equal(keyframe_count(m), options_.loss.reset_keyframes);
  atlas_.close(m, reset);
  tracking_ = TrackingState{};
  closing_ = LoopState{};
  state_ = State::kNotInitialized;
  return LostSummary{.keyframes = static_cast<std::int64_t>(keyframe_count(m)),
                     .points = static_cast<std::int64_t>(point_count(m)),
                     .reset = reset};
}

Result<Pipeline::Step> Pipeline::initialize(const Frame& frame) {
  auto attempt = mode_ == Mode::kMonocular
                     ? initialize_monocular(tracking_, camera_, frame, options_)
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
  atlas_.start();
  state_ = State::kTracking;
  Tracked tracked = attempt->started->tracked;
  const Map& started = attempt->started->map;
  atlas_.place_frame(started, frame.timestamp, started.kf_id.back(), tracked.pose);
  if (auto ok = owner_.start(std::move(attempt->started->map)); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = accept(tracking_, frame, tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  step.made = tracked.points;  // every point the frame observes was made of it
  step.tracked = std::move(tracked);
  step.keyframe = true;
  step.summary.placed_by = Placement::kInitialization;
  return step;
}

Result<Pipeline::Keyframed> Pipeline::make_keyframe(const Map& m, const Frame& frame,
                                                    const Tracked& tracked, FrameSummary& summary) {
  const auto start = std::chrono::steady_clock::now();
  const std::vector<std::int64_t> holding = held(closing_);
  // Local mapping's step: insert the frame as a keyframe and map around it.
  auto mapping = owner_.apply_mapping([&](Map& changed) {
    return process(changed, camera_, frame, tracked, mode_, options_, holding);
  });
  if (!mapping) {
    return std::unexpected(std::move(mapping.error()));
  }
  const std::chrono::duration<double> mapped = std::chrono::steady_clock::now() - start;
  summary.mapping = std::move(mapping->summary);
  // The frame's reference is its own keyframe, placed where the frame was.
  atlas_.place_keyframe(m, frame.timestamp, tracked.pose);
  const auto searching = std::chrono::steady_clock::now();
  auto looped = close_loops(closing_, owner_, camera_, mode_, options_);
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
  const std::optional<Map>& current = owner_.map();
  if (!current) {
    return fail(ErrorKind::kInvalidInput, "track: no map to track against");
  }
  const Map& m = *current;  // the owner changes it in place
  auto found =
      track_step(tracking_, owner_, camera_, frame, mode_, options_, atlas_.keyframe_base());
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  if (!found->tracked) {
    return Step{.tracked = std::nullopt,
                .keyframe = false,
                .map_seconds = 0.0,
                .loop_seconds = 0.0,
                .summary = std::move(found->summary),
                .matched = std::move(found->matched),
                .made = {},
                .lost = found->lost};
  }
  const Tracked& tracked = *found->tracked;
  Keyframed keyframed;
  if (found->keyframe) {
    auto made = make_keyframe(m, frame, tracked, found->summary);
    if (!made) {
      return std::unexpected(std::move(made.error()));
    }
    keyframed = std::move(*made);
  } else {
    const std::int64_t row =
        found->reference >= 0 ? found->reference : static_cast<std::int64_t>(keyframe_count(m)) - 1;
    atlas_.place_frame(m, frame.timestamp, m.kf_id[static_cast<std::size_t>(row)], tracked.pose);
  }
  if (auto ok = accept(tracking_, frame, tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (keyframed.corrected) {  // the last frame, the new keyframe's own, moved with it
    move_last(tracking_, m.kf_pose.back());
  }
  return Step{.tracked = std::move(found->tracked),
              .keyframe = found->keyframe,
              .map_seconds = keyframed.map_seconds,
              .loop_seconds = keyframed.loop_seconds,
              .summary = std::move(found->summary),
              .matched = std::move(found->matched),
              .made = std::move(keyframed.made),
              .lost = false};
}

}  // namespace slam_it::edge
