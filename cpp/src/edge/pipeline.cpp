/// @file
/// @brief Implementation of the pipeline declared in slam_it/edge/pipeline.hpp.
#include "slam_it/edge/pipeline.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "carry.hpp"
#include "slam_it/edge/covisibility.hpp"
#include "slam_it/edge/loops.hpp"
#include "slam_it/edge/mapping.hpp"
#include "slam_it/edge/threads.hpp"
#include "slam_it/edge/tracking.hpp"
#include "slam_it/foundations/se3.hpp"
#include "slam_it/foundations/sim3.hpp"

namespace slam_it::edge {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

namespace {

/// @brief The clock the steps are timed by.
using Clock = std::chrono::steady_clock;

/// @brief Seconds, as the results give them.
using Seconds = std::chrono::duration<double>;

/// @brief A frame as tracked, its points no longer in the map dropped (`ProcessNewKeyFrame`;
///        `slam_it.edge.pipeline._still_mapped`): threaded, mapping may cull or fuse a point a
///        keyframe tracked before it inserts the keyframe, and ORB-SLAM skips the points gone
///        bad.
/// @param tracked The frame's pose and matches.
/// @param m The map.
/// @return Its pose, and its matches still in the map.
Tracked still_mapped(const Tracked& tracked, const Map& m) {
  Tracked out = tracked;
  for (std::int64_t& id : out.points) {
    if (id >= 0 && point_row(m, id) < 0) {
      id = -1;
    }
  }
  return out;
}

/// @brief Add a frame's counts to the map: the points seen, and found, in it (a point gone is
///        skipped).
/// @param m The map.
/// @param visible The points the frame's view held, by the map's ids.
/// @param found The points it tracked.
/// @return Nothing; `count`'s error.
Result<void> add_counts(Map& m, const std::vector<std::int64_t>& visible,
                        const std::vector<std::int64_t>& found) {
  const std::vector<std::int64_t> seen(visible.size(), 1);
  const std::vector<std::int64_t> unseen(visible.size(), 0);
  if (auto ok = edge::count(m, visible, seen, unseen); !ok) {
    return ok;
  }
  const std::vector<std::int64_t> tracked(found.size(), 1);
  const std::vector<std::int64_t> untracked(found.size(), 0);
  return edge::count(m, found, untracked, tracked);
}

/// @brief Ids moved on by an offset, -1 (none) kept.
/// @param ids The ids, changed in place.
/// @param offset The offset.
void offset_ids(std::vector<std::int64_t>& ids, std::int64_t offset) {
  for (std::int64_t& id : ids) {
    id = id >= 0 ? id + offset : id;
  }
}

/// @brief A merge made (`slam_it.edge.pipeline._Joined`): how it went, how the active map's
///        keyframe and point ids moved on, and `G`.
struct Joined {
  /// @brief How it went.
  loops::MergeSummary summary;
  /// @brief How far the active map's keyframe ids moved on.
  std::int64_t keyframe_offset{0};
  /// @brief How far its point ids moved on.
  std::int64_t point_offset{0};
  /// @brief `G`, carrying the active map into the matched map's world.
  Eigen::Matrix4d to_world{Eigen::Matrix4d::Identity()};
};

/// @brief A correction with its matched and current keyframes by the run's ids
///        (`slam_it.edge.pipeline._corrected`).
/// @param atlas The atlas, its current map the corrected one, recorded.
/// @param summary The correction.
/// @return The correction, the matched keyframe's run id first.
Corrected corrected(const Atlas& atlas, loops::CorrectionSummary summary) {
  const std::array<std::int64_t, 2> run_ids{atlas.run_id(summary.matched),
                                            atlas.run_id(summary.current)};
  return Corrected{.summary = std::move(summary), .run_ids = run_ids};
}

/// @brief Merge `m`, the map being tracked, held with mapping stopped, into the kept map a
///        merge found (`slam_it.edge.pipeline._merged`), by `loops::merge`: the merged map is
///        tracked on, with the matched map's number and database, published with the
///        correction it counts (`MapOwner::take_merged`); the loop being confirmed is carried
///        into it.
/// @param state Loop closing's state.
/// @param owner The map's owner.
/// @param atlas The atlas, holding the matched map.
/// @param m The map being tracked, held: the owner's, which the merged map then replaces.
/// @param camera The camera.
/// @param found The merge.
/// @param mode The sensor mode.
/// @param options The settings.
/// @return The merge made; `InvalidInput` naming the matched map if it is not kept; a stage's
///         error.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the maps' holders, as Python's
Result<Joined> merge_held(LoopState& state, MapOwner& owner, Atlas& atlas, const Map& m,
                          const Camera& camera, const loops::Merge& found, Mode mode,
                          const Options& options) {
  const std::vector<MapRef> others = atlas.kept();
  const auto matched = std::ranges::find(others, found.map, &MapRef::number);
  if (matched == others.end()) {
    return fail(ErrorKind::kInvalidInput,
                "merge: map " + std::to_string(found.map) + " is not kept");
  }
  auto to_world = loops::merge_transform(m, found);
  if (!to_world) {
    return std::unexpected(std::move(to_world.error()));
  }
  auto merged = loops::merge(m, *matched->map, camera, found, name(mode), options);
  if (!merged) {
    return std::unexpected(std::move(merged.error()));
  }
  auto database = atlas.merge(found.map, *to_world);  // the matched map leaves the kept
  if (!database) {
    return std::unexpected(std::move(database.error()));
  }
  if (auto ok = owner.take_merged(std::move(merged->map), std::move(*database),
                                  merged->keyframe_offset, merged->point_offset);
      !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (state.pending) {
    detail::carry_through(*state.pending, *merged, *to_world);
  }
  return Joined{.summary = std::move(merged->summary),
                .keyframe_offset = merged->keyframe_offset,
                .point_offset = merged->point_offset,
                .to_world = *to_world};
}

/// @brief A keyframe searched (`search`): for a loop, and with maps kept, for a merge.
struct Found {
  /// @brief The loop detection, a loop dropped for a merge found on the keyframe.
  loops::Detection loop;
  /// @brief The merge detection; none with no map kept.
  std::optional<loops::MergeDetection> merging;
};

/// @brief Search a keyframe of `m` for a loop and, with maps kept, for a merge, and carry loop
///        closing's state on (`slam_it.edge.pipeline._search`): a merge goes before a loop
///        closed on the same keyframe (`loops::merge_first`).
/// @param state Loop closing's state, carried on.
/// @param m The map.
/// @param db Its database.
/// @param atlas The atlas: the maps kept.
/// @param camera The camera.
/// @param keyframe The keyframe, by `m`'s id.
/// @param mode The sensor mode.
/// @param options The settings.
/// @return The two detections; a stage's error.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the map's holders, as Python's
Result<Found> search(LoopState& state, const Map& m, const Database& db, const Atlas& atlas,
                     const Camera& camera, std::int64_t keyframe, Mode mode,
                     const Options& options) {
  auto found = loops::detect(m, db, camera, keyframe, state.pending, name(mode), options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  std::optional<loops::MergeDetection> merging;
  if (const std::vector<MapRef> others = atlas.kept(); !others.empty()) {
    auto searched =
        loops::detect_merge(m, db, camera, keyframe, others, state.merging, name(mode), options);
    if (!searched) {
      return std::unexpected(std::move(searched.error()));
    }
    state.merging = searched->pending;
    *found = loops::merge_first(std::move(*found), *searched);
    merging = std::move(*searched);
  }
  state.pending = std::move(found->pending);
  return Found{.loop = std::move(*found), .merging = std::move(merging)};
}

}  // namespace

/// @brief Local mapping, loop closing and the global BAs, each in a thread of its own
///        (`slam_it.edge.pipeline._Threads`, the module docstring's "In threads of their own"):
///        tracking asks it whether mapping takes a keyframe, and gives it the keyframes; the
///        threads give back `Completion` records.
class PipelineThreads {
 public:
  /// @brief A keyframe tracking asked local mapping for (`slam_it.edge.pipeline._Request`):
  ///        mapping places it by its reference keyframe if the map was corrected since.
  struct Request {
    /// @brief Its frame's number.
    std::int64_t index{0};
    /// @brief The frame.
    Frame frame;
    /// @brief Its pose and matches.
    Tracked tracked;
    /// @brief The frame's reference keyframe, by the map's id.
    std::int64_t reference{-1};
    /// @brief Its pose relative to it, `T_frame_reference`.
    Eigen::Matrix4d relative{Eigen::Matrix4d::Identity()};
    /// @brief The corrections the view it was tracked on carried (`View::corrections`).
    std::int64_t corrections{0};
  };

  /// @brief A keyframe mapped, given to loop closing.
  struct Searched {
    /// @brief Its frame's number.
    std::int64_t index{0};
    /// @brief Its id in the map.
    std::int64_t keyframe{0};
  };

  /// @brief Start the threads.
  /// @param owner The map's owner, its writer given.
  /// @param atlas The run's records.
  /// @param closing Loop closing's state.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  PipelineThreads(MapOwner& owner, Atlas& atlas, LoopState& closing, Mode mode, Camera camera,
                  const Options& options)
      : owner_(owner),
        atlas_(atlas),
        closing_(closing),
        mode_(mode),
        camera_(std::move(camera)),
        options_(options),
        without_global_ba_(options),
        loop_closing_([this](Searched& item) { return close_loops(item); }),
        mapping_([this](Request& request) { return map(request); }) {
    without_global_ba_.loops.global_ba = false;  // it runs in a thread of its own
  }

  /// @brief Supersede a global BA running, end the threads after their work, and wait for the
  ///        global BAs.
  ~PipelineThreads() {
    generation_.next();
    stop();
    for (std::thread& thread : running()) {
      thread.join();
    }
  }

  PipelineThreads(const PipelineThreads&) = delete;
  PipelineThreads& operator=(const PipelineThreads&) = delete;
  PipelineThreads(PipelineThreads&&) = delete;
  PipelineThreads& operator=(PipelineThreads&&) = delete;

  /// @brief Whether local mapping takes a keyframe at once (`AcceptKeyFrames`): none in hand,
  ///        none waiting.
  /// @return Whether it is idle.
  [[nodiscard]] bool idle() const { return mapping_.unfinished() == 0; }

  /// @brief Whether local mapping is stopped or being stopped.
  /// @return Whether it is.
  [[nodiscard]] bool stopping() const { return writer().stopping(); }

  /// @brief The keyframes waiting for local mapping (`KeyframesInQueue`).
  /// @return The count.
  [[nodiscard]] std::int64_t waiting() const {
    return static_cast<std::int64_t>(mapping_.waiting());
  }

  /// @brief Interrupt local mapping's local BA (`InterruptBA`).
  void interrupt() { writer().interrupt(); }

  /// @brief Queue a keyframe for local mapping, aborting its local BA (`InsertKeyFrame`).
  /// @param request The keyframe.
  void request(Request request) {
    mapping_.put(std::move(request));
    writer().interrupt();
  }

  /// @brief The completions since the last call, in the order they came.
  /// @return The completions.
  [[nodiscard]] std::vector<Completion> completed() {
    const std::scoped_lock lock(mutex_);
    return std::exchange(completions_, {});
  }

  /// @brief The error a thread failed with, if one did.
  /// @return Nothing, or the error.
  [[nodiscard]] Result<void> check() const {
    if (auto ok = mapping_.check(); !ok) {
      return ok;
    }
    if (auto ok = loop_closing_.check(); !ok) {
      return ok;
    }
    const std::scoped_lock lock(mutex_);
    if (error_) {
      return std::unexpected(*error_);
    }
    return {};
  }

  /// @brief Wait for local mapping and loop closing to handle every keyframe given, and for the
  ///        global BAs.
  /// @param apply Whether a global BA still running applies its result when it ends (the run's
  ///        end); if not (a map lost), it is superseded.
  /// @return Nothing; the error a thread failed with.
  Result<void> settle(bool apply) {
    mapping_.drain();
    loop_closing_.drain();
    if (!apply) {
      generation_.next();
    }
    for (std::thread& thread : running()) {
      thread.join();
    }
    return check();
  }

  /// @brief End local mapping and loop closing, after the keyframes given (`settle` first).
  void stop() {
    mapping_.stop();
    loop_closing_.stop();
  }

  /// @brief Make the merge loop closing found, if one waits: tracking's, between two frames
  ///        (`slam_it.edge.pipeline._Threads.merge_waiting`; the online-SLAM design's section
  ///        14, item 4, threaded: option B). Tracking pauses for the whole merge, as
  ///        ORB-SLAM3's tracking waits for its map while `MergeLocal` runs. Mapping first maps
  ///        every keyframe queued, which loop closing passes by while the merge waits
  ///        (`EmptyQueue`); then, mapping stopped, a global BA still running is superseded,
  ///        the merge is made as inline (`merge_held`) without global BA, and the merged map
  ///        is published with the correction it counts; a global BA starts on it only if one
  ///        was superseded (`bRelaunchBA`). Tracking's last frame is carried into it
  ///        (`merged`), and the merge is recorded on the frame whose keyframe found it.
  /// @param tracking Tracking's state, carried into the merged map.
  /// @return The time tracking waited, 0 if no merge waits; the error a thread failed with,
  ///         or the merge's.
  Result<double> merge_waiting(TrackingState& tracking) {
    if (!waiting_.load()) {
      return 0.0;
    }
    const auto asked = Clock::now();
    if (auto ok = drain(); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
    const auto start = Clock::now();
    std::optional<WaitingMerge> waiting;
    Started started;
    auto made = owner_.stopped([&](Map& m) -> Result<std::optional<Joined>> {
      waiting = std::exchange(closing_.waiting, std::nullopt);
      waiting_ = false;
      if (!waiting) {
        return std::nullopt;
      }
      const bool superseded = due_ && generation_.current(*due_);
      if (superseded) {
        generation_.next();
      }
      auto joined = merge_held(closing_, owner_, atlas_, m, camera_, waiting->merge, mode_,
                               without_global_ba_);
      if (!joined) {
        return std::unexpected(std::move(joined.error()));
      }
      atlas_.record(&m);  // the merged map, taken where the map was
      owner_.clear_gone();
      if (superseded) {
        started.generation = generation_.next();
        started.superseded = generation_.stop();
        started.copy = m;
        due_ = started.generation;
      }
      return std::optional<Joined>(std::move(*joined));
    });
    if (!made) {
      return std::unexpected(std::move(made.error()));
    }
    if (!*made || !waiting) {
      return Seconds(Clock::now() - asked).count();
    }
    const Joined& joined = **made;
    merged(tracking, joined.keyframe_offset, joined.point_offset, joined.to_world);
    waiting->summary.merge = joined.summary;
    Completion done;
    done.frame = waiting->index;
    done.loop_seconds = waiting->seconds + Seconds(Clock::now() - start).count();
    done.loop = std::move(waiting->summary);
    complete(std::move(done));
    launch(std::move(started));
    return Seconds(Clock::now() - asked).count();
  }

  /// @brief Drop a merge waiting: the map it would merge was left (`Pipeline::leave`, loop
  ///        closing's state cleared with the threads settled).
  void drop_merge() { waiting_ = false; }

 private:
  /// @brief Wait for mapping and loop closing to handle every keyframe given, in turn.
  /// @return Nothing; the error a thread failed with.
  Result<void> drain() {
    mapping_.drain();
    loop_closing_.drain();
    return check();
  }

  /// @brief The handshake between the threads.
  /// @return The owner's writer.
  [[nodiscard]] threads::Writer& writer() const { return *owner_.writer(); }

  /// @brief Take the global BAs' threads started so far.
  /// @return The threads.
  std::vector<std::thread> running() {
    const std::scoped_lock lock(mutex_);
    return std::exchange(global_, {});
  }

  /// @brief Keep a completion for the run.
  /// @param completion The completion.
  void complete(Completion completion) {
    const std::scoped_lock lock(mutex_);
    completions_.push_back(std::move(completion));
  }

  /// @brief Keep a global BA's error, for tracking's thread.
  /// @param error The error.
  void keep(foundations::Error error) {
    const std::scoped_lock lock(mutex_);
    if (!error_) {
      error_ = std::move(error);
    }
  }

  /// @brief A keyframe as tracked, placed by its reference keyframe if the map moved since
  ///        (`slam_it.edge.pipeline._Threads._placed`): a correction (a loop's, or a global
  ///        BA's carried onto the map) since the frame was tracked moved the map from under its
  ///        pose, so it is placed where its reference keyframe now is,
  ///        `T_frame_reference T_reference_world` (ORB-SLAM's `UpdateLastFrame` rule; B3b's
  ///        review panel, A2 and L1), through the reference's culled ancestors. If that chain
  ///        ends at no keyframe of the map, its pose as tracked is kept, as the trajectory keeps
  ///        such a frame's. Mapping's turn held.
  /// @param request The keyframe.
  /// @param m The map.
  /// @return Its pose and matches.
  [[nodiscard]] Tracked placed(const Request& request, const Map& m) const {
    Tracked out = request.tracked;
    if (request.corrections != owner_.corrections()) {
      if (const auto pose = atlas_.pose_by(m, request.reference, request.relative)) {
        out.pose = *pose;
      }
    }
    return out;
  }

  /// @brief Local mapping's thread: map a keyframe, the one writer while it does, interrupted
  ///        between its steps; then give it to loop closing.
  /// @param request The keyframe.
  /// @return Nothing; a step's error.
  Result<void> map(Request& request) {
    Completion done;
    done.frame = request.index;
    std::int64_t keyframe = 0;
    {
      const threads::Writer::Mapping turn(writer());
      std::unique_lock<std::mutex> lock = owner_.locked();  // the baseline's, through the steps
      writer().resume();
      if (auto ok = owner_.apply_counts(); !ok) {
        return ok;
      }
      const auto start = Clock::now();
      const Around around{.leave = [&lock] { lock.unlock(); }, .enter = [&lock] { lock.lock(); }};
      const Interrupts asked{.waiting = [this] { return mapping_.waiting() > 0; },
                             .stopping = [this] { return writer().stopping(); },
                             .abort = &writer().abort(),
                             .publish = [this, &lock](const Map& m) { publish(m, lock); },
                             .around = lock.owns_lock() ? &around : nullptr};
      const std::vector<std::int64_t> holding = held(closing_);
      auto mapped = owner_.apply_mapping([&](Map& m) {
        return process(m, camera_, request.frame, still_mapped(placed(request, m), m), mode_,
                       options_, holding, &asked);
      });
      if (!mapped) {
        return std::unexpected(std::move(mapped.error()));
      }
      done.map_seconds = Seconds(Clock::now() - start).count();
      const std::optional<Map>& current = owner_.map();
      if (!current) {
        return fail(ErrorKind::kInvalidInput, "mapping: no map");
      }
      const Map& m = *current;
      atlas_.resolve(request.index, m);
      atlas_.record(&m);
      owner_.clear_gone();
      keyframe = m.kf_id.back();
      for (const std::int64_t id : mapped->made) {
        done.made.push_back(atlas_.point_run_id(id));
      }
      done.mapping = std::move(mapped->summary);
      done.schedule = mapped->schedule;
    }
    complete(std::move(done));
    loop_closing_.put({.index = request.index, .keyframe = keyframe});
    return {};
  }

  /// @brief Loop closing's thread: search a keyframe for a loop and a merge with mapping held
  ///        off, and correct a loop that closes with mapping stopped, without global BA, which
  ///        then solves a copy of the map in a thread of its own. A merge found waits for
  ///        tracking to make it (`merge_waiting`); until it has, the keyframes mapped are passed
  ///        by, their ids about to move.
  /// @param item The keyframe.
  /// @return Nothing; a step's error.
  Result<void> close_loops(Searched& item) {
    if (!options_.loops.enabled || item.keyframe == 0 || waiting_.load()) {
      return {};
    }
    const auto start = Clock::now();
    Started started;
    std::optional<loops::Loop> later;  // the baseline corrects once its search lets the map go
    LoopSummary summary;
    auto searched = owner_.reading([&] -> Result<bool> {
      const std::optional<Map>& current = owner_.map();
      const std::optional<Database>& db = owner_.database();
      if (!current || !db || keyframe_row(*current, item.keyframe) < 0) {
        return false;  // not relocalising, or culled since it was mapped
      }
      auto found = search(closing_, *current, *db, atlas_, camera_, item.keyframe, mode_, options_);
      if (!found) {
        return std::unexpected(std::move(found.error()));
      }
      summary.detection = std::move(found->loop.summary);
      if (found->merging) {
        summary.merging = found->merging->summary;
        if (found->merging->merge) {  // recorded once tracking has made it
          closing_.waiting = WaitingMerge{.index = item.index,
                                          .merge = std::move(*found->merging->merge),
                                          .summary = std::move(summary),
                                          .seconds = Seconds(Clock::now() - start).count()};
          waiting_ = true;
          return false;
        }
      }
      if (!found->loop.loop) {
        return true;
      }
      if (!owner_.publishing()) {
        later = std::move(found->loop.loop);
        return true;
      }
      auto fixed = correct_loop(*found->loop.loop, started);
      if (!fixed) {
        return std::unexpected(std::move(fixed.error()));
      }
      summary.correction = std::move(*fixed);
      return true;
    });
    if (!searched) {
      return std::unexpected(std::move(searched.error()));
    }
    if (!*searched) {  // not relocalising, culled since it was mapped, or a merge waiting
      return {};
    }
    if (later) {
      auto fixed = correct_loop(*later, started);
      if (!fixed) {
        return std::unexpected(std::move(fixed.error()));
      }
      summary.correction = std::move(*fixed);
    }
    Completion done;
    done.frame = item.index;
    done.loop_seconds = Seconds(Clock::now() - start).count();
    done.loop = std::move(summary);
    complete(std::move(done));
    launch(std::move(started));
    return {};
  }

  /// @brief A global BA started by a loop's correction, or by a merge in place of one it
  ///        superseded.
  struct Started {
    /// @brief The map as the correction or merge left it, which global BA solves
    ///        (`loops.global_ba`).
    std::optional<Map> copy;
    /// @brief Its generation.
    std::int64_t generation{0};
    /// @brief Its stop flag, set once a newer generation starts.
    std::shared_ptr<const std::atomic<bool>> superseded;
  };

  /// @brief Start a global BA in a thread of its own, if one was started.
  /// @param started Its copy of the map, generation and stop flag; no copy, none.
  void launch(Started started) {
    if (!started.copy) {
      return;
    }
    std::thread thread([this, solving = std::move(*started.copy), generation = started.generation,
                        stop = std::move(started.superseded)] mutable {
      global_ba(std::move(solving), generation, stop.get());
    });
    const std::scoped_lock lock(mutex_);
    global_.push_back(std::move(thread));
  }

  /// @brief Correct a loop with mapping stopped, without global BA, and take the copy global BA
  ///        solves. The correction's keyframes are read by the run's ids inside the stop, once
  ///        the atlas has recorded the corrected map, as Python reads them (B4's review panel,
  ///        run 3, A18): nothing can change the atlas's current map between the two.
  /// @param loop The loop.
  /// @param started The global BA it starts, set.
  /// @return The correction, with its keyframes by the run's ids, or none if one of the loop's
  ///         keyframes went since it was found (the baseline, its search and its correction
  ///         apart); a step's error.
  Result<std::optional<Corrected>> correct_loop(const loops::Loop& loop, Started& started) {
    return owner_.stopped([&](Map& m) -> Result<std::optional<Corrected>> {
      if (keyframe_row(m, loop.current) < 0 || keyframe_row(m, loop.matched) < 0) {
        return std::nullopt;
      }
      started.generation = generation_.next();
      started.superseded = generation_.stop();
      auto fixed = loops::correct(m, camera_, loop, name(mode_), without_global_ba_);
      if (!fixed) {
        return std::unexpected(std::move(fixed.error()));
      }
      if (auto ok = owner_.corrected(); !ok) {
        return std::unexpected(std::move(ok.error()));
      }
      atlas_.record(&m);
      Corrected correction = corrected(atlas_, std::move(*fixed));
      owner_.clear_gone();
      if (options_.loops.global_ba) {
        started.copy = m;
        due_ = started.generation;
      }
      return std::optional<Corrected>(std::move(correction));
    });
  }

  /// @brief Publish the map so far, mapping's keyframe's points made: a copy, our design; the
  ///        baseline publishes none, and lets the map's lock go there, between its steps.
  /// @param m The map so far.
  /// @param lock The baseline's lock, held; none otherwise.
  void publish(const Map& m, std::unique_lock<std::mutex>& lock) {
    if (lock.owns_lock()) {
      lock.unlock();
      lock.lock();
    } else {
      owner_.publish(m);
    }
  }

  /// @brief A global BA's thread: solve a copy, then carry the result onto the map as it has
  ///        become if no newer loop superseded it, mapping stopped.
  /// @param copy The map as the loop's correction left it.
  /// @param generation The global BA's generation.
  /// @param superseded Its stop flag, set once a newer generation starts.
  void global_ba(Map copy, std::int64_t generation, const std::atomic<bool>* superseded) {
    const auto start = Clock::now();
    auto solved = loops::global_bundle_adjust(copy, camera_, options_, superseded);
    if (!solved) {
      keep(std::move(solved.error()));
      return;
    }
    std::optional<loops::Propagated> moved;
    std::int64_t frame = 0;
    auto applied = owner_.stopped([&](Map& m) -> Result<void> {
      if (due_ == generation) {  // applied or dropped now: none to supersede
        due_.reset();
      }
      if (generation_.current(generation)) {
        auto carried = loops::propagate_global_ba(m, copy);
        if (!carried) {
          return std::unexpected(std::move(carried.error()));
        }
        moved = *carried;
        if (auto ok = owner_.corrected(); !ok) {
          return ok;
        }
        atlas_.record(&m);
        owner_.clear_gone();
      }
      frame = atlas_.frame();
      return {};
    });
    if (!applied) {
      keep(std::move(applied.error()));
      return;
    }
    Completion done;
    done.frame = frame;
    done.global_ba = loops::GlobalSummary{.generation = generation,
                                          .applied = moved.has_value(),
                                          .solve = *solved,
                                          .propagation = moved};
    done.global_ba_seconds = Seconds(Clock::now() - start).count();
    complete(std::move(done));
  }

  /// @brief The map's owner.
  MapOwner& owner_;
  /// @brief The run's records.
  Atlas& atlas_;
  /// @brief Loop closing's state.
  LoopState& closing_;
  /// @brief The sensor mode.
  Mode mode_;
  /// @brief The camera.
  Camera camera_;
  /// @brief The settings.
  Options options_;
  /// @brief The settings, global BA left out of a loop's correction.
  Options without_global_ba_;
  /// @brief The global BAs' generations.
  threads::Generation generation_;
  /// @brief The latest global BA whose result is still to come; read and set with mapping
  ///        stopped.
  std::optional<std::int64_t> due_;
  /// @brief Whether a merge waits for tracking (`LoopState::waiting`): set by loop closing,
  ///        cleared by tracking as it makes it, read by both without a hold.
  std::atomic<bool> waiting_{false};
  /// @brief Guards the fields below.
  mutable std::mutex mutex_;
  /// @brief The completions not yet taken.
  std::vector<Completion> completions_;
  /// @brief The global BAs' threads started and not yet joined.
  std::vector<std::thread> global_;
  /// @brief A global BA's error, if one failed.
  std::optional<foundations::Error> error_;
  /// @brief Loop closing's thread, started after the fields above.
  threads::Worker<Searched> loop_closing_;
  /// @brief Local mapping's thread, ended first: it gives loop closing its keyframes.
  threads::Worker<Request> mapping_;
};

namespace {

/// @brief Give a fate to the keypoints a stage gave a point, over any fate before it
///        (`slam_it.edge.pipeline._Step.fates`, applied rejected, tracked, then made).
/// @param fates The frame's fates, changed.
/// @param fate The fate.
/// @param ids Each keypoint's point in the map, -1 where none; empty if no stage gave them.
/// @param atlas The atlas, for the points' run ids.
void mark(Fates& fates, Fate fate, const std::vector<std::int64_t>& ids, const Atlas& atlas) {
  for (std::size_t j = 0; j < std::min(ids.size(), fates.fate.size()); ++j) {
    if (ids[j] >= 0) {
      fates.fate[j] = fate;
      fates.point[j] = atlas.point_run_id(ids[j]);
    }
  }
}

/// @brief A frame's keypoints' fates (`slam_it.edge.pipeline._Step.fates`): rejected where its
///        last stage matched them, then tracked where its pose's inliers are, then made where a
///        point was made of them; unmatched elsewhere.
/// @param frame The frame.
/// @param matched The last stage's matches, by keypoint, -1 where none.
/// @param tracked Its pose and matches, if it was tracked.
/// @param made The points made of its keypoints, by keypoint, -1 where none.
/// @param atlas The atlas, for the points' run ids.
/// @return The fates.
Fates fates_of(const Frame& frame, const std::vector<std::int64_t>& matched,
               const std::optional<Tracked>& tracked, const std::vector<std::int64_t>& made,
               const Atlas& atlas) {
  Fates out{.fate = std::vector<Fate>(keypoint_count(frame), Fate::kUnmatched),
            .point = std::vector<std::int64_t>(keypoint_count(frame), -1)};
  mark(out, Fate::kRejected, matched, atlas);
  if (tracked) {
    mark(out, Fate::kTracked, tracked->points, atlas);
  }
  mark(out, Fate::kMade, made, atlas);
  return out;
}

/// @brief A frame placed, as `place` gives it.
struct Placed {
  /// @brief The placed frame, if any.
  std::optional<Tracked> tracked;
  /// @brief How it was placed.
  FrameSummary summary;
  /// @brief The last stage's match for each keypoint before refinement.
  std::vector<std::int64_t> matched;
  /// @brief The time spent waiting for the map, 0 inline.
  double waited{0.0};
  /// @brief The number of the kept map relocalisation found the frame in, if not the one
  ///        tracked.
  std::optional<std::int64_t> moved;
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
  /// @brief The map tracking read: the view, held for the frame.
  View view;
  /// @brief The tracked frame, if it kept enough inliers.
  std::optional<Tracked> tracked;
  /// @brief How its stages went.
  FrameSummary summary;
  /// @brief The last stage's match for each keypoint before refinement.
  std::vector<std::int64_t> matched;
  /// @brief The frame's reference keyframe's row, -1 when not tracked.
  std::int64_t reference{-1};
  /// @brief Whether it becomes a keyframe.
  bool keyframe{false};
  /// @brief Whether the frame lost the track.
  bool lost{false};
  /// @brief The time spent waiting for the map, 0 inline.
  double wait_seconds{0.0};
  /// @brief Whether it wanted a keyframe.
  bool asked{false};
  /// @brief Whether mapping could not take the keyframe it wanted.
  bool refused{false};
  /// @brief The keyframes waiting for mapping when it asked.
  std::int64_t queue{0};
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

/// @brief Relocalise a frame in the map tracked, and in every kept map with it (the online-SLAM
///        design's section 14, item 2) when there are any.
/// @param m The map tracked.
/// @param db Its database.
/// @param words The vocabulary.
/// @param atlas The atlas: the map's number, and the kept maps.
/// @param camera The camera.
/// @param frame The frame.
/// @param options The settings.
/// @return The relocalisation, and the number of the kept map it placed the frame in, if not
///         the one tracked; a stage's error.
Result<std::pair<Relocalized, std::optional<std::int64_t>>> relocalize_everywhere(
    const Map& m, const Database& db, const foundations::vocabulary::Vocabulary& words,
    const Atlas& atlas, const Camera& camera, const Frame& frame, const Options& options) {
  using Found = std::pair<Relocalized, std::optional<std::int64_t>>;
  const std::vector<MapRef> others = atlas.kept();
  if (others.empty()) {
    return relocalize(m, db, words, camera, frame, options).transform([](Relocalized found) {
      return Found{std::move(found), std::nullopt};
    });
  }
  const std::int64_t number = atlas.map_id().value_or(0);
  std::vector<MapRef> searched{{.number = number, .map = &m, .database = &db}};
  searched.insert(searched.end(), others.begin(), others.end());
  return relocalize_across(searched, words, camera, frame, options).transform([number](Across a) {
    const std::optional<std::int64_t> moved =
        a.map && *a.map != number ? a.map : std::optional<std::int64_t>();
    if (!a.map && !a.maps.empty() && a.maps.back() != number) {
      // The last candidate's matches are points of another map, not this one's.
      std::ranges::fill(a.relocalized.matched, -1);
    }
    return Found{std::move(a.relocalized), moved};
  });
}

/// @brief Track the last frame against the view, or recover by PnP, or relocalise, in every
///        kept map as well. Threaded, relocalisation reads the database with mapping held off,
///        against the whole map, and the frame goes on with the view of where it was placed,
///        published for the frames after it.
/// @param state Tracking's state.
/// @param owner The map's owner, for its database and the view of a relocalised frame.
/// @param atlas The atlas, for the kept maps.
/// @param view What tracking reads; where it reads views, a relocalisation's replaces it.
/// @param camera The camera.
/// @param frame The frame.
/// @param mode The sensor mode.
/// @param options The settings.
/// @return The placed frame, if any, how it was placed, the last stage's matches, the time
///         spent waiting for the map, and the kept map relocalisation found it in; a stage's
///         error.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the state, as Python's _place
Result<Placed> place(const TrackingState& state, MapOwner& owner, const Atlas& atlas, View& view,
                     const Camera& camera, const Frame& frame, Mode mode, const Options& options) {
  const Map& m = *view.map;
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
                  .matched = std::move(motion->matched),
                  .waited = 0.0,
                  .moved = std::nullopt};
  }
  auto recovery = relocalize_locally(m, camera, frame, state.last_tracked->points, options);
  if (!recovery) {
    return std::unexpected(std::move(recovery.error()));
  }
  summary.pnp = recovery->summary;
  const std::optional<foundations::vocabulary::Vocabulary>& words = owner.vocabulary();
  if (recovery->tracked || !words ||
      !options.relocalization.enabled) {  // placed, or not relocalising
    if (recovery->tracked) {
      summary.placed_by = Placement::kLocalPnp;
    }
    return Placed{.tracked = std::move(recovery->tracked),
                  .summary = std::move(summary),
                  .matched = std::move(recovery->matched),
                  .waited = 0.0,
                  .moved = std::nullopt};
  }
  const auto asked = Clock::now();
  auto held = asked;
  const auto search = [&] {
    held = Clock::now();
    // A view holds the part of the map its frame may read: relocalisation reads the whole map,
    // and a frame placed in it goes on with the view of where it was placed; one placed in a
    // kept map goes on in that map, once tracking has taken it up.
    const Map& whole = owner.views() ? *owner.map() : *view.map;
    auto found =
        relocalize_everywhere(whole, *owner.database(), *words, atlas, camera, frame, options);
    if (found && found->first.tracked && !found->second && owner.views()) {
      view = owner.relocalized(found->first.tracked->points);
    }
    return found;
  };
  // The baseline's tracking holds the map's lock already, through its frame.
  auto everywhere = owner.publishing() ? owner.reading(search) : search();
  if (!everywhere) {
    return std::unexpected(std::move(everywhere.error()));
  }
  auto& [found, moved] = *everywhere;
  summary.relocalization = found.summary;
  if (found.tracked) {
    summary.placed_by = Placement::kRelocalization;
  }
  const bool placed = found.tracked.has_value();
  return Placed{.tracked = std::move(found.tracked),
                .summary = std::move(summary),
                .matched = std::move(found.matched),
                .waited = owner.publishing() ? Seconds(held - asked).count() : 0.0,
                .moved = placed ? moved : std::nullopt};
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
/// @param atlas The atlas, for the reference keyframe's run id in the summary.
/// @return The tracked frame, if it keeps enough inliers, how the stage went, and its
///         matches; a stage's error.
Result<Refined> refine(MapOwner& owner, const Map& m, const Camera& camera, const Frame& frame,
                       const Tracked& placed, bool wide, bool recent, Mode mode,
                       const Options& options, const Atlas& atlas) {
  auto local = track_local_map(m, camera, frame, placed, mode, options, wide);
  if (!local) {
    return std::unexpected(std::move(local.error()));
  }
  if (auto ok = owner.count(local->visible, local->found); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  Refined out{.tracked = std::nullopt,
              .summary = edge::summary(*local),
              .matched = std::move(local->matched),
              .reference = local->reference};
  out.summary.reference = atlas.run_id(out.summary.reference);
  const std::int64_t needed =
      recent ? options.tracking.recovery_tracked : options.tracking.min_tracked;
  if (inliers(local->tracked) >= needed) {
    out.tracked = std::move(local->tracked);
  }
  return out;
}

/// @brief Ask local mapping for a keyframe a frame wants, as ORB-SLAM's `NeedNewKeyFrame` does:
///        none while mapping is stopped or being stopped; taken at once with mapping idle (as
///        inline, always); busy, the local BA is interrupted, and a stereo or RGB-D keyframe is
///        queued while fewer than `mapping.max_queued` wait, a monocular one refused.
/// @param out Tracking's result, its keyframe, refusal and queue set.
/// @param threads The threads; null inline.
/// @param idle Whether mapping was idle when the frame asked.
/// @param mode The sensor mode.
/// @param options The settings.
void ask_mapping(Tracking& out, PipelineThreads* threads, bool idle, Mode mode,
                 const Options& options) {
  if (threads == nullptr) {
    out.keyframe = true;
    return;
  }
  out.queue = threads->waiting();
  if (threads->stopping()) {
    out.refused = true;
  } else if (idle) {
    out.keyframe = true;
  } else {
    threads->interrupt();
    out.keyframe = mode != Mode::kMonocular && out.queue < options.mapping.max_queued;
    out.refused = !out.keyframe;
  }
}

/// @brief A map taken up by tracking (`Pipeline::switch_map`): how the map left was closed,
///        and the time spent waiting for the threads.
using Switched = std::pair<LostSummary, double>;

/// @brief Tracking's step (`slam_it.edge.pipeline._track_frame`): place a frame against its
///        view, track the local map, and ask for a keyframe; a frame not tracked is counted as a
///        failure, and one relocalisation found in a kept map goes on in it.
/// @param state Tracking's state.
/// @param owner The map's owner, given the frame's counts.
/// @param view The map's view, read once for the frame.
/// @param atlas The atlas: the kept maps, and the run's ids for the summary.
/// @param camera The camera.
/// @param frame The frame.
/// @param mode The sensor mode.
/// @param options The settings.
/// @param threads The threads, asked whether mapping takes a keyframe; null inline.
/// @param switch_to Takes up the kept map relocalisation found the frame in, leaving the map
///        tracked, and says how it left it and how long it waited for the threads.
/// @return How tracking went; a stage's error, or `InvalidInput` with no map.
// NOLINTNEXTLINE(readability-function-cognitive-complexity): Python's _track_frame, step for step
Result<Tracking> track_step(TrackingState& state, MapOwner& owner, View view, Atlas& atlas,
                            const Camera& camera, const Frame& frame, Mode mode,
                            const Options& options, PipelineThreads* threads,
                            const std::function<Result<Switched>(std::int64_t)>& switch_to) {
  ++state.since_keyframe;
  if (!view.map) {
    return fail(ErrorKind::kInvalidInput, "track: no map to track against");
  }
  auto placed = place(state, owner, atlas, view, camera, frame, mode, options);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  auto& [where, summary, matched, waited, moved] = *placed;
  if (moved && where) {  // placed in a kept map: relocalisation placed it
    auto switched = switch_to(*moved);
    if (!switched) {
      return std::unexpected(std::move(switched.error()));
    }
    summary.left = switched->first;
    waited += switched->second;
    view = owner.relocalized(where->points);  // the map taken up, where the frame was placed
  }
  if (threads != nullptr) {  // the view may hold what the threads have not recorded yet
    atlas.extend(*view.map);
  }
  const Map& m = *view.map;
  if (summary.placed_by == Placement::kLocalPnp ||
      summary.placed_by == Placement::kRelocalization) {
    state.since_recovery = 0;
  } else if (state.since_recovery) {
    ++*state.since_recovery;
  }
  const bool recent = state.since_recovery && *state.since_recovery < options.tracking.max_frames;
  const bool wide =
      state.since_recovery && *state.since_recovery < options.tracking.recovery_window_frames;
  Tracking out{.view = view,
               .tracked = std::nullopt,
               .summary = {},
               .matched = {},
               .reference = -1,
               .keyframe = false,
               .lost = false,
               .wait_seconds = waited,
               .asked = false,
               .refused = false,
               .queue = 0};
  if (where) {
    auto refined = refine(owner, m, camera, frame, *where, wide, recent, mode, options, atlas);
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
  // The rules count the whole map's keyframes, which a view holds a part of.
  if (!out.tracked) {
    out.lost = record_failure(state, frame, static_cast<std::size_t>(view.keyframes), options);
    return out;
  }
  if (recent && std::cmp_greater(view.keyframes, options.tracking.max_frames)) {
    return out;  // none just after a recovery
  }
  const bool idle = threads == nullptr || threads->idle();
  auto need = need_keyframe(m, camera, frame.depth, *out.tracked, out.reference, mode, options,
                            idle, state.since_keyframe, view.keyframes);
  if (!need) {
    return std::unexpected(std::move(need.error()));
  }
  out.asked = *need;
  if (out.asked) {
    ask_mapping(out, threads, idle, mode, options);
  }
  if (out.keyframe) {
    state.since_keyframe = 0;
  }
  return out;
}

/// @brief Loop closing's result on a keyframe: how it went, whether a loop was corrected, and
///        how a merge moved the map's point ids on, if one was made.
struct Looped {
  /// @brief How it went; none when loop closing did not run.
  std::optional<LoopSummary> summary;
  /// @brief Whether a loop was corrected.
  bool corrected{false};
  /// @brief The merge's point offset, if one was made.
  std::optional<std::int64_t> point_offset;
};

/// @brief Merge the map being tracked into the kept map a merge found, mapping stopped
///        (`slam_it.edge.pipeline._merge_maps`; the merge is `merge_held`'s).
/// @param state Loop closing's state.
/// @param owner The map's owner.
/// @param atlas The atlas, holding the matched map.
/// @param camera The camera.
/// @param found The merge.
/// @param mode The sensor mode.
/// @param options The settings.
/// @return The merge made; a stage's error.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the maps' holders, as Python's
Result<Joined> merge_maps(LoopState& state, MapOwner& owner, Atlas& atlas, const Camera& camera,
                          const loops::Merge& found, Mode mode, const Options& options) {
  return owner.stopped(
      [&](Map& m) { return merge_held(state, owner, atlas, m, camera, found, mode, options); });
}

/// @brief Loop closing's step (`slam_it.edge.pipeline._close_loops`): search the newest
///        keyframe for a loop and, with maps kept, for a merge, and make one, through the owner;
///        a merge goes before a loop closed on the same keyframe (`loops::merge_first`).
/// @param state Loop closing's state.
/// @param owner The map's owner.
/// @param atlas The atlas: the kept maps.
/// @param camera The camera.
/// @param mode The sensor mode.
/// @param options The settings.
/// @return How it went (none when loop closing is off, there is no database, or the keyframe
///         is the map's first), whether a loop was corrected, and the merge's point offset if
///         one was made; a stage's error.
Result<Looped> close_loops(LoopState& state, MapOwner& owner, Atlas& atlas, const Camera& camera,
                           Mode mode, const Options& options) {
  const std::optional<Map>& current = owner.map();
  const std::optional<Database>& db = owner.database();
  if (!current || !db || !options.loops.enabled || current->kf_id.back() == 0) {
    return Looped{};
  }
  const std::int64_t keyframe = current->kf_id.back();
  auto searched = search(state, *current, *db, atlas, camera, keyframe, mode, options);
  if (!searched) {
    return std::unexpected(std::move(searched.error()));
  }
  auto& [found, merging] = *searched;
  LoopSummary summary{.detection = std::move(found.summary),
                      .correction = std::nullopt,
                      .merging = merging ? std::optional(merging->summary) : std::nullopt,
                      .merge = std::nullopt};
  if (merging && merging->merge) {
    auto made = merge_maps(state, owner, atlas, camera, *merging->merge, mode, options);
    if (!made) {
      return std::unexpected(std::move(made.error()));
    }
    summary.merge = std::move(made->summary);
    return Looped{
        .summary = std::move(summary), .corrected = false, .point_offset = made->point_offset};
  }
  if (found.loop) {
    const loops::Loop& loop = *found.loop;
    auto fixed = owner.apply_correction(
        [&](Map& m) { return loops::correct(m, camera, loop, name(mode), options); });
    if (!fixed) {
      return std::unexpected(std::move(fixed.error()));
    }
    summary.correction = corrected(atlas, std::move(*fixed));
  }
  const bool closed = summary.correction.has_value();
  return Looped{.summary = std::move(summary), .corrected = closed, .point_offset = std::nullopt};
}

}  // namespace

namespace detail {

void carry_through(loops::Pending& pending, const loops::Merged& merged,
                   const Eigen::Matrix4d& to_world) {
  pending.matched += merged.keyframe_offset;
  pending.last += merged.keyframe_offset;
  const Eigen::Matrix4d back = foundations::sim3::inverse(to_world);
  pending.similarity = pending.similarity * back;
  pending.similarity.topRows<3>() /= loops::scale_of(back);
  offset_ids(pending.points, merged.point_offset);
  offset_ids(pending.matched_points, merged.point_offset);
}

}  // namespace detail

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
  const double seconds = options.loss.relocalize_seconds;
  return seconds == 0.0 || frame.timestamp - since > seconds;
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

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (row, corrections), as Python's
void anchor_to(TrackingState& state, const Map& view, std::size_t row, std::int64_t corrections) {
  if (!state.last_tracked) {
    return;
  }
  state.anchor = view.kf_id[row];
  state.anchor_relative =
      Eigen::Matrix4d(state.last_tracked->pose * foundations::sim3::inverse(view.kf_pose[row]));
  state.corrections = corrections;
}

void follow(TrackingState& state, const Map& view, std::int64_t corrections) {
  if (corrections == state.corrections) {
    return;
  }
  state.corrections = corrections;
  if (!state.last_tracked || !state.anchor || !state.anchor_relative) {
    return;
  }
  if (const std::int64_t row = keyframe_row(view, *state.anchor); row >= 0) {
    move_last(state, *state.anchor_relative * view.kf_pose[static_cast<std::size_t>(row)]);
  }
}

void change_map(TrackingState& state) {
  if (state.last_tracked) {
    std::ranges::fill(state.last_tracked->points, -1);
  }
  state.velocity.reset();
  state.lost_since.reset();
  state.failures = std::max<std::int64_t>(state.failures, 1);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the merge's two offsets, as Python's
void merged(TrackingState& state, std::int64_t keyframe_offset, std::int64_t point_offset,
            const Eigen::Matrix4d& to_world) {
  const double scale = loops::scale_of(to_world);
  if (state.last_tracked) {
    offset_ids(state.last_tracked->points, point_offset);
  }
  if (state.anchor) {
    *state.anchor += keyframe_offset;
  }
  for (std::optional<Eigen::Matrix4d>* motion : {&state.velocity, &state.anchor_relative}) {
    if (*motion) {
      (*motion)->block<3, 1>(0, 3) *= scale;
    }
  }
}

std::vector<std::int64_t> held(const LoopState& state) {
  std::vector<std::int64_t> out =
      state.pending ? loops::held(*state.pending) : std::vector<std::int64_t>{};
  if (state.merging) {
    out.push_back(state.merging->pending.last);
  }
  if (state.waiting) {
    out.push_back(state.waiting->merge.loop.current);
  }
  std::ranges::sort(out);
  const auto [first, last] = std::ranges::unique(out);
  out.erase(first, last);
  return out;
}

ViewSize size_of(const Map& m) {
  const auto bytes = [](const auto& values) {
    return static_cast<std::int64_t>(values.size() * sizeof(values.front()));
  };
  std::int64_t observations = 0;
  for (const std::vector<std::int64_t>& rows : m.pt_observations) {
    observations += bytes(rows);
  }
  return {.keyframes = static_cast<std::int64_t>(keyframe_count(m)),
          .points = static_cast<std::int64_t>(point_count(m)),
          .bytes = bytes(m.kf_id) + bytes(m.kf_time) + bytes(m.kf_pose) + bytes(m.kf_parent) +
                   bytes(m.kf_start) + bytes(m.kp_xy) + bytes(m.kp_level) + bytes(m.kp_angle) +
                   bytes(m.kp_descriptor) + bytes(m.kp_bearing) + bytes(m.kp_depth) +
                   bytes(m.kp_right) + bytes(m.kp_point) + bytes(m.pt_id) + bytes(m.pt_xyz) +
                   bytes(m.pt_descriptor) + bytes(m.pt_normal) + bytes(m.pt_min_distance) +
                   bytes(m.pt_max_distance) + bytes(m.pt_visible) + bytes(m.pt_found) +
                   bytes(m.pt_first_kf) + bytes(m.pt_reference_kf) + observations};
}

View MapOwner::view() const {
  if (publishing()) {
    const std::scoped_lock lock(view_mutex_);
    return view_;
  }
  if (!map_) {
    return {};
  }
  if (views_ && writer_ == nullptr) {  // inline, the test: the view a writer would publish now
    std::vector<std::int64_t> points;
    std::int64_t reference = -1;
    {
      const std::scoped_lock lock(view_mutex_);
      points = anchor_points_;
      reference = anchor_reference_;
    }
    return view_of(points, reference);
  }
  // The map itself, not owned; tracking holds the baseline's lock, so no correction runs now.
  return {.map = std::shared_ptr<const Map>(std::shared_ptr<const Map>(), &*map_),
          .keyframes = static_cast<std::int64_t>(keyframe_count(*map_)),
          .corrections = corrections_.load()};
}

View MapOwner::view_of(std::span<const std::int64_t> points, std::int64_t reference) const {
  if (!map_) {
    return {};
  }
  const Map& m = *map_;
  const auto rows = view_keyframes(m, points, reference, options_);
  // Tracking's points are ids, -1 where none, which view_keyframes takes: it cannot fail here,
  // and a debug build says so if it did, rather than fall back to the whole map silently.
  assert(rows.has_value() && "view_keyframes failed on tracking's points");
  return {.map = std::make_shared<const Map>(rows ? sub_map(m, *rows) : m),
          .keyframes = static_cast<std::int64_t>(keyframe_count(m)),
          .corrections = corrections_.load()};
}

void MapOwner::anchor(std::vector<std::int64_t> points, std::int64_t reference) {
  const std::scoped_lock lock(view_mutex_);
  anchor_points_ = std::move(points);
  anchor_reference_ = reference;
}

View MapOwner::relocalized(std::vector<std::int64_t> points) {
  anchor(std::move(points), -1);
  if (publishing() && map_) {
    publish(*map_);
  }
  return view();
}

std::vector<Completion> MapOwner::published() {
  const std::scoped_lock lock(view_mutex_);
  return std::exchange(published_, {});
}

Result<void> MapOwner::start(Map m) {
  map_ = std::move(m);
  if (publishing()) {
    publish(*map_);
  }
  if (vocabulary_) {
    database_.emplace();
    return sync_database();
  }
  return {};
}

Result<void> MapOwner::count(const std::vector<std::int64_t>& visible,
                             const std::vector<std::int64_t>& found) {
  if (publishing()) {
    const std::scoped_lock lock(counts_mutex_);
    counts_.emplace_back(visible, found);
    return {};
  }
  if (!map_) {
    return fail(ErrorKind::kInvalidInput, "count: no map");
  }
  return add_counts(*map_, visible, found);
}

Result<void> MapOwner::apply_counts() {
  std::vector<std::pair<std::vector<std::int64_t>, std::vector<std::int64_t>>> counts;
  {
    const std::scoped_lock lock(counts_mutex_);
    counts.swap(counts_);
  }
  if (!map_) {
    return fail(ErrorKind::kInvalidInput, "apply_counts: no map");
  }
  for (const auto& [visible, found] : counts) {
    if (auto ok = add_counts(*map_, visible, found); !ok) {
      return ok;
    }
  }
  return {};
}

Result<void> MapOwner::corrected() {
  ++corrections_;  // counted first: the view published next carries the new count
  return changed();
}

void MapOwner::publish(const Map& m) {
  std::vector<std::int64_t> points;
  std::int64_t reference = -1;
  {
    const std::scoped_lock lock(view_mutex_);
    points = anchor_points_;
    reference = anchor_reference_;
  }
  const auto rows = view_keyframes(m, points, reference, options_);
  assert(rows.has_value() && "view_keyframes failed on tracking's points");  // as in view_of
  // A writer publishes in its turn, where no correction runs: the count is the map's.
  View made{.map = std::make_shared<const Map>(rows ? sub_map(m, *rows) : m),
            .keyframes = static_cast<std::int64_t>(keyframe_count(m)),
            .corrections = corrections_.load()};
  Completion done;
  done.frame = clock_ ? clock_() : 0;
  done.view = size_of(*made.map);
  const std::scoped_lock lock(view_mutex_);
  view_ = std::move(made);
  published_.push_back(std::move(done));
}

void MapOwner::clear_gone() {
  if (map_) {
    edge::clear_gone(*map_);
  }
}

std::pair<std::optional<Map>, std::optional<Database>> MapOwner::close() {
  std::pair<std::optional<Map>, std::optional<Database>> out{std::move(map_), std::move(database_)};
  map_.reset();
  database_.reset();
  {
    const std::scoped_lock lock(view_mutex_);
    view_ = {};
    anchor_points_.clear();
    anchor_reference_ = -1;
  }
  const std::scoped_lock lock(counts_mutex_);
  counts_.clear();
  return out;
}

void MapOwner::resume(Map m, std::optional<Database> database) {
  ++corrections_;  // counted first: the view made once tracking anchors carries the new count
  map_ = std::move(m);
  database_ = vocabulary_ ? std::move(database) : std::nullopt;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the merge's two offsets, as Python's
Result<void> MapOwner::take_merged(Map m, Database database, std::int64_t keyframe_offset,
                                   std::int64_t point_offset) {
  map_ = std::move(m);
  if (vocabulary_) {
    database_ = std::move(database);
  }
  {
    const std::scoped_lock lock(view_mutex_);  // where tracking was, by the merged map's ids
    offset_ids(anchor_points_, point_offset);
    if (anchor_reference_ >= 0) {
      anchor_reference_ += keyframe_offset;
    }
  }
  ++corrections_;  // counted first, as a correction: the view published next carries the new count
  return changed();
}

Result<void> MapOwner::changed() {
  if (auto ok = sync_database(); !ok) {
    return ok;
  }
  if (publishing() && map_) {
    publish(*map_);
  }
  return {};
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

Pipeline::Pipeline(Mode mode, Camera camera, Options options,
                   std::optional<foundations::vocabulary::Vocabulary> vocabulary,
                   Threading threading)
    : mode_(mode),
      camera_(std::move(camera)),
      options_(options),
      shared_(std::make_unique<Shared>(std::move(vocabulary), options_, threading)) {
  if (threaded(threading)) {
    threads_ = std::make_unique<PipelineThreads>(shared_->owner(), shared_->atlas(),
                                                 shared_->closing(), mode_, camera_, options_);
  }
}

Pipeline::~Pipeline() = default;

Pipeline::Pipeline(Pipeline&&) noexcept = default;

Result<Pipeline> Pipeline::create(Mode mode, Camera camera, Options options,
                                  std::optional<foundations::vocabulary::Vocabulary> vocabulary,
                                  Threading threading) {
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
  return Pipeline(mode, std::move(camera), options, std::move(vocabulary), threading);
}

foundations::orb::Options Pipeline::extraction() const {
  return edge::extraction(options_, mode_ == Mode::kMonocular && state_ == State::kNotInitialized);
}

Result<FrameResult> Pipeline::track(const Frame& frame) {
  if (finished_) {
    return fail(ErrorKind::kInvalidInput, "track: the run is finished");
  }
  const auto start = Clock::now();
  // Between frames the state is never kLost: a lost track closes its map on its frame.
  const bool tracking_map = state_ == State::kTracking;
  auto stepped = tracking_map ? merge_then_track(frame) : initialize(frame);
  if (!stepped) {
    return std::unexpected(std::move(stepped.error()));
  }
  Step step = std::move(*stepped);
  MapOwner& owner = shared_->owner();
  Atlas& atlas = shared_->atlas();
  double waited = step.wait_seconds;
  if (!threads_) {
    atlas.date(owner.map() ? &*owner.map() : nullptr);
    owner.clear_gone();
  } else {
    if (auto ok = threads_->check(); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
    if (!tracking_map && owner.map()) {  // a map started on it: the threads have none of it
      atlas.record(&*owner.map());
      owner.clear_gone();
    }
    if (step.lost && owner.map()) {  // the threads end the map's work first
      const auto settling = Clock::now();
      if (auto ok = threads_->settle(false); !ok) {
        return std::unexpected(std::move(ok.error()));
      }
      waited += Seconds(Clock::now() - settling).count();
    }
    atlas.tick();
  }
  const State state = step.lost ? State::kLost : state_;
  const std::optional<std::int64_t> map_id = step.merged_from ? step.merged_from : atlas.map_id();
  const std::optional<Tracked>& tracked = step.tracked;
  Fates fates = fates_of(frame, step.matched, tracked, step.made, atlas);
  if (auto ok = close_lost(step); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const Seconds took = Clock::now() - start;
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
                     .fates = std::move(fates),
                     .wait_seconds = waited,
                     .asked = step.asked,
                     .refused = step.refused,
                     .queue = step.queue,
                     .schedule = step.schedule};
}

Result<void> Pipeline::finish() {
  if (threads_ && !finished_) {
    finished_ = true;
    shared_->atlas().end();
    auto settled = threads_->settle(true);
    if (settled) {  // a merge found last is made
      if (auto made = threads_->merge_waiting(tracking_); !made) {
        settled = std::unexpected(std::move(made.error()));
      }
    }
    threads_->stop();
    return settled;
  }
  finished_ = true;
  return {};
}

std::vector<Completion> Pipeline::completed() {
  std::vector<Completion> out = threads_ ? threads_->completed() : std::vector<Completion>{};
  std::ranges::move(shared_->owner().published(), std::back_inserter(out));
  return out;
}

Lifetimes Pipeline::lifetimes() const {
  const MapOwner& owner = shared_->owner();
  return owner.reading(
      [&] { return shared_->atlas().lifetimes(owner.map() ? &*owner.map() : nullptr); });
}

Trajectory Pipeline::trajectory() const {
  const MapOwner& owner = shared_->owner();
  return owner.reading(
      [&] { return shared_->atlas().trajectory(owner.map() ? &*owner.map() : nullptr); });
}

Result<LostSummary> Pipeline::leave(Closing why) {
  auto [m, database] = shared_->owner().close();
  const std::size_t keyframes = m ? keyframe_count(*m) : 0;
  const LossOptions& loss = options_.loss;
  const LostSummary out{.keyframes = static_cast<std::int64_t>(keyframes),
                        .points = m ? static_cast<std::int64_t>(point_count(*m)) : 0,
                        .reset = why == Closing::kSession
                                     ? std::cmp_less(keyframes, loss.session_keyframes)
                                     : std::cmp_less_equal(keyframes, loss.reset_keyframes)};
  if (m) {
    const bool dated = why == Closing::kLost;
    if (auto ok = shared_->atlas().close(std::move(*m), std::move(database), out.reset, dated);
        !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  shared_->closing() = LoopState{};  // shared with loop closing's thread, settled by now
  if (threads_) {
    threads_->drop_merge();  // with the loop-closing state it was in
  }
  return out;
}

Result<void> Pipeline::close_lost(Step& step) {
  if (!step.lost || !shared_->owner().map()) {  // a lost frame was tracking a map
    return {};
  }
  auto closed = leave(Closing::kLost);
  if (!closed) {
    return std::unexpected(std::move(closed.error()));
  }
  step.summary.lost = *closed;
  tracking_ = TrackingState{};
  state_ = State::kNotInitialized;
  return {};
}

Result<std::optional<LostSummary>> Pipeline::start_session() {
  if (finished_) {
    return fail(ErrorKind::kInvalidInput, "start_session: the run is finished");
  }
  std::optional<LostSummary> closed;
  if (shared_->owner().map()) {
    if (threads_) {  // the threads end the map's work first, as for a map lost
      if (auto ok = threads_->settle(false); !ok) {
        return std::unexpected(std::move(ok.error()));
      }
    }
    auto left = leave(Closing::kSession);
    if (!left) {
      return std::unexpected(std::move(left.error()));
    }
    closed = *left;
  }
  tracking_ = TrackingState{};
  state_ = State::kNotInitialized;
  return closed;
}

Result<std::pair<LostSummary, double>> Pipeline::switch_map(std::int64_t number,
                                                            std::unique_lock<std::mutex>& lock) {
  double waited = 0.0;
  if (threads_) {  // the threads end the map's work first, as for a map lost
    const auto settling = Clock::now();
    const bool holding = lock.owns_lock();  // the baseline's: mapping takes it to finish
    if (holding) {
      lock.unlock();
    }
    auto settled = threads_->settle(false);
    if (holding) {
      lock.lock();
    }
    if (!settled) {
      return std::unexpected(std::move(settled.error()));
    }
    waited = Seconds(Clock::now() - settling).count();
  }
  auto left = leave(Closing::kLeft);
  if (!left) {
    return std::unexpected(std::move(left.error()));
  }
  auto kept = shared_->atlas().resume(number);
  if (!kept) {
    return std::unexpected(std::move(kept.error()));
  }
  shared_->owner().resume(std::move(kept->map), std::move(kept->database));
  change_map(tracking_);
  return std::pair{*left, waited};
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
                        .lost = std::nullopt,
                        .left = std::nullopt},
            .matched = {},
            .made = {},
            .lost = false,
            .wait_seconds = 0.0,
            .asked = false,
            .refused = false,
            .queue = 0,
            .schedule = std::nullopt,
            .merged_from = std::nullopt};
  if (!attempt->started) {
    return step;
  }
  Atlas& atlas = shared_->atlas();
  MapOwner& owner = shared_->owner();
  atlas.start();
  state_ = State::kTracking;
  Tracked tracked = attempt->started->tracked;
  const Map& started = attempt->started->map;
  const std::int64_t first = started.kf_id.back();  // the keyframe it started on
  atlas.place_frame(started, frame.timestamp, first, tracked.pose);
  owner.anchor(tracked.points, first);  // what the first view follows
  if (auto ok = owner.start(std::move(attempt->started->map)); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = accept(tracking_, frame, tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (threads_) {
    const View view = owner.view();  // the map's first view, of the keyframe it started on
    const std::int64_t newest = keyframe_row(*view.map, first);
    anchor_to(tracking_, *view.map, static_cast<std::size_t>(newest), view.corrections);
  }
  step.made = tracked.points;  // every point the frame observes was made of it
  step.tracked = std::move(tracked);
  step.keyframe = true;
  step.summary.placed_by = Placement::kInitialization;
  return step;
}

Result<Pipeline::Keyframed> Pipeline::make_keyframe(const Map& m, const Frame& frame,
                                                    const Tracked& tracked, FrameSummary& summary) {
  const auto start = Clock::now();
  const std::vector<std::int64_t> holding = held(shared_->closing());
  // Local mapping's step: insert the frame as a keyframe and map around it.
  auto mapping = shared_->owner().apply_mapping([&](Map& changed) {
    return process(changed, camera_, frame, tracked, mode_, options_, holding);
  });
  if (!mapping) {
    return std::unexpected(std::move(mapping.error()));
  }
  const Seconds mapped = Clock::now() - start;
  summary.mapping = std::move(mapping->summary);
  // The frame's reference is its own keyframe, placed where the frame was.
  shared_->atlas().place_keyframe(m, frame.timestamp, tracked.pose);
  const auto searching = Clock::now();
  auto looped =
      close_loops(shared_->closing(), shared_->owner(), shared_->atlas(), camera_, mode_, options_);
  if (!looped) {
    return std::unexpected(std::move(looped.error()));
  }
  Keyframed out{.map_seconds = mapped.count(),
                .loop_seconds = 0.0,
                .corrected = looped->corrected,
                .made = std::move(mapping->made),
                .schedule = mapping->schedule,
                .point_offset = looped->point_offset};
  if (looped->summary) {
    out.loop_seconds = Seconds(Clock::now() - searching).count();
    summary.loop = std::move(looped->summary);
  }
  return out;
}

Result<void> Pipeline::hand_over(const Frame& frame, const View& seen, const Tracked& tracked,
                                 std::int64_t reference, bool keyframe) {
  Atlas& atlas = shared_->atlas();
  const Map& view = *seen.map;
  const std::int64_t row =
      reference >= 0 ? reference : static_cast<std::int64_t>(keyframe_count(view)) - 1;
  const auto k = static_cast<std::size_t>(row);
  if (keyframe) {
    const std::int64_t index = atlas.frames();
    atlas.place_pending(index, frame.timestamp, tracked.pose);
    threads_->request(
        {.index = index,
         .frame = frame,
         .tracked = tracked,
         .reference = view.kf_id[k],
         .relative = Eigen::Matrix4d(tracked.pose * foundations::sim3::inverse(view.kf_pose[k])),
         .corrections = seen.corrections});
  } else {
    atlas.place_frame(view, frame.timestamp, view.kf_id[k], tracked.pose);
  }
  if (auto ok = accept(tracking_, frame, tracked); !ok) {
    return ok;
  }
  anchor_to(tracking_, view, k, seen.corrections);
  shared_->owner().anchor(tracked.points, view.kf_id[k]);  // what the next views follow
  return {};
}

Result<Pipeline::Step> Pipeline::merge_then_track(const Frame& frame) {
  double paused = 0.0;
  if (threads_) {  // a merge found is made between two frames
    auto made = threads_->merge_waiting(tracking_);
    if (!made) {
      return std::unexpected(std::move(made.error()));
    }
    paused = *made;
  }
  auto step = track_frame(frame);
  if (step) {
    step->wait_seconds += paused;
  }
  return step;
}

Result<Pipeline::Step> Pipeline::track_frame(const Frame& frame) {
  MapOwner& owner = shared_->owner();
  const auto asked = Clock::now();
  std::unique_lock<std::mutex> lock = owner.locked();  // the baseline's, through the frame
  // The baseline's tracking waits for the map's lock as ours waits for a hold: its wait.
  const double locked = lock.owns_lock() ? Seconds(Clock::now() - asked).count() : 0.0;
  // Read once for the frame: the map and the corrections it carries, published together.
  View view = owner.view();
  if (threads_ && view.map) {
    follow(tracking_, *view.map, view.corrections);
  }
  auto found = track_step(tracking_, owner, std::move(view), shared_->atlas(), camera_, frame,
                          mode_, options_, threads_.get(),
                          [this, &lock](std::int64_t number) { return switch_map(number, lock); });
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  Step step{.tracked = std::nullopt,
            .keyframe = false,
            .map_seconds = 0.0,
            .loop_seconds = 0.0,
            .summary = std::move(found->summary),
            .matched = std::move(found->matched),
            .made = {},
            .lost = found->lost,
            .wait_seconds = found->wait_seconds + locked,
            .asked = found->asked,
            .refused = found->refused,
            .queue = found->queue,
            .schedule = std::nullopt,
            .merged_from = std::nullopt};
  if (!found->tracked) {
    return step;
  }
  Tracked& tracked = *found->tracked;
  step.keyframe = found->keyframe;
  if (threads_) {
    if (auto ok = hand_over(frame, found->view, tracked, found->reference, found->keyframe); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
    step.tracked = std::move(found->tracked);
    return step;
  }
  // Inline, the map itself, which mapping changes in place; the frame's reference by its id in
  // the view it was tracked on (the map itself, or the test's view of it).
  const std::optional<Map>& current = owner.map();
  if (!current) {
    return fail(ErrorKind::kInvalidInput, "track: no map to track against");
  }
  const Map& m = *current;
  const Map& seen = *found->view.map;
  const std::int64_t reference = seen.kf_id[static_cast<std::size_t>(
      found->reference >= 0 ? found->reference
                            : static_cast<std::int64_t>(keyframe_count(seen)) - 1)];
  if (owner.views()) {
    owner.anchor(tracked.points, reference);  // what the next frame's view follows
  }
  Keyframed keyframed;
  if (found->keyframe) {
    const std::optional<std::int64_t> before = shared_->atlas().map_id();
    auto made = make_keyframe(m, frame, tracked, step.summary);
    if (!made) {
      return std::unexpected(std::move(made.error()));
    }
    keyframed = std::move(*made);
    step.schedule = keyframed.schedule;
    if (keyframed.point_offset) {  // the frame's ids are the merged map's now
      step.merged_from = before;
      offset_ids(tracked.points, *keyframed.point_offset);
      offset_ids(step.matched, *keyframed.point_offset);
      offset_ids(keyframed.made, *keyframed.point_offset);
      if (owner.views()) {  // the next frame's view follows the merged map's ids
        owner.anchor(tracked.points, m.kf_id.back());
      }
    }
  } else {
    shared_->atlas().place_frame(m, frame.timestamp, reference, tracked.pose);
  }
  if (auto ok = accept(tracking_, frame, tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (keyframed.corrected || keyframed.point_offset) {  // the last frame, the keyframe's own, moved
    move_last(tracking_, m.kf_pose.back());
  }
  step.tracked = std::move(found->tracked);
  step.map_seconds = keyframed.map_seconds;
  step.loop_seconds = keyframed.loop_seconds;
  step.made = std::move(keyframed.made);
  return step;
}

}  // namespace slam_it::edge
