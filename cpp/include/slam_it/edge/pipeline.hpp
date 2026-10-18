/// @file
/// @brief The edge pipeline, initialising, tracking each frame and mapping on keyframes: the
///        C++ core's mirror of `slam_it.edge.pipeline`, whose docstring states the per-frame
///        loop, the steps' own state, the map's owner and the steps in threads of their own. It
///        runs in the native program, not across the binding.
///
/// Threaded, the published view is a copy of the map (`std::shared_ptr<const Map>`), made
/// each time a writer publishes: tracking holds the copy it read for its whole frame, and the
/// writers go on changing the map in place. Inline, the view is the map itself.
///
/// C++ also runs the threads by ORB-SLAM's own locking (`Threading::kLocks`), the baseline the
/// design compares ours with (section 12, item 7).
#pragma once

#include <Eigen/Core>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "slam_it/edge/atlas.hpp"
#include "slam_it/edge/database.hpp"
#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/initialization.hpp"
#include "slam_it/edge/loops.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/mapping.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/edge/relocalization.hpp"
#include "slam_it/edge/threads.hpp"
#include "slam_it/edge/tracking.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/orb.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace slam_it::edge {

/// @brief Where the pipeline is, with the codes it is written as.
enum class State : std::uint8_t {
  /// @brief No map: before the first, or after a lost one, until a new one starts.
  kNotInitialized = 0,
  /// @brief Tracking frames against the map.
  kTracking = 1,
  /// @brief The track was lost on this frame: its map was kept or reset, and the next frame
  ///        starts a new one. Only a frame's result has it.
  kLost = 2,
  /// @brief The number of states (a sentinel, not a state).
  kCount = 3,
};

/// @brief A state's name, as `slam_it.edge.pipeline.State` spells it.
/// @param state The state.
/// @return A static string.
[[nodiscard]] const char* name(State state);

/// @brief How a frame was placed, before the local map refined it, with the codes it is
///        written as (`slam_it.edge.pipeline.Placement`).
enum class Placement : std::uint8_t {
  /// @brief It started the map.
  kInitialization = 0,
  /// @brief Tracking the last frame, from the constant-velocity prediction.
  kMotion = 1,
  /// @brief The fallback: PnP against the local map.
  kLocalPnp = 2,
  /// @brief Relocalised against the whole map, through its keyframe database.
  kRelocalization = 3,
  /// @brief The number of placements (a sentinel, not a placement).
  kCount = 4,
};

/// @brief A placement's name, as `slam_it.edge.pipeline.Placement` spells it.
/// @param placement The placement.
/// @return A static string.
[[nodiscard]] const char* name(Placement placement);

/// @brief What became of a keypoint on its frame, with the codes a run records
///        (`slam_it.edge.pipeline.Fate`, whose docstring defines each): the first that
///        applies.
enum class Fate : std::uint8_t {
  /// @brief The frame's last tracking stage did not match it, and no point was made of it.
  kUnmatched = 0,
  /// @brief The frame's last tracking stage matched it, and it was not tracked in the end.
  kRejected = 1,
  /// @brief An inlier of the frame's final pose.
  kTracked = 2,
  /// @brief A point was made of it on its frame.
  kMade = 3,
};

/// @brief What became of each of a frame's keypoints, and the point concerned
///        (`slam_it.edge.pipeline.Fates`).
struct Fates {
  /// @brief Each keypoint's fate.
  std::vector<Fate> fate;
  /// @brief The id of the point concerned: the one made of it, the one it was tracked to, or the
  ///        one it was matched to and rejected from; -1 if unmatched.
  std::vector<std::int64_t> point;
};

/// @brief The map a lost track closed, as a run records it on the frame it was lost on
///        (`slam_it.edge.pipeline.LostSummary`).
struct LostSummary {
  /// @brief The map's keyframes then.
  std::int64_t keyframes{0};
  /// @brief Its points.
  std::int64_t points{0};
  /// @brief Whether it was reset, having `loss.reset_keyframes` keyframes or fewer, rather than
  ///        kept.
  bool reset{false};
};

/// @brief How loop closing went on a keyframe (`slam_it.edge.pipeline.LoopSummary`); its time
///        is the frame's `loop_seconds`.
struct LoopSummary {
  /// @brief The search.
  loops::DetectionSummary detection;
  /// @brief The correction, when a loop closed.
  std::optional<loops::CorrectionSummary> correction;
};

/// @brief How the stages went on one frame (`slam_it.edge.pipeline.FrameSummary`); each
///        empty where the stage did not run.
struct FrameSummary {
  /// @brief How the frame was placed, if it was; a frame placed may still fail the local map.
  std::optional<Placement> placed_by;
  /// @brief For a frame before the map existed.
  std::optional<InitSummary> init;
  /// @brief Tracking the last frame.
  std::optional<MotionSummary> motion;
  /// @brief The PnP fallback, run only when tracking the last frame failed.
  std::optional<PnpSummary> pnp;
  /// @brief Relocalisation, run only when the fallback failed too.
  std::optional<RelocalizationSummary> relocalization;
  /// @brief Tracking the local map, run on a placed frame.
  std::optional<LocalMapSummary> local_map;
  /// @brief Local mapping, on a keyframe after the first.
  std::optional<MappingSummary> mapping;
  /// @brief Loop closing, on a keyframe after the first while it is on.
  std::optional<LoopSummary> loop;
  /// @brief The map closed, on the frame the track was lost on.
  std::optional<LostSummary> lost;
};

/// @brief What tracking one frame gave (`slam_it.edge.pipeline.FrameResult`).
struct FrameResult {
  /// @brief The frame's time.
  double timestamp{0.0};
  /// @brief The frame's: the pipeline's after it, or `kLost` on the frame the track was lost on.
  State state{State::kNotInitialized};
  /// @brief The map it was tracked in, or lost from; empty when there was none.
  std::optional<std::int64_t> map;
  /// @brief The frame's `T_cam_world`, if it was tracked.
  std::optional<Eigen::Matrix4d> pose;
  /// @brief Its keypoints matched to map points.
  std::int64_t tracked{0};
  /// @brief Whether it became a keyframe (or started the map).
  bool keyframe{false};
  /// @brief The time tracking it took, in seconds, initialisation included: what ORB-SLAM's
  ///        tracking thread does.
  double track_seconds{0.0};
  /// @brief The time mapping it took as a keyframe (`process`), 0 for any other frame: what
  ///        ORB-SLAM's local-mapping thread does.
  double map_seconds{0.0};
  /// @brief The time loop closing took on it as a keyframe, 0 for any other frame: what
  ///        ORB-SLAM's loop-closing thread does.
  double loop_seconds{0.0};
  /// @brief How its stages went.
  FrameSummary summary;
  /// @brief What became of each of its keypoints.
  Fates fates;
  /// @brief The time tracking waited for the map, part of `track_seconds`: 0 inline; threaded,
  ///        relocalisation's wait for mapping to let the map go, and a lost map's for the
  ///        threads; by ORB-SLAM's locks, its wait for the map's lock too.
  double wait_seconds{0.0};
  /// @brief Whether it wanted a keyframe (`need_keyframe`).
  bool asked{false};
  /// @brief Whether it wanted one and mapping could not take it: stopped, or busy and its queue
  ///        full (a monocular frame's queue being none). Never inline.
  bool refused{false};
  /// @brief The keyframes waiting for mapping when it asked; 0 inline, and when it did not ask.
  std::int64_t queue{0};
  /// @brief How mapping scheduled its keyframe's steps, inline; threaded, it comes later
  ///        (`Completion`).
  std::optional<Schedule> schedule;
};

/// @brief What a thread of its own finished after its frame's result was given
///        (`slam_it.edge.pipeline.Completion`, threaded runs): a keyframe's mapping and loop
///        closing on the keyframe's frame; a global BA's on the frame being tracked when it
///        ended, or the last frame when the run's end waited for it. Each part is set only by
///        the step that finished.
struct Completion {
  /// @brief The frame, counted from 0, one per `Pipeline::track`.
  std::int64_t frame{0};
  /// @brief A keyframe's mapping: its time, in seconds.
  std::optional<double> map_seconds;
  /// @brief How it went.
  std::optional<MappingSummary> mapping;
  /// @brief How its steps were scheduled.
  std::optional<Schedule> schedule;
  /// @brief The keyframe's keypoints' points made by mapping, by the run's ids, -1 where none:
  ///        their fate is `Fate::kMade`. Empty but for a keyframe's mapping.
  std::vector<std::int64_t> made;
  /// @brief Loop closing on a keyframe: its time, in seconds.
  std::optional<double> loop_seconds;
  /// @brief How it went.
  std::optional<LoopSummary> loop;
  /// @brief A global BA in a thread of its own, ended.
  std::optional<loops::GlobalSummary> global_ba;
  /// @brief Its time, in seconds.
  double global_ba_seconds{0.0};
};

/// @brief Tracking's own state from frame to frame (`slam_it.edge.pipeline.TrackingState`,
///        ORB-SLAM's `Tracking`'s). A map closed takes it with it.
struct TrackingState {
  /// @brief The last tracked frame.
  std::optional<Frame> last_frame;
  /// @brief Its pose and matches.
  std::optional<Tracked> last_tracked;
  /// @brief The last motion `T_last_previous`, if the last two frames were tracked.
  std::optional<Eigen::Matrix4d> velocity;
  /// @brief The monocular initialisation's reference frame.
  std::optional<Frame> reference;
  /// @brief Where each of the reference's keypoints was last matched.
  std::vector<std::array<double, 2>> previous;
  /// @brief Frames in a row not tracked.
  std::int64_t failures{0};
  /// @brief Frames since the last recovery, if any.
  std::optional<std::int64_t> since_recovery;
  /// @brief The first failed frame's time, while the track is recently lost.
  std::optional<double> lost_since;
  /// @brief The frames since the last that became a keyframe (ORB-SLAM's `mnLastKeyFrameId`).
  std::int64_t since_keyframe{0};
  /// @brief Threaded: the keyframe the last frame was tracked against, by the map's id.
  std::optional<std::int64_t> anchor;
  /// @brief Threaded: the last frame's pose relative to it, `T_last_anchor`.
  std::optional<Eigen::Matrix4d> anchor_relative;
  /// @brief Threaded: the corrections of the map seen (`MapOwner::corrections`).
  std::int64_t corrections{0};
};

/// @brief Make a tracked frame the last one; the velocity only spans consecutive frames
///        (`slam_it.edge.pipeline.TrackingState.accept`).
/// @param state Tracking's state.
/// @param frame The frame.
/// @param tracked Its pose and matches.
/// @return Nothing, or an error if the last pose cannot be inverted.
foundations::Result<void> accept(TrackingState& state, const Frame& frame, const Tracked& tracked);

/// @brief Count a frame that was not tracked, which ends the velocity
///        (`slam_it.edge.pipeline.TrackingState.fail`).
/// @param state Tracking's state.
/// @param frame The frame.
/// @param keyframes The map's keyframes.
/// @param options The settings.
/// @return Whether it loses the track: past `loss.relocalize_seconds` since the first failure,
///         or for a small map or with `relocalization.enabled` off, on the `loss.lost_frames`-th
///         in a row.
bool record_failure(TrackingState& state, const Frame& frame, std::size_t keyframes,
                    const Options& options);

/// @brief Make a frame the monocular reference, each keypoint searched from its own position.
/// @param state Tracking's state.
/// @param frame The frame.
void set_reference(TrackingState& state, const Frame& frame);

/// @brief Move the last frame tracked to its keyframe's pose, after a correction moved it
///        (`slam_it.edge.pipeline.TrackingState.move_last`).
/// @param state Tracking's state.
/// @param pose Its new `T_cam_world`.
void move_last(TrackingState& state, const Eigen::Matrix4d& pose);

/// @brief Keep the last frame's pose relative to the keyframe it was tracked against
///        (`slam_it.edge.pipeline.TrackingState.anchor_to`, threaded).
/// @param state Tracking's state, its last frame tracked.
/// @param view The map tracking read.
/// @param row The keyframe's row in it.
void anchor_to(TrackingState& state, const Map& view, std::size_t row);

/// @brief Move the last frame with its anchor keyframe, if the map was corrected since
///        (`slam_it.edge.pipeline.TrackingState.follow`, threaded): ORB-SLAM's
///        `UpdateLastFrame` places the last frame by its reference keyframe; here only after a
///        correction, as inline only the keyframe's own frame moves.
/// @param state Tracking's state.
/// @param view The map's view now.
/// @param corrections The corrections of the map so far.
void follow(TrackingState& state, const Map& view, std::int64_t corrections);

/// @brief Loop closing's own state from keyframe to keyframe (`slam_it.edge.pipeline.LoopState`,
///        ORB-SLAM3's `LoopClosing`'s). A map closed takes it with it.
struct LoopState {
  /// @brief The loop being confirmed in the map, if any.
  std::optional<loops::Pending> pending;
};

/// @brief The keyframes the loop being confirmed holds, which mapping's culling spares
///        (`slam_it.edge.pipeline.LoopState.held`).
/// @param state Loop closing's state.
/// @return Their ids in the map; none when no loop is being confirmed.
[[nodiscard]] std::vector<std::int64_t> held(const LoopState& state);

/// @brief The map being tracked and its keyframe database (`slam_it.edge.pipeline.MapOwner`):
///        every change to them goes through here. The steps change the map in place, so the
///        owner runs each step that changes it on its map, then keeps the database in step.
///        Inline, it holds the map between the steps, and the view is the map. Threaded (a
///        writer given), one thread writes the map at a time (`threads::Writer`), tracking reads
///        the view the writers publish, a copy, and tracking's counts wait for local mapping to
///        apply them (`apply_counts`). The map and the database are read and written only under
///        the writer's hand-over: local mapping's turn, a hold or a stop.
class MapOwner {
 public:
  /// @brief An owner with no map yet.
  /// @param vocabulary The database's vocabulary; none when neither relocalising nor closing
  ///        loops, and there is no database.
  /// @param writer The handshake between the threads, not owned; null inline.
  /// @param lock The map's lock, ORB-SLAM's `mMutexMapUpdate`, not owned: the baseline's
  ///        (`Threading::kLocks`), which publishes no view; null otherwise.
  MapOwner(std::optional<foundations::vocabulary::Vocabulary> vocabulary, threads::Writer* writer,
           std::mutex* lock)
      : vocabulary_(std::move(vocabulary)), writer_(writer), lock_(lock) {}

  /// @brief The map being tracked: none until one starts, and from a lost one to the next.
  ///        Threaded, read it only under the writer's hand-over.
  /// @return The map, if any.
  [[nodiscard]] const std::optional<Map>& map() const { return map_; }

  /// @brief The map as last published, what tracking reads: a copy, threaded; the map itself,
  ///        inline and in the baseline (not owned, valid while the map is).
  /// @return The view, or null with no map.
  [[nodiscard]] std::shared_ptr<const Map> view() const;

  /// @brief Whether the writers publish a view: threaded, our design.
  /// @return Whether they do.
  [[nodiscard]] bool publishing() const { return writer_ != nullptr && lock_ == nullptr; }

  /// @brief Take the map's lock, the baseline's (tracking through its frame, mapping through
  ///        its steps); nothing otherwise.
  /// @return The lock, held, or none.
  [[nodiscard]] std::unique_lock<std::mutex> locked() const {
    return lock_ != nullptr ? std::unique_lock<std::mutex>(*lock_) : std::unique_lock<std::mutex>();
  }

  /// @brief Its keyframe database: none when neither relocalising nor closing loops, or with no
  ///        map. Threaded, read it only under the writer's hand-over.
  /// @return The database, if any.
  [[nodiscard]] const std::optional<Database>& database() const { return database_; }

  /// @brief The database's vocabulary, when relocalising or closing loops.
  /// @return The vocabulary, if any.
  [[nodiscard]] const std::optional<foundations::vocabulary::Vocabulary>& vocabulary() const {
    return vocabulary_;
  }

  /// @brief The handshake between the threads.
  /// @return The writer; null inline.
  [[nodiscard]] threads::Writer* writer() const { return writer_; }

  /// @brief The corrections applied: loops corrected, and global BAs carried onto the map.
  /// @return The count.
  [[nodiscard]] std::int64_t corrections() const { return corrections_.load(); }

  /// @brief Take a map just started, publish it, and start its database with its keyframes.
  /// @param m The new map.
  /// @return Nothing; a quantisation's error.
  foundations::Result<void> start(Map m);

  /// @brief Count a frame tracked against the local map: the points seen, and found, in it.
  ///        Threaded, the counts wait for local mapping (`apply_counts`).
  /// @param visible The points the frame's view held, by the map's ids.
  /// @param found The points it tracked.
  /// @return Nothing; `InvalidInput` with no map, inline.
  foundations::Result<void> count(const std::vector<std::int64_t>& visible,
                                  const std::vector<std::int64_t>& found);

  /// @brief Apply the counts tracking sent since the last (threaded; local mapping's, in its
  ///        turn). A point gone since is skipped.
  /// @return Nothing; `InvalidInput` with no map.
  foundations::Result<void> apply_counts();

  /// @brief Run a mapping step on the map, then bring the database to its keyframes and,
  ///        threaded, publish the map.
  /// @tparam Step A callable taking the map, changed in place, and giving a `Result`.
  /// @param step The step.
  /// @return The step's result; `InvalidInput` with no map, or the database's error.
  template <typename Step>
  auto apply_mapping(Step&& step) -> std::invoke_result_t<Step, Map&> {
    if (!map_) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput, "mapping: no map");
    }
    auto out = std::forward<Step>(step)(*map_);
    if (!out) {
      return out;
    }
    if (auto ok = changed(); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
    return out;
  }

  /// @brief Run a loop's correction on the map, held through the handshake (`stopped`), then
  ///        take it as corrected (`corrected`).
  /// @tparam Step A callable taking the map, changed in place, and giving a `Result`.
  /// @param step The correction.
  /// @return The correction's result; `InvalidInput` with no map, or the database's error.
  template <typename Step>
  auto apply_correction(Step&& step) -> std::invoke_result_t<Step, Map&> {
    return stopped([this, &step](Map& m) -> std::invoke_result_t<Step, Map&> {
      auto out = std::forward<Step>(step)(m);
      if (!out) {
        return out;
      }
      if (auto ok = corrected(); !ok) {
        return std::unexpected(std::move(ok.error()));
      }
      return out;
    });
  }

  /// @brief The handshake: run a change on the map held, mapping stopped and its local BA
  ///        aborted, then mapping released, as ORB-SLAM's loop closing does (`RequestStop`,
  ///        `Release`; a global BA running is superseded by the caller's new generation).
  ///        Inline nothing runs beside the change, so there is nothing to stop. The baseline
  ///        then takes the map's lock, as `CorrectLoop` does. Loop corrections, global BAs'
  ///        results and milestone 2B's B4's merges all enter here.
  /// @tparam Step A callable taking the map, changed in place, and giving a `Result`.
  /// @param step The change.
  /// @return The change's result; `InvalidInput` with no map.
  template <typename Step>
  auto stopped(Step&& step) -> std::invoke_result_t<Step, Map&> {
    std::optional<threads::Writer::Stopped> stop;
    if (writer_ != nullptr) {
      stop.emplace(*writer_);
    }
    const std::unique_lock<std::mutex> held = locked();
    if (!map_) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput, "stopped: no map");
    }
    return std::forward<Step>(step)(*map_);
  }

  /// @brief Take the map as corrected, inside the handshake (`stopped`): bring the database to
  ///        its keyframes, publish it, and count the correction.
  /// @return Nothing; the database's error.
  foundations::Result<void> corrected();

  /// @brief Publish the map so far for tracking to read, while its writer goes on (threaded):
  ///        a copy.
  /// @param m The map so far.
  void publish(const Map& m);

  /// @brief Run something with the map held whole, local mapping held off (a hold, threaded;
  ///        the map's lock, in the baseline; nothing to hold inline).
  /// @tparam Read A callable taking nothing.
  /// @param read What to run.
  /// @return What it gives.
  template <typename Read>
  auto reading(Read&& read) const -> std::invoke_result_t<Read> {
    const std::unique_lock<std::mutex> held = locked();
    std::optional<threads::Writer::Held> hold;
    if (writer_ != nullptr && lock_ == nullptr) {
      hold.emplace(*writer_);
    }
    return std::forward<Read>(read)();
  }

  /// @brief Empty the map's archive of removals, once the atlas has dated them.
  void clear_gone();

  /// @brief Give up the map, lost: the database and the view go with it.
  /// @return The map, if there was one.
  std::optional<Map> close();

 private:
  /// @brief After a step changed the map: bring the database to it and, threaded, publish it.
  /// @return Nothing; a quantisation's error.
  foundations::Result<void> changed();

  /// @brief Bring the keyframe database to the map's keyframes: add the new, erase the culled.
  /// @return Nothing; a quantisation's error.
  foundations::Result<void> sync_database();

  /// @brief The vocabulary, when relocalising or closing loops.
  std::optional<foundations::vocabulary::Vocabulary> vocabulary_;
  /// @brief The handshake between the threads; null inline.
  threads::Writer* writer_;
  /// @brief The map's lock, the baseline's; null otherwise.
  std::mutex* lock_;
  /// @brief The map, once initialised.
  std::optional<Map> map_;
  /// @brief The map's keyframe database, when relocalising or closing loops and a map exists.
  std::optional<Database> database_;
  /// @brief Guards `view_`.
  mutable std::mutex view_mutex_;
  /// @brief The map as last published.
  std::shared_ptr<const Map> view_;
  /// @brief Guards `counts_`.
  std::mutex counts_mutex_;
  /// @brief Tracking's counts waiting for local mapping: the points seen, and found, per frame.
  std::vector<std::pair<std::vector<std::int64_t>, std::vector<std::int64_t>>> counts_;
  /// @brief The corrections applied.
  std::atomic<std::int64_t> corrections_{0};
};

/// @brief Local mapping, loop closing and the global BAs, each in a thread of its own
///        (`slam_it.edge.pipeline._Threads`): defined in pipeline.cpp, for a threaded
///        pipeline.
class PipelineThreads;

/// @brief How a pipeline's steps run and share the map (`Pipeline::create`).
enum class Threading : std::uint8_t {
  /// @brief In turn, in the caller's thread: a run repeats exactly.
  kInline = 0,
  /// @brief In threads of their own, our design: one writer at a time, and a view published for
  ///        tracking.
  kView = 1,
  /// @brief In threads of their own, by ORB-SLAM's locks (the baseline, section 12, item 7),
  ///        the map's lock taken where ORB-SLAM takes a keyframe's or a point's: tracking holds
  ///        it through its frame, reading the map itself and counting its points there;
  ///        mapping holds it through its steps but the local BA's solves (`Around`), letting
  ///        it go once the keyframe's points are made; loop closing searches under it, then
  ///        corrects with mapping stopped and under it, a loop whose keyframes went in between
  ///        left; a global BA's result is carried under it, mapping stopped.
  kLocks = 2,
};

/// @brief The edge pipeline for one sequence (`slam_it.edge.pipeline.Pipeline`): tracking, local
///        mapping and loop closing as steps over state of their own, called in turn, or each in
///        a thread of its own (threaded).
class Pipeline {
 public:
  /// @brief Start a pipeline.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  /// @param vocabulary The vocabulary of the keyframe database's words: required with
  ///        `relocalization.enabled` or `loops.enabled` on, refused with both off.
  /// @param threading How the steps run: inline, the default, a run repeating exactly, or in
  ///        threads of their own.
  /// @return The pipeline; `InvalidInput` for an invalid camera or settings, a stereo or
  ///         RGB-D camera without a baseline, or a vocabulary missing while relocalising or
  ///         closing loops, or given while doing neither.
  [[nodiscard]] static foundations::Result<Pipeline> create(
      Mode mode, Camera camera, Options options,
      std::optional<foundations::vocabulary::Vocabulary> vocabulary,
      Threading threading = Threading::kInline);

  /// @brief End the threads, if they still run.
  ~Pipeline();
  Pipeline(const Pipeline&) = delete;
  Pipeline& operator=(const Pipeline&) = delete;
  /// @brief Move a pipeline: its threads go on, their state where it was.
  Pipeline(Pipeline&&) noexcept;
  Pipeline& operator=(Pipeline&&) = delete;

  /// @brief The extraction settings the next frame should be built with.
  /// @return `features.init_factor` times the features for a monocular frame before the map
  ///         exists, the usual ones otherwise.
  [[nodiscard]] foundations::orb::Options extraction() const;

  /// @brief Track one frame (and map, if it becomes a keyframe; threaded, local mapping is
  ///        given it).
  /// @param frame The next frame, built with `extraction`'s settings.
  /// @return Its pose, or why there is none; a stage's error, the error a thread failed with,
  ///         or `InvalidInput` once the run is finished.
  [[nodiscard]] foundations::Result<FrameResult> track(const Frame& frame);

  /// @brief End the run: threaded, wait for the threads to finish their work, then end them.
  ///        Local mapping and loop closing handle every keyframe given, and each global BA
  ///        running ends, the latest carried onto the map. Inline, nothing waits. No frame is
  ///        tracked after.
  /// @return Nothing; the error a thread failed with.
  foundations::Result<void> finish();

  /// @brief What the threads finished since the last call (threaded; none inline).
  /// @return The completions, in the order they came.
  [[nodiscard]] std::vector<Completion> completed();

  /// @brief The handshake between the threads, threaded: a test holds a stop through it.
  /// @return The writer; null inline.
  [[nodiscard]] threads::Writer* writer() const { return shared_->writer(); }

  /// @brief Where the pipeline is: never `kLost`, which a lost frame's result has.
  /// @return The state.
  [[nodiscard]] State state() const { return state_; }

  /// @brief The current map: none until one starts, and from a lost one to the next. Threaded,
  ///        read it once the run is finished.
  /// @return The map, if any.
  [[nodiscard]] const std::optional<Map>& map() const { return shared_->owner().map(); }

  /// @brief Every point and keyframe the run has made, with their lives and maps
  ///        (`Atlas::lifetimes`, with the current map).
  /// @return The points and keyframes by the run's id: those in the current map where it has
  ///         them, the others where they last were.
  [[nodiscard]] Lifetimes lifetimes() const;

  /// @brief The run's trajectory so far (`Atlas::trajectory`, with the current map).
  /// @return The frames tracked, but a reset map's.
  [[nodiscard]] Trajectory trajectory() const;

 private:
  /// @brief A frame handled, as `initialize` and `track_frame` give it.
  struct Step {
    /// @brief The tracked frame, if it kept enough inliers.
    std::optional<Tracked> tracked;
    /// @brief Whether it became a keyframe.
    bool keyframe{false};
    /// @brief The time mapping it took, in seconds, if it did.
    double map_seconds{0.0};
    /// @brief The time loop closing took, in seconds, if it ran.
    double loop_seconds{0.0};
    /// @brief How its stages went.
    FrameSummary summary;
    /// @brief The last tracking stage's match for each keypoint before refinement; empty if no
    ///        stage gave them.
    std::vector<std::int64_t> matched;
    /// @brief The point made of each keypoint, -1 where none; empty if none were made.
    std::vector<std::int64_t> made;
    /// @brief Whether the frame lost the track.
    bool lost{false};
    /// @brief The time tracking waited for the map.
    double wait_seconds{0.0};
    /// @brief Whether it wanted a keyframe.
    bool asked{false};
    /// @brief Whether mapping could not take the keyframe it wanted.
    bool refused{false};
    /// @brief The keyframes waiting for mapping when it asked.
    std::int64_t queue{0};
    /// @brief How mapping scheduled its keyframe's steps, inline.
    std::optional<Schedule> schedule;
  };

  /// @brief A keyframe mapped and searched for loops, as `make_keyframe` gives it.
  struct Keyframed {
    /// @brief The time mapping it took, in seconds.
    double map_seconds{0.0};
    /// @brief The time loop closing took, in seconds.
    double loop_seconds{0.0};
    /// @brief Whether a loop was corrected.
    bool corrected{false};
    /// @brief The point made of each of its keypoints, -1 where none.
    std::vector<std::int64_t> made;
    /// @brief How mapping scheduled its steps.
    Schedule schedule;
  };

  /// @brief What the threads share with tracking, on the heap so that a pipeline moved leaves
  ///        it where the threads hold it.
  class Shared {
   public:
    /// @brief Start what a pipeline shares.
    /// @param vocabulary The keyframe database's vocabulary, when relocalising.
    /// @param options The settings.
    /// @param threading How the steps run: a writer made for threads of their own, and the map's
    ///        lock used by the baseline.
    Shared(std::optional<foundations::vocabulary::Vocabulary> vocabulary, const Options& options,
           Threading threading)
        : writer_(threading != Threading::kInline ? std::make_unique<threads::Writer>() : nullptr),
          owner_(std::move(vocabulary), writer_.get(),
                 threading == Threading::kLocks ? &lock_ : nullptr),
          atlas_(options) {}

    /// @brief The handshake between the threads.
    /// @return The writer; null inline.
    [[nodiscard]] threads::Writer* writer() const { return writer_.get(); }

    /// @brief The map being tracked and its database.
    /// @return The owner.
    [[nodiscard]] MapOwner& owner() { return owner_; }

    /// @brief The map being tracked and its database.
    /// @return The owner.
    [[nodiscard]] const MapOwner& owner() const { return owner_; }

    /// @brief The run's records.
    /// @return The atlas.
    [[nodiscard]] Atlas& atlas() { return atlas_; }

    /// @brief The run's records.
    /// @return The atlas.
    [[nodiscard]] const Atlas& atlas() const { return atlas_; }

    /// @brief Loop closing's own state.
    /// @return The state.
    [[nodiscard]] LoopState& closing() { return closing_; }

   private:
    /// @brief The map's lock, the baseline's.
    std::mutex lock_;
    /// @brief The handshake between the threads; null inline.
    std::unique_ptr<threads::Writer> writer_;
    /// @brief The map being tracked and its database.
    MapOwner owner_;
    /// @brief The run's records.
    Atlas atlas_;
    /// @brief Loop closing's own state.
    LoopState closing_;
  };

  /// @brief A pipeline before its first frame.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  /// @param vocabulary The keyframe database's vocabulary, when relocalising or closing loops.
  /// @param threading How the steps run.
  Pipeline(Mode mode, Camera camera, Options options,
           std::optional<foundations::vocabulary::Vocabulary> vocabulary, Threading threading);

  /// @brief Try to start the map with a frame.
  /// @param frame The frame.
  /// @return The frame tracked if it started the map, and how the attempt went; a stage's
  ///         error.
  foundations::Result<Step> initialize(const Frame& frame);

  /// @brief Run the steps on a frame: track it, and map and close loops if it becomes a
  ///        keyframe.
  /// @param frame The frame.
  /// @return The tracked frame, if any, whether it became a keyframe, the times mapping and
  ///         loop closing took, and how its stages went; a stage's error.
  foundations::Result<Step> track_frame(const Frame& frame);

  /// @brief Threaded: place a tracked frame, and give local mapping its keyframe if it is one.
  /// @param frame The frame.
  /// @param view The map tracking read.
  /// @param tracked Its pose and matches.
  /// @param reference Its reference keyframe's row in the view.
  /// @param keyframe Whether it becomes a keyframe.
  /// @return Nothing; an error if the last pose cannot be inverted.
  foundations::Result<void> hand_over(const Frame& frame, const Map& view, const Tracked& tracked,
                                      std::int64_t reference, bool keyframe);

  /// @brief Map a tracked frame as a keyframe, place it, and close loops on it.
  /// @param m The map.
  /// @param frame The frame.
  /// @param tracked Its pose and matches.
  /// @param summary Its frame's summary, given the mapping's and the loop's.
  /// @return The times, whether a loop was corrected, and the points made; a stage's error.
  foundations::Result<Keyframed> make_keyframe(const Map& m, const Frame& frame,
                                               const Tracked& tracked, FrameSummary& summary);

  /// @brief Close the map the track was lost in, on this frame: keep it, or reset it if it has
  ///        `loss.reset_keyframes` keyframes or fewer (`Atlas::close`). Tracking's and loop
  ///        closing's state go with it, so the next frame starts a new map.
  /// @param m The map, given up by the owner.
  /// @return The closed map's summary.
  LostSummary close(const Map& m);

  /// @brief The sensor mode.
  Mode mode_;
  /// @brief The camera.
  Camera camera_;
  /// @brief The settings.
  Options options_;
  /// @brief Where the pipeline is.
  State state_{State::kNotInitialized};
  /// @brief Tracking's own state.
  TrackingState tracking_;
  /// @brief Whether the run is finished (`finish`).
  bool finished_{false};
  /// @brief The writer, the map's owner, the atlas and loop closing's state.
  std::unique_ptr<Shared> shared_;
  /// @brief The threads, threaded; ended before what they share.
  std::unique_ptr<PipelineThreads> threads_;
};

}  // namespace slam_it::edge
