/// @file
/// @brief The edge pipeline, initialising, tracking each frame and mapping on keyframes: the
///        C++ core's mirror of `slam_it.edge.pipeline`, whose docstring states the per-frame
///        loop, the steps' own state and the map's owner. It runs in the native program, not
///        across the binding.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
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
///        owner runs each step that changes it on its map, then keeps the database in step. In
///        one thread it holds the map between the steps; it is the one place milestone 2B's B3b
///        gives the map one writer at a time and tracking a published view of it (the
///        online-SLAM design, section 12).
class MapOwner {
 public:
  /// @brief An owner with no map yet.
  /// @param vocabulary The database's vocabulary; none when neither relocalising nor closing
  ///        loops, and there is no database.
  explicit MapOwner(std::optional<foundations::vocabulary::Vocabulary> vocabulary)
      : vocabulary_(std::move(vocabulary)) {}

  /// @brief The map being tracked: none until one starts, and from a lost one to the next.
  /// @return The map, if any.
  [[nodiscard]] const std::optional<Map>& map() const { return map_; }

  /// @brief Its keyframe database: none when neither relocalising nor closing loops, or with no
  ///        map.
  /// @return The database, if any.
  [[nodiscard]] const std::optional<Database>& database() const { return database_; }

  /// @brief The database's vocabulary, when relocalising or closing loops.
  /// @return The vocabulary, if any.
  [[nodiscard]] const std::optional<foundations::vocabulary::Vocabulary>& vocabulary() const {
    return vocabulary_;
  }

  /// @brief Take a map just started, and start its database with its keyframes.
  /// @param m The new map.
  /// @return Nothing; a quantisation's error.
  foundations::Result<void> start(Map m);

  /// @brief Count a frame tracked against the local map: the points seen, and found, in it.
  /// @param visible The points the frame's view held, by the map's ids.
  /// @param found The points it tracked.
  /// @return Nothing; `InvalidInput` with no map, or for an id the map does not hold.
  foundations::Result<void> count(const std::vector<std::int64_t>& visible,
                                  const std::vector<std::int64_t>& found);

  /// @brief Run a mapping step on the map, then bring the database to its keyframes.
  /// @tparam Step A callable taking the map, changed in place, and giving a `Result`.
  /// @param step The step.
  /// @return The step's result; `InvalidInput` with no map, or the database's error.
  template <typename Step>
  auto apply_mapping(Step&& step) -> std::invoke_result_t<Step, Map&> {
    if (!map_) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput, "mapping: no map");
    }
    return synced(std::forward<Step>(step)(*map_));
  }

  /// @brief Run a loop's correction on the map, held through the handshake (`stopped`), then
  ///        bring the database to its keyframes.
  /// @tparam Step A callable taking the map, changed in place, and giving a `Result`.
  /// @param step The correction.
  /// @return The correction's result; `InvalidInput` with no map, or the database's error.
  template <typename Step>
  auto apply_correction(Step&& step) -> std::invoke_result_t<Step, Map&> {
    return synced(stopped(std::forward<Step>(step)));
  }

  /// @brief The handshake: run a change on the map held, mapping stopped and global BA aborted,
  ///        then both released, as ORB-SLAM's loop closing does (`RequestStop`, `mbStopGBA`,
  ///        `Release`). In one thread nothing runs beside the change, so there is nothing to
  ///        stop or abort: this is the one place milestone 2B's B3b's threads do it, and its
  ///        merge (B4) enters it too.
  /// @tparam Step A callable taking the map, changed in place, and giving a `Result`.
  /// @param step The change.
  /// @return The change's result; `InvalidInput` with no map.
  template <typename Step>
  auto stopped(Step&& step) -> std::invoke_result_t<Step, Map&> {
    if (!map_) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput, "stopped: no map");
    }
    return std::forward<Step>(step)(*map_);
  }

  /// @brief Empty the map's archive of removals, once the atlas has dated them.
  void clear_gone();

  /// @brief Give up the map, lost: the database goes with it.
  /// @return The map, if there was one.
  std::optional<Map> close();

 private:
  /// @brief Bring the database to the map after a step that changed it.
  /// @tparam T The step's value.
  /// @param out The step's result.
  /// @return It, or the database's error.
  template <typename T>
  foundations::Result<T> synced(foundations::Result<T> out) {
    if (!out) {
      return out;
    }
    if (auto ok = sync_database(); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
    return out;
  }

  /// @brief Bring the keyframe database to the map's keyframes: add the new, erase the culled.
  /// @return Nothing; a quantisation's error.
  foundations::Result<void> sync_database();

  /// @brief The vocabulary, when relocalising or closing loops.
  std::optional<foundations::vocabulary::Vocabulary> vocabulary_;
  /// @brief The map, once initialised.
  std::optional<Map> map_;
  /// @brief The map's keyframe database, when relocalising or closing loops and a map exists.
  std::optional<Database> database_;
};

/// @brief The edge pipeline for one sequence (`slam_it.edge.pipeline.Pipeline`): tracking, local
///        mapping and loop closing as steps over state of their own, called in turn.
class Pipeline {
 public:
  /// @brief Start a pipeline.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  /// @param vocabulary The vocabulary of the keyframe database's words: required with
  ///        `relocalization.enabled` or `loops.enabled` on, refused with both off.
  /// @return The pipeline; `InvalidInput` for an invalid camera or settings, a stereo or
  ///         RGB-D camera without a baseline, or a vocabulary missing while relocalising or
  ///         closing loops, or given while doing neither.
  [[nodiscard]] static foundations::Result<Pipeline> create(
      Mode mode, Camera camera, Options options,
      std::optional<foundations::vocabulary::Vocabulary> vocabulary);

  /// @brief The extraction settings the next frame should be built with.
  /// @return `features.init_factor` times the features for a monocular frame before the map
  ///         exists, the usual ones otherwise.
  [[nodiscard]] foundations::orb::Options extraction() const;

  /// @brief Track one frame (and map, if it becomes a keyframe).
  /// @param frame The next frame, built with `extraction`'s settings.
  /// @return Its pose, or why there is none; a stage's error.
  [[nodiscard]] foundations::Result<FrameResult> track(const Frame& frame);

  /// @brief Where the pipeline is: never `kLost`, which a lost frame's result has.
  /// @return The state.
  [[nodiscard]] State state() const { return state_; }

  /// @brief The current map: none until one starts, and from a lost one to the next.
  /// @return The map, if any.
  [[nodiscard]] const std::optional<Map>& map() const { return owner_.map(); }

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
  };

  /// @brief A pipeline before its first frame.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  /// @param vocabulary The keyframe database's vocabulary, when relocalising or closing loops.
  Pipeline(Mode mode, Camera camera, Options options,
           std::optional<foundations::vocabulary::Vocabulary> vocabulary)
      : mode_(mode),
        camera_(std::move(camera)),
        options_(options),
        owner_(std::move(vocabulary)),
        atlas_(options) {}

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
  /// @brief Loop closing's own state.
  LoopState closing_;
  /// @brief The map being tracked and its database.
  MapOwner owner_;
  /// @brief The run's records.
  Atlas atlas_;
};

}  // namespace slam_it::edge
