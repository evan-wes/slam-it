/// @file
/// @brief The edge pipeline, initialising, tracking each frame and mapping on keyframes: the
///        C++ core's mirror of `slam_it.edge.pipeline`, whose docstring states the per-frame
///        loop. It runs in the native program, not across the binding.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/initialization.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/mapping.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/edge/tracking.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/orb.hpp"

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
  /// @brief The number of placements (a sentinel, not a placement).
  kCount = 3,
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
  /// @brief Whether it was reset, having `reset_keyframes` keyframes or fewer, rather than kept.
  bool reset{false};
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
  /// @brief Tracking the local map, run on a placed frame.
  std::optional<LocalMapSummary> local_map;
  /// @brief Local mapping, on a keyframe after the first.
  std::optional<MappingSummary> mapping;
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
  /// @brief How its stages went.
  FrameSummary summary;
  /// @brief What became of each of its keypoints.
  Fates fates;
};

/// @brief A life: the frame that made a point or keyframe, the frame that removed it or -1, and
///        why (a `Removal` code) or -1.
using Life = std::array<std::int64_t, 3>;

/// @brief Every point and keyframe a run made, by the run's id (continuing across maps), where
///        each last was, its life and its map (`slam_it.edge.pipeline.Lifetimes`).
struct Lifetimes {
  /// @brief Every point, its last position in the world frame.
  std::vector<Eigen::Vector3d> points;
  /// @brief Their lives.
  std::vector<Life> point_lives;
  /// @brief Every keyframe's time, ascending.
  std::vector<double> keyframe_times;
  /// @brief Its last `T_cam_world`.
  std::vector<Eigen::Matrix4d> keyframe_poses;
  /// @brief Their lives.
  std::vector<Life> keyframe_lives;
  /// @brief The map each point was made in.
  std::vector<std::int64_t> point_maps;
  /// @brief The map each keyframe was made in.
  std::vector<std::int64_t> keyframe_maps;
};

/// @brief The edge pipeline for one sequence (`slam_it.edge.pipeline.Pipeline`).
class Pipeline {
 public:
  /// @brief Start a pipeline.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  /// @return The pipeline; `InvalidInput` for an invalid camera or settings, or a stereo or
  ///         RGB-D camera without a baseline.
  [[nodiscard]] static foundations::Result<Pipeline> create(Mode mode, Camera camera,
                                                            Options options);

  /// @brief The extraction settings the next frame should be built with.
  /// @return `init_feature_factor` times the features for a monocular frame before the map
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
  [[nodiscard]] const std::optional<Map>& map() const { return map_; }

  /// @brief Every point and keyframe the run has made, with their lives and maps
  ///        (`slam_it.edge.pipeline.Pipeline.lifetimes`).
  /// @return The points and keyframes by the run's id: those in the current map where it has
  ///         them, the others where they last were.
  [[nodiscard]] Lifetimes lifetimes() const;

 private:
  /// @brief A frame handled, as `initialize` and `track_frame` give it.
  struct Step {
    /// @brief The tracked frame, if it kept enough inliers.
    std::optional<Tracked> tracked;
    /// @brief Whether it became a keyframe.
    bool keyframe{false};
    /// @brief The time mapping it took, in seconds, if it did.
    double map_seconds{0.0};
    /// @brief How its stages went.
    FrameSummary summary;
    /// @brief The last tracking stage's match for each keypoint before refinement; empty if no
    ///        stage gave them.
    std::vector<std::int64_t> matched;
    /// @brief The point made of each keypoint, -1 where none; empty if none were made.
    std::vector<std::int64_t> made;
  };

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
  };

  /// @brief A pipeline before its first frame.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  Pipeline(Mode mode, Camera camera, Options options)
      : mode_(mode), camera_(std::move(camera)), options_(options) {}

  /// @brief Try to start the map with a frame.
  /// @param frame The frame.
  /// @return The frame tracked if it started the map, and how the attempt went; a stage's
  ///         error.
  foundations::Result<Step> initialize(const Frame& frame);

  /// @brief Pair a monocular frame with the reference, or make it the reference.
  /// @param frame The frame.
  /// @return The started map, if any, and how the attempt went; a stage's error.
  foundations::Result<Initialization> initialize_monocular(const Frame& frame);

  /// @brief Make a frame the monocular reference, each keypoint searched from its own position.
  /// @param frame The frame.
  void set_reference(const Frame& frame);

  /// @brief Make a tracked frame the last one; the velocity only spans consecutive frames.
  /// @param frame The frame.
  /// @param tracked Its pose and matches.
  /// @return Nothing, or an error if the last pose cannot be inverted.
  foundations::Result<void> accept(const Frame& frame, const Tracked& tracked);

  /// @brief Count a frame that was not tracked and end the velocity; enough in a row lose the
  ///        track.
  void record_failure();

  /// @brief Track the last frame, or recover by PnP.
  /// @param m The map.
  /// @param frame The frame.
  /// @return The placed frame, if any, how it was placed, and the last stage's matches; a
  ///         stage's error.
  [[nodiscard]] foundations::Result<Placed> place(const Map& m, const Frame& frame) const;

  /// @brief Track the local map from a placed frame and count its points.
  /// @param m The map, its counts changed.
  /// @param frame The frame.
  /// @param placed The placed frame.
  /// @param wide Whether the frame is a recovery's or the next (the search widened).
  /// @param recent Whether a recovery was recent (more inliers needed).
  /// @return The tracked frame, if it keeps enough inliers, how the stage went, and its
  ///         matches; a stage's error.
  [[nodiscard]] foundations::Result<Refined> refine(Map& m, const Frame& frame,
                                                    const Tracked& placed, bool wide,
                                                    bool recent) const;

  /// @brief Date what this frame made and removed, and empty the map's archive of removals.
  void date();

  /// @brief Close the map the track was lost in, on this frame: keep it, or reset it if it
  ///        has `reset_keyframes` keyframes or fewer (its lives end, as `Removal::kReset`). The
  ///        tracking state goes with it, so the next frame starts a new map.
  /// @param m The map, taken out of the pipeline.
  /// @return The closed map's summary.
  LostSummary close(const Map& m);

  /// @brief Track a frame against the map; insert a keyframe if one is needed (and time it).
  /// @param frame The frame.
  /// @return The tracked frame, if any, whether it became a keyframe, the time mapping it
  ///         took, and how its stages went; a stage's error.
  foundations::Result<Step> track_frame(const Frame& frame);

  /// @brief The sensor mode.
  Mode mode_;
  /// @brief The camera.
  Camera camera_;
  /// @brief The settings.
  Options options_;
  /// @brief Where the pipeline is.
  State state_{State::kNotInitialized};
  /// @brief The map, once initialised.
  std::optional<Map> map_;
  /// @brief The last tracked frame.
  std::optional<Frame> last_frame_;
  /// @brief Its pose and matches.
  std::optional<Tracked> last_tracked_;
  /// @brief The last motion `T_last_previous`, if the last two frames were tracked.
  std::optional<Eigen::Matrix4d> velocity_;
  /// @brief The monocular initialisation's reference frame.
  std::optional<Frame> reference_;
  /// @brief Where each of the reference's keypoints was last matched.
  std::vector<std::array<double, 2>> previous_;
  /// @brief Frames in a row not tracked.
  std::int64_t failures_{0};
  /// @brief Frames since the last recovery, if any.
  std::optional<std::int64_t> since_recovery_;
  /// @brief Frames given so far.
  std::int64_t frames_{0};
  /// @brief The current map's id, if there is one.
  std::optional<std::int64_t> map_id_;
  /// @brief The maps started so far: the next one's id.
  std::int64_t maps_{0};
  /// @brief The run's id of the current map's keyframe 0.
  std::int64_t kf_base_{0};
  /// @brief The run's id of the current map's point 0.
  std::int64_t pt_base_{0};
  /// @brief Each point's life, by the run's id.
  std::vector<Life> point_lives_;
  /// @brief Each keyframe's life, by the run's id.
  std::vector<Life> keyframe_lives_;
  /// @brief The map each point was made in, by the run's id.
  std::vector<std::int64_t> point_maps_;
  /// @brief The map each keyframe was made in, by the run's id.
  std::vector<std::int64_t> keyframe_maps_;
  /// @brief Where each point no longer in the current map last was, by the run's id: removed,
  ///        or in a closed map (unset for the others).
  std::vector<Eigen::Vector3d> past_points_;
  /// @brief The same keyframes' times (unset for the others).
  std::vector<double> past_times_;
  /// @brief Their last poses (unset for the others).
  std::vector<Eigen::Matrix4d> past_poses_;
};

}  // namespace slam_it::edge
