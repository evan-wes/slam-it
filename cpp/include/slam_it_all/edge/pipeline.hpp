/// @file
/// @brief The edge pipeline, initialising, tracking each frame and mapping on keyframes: the
///        C++ core's mirror of `slam_it_all.edge.pipeline`, whose docstring states the per-frame
///        loop. It runs in the native program, not across the binding.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <utility>

#include "slam_it_all/edge/frames.hpp"
#include "slam_it_all/edge/initialization.hpp"
#include "slam_it_all/edge/map.hpp"
#include "slam_it_all/edge/options.hpp"
#include "slam_it_all/foundations/error.hpp"
#include "slam_it_all/foundations/orb.hpp"

namespace slam_it_all::edge {

/// @brief Where the pipeline is, with the codes it is written as.
enum class State : std::uint8_t {
  /// @brief No map yet.
  kNotInitialized = 0,
  /// @brief Tracking frames against the map.
  kTracking = 1,
  /// @brief The track was lost: the run ends.
  kLost = 2,
  /// @brief The number of states (a sentinel, not a state).
  kCount = 3,
};

/// @brief A state's name, as `slam_it_all.edge.pipeline.State` spells it.
/// @param state The state.
/// @return A static string.
[[nodiscard]] const char* name(State state);

/// @brief What tracking one frame gave (`slam_it_all.edge.pipeline.FrameResult`).
struct FrameResult {
  /// @brief The frame's time.
  double timestamp{0.0};
  /// @brief The pipeline's state after the frame.
  State state{State::kNotInitialized};
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
};

/// @brief The edge pipeline for one sequence (`slam_it_all.edge.pipeline.Pipeline`).
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

  /// @brief Where the pipeline is.
  /// @return The state.
  [[nodiscard]] State state() const { return state_; }

  /// @brief The map, once initialised.
  /// @return The map, if any.
  [[nodiscard]] const std::optional<Map>& map() const { return map_; }

 private:
  /// @brief A frame tracked against the map, as `track_frame` gives it.
  struct Step {
    /// @brief The tracked frame, if it kept enough inliers.
    std::optional<Tracked> tracked;
    /// @brief Whether it became a keyframe.
    bool keyframe{false};
    /// @brief The time mapping it took, in seconds, if it did.
    double map_seconds{0.0};
  };

  /// @brief A pipeline before its first frame.
  /// @param mode The sensor mode.
  /// @param camera The camera.
  /// @param options The settings.
  Pipeline(Mode mode, Camera camera, Options options)
      : mode_(mode), camera_(std::move(camera)), options_(options) {}

  /// @brief Try to start the map with a frame.
  /// @param frame The frame.
  /// @return The frame tracked if it started the map; a stage's error.
  foundations::Result<std::optional<Tracked>> initialize(const Frame& frame);

  /// @brief Pair a monocular frame with the reference, or make it the reference.
  /// @param frame The frame.
  /// @return The started map, if any; a stage's error.
  foundations::Result<std::optional<Started>> initialize_monocular(const Frame& frame);

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
  /// @return The placed frame, if any, and whether it recovered; a stage's error.
  [[nodiscard]] foundations::Result<std::pair<std::optional<Tracked>, bool>> place(
      const Map& m, const Frame& frame) const;

  /// @brief Track the local map from a placed frame and count its points.
  /// @param m The map, its counts changed.
  /// @param frame The frame.
  /// @param placed The placed frame.
  /// @param wide Whether the frame is a recovery's or the next (the search widened).
  /// @param recent Whether a recovery was recent (more inliers needed).
  /// @return The tracked frame, if it keeps enough inliers; a stage's error.
  [[nodiscard]] foundations::Result<std::optional<Tracked>> refine(Map& m, const Frame& frame,
                                                                   const Tracked& placed, bool wide,
                                                                   bool recent) const;

  /// @brief Track a frame against the map; insert a keyframe if one is needed (and time it).
  /// @param frame The frame.
  /// @return The tracked frame, if any, whether it became a keyframe, and the time mapping it
  ///         took; a stage's error.
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
};

}  // namespace slam_it_all::edge
