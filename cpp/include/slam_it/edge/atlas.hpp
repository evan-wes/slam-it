/// @file
/// @brief The run's records across its maps, each frame's place and every point's and keyframe's
///        life: the C++ core's mirror of `slam_it.edge.atlas`, whose docstring states them.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"

namespace slam_it::edge {

/// @brief A life: the frame that made a point or keyframe, the frame that removed it or -1, and
///        why (a `Removal` code) or -1.
using Life = std::array<std::int64_t, 3>;

/// @brief Every point and keyframe a run made, by the run's id (continuing across maps), where
///        each last was, its life and its map (`slam_it.edge.atlas.Lifetimes`).
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
  /// @brief Each one's last parent in its map's spanning tree, by the run's id, or -1.
  std::vector<std::int64_t> keyframe_parents;
  /// @brief Each map's strong links when it closed, and the current map's now, by the run's
  ///        ids, ascending.
  std::vector<std::array<std::int64_t, 3>> keyframe_links;
  /// @brief Each map's loop edges, the older keyframe first, by the run's ids, in the order
  ///        closed.
  std::vector<std::array<std::int64_t, 2>> keyframe_loops;
  /// @brief The map each point was made in.
  std::vector<std::int64_t> point_maps;
  /// @brief The map each keyframe was made in.
  std::vector<std::int64_t> keyframe_maps;
};

/// @brief The run's trajectory: each tracked frame's final pose, and its pose as tracked
///        (`slam_it.edge.atlas.Trajectory`); a reset map's frames are left out.
struct Trajectory {
  /// @brief Each tracked frame's time, in the order tracked.
  std::vector<double> timestamps;
  /// @brief Its final `T_cam_world`: its pose relative to its reference keyframe, carried by
  ///        the keyframe's last pose.
  std::vector<Eigen::Matrix4d> poses;
  /// @brief Its `T_cam_world` as tracked.
  std::vector<Eigen::Matrix4d> tracked;
  /// @brief The map it was tracked in.
  std::vector<std::int64_t> maps;
};

/// @brief The run's records across its maps (`slam_it.edge.atlas.Atlas`): the maps' numbers and
///        the run's id bases, every point's and keyframe's life, the closed maps' last places,
///        and each tracked frame's place for the final trajectory. It only reads a map.
///
/// With the steps in threads of their own (milestone 2B's B3b), the threads that change the map
/// record what they made and removed as they change it (`record`), dated by the frame being
/// tracked then, and tracking counts the frames (`tick`); a frame that became a keyframe is
/// placed before local mapping makes its keyframe (`place_pending`), its reference set once it
/// has (`resolve`). Every method holds the atlas's lock.
class Atlas {
 public:
  /// @brief An atlas before the run's first frame.
  /// @param options The settings: the strong links' threshold.
  explicit Atlas(const Options& options) : options_(options) {}

  /// @brief The current map's number, if a map is being tracked.
  /// @return The number.
  [[nodiscard]] std::optional<std::int64_t> map_id() const;

  /// @brief The run's id of the current map's keyframe 0 (the next map's, between maps).
  /// @return The id.
  [[nodiscard]] std::int64_t keyframe_base() const;

  /// @brief The run's id of the current map's point 0 (the next map's, between maps).
  /// @return The id.
  [[nodiscard]] std::int64_t point_base() const;

  /// @brief The frames counted so far: the number of the frame being tracked.
  /// @return The count.
  [[nodiscard]] std::int64_t frames() const;

  /// @brief The frame a record is dated by now: the one being tracked, or the last after `end`.
  /// @return The frame's number.
  [[nodiscard]] std::int64_t frame() const;

  /// @brief End the run's frames: what threads record after the last is dated by it.
  void end();

  /// @brief Give the map starting on this frame its number, and its ids past every map before
  ///        it.
  void start();

  /// @brief Date what this frame made in the current map and what it removed from it, read
  ///        from its archive of removals, which the map's owner then empties. Every frame is
  ///        dated, one a frame, whether or not a map is being tracked: `record`, then `tick`.
  /// @param m The current map after the frame, or null if there is none.
  void date(const Map* m);

  /// @brief Count a frame: what is recorded from now on is dated by the next.
  void tick();

  /// @brief Record what the current map made and removed since the last record, dated by the
  ///        frame being tracked (`frame`); what it removed is read from its archive, which the
  ///        map's owner then empties.
  /// @param m The current map, or null if there is none.
  void record(const Map* m);

  /// @brief Close the current map on the frame just dated: keep its record, or reset it (its
  ///        lives end as `Removal::kReset`, its frames' places dropped). A stand-in until
  ///        milestone 2B's B4: the map's strong links are counted here, for the run's record,
  ///        inside the frame's timed step (58 to 360 ms on the test set's largest maps); B4's
  ///        atlas keeps maps whole, and the record moves with it (the online-SLAM design,
  ///        section 4).
  /// @param m The map the track was lost in.
  /// @param reset Whether it is reset rather than kept.
  void close(const Map& m, bool reset);

  /// @brief Keep a tracked frame's place: its reference keyframe and its pose relative to it.
  /// @param m The current map.
  /// @param timestamp The frame's time.
  /// @param reference Its reference keyframe's id in the map.
  /// @param pose Its `T_cam_world` as tracked.
  void place_frame(const Map& m, double timestamp, std::int64_t reference,
                   const Eigen::Matrix4d& pose);

  /// @brief Keep the place of a frame that became the map's newest keyframe: its own, where it
  ///        was.
  /// @param m The current map, the frame's keyframe its last.
  /// @param timestamp The frame's time.
  /// @param pose Its `T_cam_world` as tracked.
  void place_keyframe(const Map& m, double timestamp, const Eigen::Matrix4d& pose);

  /// @brief Place a frame that will be the current map's next keyframe, once local mapping
  ///        makes it: its reference its own keyframe, its pose relative to it the identity, set
  ///        by `resolve`; until then the trajectory gives its pose as tracked.
  /// @param frame The frame's number (`frames` when it was tracked).
  /// @param timestamp Its time.
  /// @param pose Its `T_cam_world` as tracked.
  void place_pending(std::int64_t frame, double timestamp, const Eigen::Matrix4d& pose);

  /// @brief Set a pending frame's reference to the map's newest keyframe, the one made of it.
  /// @param frame The frame's number, as `place_pending` was given it.
  /// @param m The current map, the frame's keyframe its last.
  void resolve(std::int64_t frame, const Map& m);

  /// @brief Every point and keyframe the run has made, with their lives and maps.
  /// @param m The current map, or null if there is none.
  /// @return The points and keyframes by the run's id: those in the current map where it has
  ///         them, the others where they last were.
  [[nodiscard]] Lifetimes lifetimes(const Map* m) const;

  /// @brief The run's trajectory so far: each tracked frame placed by its reference keyframe's
  ///        last pose, through the keyframe's culled ancestors where it was culled; a frame
  ///        whose chain ends at none keeps its pose as tracked.
  /// @param m The current map, or null if there is none.
  /// @return The frames tracked, but a reset map's.
  [[nodiscard]] Trajectory trajectory(const Map* m) const;

  /// @brief Place a pose given relative to one of the current map's keyframes by where that
  ///        keyframe is now (`slam_it.edge.atlas.Atlas.pose_by`): a keyframe culled since is
  ///        followed through its culled ancestors, as the trajectory places a frame. Local
  ///        mapping places a keyframe tracked before a correction so (the online-SLAM design,
  ///        section 12): ORB-SLAM's `UpdateLastFrame` rule.
  /// @param m The current map.
  /// @param keyframe The keyframe, by the map's id.
  /// @param relative The pose relative to it, `T_pose_keyframe`.
  /// @return The pose's `T_cam_world` in the map now; none if the keyframe's chain of culled
  ///         ancestors ends at no keyframe of the map.
  [[nodiscard]] std::optional<Eigen::Matrix4d> pose_by(const Map& m, std::int64_t keyframe,
                                                       const Eigen::Matrix4d& relative) const;

  /// @brief A keyframe of the current map, by the run's id.
  /// @param id Its id in the map, or -1.
  /// @return The run's id, or -1.
  [[nodiscard]] std::int64_t run_id(std::int64_t id) const;

 private:
  /// @brief Record what a map made and removed (the lock held).
  /// @param m The current map, or null.
  void record_held(const Map* m);

  /// @brief Follow a keyframe through its culled ancestors (the lock held).
  /// @param keyframe The keyframe, by the run's id.
  /// @param relative A pose relative to it.
  /// @return The first ancestor not culled, by the run's id, and the pose relative to it.
  [[nodiscard]] std::pair<std::int64_t, Eigen::Matrix4d> through_culled(
      std::int64_t keyframe, Eigen::Matrix4d relative) const;

  /// @brief A tracked frame's place, for the final trajectory (`slam_it.edge.atlas._Placement`).
  struct FramePlace {
    /// @brief The frame's time.
    double timestamp{0.0};
    /// @brief The map it was tracked in.
    std::int64_t map{0};
    /// @brief The run's id of its reference keyframe.
    std::int64_t reference{0};
    /// @brief Its pose relative to the reference, `T_cam_ref`.
    Eigen::Matrix4d relative{Eigen::Matrix4d::Identity()};
    /// @brief Its `T_cam_world` as tracked.
    Eigen::Matrix4d tracked{Eigen::Matrix4d::Identity()};
  };

  /// @brief The settings.
  Options options_;
  /// @brief Frames dated so far.
  std::int64_t frames_{0};
  /// @brief The current map's number, if there is one.
  std::optional<std::int64_t> map_id_;
  /// @brief The maps started so far: the next one's number.
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
  /// @brief Their last parents, by the run's id (unset for the others).
  std::vector<std::int64_t> past_parents_;
  /// @brief The closed maps' strong links, by the run's ids.
  std::vector<std::array<std::int64_t, 3>> past_links_;
  /// @brief The closed maps' loop edges, by the run's ids.
  std::vector<std::array<std::int64_t, 2>> past_loops_;
  /// @brief Each tracked frame's place, for the final trajectory.
  std::vector<FramePlace> placements_;
  /// @brief Each culled keyframe's parent and its pose relative to it, by the run's ids.
  std::map<std::int64_t, std::pair<std::int64_t, Eigen::Matrix4d>> culled_;
  /// @brief Each frame placed before its keyframe was made: its place's index, by the frame.
  std::map<std::int64_t, std::size_t> pending_;
  /// @brief Whether the run's frames have ended.
  bool ended_{false};
  /// @brief Guards every field.
  mutable std::mutex mutex_;
};

}  // namespace slam_it::edge
