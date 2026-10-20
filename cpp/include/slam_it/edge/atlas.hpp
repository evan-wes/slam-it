/// @file
/// @brief The run's maps and records: every map kept whole, each frame's place, every point's and
///        keyframe's life: the C++ core's mirror of `slam_it.edge.atlas`, whose docstring states
///        them.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "slam_it/edge/database.hpp"
#include "slam_it/edge/loops.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"

namespace slam_it::edge {

/// @brief A life: the frame that made a point or keyframe, the frame that removed it or -1, and
///        why (a `Removal` code) or -1.
using Life = std::array<std::int64_t, 3>;

/// @brief Every point and keyframe a run made, by the run's id (continuing across maps), where
///        each last was, its life and the map it ends in (`slam_it.edge.atlas.Lifetimes`).
struct Lifetimes {
  /// @brief Every point, its last position in its map's world frame.
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
  /// @brief Each kept map's strong links at the end, and each reset map's when it was reset, by
  ///        the run's ids, the rows ascending.
  std::vector<std::array<std::int64_t, 3>> keyframe_links;
  /// @brief Each map's loop edges, by the run's ids, the lower first, in the order closed.
  std::vector<std::array<std::int64_t, 2>> keyframe_loops;
  /// @brief Each map's merge edges, by the run's ids, the lower first, in the order made.
  std::vector<std::array<std::int64_t, 2>> keyframe_merges;
  /// @brief The map each point ends in.
  std::vector<std::int64_t> point_maps;
  /// @brief The map each keyframe ends in.
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
  /// @brief The map its final pose is in: the map it was tracked in, or the one that map was
  ///        merged into.
  std::vector<std::int64_t> maps;
  /// @brief The map it was tracked in.
  std::vector<std::int64_t> tracked_maps;
};

/// @brief The run's maps and records (`slam_it.edge.atlas.Atlas`): every map kept whole with its
///        database, the maps' numbers and each one's ids' run ids, every point's and keyframe's
///        life, and each tracked frame's place for the final trajectory. It only reads the map
///        being tracked.
///
/// With the steps in threads of their own (milestone 2B's B3b), the threads that change the map
/// record what they made and removed as they change it (`record`), dated by the frame being
/// tracked then, and tracking counts the frames (`tick`); a frame that became a keyframe is
/// placed before local mapping makes its keyframe (`place_pending`), its reference set once it
/// has (`resolve`). Every method holds the atlas's lock but `kept`, whose maps only tracking's
/// thread reads and changes.
class Atlas {
 public:
  /// @brief An atlas before the run's first frame.
  /// @param options The settings: the strong links' threshold.
  explicit Atlas(const Options& options) : options_(options) {}

  /// @brief The current map's number, if a map is being tracked.
  /// @return The number.
  [[nodiscard]] std::optional<std::int64_t> map_id() const;

  /// @brief The frames counted so far: the number of the frame being tracked.
  /// @return The count.
  [[nodiscard]] std::int64_t frames() const;

  /// @brief The frame a record is dated by now: the one being tracked, or the last after `end`.
  /// @return The frame's number.
  [[nodiscard]] std::int64_t frame() const;

  /// @brief End the run's frames: what threads record after the last is dated by it.
  void end();

  /// @brief The maps kept, not being tracked, by number, each with its database (empty when the
  ///        run does not relocalise). Changed by tracking's thread alone (`close`, `resume`,
  ///        `merge`), between frames with the threads settled or drained, and read by it and,
  ///        threaded, by loop closing's search, which those changes wait for: so unlocked.
  /// @return The maps.
  [[nodiscard]] std::span<const Indexed> kept() const { return kept_; }

  /// @brief Give the map starting on this frame its number; its ids are the run's next.
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

  /// @brief Give each of the current map's ids without a run id one, its life starting on this
  ///        frame: threaded, tracking reads a view that may hold points and keyframes the
  ///        threads have not recorded yet, and gives them their run ids before it reads them by
  ///        the run's ids.
  /// @param m The current map, or a view of it (`sub_map` keeps its next ids).
  void extend(const Map& m);

  /// @brief Close the current map on this frame: keep it whole with its database, or reset it
  ///        (its lives end as `Removal::kReset`, its strong links and loops kept, its frames'
  ///        places dropped).
  /// @param m The map the track left.
  /// @param database Its keyframe database, if the run relocalises.
  /// @param reset Whether it is reset rather than kept.
  /// @param dated Whether this frame is dated already: a map lost on a frame is closed after
  ///        it, one left for another kept map while tracking it.
  void close(Map m, std::optional<Database> database, bool reset, bool dated);

  /// @brief Take a kept map up again as the current one: relocalisation found the track in it.
  /// @param number The kept map's number.
  /// @return The map and its database, for the map's owner.
  [[nodiscard]] Indexed resume(std::int64_t number);

  /// @brief Take the current map into the kept map it merged with (`loops::merge`): the merged
  ///        map is the current one after, with the matched map's number, its run ids the
  ///        matched map's then the current map's, as the merge's offsets (the matched map's
  ///        next ids) moved them; what the current map removed before, and its frames' places,
  ///        are carried into the matched map's world by `to_world`, the frames' and the culled
  ///        keyframes' poses relative to their keyframes into its units (their translations
  ///        divided by `scale_of(G^-1)`, 1 but in monocular).
  /// @param matched The kept map's number.
  /// @param to_world `G`, the similarity carrying the current map's world into the matched
  ///        map's (`loops::merge_transform`).
  /// @return The matched map's database, which becomes the merged map's.
  [[nodiscard]] Database merge(std::int64_t matched, const Eigen::Matrix4d& to_world);

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
  /// @return The points and keyframes by the run's id: those in a map where it has them, the
  ///         others where they last were.
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
  ///        The map's ids without a run id are given one first, as a record does.
  /// @param m The current map.
  /// @param keyframe The keyframe, by the map's id.
  /// @param relative The pose relative to it, `T_pose_keyframe`.
  /// @return The pose's `T_cam_world` in the map now; none if the keyframe's chain of culled
  ///         ancestors ends at no keyframe of the map.
  [[nodiscard]] std::optional<Eigen::Matrix4d> pose_by(const Map& m, std::int64_t keyframe,
                                                       const Eigen::Matrix4d& relative);

  /// @brief A keyframe of the current map, by the run's id.
  /// @param id Its id in the map, given a run id, or -1.
  /// @return The run's id, or -1.
  [[nodiscard]] std::int64_t run_id(std::int64_t id) const;

  /// @brief A point of the current map, by the run's id.
  /// @param id Its id in the map, given a run id, or -1.
  /// @return The run's id, or -1.
  [[nodiscard]] std::int64_t point_run_id(std::int64_t id) const;

 private:
  /// @brief A map's ids' run ids, by its ids.
  struct Runs {
    /// @brief Each keyframe id's.
    std::vector<std::int64_t> keyframes;
    /// @brief Each point id's.
    std::vector<std::int64_t> points;
  };

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

  /// @brief Give each of the current map's ids without one a run id, its life starting on this
  ///        frame (the lock held).
  /// @param m The current map.
  void extend_held(const Map& m);

  /// @brief Record what a map made and removed (the lock held).
  /// @param m The current map, or null.
  void record_held(const Map* m);

  /// @brief The frame a record is dated by now (the lock held).
  /// @return The frame's number.
  [[nodiscard]] std::int64_t frame_held() const;

  /// @brief The map a map's points and keyframes end in: itself, or the one it merged into.
  /// @param number The map's number.
  /// @return The number.
  [[nodiscard]] std::int64_t ends_in(std::int64_t number) const;

  /// @brief A kept map's place in `kept_`.
  /// @param number Its number.
  /// @return The place.
  [[nodiscard]] std::size_t kept_at(std::int64_t number) const;

  /// @brief Write a whole map's points and keyframes, links, loops and merges into the record.
  /// @param m The map.
  /// @param runs Its ids' run ids.
  /// @param out The record, changed.
  void record_whole(const Map& m, const Runs& runs, Lifetimes& out) const;

  /// @brief The settings.
  Options options_;
  /// @brief Frames dated so far.
  std::int64_t frames_{0};
  /// @brief The current map's number, if there is one.
  std::optional<std::int64_t> map_id_;
  /// @brief The maps started so far: the next one's number.
  std::int64_t maps_{0};
  /// @brief The current map's ids' run ids.
  Runs runs_;
  /// @brief The maps kept, by number, each with its database.
  std::vector<Indexed> kept_;
  /// @brief Each kept map's ids' run ids, in `kept_`'s order.
  std::vector<Runs> kept_runs_;
  /// @brief Each merged map's number to the number of the map it was merged into.
  std::map<std::int64_t, std::int64_t> merged_into_;
  /// @brief Each point's life, by the run's id.
  std::vector<Life> point_lives_;
  /// @brief Each keyframe's life, by the run's id.
  std::vector<Life> keyframe_lives_;
  /// @brief The map each point was made in, by the run's id.
  std::vector<std::int64_t> point_maps_;
  /// @brief The map each keyframe was made in, by the run's id.
  std::vector<std::int64_t> keyframe_maps_;
  /// @brief Where each removed point last was, by the run's id (unset for the others).
  std::vector<Eigen::Vector3d> past_points_;
  /// @brief Each removed keyframe's time (unset for the others).
  std::vector<double> past_times_;
  /// @brief Its last pose (unset for the others).
  std::vector<Eigen::Matrix4d> past_poses_;
  /// @brief Its last parent, by the run's id (unset for the others).
  std::vector<std::int64_t> past_parents_;
  /// @brief The reset maps' strong links, by the run's ids.
  std::vector<std::array<std::int64_t, 3>> past_links_;
  /// @brief The reset maps' loop edges, by the run's ids.
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
