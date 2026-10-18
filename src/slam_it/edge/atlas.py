"""The run's maps and records: every map kept whole, each frame's place, every point's life.

The pipeline (:mod:`slam_it.edge.pipeline`) tracks one map at a time; the atlas keeps every
other map the run has made, and what the run needs of each, from the frame that made each point
and keyframe to the end of the run (ORB-SLAM3's ``Atlas``; the online-SLAM design's section 14,
item 1):

- **Maps kept whole.** A map the track left is kept with its keyframes, keypoints, points,
  observations and links, and its keyframe database (:class:`Kept`), unless it was too small to
  keep and was reset. A kept map may be tracked again, found by relocalisation
  (:meth:`Atlas.resume`), or merged with the map being tracked (:meth:`Atlas.merge`).
- **Map numbers and the run's ids.** Maps are numbered from 0 in the order they start; a reset
  map's number is not given again, nor a merged one's. A map numbers its own keyframes and
  points from 0, so its first keyframe, which local BA holds fixed and culling keeps, is always
  keyframe 0. The run's ids continue across maps, in the order the run made them: each map
  keeps its ids' run ids, by its own id (:meth:`Atlas.run_ids`). A merge takes the matched map's
  ids as they were and moves the active map's on by offsets (:class:`slam_it.edge.loops.Merged`),
  so the merged map's run ids are the matched map's, then the active map's.
- **Lives** (:class:`Lifetimes`): every point and keyframe has a life, from the frame that made
  it to the frame that removed it, if one did, and the map it ends in: the one it was made in,
  or the map that map was merged into.
- **The final trajectory** (:meth:`Atlas.trajectory`, ORB-SLAM's ``SaveTrajectoryTUM``): each
  tracked frame keeps its reference keyframe (the one it shared the most points with, or its own
  when it became one) and its pose relative to it; a culled keyframe keeps its pose relative to
  its parent (:attr:`slam_it.edge.map.Map.gone_kf_relative`). Each frame is placed by its
  reference keyframe's last pose, through the keyframe's culled ancestors where it was culled,
  so the trajectory follows what loop closing, merging and bundle adjustment did to the
  keyframes after the frame was tracked. The poses as tracked are kept beside it, with the map
  each frame was tracked in.
- **Records carried through a merge.** What the active map removed before a merge (its points'
  and keyframes' last places, and each culled keyframe's pose relative to its parent) is carried
  into the matched map's world by the merge's similarity, as the map itself is.

The atlas only reads the map being tracked: it, and every change to it, belong to the pipeline's
owner (:class:`slam_it.edge.pipeline.MapOwner`).

**With the steps in threads of their own** (milestone 2B's B3b), the threads that change the map
record what they made and removed as they change it (:meth:`Atlas.record`), dated by the frame
being tracked then, and tracking counts the frames (:meth:`Atlas.tick`); a frame that became a
keyframe is placed before local mapping makes its keyframe (:meth:`Atlas.place_pending`), its
reference set once it has (:meth:`Atlas.resolve`); a keyframe tracked before a correction is
placed by its reference keyframe as the map now has it (:meth:`Atlas.pose_by`), through the
reference's culled ancestors as the trajectory places a frame. Every method holds the atlas's
lock but the properties, which read single values that Python's global interpreter lock keeps
whole.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field, replace

import numpy as np
from numpy.typing import NDArray

from slam_it.edge import covisibility, loops
from slam_it.edge.database import Database, Indexed
from slam_it.edge.map import Map, Removal
from slam_it.edge.options import Options
from slam_it.foundations import se3, sim3

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class Lifetimes:
    """Every point and keyframe a run made, by id, where each last was, its life and its map.

    The ids are the run's, continuing across maps. A life is three numbers: the frame that made
    it (frames counted from 0, each dated once, :meth:`Atlas.date`), the frame that removed it or
    -1, and why (a :class:`slam_it.edge.map.Removal` code) or -1. A kept map's points and
    keyframes stay alive; a reset map's end where it was reset, as
    :attr:`slam_it.edge.map.Removal.RESET`.

    Attributes
    ----------
    points : ndarray of float64, shape (N, 3)
        Every point, its last position in its map's world frame.
    point_lives : ndarray of int64, shape (N, 3)
        Their lives.
    keyframe_times : ndarray of float64, shape (K,)
        Every keyframe's time, ascending.
    keyframe_poses : ndarray of float64, shape (K, 4, 4)
        Its last ``T_cam_world``.
    keyframe_lives : ndarray of int64, shape (K, 3)
        Their lives.
    keyframe_parents : ndarray of int64, shape (K,)
        Each one's last parent in its map's spanning tree, by the run's id, or -1.
    keyframe_links : ndarray of int64, shape (L, 3)
        Each kept map's strong links (:func:`slam_it.edge.covisibility.strong_links`) at the
        end, and each reset map's when it was reset, by the run's ids, the rows ascending.
    keyframe_loops : ndarray of int64, shape (E, 2)
        Each map's loop edges, by the run's ids, the lower first, in the order closed.
    keyframe_merges : ndarray of int64, shape (G, 2)
        Each map's merge edges, by the run's ids, the lower first, in the order made.
    point_maps : ndarray of int64, shape (N,)
        The map each point ends in.
    keyframe_maps : ndarray of int64, shape (K,)
        The map each keyframe ends in.
    """

    points: Array
    point_lives: IntArray
    keyframe_times: Array
    keyframe_poses: Array
    keyframe_lives: IntArray
    keyframe_parents: IntArray
    keyframe_links: IntArray
    keyframe_loops: IntArray
    keyframe_merges: IntArray
    point_maps: IntArray
    keyframe_maps: IntArray


@dataclass(frozen=True)
class Trajectory:
    """The run's trajectory: each tracked frame's final pose, and its pose as tracked.

    The frames of a map that was reset are left out, as ORB-SLAM3 drops a reset map's.

    Attributes
    ----------
    timestamps : ndarray of float64, shape (N,)
        Each tracked frame's time, in the order tracked.
    poses : ndarray of float64, shape (N, 4, 4)
        Its final ``T_cam_world``: its pose relative to its reference keyframe, carried by the
        keyframe's last pose (the module docstring).
    tracked : ndarray of float64, shape (N, 4, 4)
        Its ``T_cam_world`` as tracked.
    maps : ndarray of int64, shape (N,)
        The map its final pose is in: the map it was tracked in, or the one that map was merged
        into.
    tracked_maps : ndarray of int64, shape (N,)
        The map it was tracked in.
    """

    timestamps: Array
    poses: Array
    tracked: Array
    maps: IntArray
    tracked_maps: IntArray


@dataclass
class Kept:
    """A map kept whole, not being tracked: its database, and its ids' run ids.

    Attributes
    ----------
    number : int
        The map's number.
    map : slam_it.edge.map.Map
        The map, as it was when the track left it.
    database : slam_it.edge.database.Database or None
        Its keyframe database; None when the run does not relocalise.
    keyframe_runs : list of int
        The run's id of each of its keyframe ids, by id.
    point_runs : list of int
        The same for its point ids.
    """

    number: int
    map: Map
    database: Database | None
    keyframe_runs: list[int]
    point_runs: list[int]


@dataclass(frozen=True)
class _Placement:
    """A tracked frame's place, for the final trajectory: when, in which map, relative to what.

    ``reference`` is the run's id of its reference keyframe and ``relative`` its pose relative
    to it, ``T_cam_ref``; ``tracked`` its ``T_cam_world`` as tracked; ``map`` the map it was
    tracked in.
    """

    timestamp: float
    map: int
    reference: int
    relative: Array
    tracked: Array


@dataclass
class Atlas:
    """The run's maps and records (see the module docstring).

    Attributes
    ----------
    options : Options
        The settings: the strong links' threshold (``covisibility``).
    """

    options: Options
    _frames: int = field(default=0, init=False)
    _map_id: int | None = field(default=None, init=False)
    _maps: int = field(default=0, init=False)
    # The current map's ids' run ids, by its ids.
    _kf_runs: list[int] = field(default_factory=list, init=False)
    _pt_runs: list[int] = field(default_factory=list, init=False)
    _kept: dict[int, Kept] = field(default_factory=dict, init=False)
    # Each merged map's number to the number of the map it was merged into.
    _merged_into: dict[int, int] = field(default_factory=dict, init=False)
    _point_lives: list[list[int]] = field(default_factory=list, init=False)
    _keyframe_lives: list[list[int]] = field(default_factory=list, init=False)
    # The map each point and keyframe was made in.
    _point_maps: list[int] = field(default_factory=list, init=False)
    _keyframe_maps: list[int] = field(default_factory=list, init=False)
    # Where each removed point and keyframe last was, by the run's id.
    _past_points: dict[int, Array] = field(default_factory=dict, init=False)
    _past_keyframes: dict[int, tuple[float, Array, int]] = field(default_factory=dict, init=False)
    # The reset maps' strong links and loop edges, by the run's ids.
    _past_links: list[IntArray] = field(default_factory=list, init=False)
    _past_loops: list[IntArray] = field(default_factory=list, init=False)
    # Each tracked frame's place, and each culled keyframe's parent and pose relative to it, by
    # the run's ids: the final trajectory's (trajectory()).
    _placements: list[_Placement] = field(default_factory=list, init=False)
    _culled: dict[int, tuple[int, Array]] = field(default_factory=dict, init=False)
    # Each frame placed before its keyframe was made: the frame, at its place's index.
    _pending: dict[int, int] = field(default_factory=dict, init=False)
    _ended: bool = field(default=False, init=False)
    _lock: threading.RLock = field(default_factory=threading.RLock, init=False, repr=False)

    @property
    def frames(self) -> int:
        """The frames counted so far: the number of the frame being tracked."""
        return self._frames

    @property
    def frame(self) -> int:
        """The frame a record is dated by now: the one tracked, or the last after :meth:`end`."""
        return self._frames - 1 if self._ended else self._frames

    def end(self) -> None:
        """End the run's frames: what threads record after the last is dated by it."""
        with self._lock:
            self._ended = True

    @property
    def map_id(self) -> int | None:
        """The current map's number; None when no map is being tracked."""
        return self._map_id

    def kept(self) -> list[Kept]:
        """Return the maps kept, not being tracked, by number.

        Returns
        -------
        list of Kept
            The maps.
        """
        with self._lock:
            return [self._kept[n] for n in sorted(self._kept)]

    def indexed(self) -> list[Indexed]:
        """Return the maps kept with their databases, as merging and relocalisation take them.

        Returns
        -------
        list of slam_it.edge.database.Indexed
            The kept maps, by number; none when the run does not relocalise.
        """
        return [
            Indexed(number=k.number, map=k.map, database=k.database)
            for k in self.kept()
            if k.database is not None
        ]

    def start(self) -> None:
        """Give the map starting on this frame its number; its ids are the run's next."""
        with self._lock:
            self._map_id, self._maps = self._maps, self._maps + 1
            self._kf_runs, self._pt_runs = [], []

    def date(self, m: Map | None) -> None:
        """Date what this frame made in `m`, the current map, and what it removed from it.

        Every frame is dated, one a frame, whether or not a map is being tracked; the frame
        numbers the lives. What `m` removed is read from its archive
        (:attr:`slam_it.edge.map.Map.gone_pt_id` and the rest), which the map's owner then
        empties. The steps in order: :meth:`record` then :meth:`tick`.

        Parameters
        ----------
        m : Map or None
            The current map after the frame, None if there is none.
        """
        with self._lock:
            self.record(m)
            self.tick()

    def tick(self) -> None:
        """Count a frame: what is recorded from now on is dated by the next."""
        with self._lock:
            self._frames += 1

    def record(self, m: Map | None) -> None:
        """Record what `m`, the current map, made and removed since the last record.

        Dated by the frame being tracked (:attr:`frame`). What `m` removed is read from its
        archive, which the map's owner then empties.

        Parameters
        ----------
        m : Map or None
            The current map, None if there is none.
        """
        with self._lock:
            self._record(m)

    def _record(self, m: Map | None) -> None:
        """Record what `m` made and removed (the lock held)."""
        frame = self.frame
        if m is not None and self._map_id is not None:
            self._extend(m)
            for point_id, xyz, how in zip(m.gone_pt_id, m.gone_pt_xyz, m.gone_pt_how, strict=True):
                run_id = self._pt_runs[int(point_id)]
                self._point_lives[run_id][1:] = [frame, int(how)]
                self._past_points[run_id] = xyz
            for keyframe_id, seconds, pose, parent, relative in zip(
                m.gone_kf_id, m.gone_kf_time, m.gone_kf_pose, self.run_ids(m.gone_kf_parent),
                m.gone_kf_relative, strict=True,
            ):  # fmt: skip
                run_id = self._kf_runs[int(keyframe_id)]
                self._keyframe_lives[run_id][1:] = [frame, int(Removal.CULLED)]
                self._past_keyframes[run_id] = (float(seconds), pose, int(parent))
                if parent >= 0:
                    self._culled[run_id] = (int(parent), relative)

    def close(self, m: Map, database: Database | None, *, reset: bool, dated: bool) -> None:
        """Close `m`, the current map, on this frame: keep it whole, or reset it.

        A reset map's points and keyframes go where they last were, their lives ending on this
        frame (:attr:`slam_it.edge.map.Removal.RESET`), with its strong links and loops then;
        its frames' places are dropped.

        Parameters
        ----------
        m : Map
            The map the track left.
        database : slam_it.edge.database.Database or None
            Its keyframe database, kept with it.
        reset : bool
            Whether it is reset rather than kept.
        dated : bool
            Whether this frame is dated already (:meth:`date`): a map lost on a frame is closed
            after it, one left for another kept map while tracking it.
        """
        with self._lock:
            self._close(m, database, reset=reset, dated=dated)

    def _close(self, m: Map, database: Database | None, *, reset: bool, dated: bool) -> None:
        """Close `m` (the lock held)."""
        number = self._current()
        self._pending.clear()
        self._map_id = None
        if not reset:
            self._kept[number] = Kept(number, m, database, self._kf_runs, self._pt_runs)
            return
        frame = self._frames - 1 if dated else self._frames
        for point_id, xyz in zip(m.pt_id, m.pt_xyz, strict=True):
            run_id = self._pt_runs[int(point_id)]
            self._past_points[run_id] = xyz
            self._point_lives[run_id][1:] = [frame, int(Removal.RESET)]
        for keyframe_id, seconds, pose, parent in zip(
            m.kf_id, m.kf_time, m.kf_pose, _runs_of(m.kf_parent, self._kf_runs), strict=True
        ):
            run_id = self._kf_runs[int(keyframe_id)]
            self._past_keyframes[run_id] = (float(seconds), pose, int(parent))
            self._keyframe_lives[run_id][1:] = [frame, int(Removal.RESET)]
        self._past_links.append(self._strong_links(m, self._kf_runs))
        self._past_loops.append(_runs_of(m.loop_edges, self._kf_runs).reshape(-1, 2))
        self._placements = [p for p in self._placements if p.map != number]

    def resume(self, number: int) -> Kept:
        """Take a kept map up again as the current one: relocalisation found the track in it.

        Parameters
        ----------
        number : int
            The kept map's number.

        Returns
        -------
        Kept
            The map and its database, for the map's owner.
        """
        with self._lock:
            kept = self._kept.pop(number)
            self._map_id = number
            self._kf_runs, self._pt_runs = kept.keyframe_runs, kept.point_runs
            return kept

    def merge(self, matched: int, merged: loops.Merged, to_world: Array) -> Kept:
        """Take the current map into the kept map it merged with (:func:`slam_it.edge.loops.merge`).

        The merged map is the current one after, with the matched map's number; its run ids are
        the matched map's, then the current map's, as the merge's offsets moved its ids. What
        the current map removed before is carried into the matched map's world by `to_world`.

        Parameters
        ----------
        matched : int
            The kept map's number.
        merged : slam_it.edge.loops.Merged
            The merge, its ids' offsets the matched map's next ids.
        to_world : ndarray of float64, shape (4, 4)
            ``G``, the similarity carrying the current map's world into the matched map's
            (:func:`slam_it.edge.loops.merge_transform`).

        Returns
        -------
        Kept
            The matched map as it was kept: its database is the merged map's to start from.
        """
        with self._lock:
            return self._merge(matched, merged, to_world)

    def _merge(self, matched: int, merged: loops.Merged, to_world: Array) -> Kept:
        """Take the current map into kept map `matched` (the lock held)."""
        current = self._current()
        kept = self._kept.pop(matched)
        assert len(kept.keyframe_runs) == merged.keyframe_offset
        assert len(kept.point_runs) == merged.point_offset
        back = sim3.inverse(to_world)
        scale = loops.scale_of(back)
        turn, shift = to_world[:3, :3], to_world[:3, 3]
        for run_id in self._pt_runs:
            if run_id in self._past_points:
                self._past_points[run_id] = turn @ self._past_points[run_id] + shift
        for run_id in self._kf_runs:
            if run_id in self._past_keyframes:
                seconds, pose, parent = self._past_keyframes[run_id]
                self._past_keyframes[run_id] = (seconds, loops.rigid_of(pose @ back), parent)
            if run_id in self._culled:
                parent, relative = self._culled[run_id]
                carried = relative.copy()
                carried[:3, 3] /= scale
                self._culled[run_id] = (parent, carried)
        self._kf_runs = kept.keyframe_runs + self._kf_runs
        self._pt_runs = kept.point_runs + self._pt_runs
        self._merged_into[current] = matched
        self._map_id = matched
        return kept

    def place_frame(self, m: Map, timestamp: float, reference: int, pose: Array) -> None:
        """Keep a tracked frame's place: its reference keyframe and its pose relative to it.

        Parameters
        ----------
        m : Map
            The current map.
        timestamp : float
            The frame's time.
        reference : int
            Its reference keyframe, by the map's id.
        pose : ndarray of float64, shape (4, 4)
            Its ``T_cam_world`` as tracked.
        """
        k = int(m.keyframe_rows(np.array([reference], dtype=np.int64))[0])
        relative = pose @ se3.inverse(m.kf_pose[k])
        with self._lock:
            self._extend(m)
            self._placements.append(
                _Placement(
                    timestamp=timestamp,
                    map=self._current(),
                    reference=self._kf_runs[reference],
                    relative=relative,
                    tracked=pose.copy(),
                )
            )

    def place_keyframe(self, m: Map, timestamp: float, pose: Array) -> None:
        """Keep the place of a frame that became `m`'s newest keyframe: its own, where it was.

        Parameters
        ----------
        m : Map
            The current map, the frame's keyframe its last.
        timestamp : float
            The frame's time.
        pose : ndarray of float64, shape (4, 4)
            Its ``T_cam_world`` as tracked.
        """
        with self._lock:
            self._extend(m)
            reference = self._kf_runs[int(m.kf_id[-1])]
            self._placements.append(
                _Placement(timestamp, self._current(), reference, np.eye(4), pose.copy())
            )

    def place_pending(self, frame: int, timestamp: float, pose: Array) -> None:
        """Place a frame that will be the current map's next keyframe, once local mapping makes it.

        Its reference is its own keyframe, its pose relative to it the identity, set by
        :meth:`resolve`; until then the trajectory gives its pose as tracked.

        Parameters
        ----------
        frame : int
            The frame's number (:attr:`frames` when it was tracked).
        timestamp : float
            Its time.
        pose : ndarray of float64, shape (4, 4)
            Its ``T_cam_world`` as tracked.
        """
        with self._lock:
            self._pending[frame] = len(self._placements)
            self._placements.append(
                _Placement(timestamp, self._current(), -1, np.eye(4), pose.copy())
            )

    def resolve(self, frame: int, m: Map) -> None:
        """Set a pending frame's reference to `m`'s newest keyframe, the one made of it.

        Parameters
        ----------
        frame : int
            The frame's number, as :meth:`place_pending` was given it.
        m : Map
            The current map, the frame's keyframe its last.
        """
        with self._lock:
            self._extend(m)
            at = self._pending.pop(frame)
            placed = self._placements[at]
            reference = self._kf_runs[int(m.kf_id[-1])]
            self._placements[at] = replace(placed, reference=reference)

    def lifetimes(self, m: Map | None) -> Lifetimes:
        """Return every point and keyframe the run has made, with their lives.

        Parameters
        ----------
        m : Map or None
            The current map, None if there is none.

        Returns
        -------
        Lifetimes
            The points and keyframes by the run's id: those in a map where it has them, the
            others where they last were.
        """
        with self._lock:
            return self._lifetimes(m)

    def _lifetimes(self, m: Map | None) -> Lifetimes:
        """Return every point's and keyframe's life (the lock held)."""
        points = np.zeros((len(self._point_lives), 3))
        times = np.zeros(len(self._keyframe_lives))
        poses = np.tile(np.eye(4), (len(self._keyframe_lives), 1, 1))
        parents = np.full(len(self._keyframe_lives), -1, dtype=np.int64)
        for point_id, xyz in self._past_points.items():
            points[point_id] = xyz
        for keyframe_id, (seconds, pose, parent) in self._past_keyframes.items():
            times[keyframe_id], poses[keyframe_id], parents[keyframe_id] = seconds, pose, parent
        links, loop_edges = list(self._past_links), list(self._past_loops)
        merge_edges: list[IntArray] = []
        for whole, kf_runs, pt_runs in self._whole(m):
            kf_ids = _runs_of(whole.kf_id, kf_runs)
            points[_runs_of(whole.pt_id, pt_runs)] = whole.pt_xyz
            times[kf_ids], poses[kf_ids] = whole.kf_time, whole.kf_pose
            parents[kf_ids] = _runs_of(whole.kf_parent, kf_runs)
            links.append(self._strong_links(whole, kf_runs))
            loop_edges.append(np.sort(_runs_of(whole.loop_edges, kf_runs).reshape(-1, 2)))
            merge_edges.append(np.sort(_runs_of(whole.merge_edges, kf_runs).reshape(-1, 2)))
        return Lifetimes(
            points=points,
            point_lives=np.array(self._point_lives, dtype=np.int64).reshape(-1, 3),
            keyframe_times=times,
            keyframe_poses=poses,
            keyframe_lives=np.array(self._keyframe_lives, dtype=np.int64).reshape(-1, 3),
            keyframe_parents=parents,
            keyframe_links=_ascending(links),
            keyframe_loops=np.concatenate([np.zeros((0, 2), dtype=np.int64), *loop_edges]),
            keyframe_merges=np.concatenate([np.zeros((0, 2), dtype=np.int64), *merge_edges]),
            point_maps=np.array([self._ends_in(n) for n in self._point_maps], dtype=np.int64),
            keyframe_maps=np.array([self._ends_in(n) for n in self._keyframe_maps], dtype=np.int64),
        )

    def trajectory(self, m: Map | None) -> Trajectory:
        """Return the run's trajectory so far: each tracked frame's final pose, and as tracked.

        Parameters
        ----------
        m : Map or None
            The current map, None if there is none.

        Returns
        -------
        Trajectory
            The frames tracked, but a reset map's, each placed by its reference keyframe's last
            pose (the module docstring); a frame whose keyframe chain ends at none (a culled
            keyframe that had no parent) keeps its pose as tracked.
        """
        with self._lock:
            return self._trajectory(m)

    def pose_by(self, m: Map, keyframe: int, relative: Array) -> Array | None:
        """Place a pose given relative to one of `m`'s keyframes by where that keyframe is now.

        A keyframe culled since is followed through its culled ancestors, as the trajectory
        places a frame (the module docstring). Local mapping places a keyframe tracked before a
        correction so (the online-SLAM design, section 12): ORB-SLAM's ``UpdateLastFrame`` rule.

        Parameters
        ----------
        m : Map
            The current map.
        keyframe : int
            The keyframe, by `m`'s id.
        relative : ndarray of float64, shape (4, 4)
            The pose relative to it, ``T_pose_keyframe``.

        Returns
        -------
        ndarray of float64, shape (4, 4) or None
            The pose's ``T_cam_world`` in `m` now; None if the keyframe's chain of culled
            ancestors ends at no keyframe of `m`.
        """
        with self._lock:
            self._extend(m)
            found, pose = self._through_culled(self._kf_runs[keyframe], relative)
            rows = np.flatnonzero(_runs_of(m.kf_id, self._kf_runs) == found)
        return None if rows.size == 0 else pose @ m.kf_pose[int(rows[0])]

    def _through_culled(self, keyframe: int, relative: Array) -> tuple[int, Array]:
        """Follow a keyframe (by the run's id) through its culled ancestors (the lock held).

        Return the first ancestor not culled, by the run's id, and the pose relative to it.
        """
        while keyframe in self._culled:
            parent, step = self._culled[keyframe]
            relative, keyframe = relative @ step, parent
        return keyframe, relative

    def _trajectory(self, m: Map | None) -> Trajectory:
        """Return the trajectory (the lock held)."""
        last: dict[int, Array] = {k: pose for k, (_, pose, _) in self._past_keyframes.items()}
        for whole, kf_runs, _ in self._whole(m):
            last.update(zip(_runs_of(whole.kf_id, kf_runs).tolist(), whole.kf_pose, strict=True))
        poses: list[Array] = []
        for placed in self._placements:
            keyframe, pose = self._through_culled(placed.reference, placed.relative)
            anchor = last.get(keyframe)
            poses.append(placed.tracked if anchor is None else pose @ anchor)
        tracked_in = [placed.map for placed in self._placements]
        return Trajectory(
            timestamps=np.array([placed.timestamp for placed in self._placements]),
            poses=np.array(poses).reshape(-1, 4, 4),
            tracked=np.array([placed.tracked for placed in self._placements]).reshape(-1, 4, 4),
            maps=np.array([self._ends_in(n) for n in tracked_in], dtype=np.int64),
            tracked_maps=np.array(tracked_in, dtype=np.int64),
        )

    def run_ids(self, ids: IntArray) -> IntArray:
        """Return the current map's keyframe ids as the run's, -1 kept.

        Parameters
        ----------
        ids : ndarray of int64, shape (N,)
            Keyframe ids by the current map's numbering, -1 for none; each given a run id.

        Returns
        -------
        ndarray of int64, shape (N,)
            The same keyframes by the run's ids.
        """
        with self._lock:
            return _runs_of(ids, self._kf_runs)

    def point_run_ids(self, ids: IntArray) -> IntArray:
        """Return the current map's point ids as the run's, -1 kept.

        Parameters
        ----------
        ids : ndarray of int64, shape (N,)
            Point ids by the current map's numbering, -1 for none; each given a run id.

        Returns
        -------
        ndarray of int64, shape (N,)
            The same points by the run's ids.
        """
        with self._lock:
            return _runs_of(ids, self._pt_runs)

    def extend(self, m: Map) -> None:
        """Give each of `m`'s ids without a run id one, its life starting on this frame.

        Threaded, tracking reads a view of the current map that may hold points and keyframes
        the threads have not recorded yet (:meth:`record`); tracking gives them their run ids
        before it reads them by the run's ids.

        Parameters
        ----------
        m : Map
            The current map, or a view of it.
        """
        with self._lock:
            self._extend(m)

    def _extend(self, m: Map) -> None:
        """Give each of `m`'s ids without one a run id, its life starting on this frame."""
        frame, number = self.frame, self._current()
        for _ in range(len(self._pt_runs), m.next_pt_id):
            self._pt_runs.append(len(self._point_lives))
            self._point_lives.append([frame, -1, -1])
            self._point_maps.append(number)
        for _ in range(len(self._kf_runs), m.next_kf_id):
            self._kf_runs.append(len(self._keyframe_lives))
            self._keyframe_lives.append([frame, -1, -1])
            self._keyframe_maps.append(number)

    def _whole(self, m: Map | None) -> list[tuple[Map, list[int], list[int]]]:
        """Return every map held whole, the kept ones and `m`, each with its ids' run ids."""
        held = [(k.map, k.keyframe_runs, k.point_runs) for k in self.kept()]
        if m is not None and self._map_id is not None:
            held.append((m, self._kf_runs, self._pt_runs))
        return held

    def _ends_in(self, number: int) -> int:
        """Return the map a map's points and keyframes end in: itself, or the one it merged into."""
        while number in self._merged_into:
            number = self._merged_into[number]
        return number

    def _current(self) -> int:
        """Return the current map's number; a frame is placed only while one is tracked."""
        assert self._map_id is not None
        return self._map_id

    def _strong_links(self, m: Map, kf_runs: list[int]) -> IntArray:
        """Return `m`'s strong links, by the run's ids, each the lower id first."""
        links = covisibility.strong_links(m, self.options)
        ends = np.sort(_runs_of(links[:, :2], kf_runs).reshape(-1, 2), axis=1)
        return np.column_stack([ends, links[:, 2]]).astype(np.int64).reshape(-1, 3)


def _runs_of(ids: IntArray, runs: list[int]) -> IntArray:
    """Return a map's ids as the run's, by its table of run ids; -1 kept."""
    values = np.asarray(ids, dtype=np.int64)
    if not runs:
        return np.full(values.shape, -1, dtype=np.int64)
    table = np.array(runs, dtype=np.int64)
    return np.where(values >= 0, table[np.clip(values, 0, None)], -1).astype(np.int64)


def _ascending(links: list[IntArray]) -> IntArray:
    """Return strong links gathered from several maps, the rows ascending."""
    out = np.concatenate([np.zeros((0, 3), dtype=np.int64), *links])
    return out[np.lexsort((out[:, 1], out[:, 0]))]
