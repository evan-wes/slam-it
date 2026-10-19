"""The run's records across its maps: each frame's place, every point's and keyframe's life.

The pipeline (:mod:`slam_it.edge.pipeline`) tracks one map at a time; the atlas keeps what the
run needs of every map it has made, from the frame that made each point and keyframe to the end
of the run (ORB-SLAM3's ``Atlas``; so far its bookkeeping):

- **Map numbers and the run's ids.** Maps are numbered from 0 in the order they start; a reset
  map's number is not given again. A map numbers its own keyframes and points from 0, so its
  first keyframe, which local BA holds fixed and culling keeps, is always keyframe 0. The run's
  ids continue across maps: each map's are offset by the points and keyframes the maps before it
  made, as ORB-SLAM3's one counter for every map gives them.
- **Lives** (:class:`Lifetimes`): after each frame the atlas dates what the frame made and
  removed (:meth:`Atlas.date`). Every point and keyframe has a life, from the frame that made it
  to the frame that removed it, if one did.
- **The final trajectory** (:meth:`Atlas.trajectory`, ORB-SLAM's ``SaveTrajectoryTUM``): each
  tracked frame keeps its reference keyframe (the one it shared the most points with, or its own
  when it became one) and its pose relative to it; a culled keyframe keeps its pose relative to
  its parent (:attr:`slam_it.edge.map.Map.gone_kf_relative`). Each frame is placed by its
  reference keyframe's last pose, through the keyframe's culled ancestors where it was culled,
  so the trajectory follows what loop closing and bundle adjustment did to the keyframes after
  the frame was tracked. The poses as tracked are kept beside it.
- **Closed maps** (:meth:`Atlas.close`): where each of a closed map's points and keyframes last
  was, and its strong links and loop edges when it closed. A kept map's descriptors and
  observations are not kept: milestone 2B's B4 keeps maps whole here
  (``docs/design/2026-10-14-online-slam.md``, section 14).

The atlas only reads a map: the map being tracked, and every change to it, belong to the
pipeline's owner (:class:`slam_it.edge.pipeline.MapOwner`).

**With the steps in threads of their own** (milestone 2B's B3b), the threads that change the map
record what they made and removed as they change it (:meth:`Atlas.record`), dated by the frame
being tracked then, and tracking counts the frames (:meth:`Atlas.tick`); a frame that became a
keyframe is placed before local mapping makes its keyframe (:meth:`Atlas.place_pending`), its
reference set once it has (:meth:`Atlas.resolve`); a keyframe tracked before a correction is
placed by its reference keyframe as the map now has it (:meth:`Atlas.pose_by`), through the
reference's culled ancestors as the trajectory places a frame. Every method holds the atlas's
lock.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field, replace

import numpy as np
from numpy.typing import NDArray

from slam_it.edge import covisibility
from slam_it.edge.map import Map, Removal
from slam_it.edge.options import Options
from slam_it.foundations import se3

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class Lifetimes:
    """Every point and keyframe a run made, by id, where each last was, its life and its map.

    The ids are the run's, continuing across maps. A life is three numbers: the frame that made
    it (frames counted from 0, each dated once, :meth:`Atlas.date`), the frame that removed it or
    -1, and why (a :class:`slam_it.edge.map.Removal` code) or -1. A kept map's points and
    keyframes stay alive after it closes; a reset map's end there, as
    :attr:`slam_it.edge.map.Removal.RESET`.

    Attributes
    ----------
    points : ndarray of float64, shape (N, 3)
        Every point, its last position in the world frame.
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
        Each map's strong links (:func:`slam_it.edge.covisibility.strong_links`) when it closed,
        and the current map's now, by the run's ids, ascending.
    keyframe_loops : ndarray of int64, shape (E, 2)
        Each map's loop edges, the older keyframe first, by the run's ids, in the order closed.
    point_maps : ndarray of int64, shape (N,)
        The map each point was made in.
    keyframe_maps : ndarray of int64, shape (K,)
        The map each keyframe was made in.
    """

    points: Array
    point_lives: IntArray
    keyframe_times: Array
    keyframe_poses: Array
    keyframe_lives: IntArray
    keyframe_parents: IntArray
    keyframe_links: IntArray
    keyframe_loops: IntArray
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
        The map it was tracked in.
    """

    timestamps: Array
    poses: Array
    tracked: Array
    maps: IntArray


@dataclass(frozen=True)
class _Placement:
    """A tracked frame's place, for the final trajectory: when, in which map, relative to what.

    ``reference`` is the run's id of its reference keyframe and ``relative`` its pose relative
    to it, ``T_cam_ref``; ``tracked`` its ``T_cam_world`` as tracked.
    """

    timestamp: float
    map: int
    reference: int
    relative: Array
    tracked: Array


@dataclass
class Atlas:
    """The run's records across its maps (see the module docstring).

    Attributes
    ----------
    options : Options
        The settings: the strong links' threshold (``covisibility``).
    """

    options: Options
    _frames: int = field(default=0, init=False)
    _map_id: int | None = field(default=None, init=False)
    _maps: int = field(default=0, init=False)
    _kf_base: int = field(default=0, init=False)
    _pt_base: int = field(default=0, init=False)
    _point_lives: list[list[int]] = field(default_factory=list, init=False)
    _keyframe_lives: list[list[int]] = field(default_factory=list, init=False)
    _point_maps: list[int] = field(default_factory=list, init=False)
    _keyframe_maps: list[int] = field(default_factory=list, init=False)
    # Where each point and keyframe no longer in the current map last was, by the run's id:
    # removed, or in a closed map.
    _past_points: dict[int, Array] = field(default_factory=dict, init=False)
    _past_keyframes: dict[int, tuple[float, Array, int]] = field(default_factory=dict, init=False)
    # The closed maps' strong links, by the run's ids.
    _past_links: list[IntArray] = field(default_factory=list, init=False)
    # The closed maps' loop edges, by the run's ids.
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

    @property
    def keyframe_base(self) -> int:
        """The run's id of the current map's keyframe 0 (the next map's, between maps)."""
        return self._kf_base

    @property
    def point_base(self) -> int:
        """The run's id of the current map's point 0 (the next map's, between maps)."""
        return self._pt_base

    def start(self) -> None:
        """Give the map starting on this frame its number, and its ids past every map before it."""
        with self._lock:
            self._map_id, self._maps = self._maps, self._maps + 1
            self._kf_base, self._pt_base = len(self._keyframe_lives), len(self._point_lives)

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
        map_id = self._map_id
        if m is None or map_id is None:
            return
        made = self._pt_base + m.next_pt_id - len(self._point_lives)
        self._point_lives.extend([frame, -1, -1] for _ in range(made))
        self._point_maps.extend([map_id] * made)
        made = self._kf_base + m.next_kf_id - len(self._keyframe_lives)
        self._keyframe_lives.extend([frame, -1, -1] for _ in range(made))
        self._keyframe_maps.extend([map_id] * made)
        for point_id, xyz, how in zip(m.gone_pt_id, m.gone_pt_xyz, m.gone_pt_how, strict=True):
            run_id = self._pt_base + int(point_id)
            self._point_lives[run_id][1:] = [frame, int(how)]
            self._past_points[run_id] = xyz
        for keyframe_id, seconds, pose, parent, relative in zip(
            m.gone_kf_id, m.gone_kf_time, m.gone_kf_pose, self.run_ids(m.gone_kf_parent),
            m.gone_kf_relative, strict=True,
        ):  # fmt: skip
            run_id = self._kf_base + int(keyframe_id)
            self._keyframe_lives[run_id][1:] = [frame, int(Removal.CULLED)]
            self._past_keyframes[run_id] = (float(seconds), pose, int(parent))
            if parent >= 0:
                self._culled[run_id] = (int(parent), relative)

    def close(self, m: Map, *, reset: bool) -> None:
        """Close `m`, the current map, on the frame just dated: keep its record, or reset it.

        Its points and keyframes go where they last were; a reset map's lives end on this frame
        (:attr:`slam_it.edge.map.Removal.RESET`), and its frames' places are dropped.

        A stand-in until milestone 2B's B4: the map's strong links are counted here, for the
        run's record, inside the frame's timed step, which costs that frame 58 to 360 ms in the
        C++ program on the test set's largest maps; B4's atlas keeps maps whole, and the record
        moves with it (``docs/design/2026-10-14-online-slam.md``, section 4).

        Parameters
        ----------
        m : Map
            The map the track was lost in.
        reset : bool
            Whether it is reset rather than kept.
        """
        with self._lock:
            self._close(m, reset=reset)

    def _close(self, m: Map, *, reset: bool) -> None:
        """Close `m` (the lock held)."""
        frame = self._frames - 1  # dated already
        for point_id, xyz in zip(m.pt_id, m.pt_xyz, strict=True):
            run_id = self._pt_base + int(point_id)
            self._past_points[run_id] = xyz
            if reset:
                self._point_lives[run_id][1:] = [frame, int(Removal.RESET)]
        for keyframe_id, seconds, pose, parent in zip(
            m.kf_id, m.kf_time, m.kf_pose, self.run_ids(m.kf_parent), strict=True
        ):
            run_id = self._kf_base + int(keyframe_id)
            self._past_keyframes[run_id] = (float(seconds), pose, int(parent))
            if reset:
                self._keyframe_lives[run_id][1:] = [frame, int(Removal.RESET)]
        self._past_links.append(self._strong_links(m))
        self._past_loops.append(self._kf_base + m.loop_edges)
        if reset:  # its frames' poses go with it
            self._placements = [p for p in self._placements if p.map != self._map_id]
        self._map_id = None
        self._pending.clear()

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
            self._placements.append(
                _Placement(
                    timestamp=timestamp,
                    map=self._current(),
                    reference=self._kf_base + reference,
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
            reference = self._kf_base + int(m.kf_id[-1])
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
            at = self._pending.pop(frame)
            placed = self._placements[at]
            reference = self._kf_base + int(m.kf_id[-1])
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
            The points and keyframes by the run's id: those in `m` where it has them, the
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
        links = list(self._past_links)
        loop_edges = list(self._past_loops)
        if m is not None:
            points[self._pt_base + m.pt_id] = m.pt_xyz
            times[self._kf_base + m.kf_id] = m.kf_time
            poses[self._kf_base + m.kf_id] = m.kf_pose
            parents[self._kf_base + m.kf_id] = self.run_ids(m.kf_parent)
            links.append(self._strong_links(m))
            loop_edges.append(self._kf_base + m.loop_edges)
        return Lifetimes(
            points=points,
            point_lives=np.array(self._point_lives, dtype=np.int64).reshape(-1, 3),
            keyframe_times=times,
            keyframe_poses=poses,
            keyframe_lives=np.array(self._keyframe_lives, dtype=np.int64).reshape(-1, 3),
            keyframe_parents=parents,
            keyframe_links=np.concatenate([np.zeros((0, 3), dtype=np.int64), *links]),
            keyframe_loops=np.concatenate([np.zeros((0, 2), dtype=np.int64), *loop_edges]),
            point_maps=np.array(self._point_maps, dtype=np.int64),
            keyframe_maps=np.array(self._keyframe_maps, dtype=np.int64),
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
            base = self._kf_base
            found, pose = self._through_culled(base + keyframe, relative)
        row = int(m.keyframe_rows(np.array([found - base], dtype=np.int64))[0])
        return None if row < 0 else pose @ m.kf_pose[row]

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
        if m is not None:
            last.update(zip((self._kf_base + m.kf_id).tolist(), m.kf_pose, strict=True))
        poses: list[Array] = []
        for placed in self._placements:
            keyframe, pose = self._through_culled(placed.reference, placed.relative)
            anchor = last.get(keyframe)
            poses.append(placed.tracked if anchor is None else pose @ anchor)
        return Trajectory(
            timestamps=np.array([placed.timestamp for placed in self._placements]),
            poses=np.array(poses).reshape(-1, 4, 4),
            tracked=np.array([placed.tracked for placed in self._placements]).reshape(-1, 4, 4),
            maps=np.array([placed.map for placed in self._placements], dtype=np.int64),
        )

    def run_ids(self, ids: IntArray) -> IntArray:
        """Return the current map's keyframe ids as the run's, -1 kept.

        Parameters
        ----------
        ids : ndarray of int64, shape (N,)
            Keyframe ids by the current map's numbering, -1 for none.

        Returns
        -------
        ndarray of int64, shape (N,)
            The same keyframes by the run's ids.
        """
        return np.where(ids >= 0, self._kf_base + ids, -1)

    def _current(self) -> int:
        """Return the current map's number; a frame is placed only while one is tracked."""
        assert self._map_id is not None
        return self._map_id

    def _strong_links(self, m: Map) -> IntArray:
        """Return `m`'s strong links, by the run's ids."""
        links = covisibility.strong_links(m, self.options)
        return links + np.array([self._kf_base, self._kf_base, 0], dtype=np.int64)
