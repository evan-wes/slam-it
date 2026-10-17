"""The edge pipeline: initialise, track each frame, map on keyframes (Python's whole run).

:class:`Pipeline` holds the map and the tracking state and runs the stages per frame
(:mod:`slam_it.edge.initialization`, :mod:`slam_it.edge.tracking`,
:mod:`slam_it.edge.mapping`):

1. **Not initialised:** a monocular frame becomes the reference, and later frames are matched
   to it (:func:`slam_it.edge.initialization.initialize_monocular`); with too few matches the
   current frame becomes the reference, and a frame with too few features drops it (ORB-SLAM
   drops its initializer). A stereo or RGB-D frame starts the map from depth. Monocular frames
   are extracted with more features until then (:meth:`Pipeline.extraction`).
2. **Tracking:** predict by constant velocity (no motion when the last frame was not
   tracked) and track the last frame; if that fails, fall back to PnP against the local map
   (:func:`slam_it.edge.tracking.relocalize_locally`), and if that fails, with
   ``relocalization.enabled`` on, relocalise against the whole map through its keyframe database
   (:func:`slam_it.edge.relocalization.relocalize`); then track the local map, its window widened
   just after a recovery (``tracking.recovery_local_window``). Fewer than ``tracking.min_tracked``
   inliers (``tracking.recovery_tracked`` within ``tracking.max_frames`` of a recovery) counts as a
   failure.
3. **Lost:** the track is recently lost from its first failed frame, and found again on the
   next frame tracked. Relocalising, it is lost once more than ``loss.relocalize_seconds`` have
   passed since that frame with none tracked (ORB-SLAM3's ``RECENTLY_LOST``); a map of
   ``loss.reset_keyframes`` keyframes or fewer is lost after ``loss.lost_frames`` failures in a
   row instead (ORB-SLAM3 loses it at once), as is any map with ``relocalization.enabled`` off.
   The map is then kept, or reset if it has ``loss.reset_keyframes`` keyframes or fewer (its
   keyframes and points removed), and the next frame starts a new map (step 1), as ORB-SLAM3's
   atlas does (``Tracking::Track``'s ``CreateMapInAtlas`` and ``ResetActiveMap``). The time runs
   from the first failed frame, as ORB-SLAM3's ``mTimeStampLost`` does. Unlike ORB-SLAM3, every
   failed frame tries the motion model and the local map before relocalising, and the first
   failed frame relocalises too: ORB-SLAM3 relocalises only the frames after the one that lost
   the track, and tries nothing else on them.
4. **Mapping:** a tracked frame that needs a keyframe
   (:func:`slam_it.edge.mapping.need_keyframe`; none within ``tracking.max_frames`` of a recovery
   once the map holds more than ``tracking.max_frames``) is inserted and mapped
   (:func:`slam_it.edge.mapping.process`), culling sparing the keyframes a loop being confirmed
   holds.
5. **Loop closing:** with ``loops.enabled`` and the keyframe database, each keyframe mapped is
   searched for a loop (:func:`slam_it.edge.loops.detect`), the loop being confirmed carried
   from keyframe to keyframe; a loop closed is corrected at once
   (:func:`slam_it.edge.loops.correct`), and the last frame, the keyframe's own, moved with its
   keyframe, as ORB-SLAM's tracking places its last frame by its reference keyframe.

**The keyframe database** (:class:`slam_it.edge.database.Database`), kept only when
relocalising, holds the words of the map being tracked, with the pipeline's vocabulary: a map's
first keyframes enter it when the map starts, a keyframe made by mapping when it is mapped
(timed as mapping, as ORB-SLAM computes a keyframe's words in its local-mapping thread), and a
culled keyframe leaves it. A map closed takes its database with it.

**The final trajectory** (:meth:`Pipeline.trajectory`, ORB-SLAM's ``SaveTrajectoryTUM``): each
tracked frame keeps its reference keyframe (the one it shared the most points with, or its own
when it became one) and its pose relative to it; a culled keyframe keeps its pose relative to
its parent (:attr:`slam_it.edge.map.Map.gone_kf_relative`). Each frame is placed by its
reference keyframe's last pose, through the keyframe's culled ancestors where it was culled, so
the trajectory follows what loop closing and bundle adjustment did to the keyframes after the
frame was tracked. The poses as tracked are kept beside it.

**Maps** are numbered from 0 in the order they start; a reset map's number is not given
again. A map numbers its own keyframes and points from 0, so its first keyframe, which local
BA holds fixed and culling keeps, is always keyframe 0. The run's ids continue across maps:
each map's are offset by the points and keyframes the maps before it made, as ORB-SLAM3's one
counter for every map gives them (:meth:`Pipeline.lifetimes`, :attr:`Fates.point`).

Each frame's result carries how its stages went (:class:`FrameSummary`), which a run records
in its diagnostics, and what became of each of its keypoints (:class:`Fates`), which a run
records if asked (:mod:`slam_it.eval.runs`). After each frame the pipeline dates what the
frame made and removed: every point and keyframe has a life, from the frame that made it to the
frame that removed it, if one did (:meth:`Pipeline.lifetimes`).

The C++ and Rust pipelines run in their native programs; this one runs in-process.
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from slam_it.edge import covisibility, initialization, loops, mapping, relocalization, tracking
from slam_it.edge import map as maps
from slam_it.edge.database import Database
from slam_it.edge.frames import Camera, Frame, Tracked
from slam_it.edge.initialization import Initialization, InitSummary, Outcome
from slam_it.edge.loops import CorrectionSummary, DetectionSummary, Pending
from slam_it.edge.map import Map, Removal
from slam_it.edge.mapping import MappingSummary
from slam_it.edge.options import MODES, Options
from slam_it.edge.relocalization import RelocalizationSummary
from slam_it.edge.tracking import LocalMapSummary, MotionSummary, PnpSummary
from slam_it.foundations import orb, se3
from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.vocabulary import Vocabulary

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


class State(enum.StrEnum):
    """Where the pipeline is."""

    NOT_INITIALIZED = "not_initialized"
    """No map: before the first, or after a lost one, until a new one starts."""
    TRACKING = "tracking"
    """Tracking frames against the map."""
    LOST = "lost"
    """The track was lost on this frame: its map was kept or reset, and the next frame starts a
    new one. Only a frame's result has it; the pipeline never does, being then not
    initialised."""


class Placement(enum.StrEnum):
    """How a frame was placed, before the local map refined it."""

    INITIALIZATION = "initialization"
    """It started the map."""
    MOTION = "motion"
    """Tracking the last frame, from the constant-velocity prediction."""
    LOCAL_PNP = "local_pnp"
    """The fallback: PnP against the local map."""
    RELOCALIZATION = "relocalization"
    """Relocalised against the whole map, through its keyframe database."""


class Fate(enum.IntEnum):
    """What became of a keypoint on its frame, with the codes a run records.

    The first that applies, in the order made, tracked, rejected: a keypoint tracked to a point
    that mapping then culls may have a new point made of it.
    """

    UNMATCHED = 0
    """Neither: the frame's last tracking stage did not match it, and no point was made of it.
    An earlier stage may have matched it (the motion model, before the local map's search)."""
    REJECTED = 1
    """The frame's last tracking stage matched it to a point, and it was not tracked in the
    end: refinement dropped it, or the frame was not tracked."""
    TRACKED = 2
    """An inlier of the frame's final pose."""
    MADE = 3
    """A point was made of it on its frame: by initialisation, or by mapping the frame as a
    keyframe (from its depth, or triangulated)."""


@dataclass(frozen=True)
class Fates:
    """What became of each of a frame's keypoints (:class:`Fate`), and the point concerned.

    Attributes
    ----------
    fate : ndarray of int64, shape (N,)
        Each keypoint's :class:`Fate` code.
    point : ndarray of int64, shape (N,)
        The run's id of the point concerned (:meth:`Pipeline.lifetimes`' ids): the one made of
        it, the one it was tracked to, or the one it was matched to and rejected from; -1 if
        unmatched.
    """

    fate: IntArray
    point: IntArray


class _Summary(Protocol):
    """A stage's summary: it gives its JSON values."""

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values."""
        ...


def _record(summary: _Summary | None) -> dict[str, Any] | None:
    """Return a stage's summary as JSON values, or None if the stage did not run."""
    return None if summary is None else summary.as_dict()


@dataclass(frozen=True)
class LostSummary:
    """The map a lost track closed, as a run records it on the frame it was lost on.

    Attributes
    ----------
    keyframes : int
        The map's keyframes then.
    points : int
        Its points.
    reset : bool
        Whether it was reset, having ``loss.reset_keyframes`` keyframes or fewer, rather than kept.
    """

    keyframes: int
    points: int
    reset: bool

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``keyframes``, ``points`` and ``reset``.
        """
        return {"keyframes": self.keyframes, "points": self.points, "reset": self.reset}


@dataclass(frozen=True)
class LoopSummary:
    """How loop closing went on a keyframe, as a run records it.

    Attributes
    ----------
    detection : slam_it.edge.loops.DetectionSummary
        The search.
    correction : slam_it.edge.loops.CorrectionSummary or None
        The correction, when a loop closed.
    detection_seconds : float
        The time the search took.
    correction_seconds : float
        The time the correction took, global BA included (0 when none closed).
    """

    detection: DetectionSummary
    correction: CorrectionSummary | None
    detection_seconds: float
    correction_seconds: float

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``detection``, ``correction`` (or None), ``detection_seconds`` and
            ``correction_seconds``.
        """
        return {
            "detection": self.detection.as_dict(),
            "correction": _record(self.correction),
            "detection_seconds": self.detection_seconds,
            "correction_seconds": self.correction_seconds,
        }


@dataclass(frozen=True)
class FrameSummary:
    """How the stages went on one frame, as a run records it; None where a stage did not run.

    Attributes
    ----------
    placed_by : Placement or None
        How the frame was placed; None if it was not. A frame placed may still fail the local
        map.
    init : slam_it.edge.initialization.InitSummary or None
        For a frame before the map existed.
    motion : slam_it.edge.tracking.MotionSummary or None
        Tracking the last frame.
    pnp : slam_it.edge.tracking.PnpSummary or None
        The PnP fallback, run only when tracking the last frame failed.
    relocalization : slam_it.edge.relocalization.RelocalizationSummary or None
        Relocalisation, run only when the fallback failed too.
    local_map : slam_it.edge.tracking.LocalMapSummary or None
        Tracking the local map, run on a placed frame.
    mapping : slam_it.edge.mapping.MappingSummary or None
        Local mapping, on a keyframe after the first.
    loop : LoopSummary or None
        Loop closing, on a keyframe after the first while it is on.
    lost : LostSummary or None
        The map closed, on the frame the track was lost on.
    """

    placed_by: Placement | None = None
    init: InitSummary | None = None
    motion: MotionSummary | None = None
    pnp: PnpSummary | None = None
    relocalization: RelocalizationSummary | None = None
    local_map: LocalMapSummary | None = None
    mapping: MappingSummary | None = None
    loop: LoopSummary | None = None
    lost: LostSummary | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values, the keys a run's diagnostics add.

        Returns
        -------
        dict
            ``placed_by`` (by name), ``init``, ``motion``, ``pnp``, ``relocalization``,
            ``local_map``, ``mapping``, ``loop`` and ``lost``, each an object or None.
        """
        return {
            "placed_by": None if self.placed_by is None else self.placed_by.value,
            "init": _record(self.init),
            "motion": _record(self.motion),
            "pnp": _record(self.pnp),
            "relocalization": _record(self.relocalization),
            "local_map": _record(self.local_map),
            "mapping": _record(self.mapping),
            "loop": _record(self.loop),
            "lost": _record(self.lost),
        }


@dataclass(frozen=True)
class FrameResult:
    """What tracking one frame gave.

    Attributes
    ----------
    timestamp : float
        The frame's time.
    state : State
        The frame's: the pipeline's after it, or :attr:`State.LOST` on the frame the track was
        lost on.
    map : int or None
        The map it was tracked in, or lost from; None when there was none.
    pose : ndarray of float64, shape (4, 4) or None
        The frame's ``T_cam_world``, None if it was not tracked.
    tracked : int
        Its keypoints matched to map points.
    keyframe : bool
        Whether it became a keyframe (or started the map).
    track_seconds : float
        The time tracking it took, initialisation included: what ORB-SLAM's tracking thread
        does.
    map_seconds : float
        The time mapping it took as a keyframe (:func:`slam_it.edge.mapping.process`), 0 for
        any other frame: what ORB-SLAM's local-mapping thread does.
    loop_seconds : float
        The time loop closing took on it as a keyframe, 0 for any other frame: what ORB-SLAM's
        loop-closing thread does.
    summary : FrameSummary
        How its stages went.
    fates : Fates
        What became of each of its keypoints.
    """

    timestamp: float
    state: State
    map: int | None
    pose: Array | None
    tracked: int
    keyframe: bool
    track_seconds: float
    map_seconds: float
    loop_seconds: float
    summary: FrameSummary
    fates: Fates


@dataclass(frozen=True)
class Lifetimes:
    """Every point and keyframe a run made, by id, where each last was, its life and its map.

    The ids are the run's, continuing across maps. A life is three numbers: the frame that made
    it (frames counted from 0, one per :meth:`Pipeline.track`), the frame that removed it or
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
    point_lives: NDArray[np.int64]
    keyframe_times: Array
    keyframe_poses: Array
    keyframe_lives: NDArray[np.int64]
    keyframe_parents: NDArray[np.int64]
    keyframe_links: NDArray[np.int64]
    keyframe_loops: NDArray[np.int64]
    point_maps: NDArray[np.int64]
    keyframe_maps: NDArray[np.int64]


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
    maps: NDArray[np.int64]


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


@dataclass(frozen=True)
class _Step:
    """A frame handled: the frame, if tracked, and what came of it.

    ``matched`` holds the last tracking stage's matches before refinement and ``made`` the
    points made of each keypoint, each None where no stage gave them; ``lost`` says the frame
    lost the track.
    """

    tracked: Tracked | None
    keyframe: bool
    map_seconds: float
    summary: FrameSummary
    matched: IntArray | None = None
    made: IntArray | None = None
    lost: bool = False
    loop_seconds: float = 0.0

    def fates(self, size: int, base: int) -> Fates:
        """Return what became of each of the frame's `size` keypoints (:class:`Fate`).

        `base` is the run's id of the map's point 0.
        """
        fate = np.full(size, int(Fate.UNMATCHED), dtype=np.int64)
        point = np.full(size, -1, dtype=np.int64)
        held = None if self.tracked is None else self.tracked.points
        for kind, ids in (
            (Fate.REJECTED, self.matched),
            (Fate.TRACKED, held),
            (Fate.MADE, self.made),
        ):
            if ids is not None:
                fate[ids >= 0] = int(kind)
                point[ids >= 0] = base + ids[ids >= 0]
        return Fates(fate=fate, point=point)


@dataclass
class Pipeline:
    """The edge pipeline for one sequence (see the module docstring).

    Attributes
    ----------
    mode : {"monocular", "stereo", "rgbd"}
        The sensor mode.
    camera : slam_it.edge.frames.Camera
        The camera.
    options : Options
        The settings.
    vocabulary : slam_it.foundations.vocabulary.Vocabulary or None
        The vocabulary the keyframe database's words are of: required with
        ``relocalization.enabled`` on, refused with it off.
    state : State
        Where the pipeline is: never :attr:`State.LOST`, which a lost frame's result has.
    map : Map or None
        The current map; None until one starts, and from a lost one to the next.
    """

    mode: str
    camera: Camera
    options: Options = field(default_factory=Options)
    vocabulary: Vocabulary | None = field(default=None, repr=False)
    state: State = field(default=State.NOT_INITIALIZED, init=False)
    map: Map | None = field(default=None, init=False)
    _database: Database | None = field(default=None, init=False, repr=False)
    _lost_since: float | None = field(default=None, init=False, repr=False)
    _last_frame: Frame | None = field(default=None, init=False, repr=False)
    _last_tracked: Tracked | None = field(default=None, init=False, repr=False)
    _velocity: Array | None = field(default=None, init=False, repr=False)
    _reference: Frame | None = field(default=None, init=False, repr=False)
    _previous: Array | None = field(default=None, init=False, repr=False)
    _failures: int = field(default=0, init=False, repr=False)
    _since_recovery: int | None = field(default=None, init=False, repr=False)
    _frames: int = field(default=0, init=False, repr=False)
    _map_id: int | None = field(default=None, init=False, repr=False)
    _maps: int = field(default=0, init=False, repr=False)
    _kf_base: int = field(default=0, init=False, repr=False)
    _pt_base: int = field(default=0, init=False, repr=False)
    _point_lives: list[list[int]] = field(default_factory=list, init=False, repr=False)
    _keyframe_lives: list[list[int]] = field(default_factory=list, init=False, repr=False)
    _point_maps: list[int] = field(default_factory=list, init=False, repr=False)
    _keyframe_maps: list[int] = field(default_factory=list, init=False, repr=False)
    # Where each point and keyframe no longer in the current map last was, by the run's id:
    # removed, or in a closed map.
    _past_points: dict[int, Array] = field(default_factory=dict, init=False, repr=False)
    _past_keyframes: dict[int, tuple[float, Array, int]] = field(
        default_factory=dict, init=False, repr=False
    )
    # The closed maps' strong links, by the run's ids.
    _past_links: list[NDArray[np.int64]] = field(default_factory=list, init=False, repr=False)
    # The closed maps' loop edges, by the run's ids.
    _past_loops: list[NDArray[np.int64]] = field(default_factory=list, init=False, repr=False)
    # The loop being confirmed in the current map, if any.
    _pending: Pending | None = field(default=None, init=False, repr=False)
    # Each tracked frame's place, and each culled keyframe's parent and pose relative to it, by
    # the run's ids: the final trajectory's (trajectory()).
    _placements: list[_Placement] = field(default_factory=list, init=False, repr=False)
    _culled: dict[int, tuple[int, Array]] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        """Check the mode.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If the mode is unknown, a stereo or RGB-D camera has no baseline, or a vocabulary
            is missing while relocalising, or given while not.
        """
        if self.mode not in MODES:
            raise InvalidInputError(f"mode: expected one of {MODES}, got {self.mode!r}")
        if self.mode != "monocular" and not self.camera.baseline > 0.0:
            raise InvalidInputError(f"camera.baseline: {self.mode} needs a positive baseline")
        if self.options.relocalization.enabled and self.vocabulary is None:
            raise InvalidInputError("vocabulary: relocalization.enabled needs one")
        if not self.options.relocalization.enabled and self.vocabulary is not None:
            raise InvalidInputError("vocabulary: given, but relocalization.enabled is off")

    def extraction(self) -> orb.Options:
        """Return the extraction settings the next frame should be built with.

        Returns
        -------
        slam_it.foundations.orb.Options
            ``features.init_factor`` times the features for a monocular frame before the map
            exists, the usual ones otherwise.
        """
        initializing = self.mode == "monocular" and self.state is State.NOT_INITIALIZED
        return self.options.extraction(initializing=initializing)

    def track(self, frame: Frame) -> FrameResult:
        """Track one frame (and map, if it becomes a keyframe).

        Parameters
        ----------
        frame : slam_it.edge.frames.Frame
            The next frame, built with :meth:`extraction`'s settings.

        Returns
        -------
        FrameResult
            Its pose, or why there is none.
        """
        start = time.perf_counter()
        step = self._track(frame) if self.state is State.TRACKING else self._initialize(frame)
        self._date()
        state = State.LOST if step.lost else self.state
        map_id, summary = self._map_id, step.summary
        fates = step.fates(frame.size, self._pt_base)
        if step.lost and self.map is not None:  # a lost frame was tracking a map
            summary = replace(summary, lost=self._close(self.map))
        track_seconds = time.perf_counter() - start - step.map_seconds - step.loop_seconds
        tracked = step.tracked
        return FrameResult(
            timestamp=frame.timestamp,
            state=state,
            map=map_id,
            pose=None if tracked is None else tracked.pose.copy(),
            tracked=0 if tracked is None else tracked.inliers,
            keyframe=step.keyframe,
            track_seconds=track_seconds,
            map_seconds=step.map_seconds,
            loop_seconds=step.loop_seconds,
            summary=summary,
            fates=fates,
        )

    def lifetimes(self) -> Lifetimes:
        """Return every point and keyframe the run has made, with their lives.

        Returns
        -------
        Lifetimes
            The points and keyframes by the run's id: those in the current map where it has
            them, the others where they last were.
        """
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
        m = self.map
        if m is not None:
            points[self._pt_base + m.pt_id] = m.pt_xyz
            times[self._kf_base + m.kf_id] = m.kf_time
            poses[self._kf_base + m.kf_id] = m.kf_pose
            parents[self._kf_base + m.kf_id] = self._run_ids(m.kf_parent)
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

    def trajectory(self) -> Trajectory:
        """Return the run's trajectory so far: each tracked frame's final pose, and as tracked.

        Returns
        -------
        Trajectory
            The frames tracked, but a reset map's, each placed by its reference keyframe's last
            pose (the module docstring); a frame whose keyframe chain ends at none (a culled
            keyframe that had no parent) keeps its pose as tracked.
        """
        last: dict[int, Array] = {k: pose for k, (_, pose, _) in self._past_keyframes.items()}
        m = self.map
        if m is not None:
            last.update(zip((self._kf_base + m.kf_id).tolist(), m.kf_pose, strict=True))
        poses: list[Array] = []
        for placed in self._placements:
            pose, keyframe = placed.relative, placed.reference
            while keyframe in self._culled:
                parent, relative = self._culled[keyframe]
                pose, keyframe = pose @ relative, parent
            anchor = last.get(keyframe)
            poses.append(placed.tracked if anchor is None else pose @ anchor)
        return Trajectory(
            timestamps=np.array([placed.timestamp for placed in self._placements]),
            poses=np.array(poses).reshape(-1, 4, 4),
            tracked=np.array([placed.tracked for placed in self._placements]).reshape(-1, 4, 4),
            maps=np.array([placed.map for placed in self._placements], dtype=np.int64),
        )

    def _date(self) -> None:
        """Date what this frame made and removed, and empty the map's archive of removals."""
        frame = self._frames
        self._frames += 1
        m, map_id = self.map, self._map_id
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
            m.gone_kf_id, m.gone_kf_time, m.gone_kf_pose, self._run_ids(m.gone_kf_parent),
            m.gone_kf_relative, strict=True,
        ):  # fmt: skip
            run_id = self._kf_base + int(keyframe_id)
            self._keyframe_lives[run_id][1:] = [frame, int(Removal.CULLED)]
            self._past_keyframes[run_id] = (float(seconds), pose, int(parent))
            if parent >= 0:
                self._culled[run_id] = (int(parent), relative)
        if len(m.gone_pt_id) or len(m.gone_kf_id):
            self.map = maps.clear_gone(m)

    def _run_ids(self, parents: NDArray[np.int64]) -> NDArray[np.int64]:
        """Return the current map's keyframe ids as the run's, -1 kept."""
        return np.where(parents >= 0, self._kf_base + parents, -1)

    def _strong_links(self, m: Map) -> NDArray[np.int64]:
        """Return the current map's strong links, by the run's ids."""
        links = covisibility.strong_links(m, self.options)
        return links + np.array([self._kf_base, self._kf_base, 0], dtype=np.int64)

    def _close(self, m: Map) -> LostSummary:
        """Close `m`, the map the track was lost in, on this frame: keep it, or reset it if small.

        Its points and keyframes go where they last were; a reset map's lives end on this
        frame (:attr:`slam_it.edge.map.Removal.RESET`). The tracking state goes with the map, so
        the next frame starts a new one.

        A stand-in until milestone 2B's B4: the map's strong links are counted here, for the
        run's record, inside the frame's timed step, which costs that frame 58 to 360 ms in the
        C++ program on the test set's largest maps; B4's atlas keeps maps whole, and the record
        moves with it
        (``docs/design/2026-10-14-online-slam.md``, section 4).
        """
        frame = self._frames - 1  # dated already
        reset = m.keyframes <= self.options.loss.reset_keyframes
        for point_id, xyz in zip(m.pt_id, m.pt_xyz, strict=True):
            run_id = self._pt_base + int(point_id)
            self._past_points[run_id] = xyz
            if reset:
                self._point_lives[run_id][1:] = [frame, int(Removal.RESET)]
        for keyframe_id, seconds, pose, parent in zip(
            m.kf_id, m.kf_time, m.kf_pose, self._run_ids(m.kf_parent), strict=True
        ):
            run_id = self._kf_base + int(keyframe_id)
            self._past_keyframes[run_id] = (float(seconds), pose, int(parent))
            if reset:
                self._keyframe_lives[run_id][1:] = [frame, int(Removal.RESET)]
        self._past_links.append(self._strong_links(m))
        self._past_loops.append(self._kf_base + m.loop_edges)
        if reset:  # its frames' poses go with it
            self._placements = [p for p in self._placements if p.map != self._map_id]
        lost = LostSummary(keyframes=m.keyframes, points=m.points, reset=reset)
        self.map, self._map_id, self._database = None, None, None
        self._pending = None
        self.state = State.NOT_INITIALIZED
        self._last_frame, self._last_tracked, self._velocity = None, None, None
        self._reference, self._previous = None, None
        self._failures, self._since_recovery, self._lost_since = 0, None, None
        return lost

    def _initialize(self, frame: Frame) -> _Step:
        """Try to start the map with this frame."""
        if self.mode == "monocular":
            attempt = self._initialize_monocular(frame)
        else:
            attempt = initialization.initialize_from_depth(frame, self.options)
        summary = FrameSummary(init=attempt.summary)
        if attempt.started is None:
            return _Step(tracked=None, keyframe=False, map_seconds=0.0, summary=summary)
        self.map, tracked = attempt.started
        if self.vocabulary is not None:
            self._database = Database(self.vocabulary)
            self._sync_database()
        self._map_id, self._maps = self._maps, self._maps + 1
        self._kf_base, self._pt_base = len(self._keyframe_lives), len(self._point_lives)
        self.state = State.TRACKING
        self._place_frame(frame, tracked, int(self.map.kf_id[-1]))
        self._accept(frame, tracked)
        summary = replace(summary, placed_by=Placement.INITIALIZATION)
        # Every point the frame observes was made of it.
        return _Step(tracked, keyframe=True, map_seconds=0.0, summary=summary, made=tracked.points)

    def _initialize_monocular(self, frame: Frame) -> Initialization:
        """Pair a monocular frame with the reference, or make it the reference."""
        if frame.size <= self.options.initialization.min_features:
            self._reference = None  # too few features: ORB-SLAM drops its initializer
            return Initialization(started=None, summary=InitSummary(Outcome.TOO_FEW_FEATURES))
        if self._reference is None or self._previous is None:
            self._reference, self._previous = frame, frame.features.keypoints.xy.copy()
            return Initialization(started=None, summary=InitSummary(Outcome.REFERENCE_SET))
        matched = initialization.match_for_initialization(
            self._reference, frame, self._previous, self.options
        )
        if len(matched.pairs) < self.options.initialization.min_matches:
            # The views no longer overlap enough: start again from this frame.
            self._reference, self._previous = frame, frame.features.keypoints.xy.copy()
            replaced = InitSummary(Outcome.REFERENCE_REPLACED, matches=len(matched.pairs))
            return Initialization(started=None, summary=replaced)
        # The search follows each matched keypoint (SearchForInitialization's vbPrevMatched).
        self._previous[matched.pairs[:, 0]] = frame.features.keypoints.xy[matched.pairs[:, 1]]
        # None keeps the reference: a later frame may have the parallax.
        return initialization.initialize_monocular(
            self.camera, self._reference, frame, matched, self.options
        )

    def _accept(self, frame: Frame, tracked: Tracked) -> None:
        """Make a tracked frame the last one; the velocity only spans consecutive frames."""
        last = self._last_tracked
        if last is not None and self._failures == 0:
            self._velocity = tracked.pose @ se3.inverse(last.pose)
        else:
            self._velocity = None
        self._last_frame, self._last_tracked = frame, tracked
        self._failures, self._lost_since = 0, None

    def _place_frame(self, frame: Frame, tracked: Tracked, reference: int) -> None:
        """Keep a tracked frame's place: its reference keyframe (by the map's id) and pose to it."""
        assert self.map is not None
        assert self._map_id is not None
        k = int(self.map.keyframe_rows(np.array([reference], dtype=np.int64))[0])
        relative = tracked.pose @ se3.inverse(self.map.kf_pose[k])
        self._placements.append(
            _Placement(
                timestamp=frame.timestamp,
                map=self._map_id,
                reference=self._kf_base + reference,
                relative=relative,
                tracked=tracked.pose.copy(),
            )
        )

    def _close_loops(self) -> tuple[LoopSummary | None, bool]:
        """Search the newest keyframe for a loop, and correct one that closes.

        Return how it went (None when loop closing is off, there is no database, or the keyframe
        is the map's first), and whether a loop was corrected.
        """
        m, db = self.map, self._database
        assert m is not None
        keyframe = int(m.kf_id[-1])
        if db is None or not self.options.loops.enabled or keyframe == 0:
            return None, False
        start = time.perf_counter()
        found = loops.detect(
            m, db, self.camera, keyframe, self._pending, mode=self.mode, options=self.options
        )
        self._pending = found.pending
        searched = time.perf_counter()
        correction = None
        if found.loop is not None:
            fixed = loops.correct(m, self.camera, found.loop, mode=self.mode, options=self.options)
            self.map, correction = fixed.map, fixed.summary
            self._sync_database()
        done = time.perf_counter()
        summary = LoopSummary(
            detection=found.summary,
            correction=correction,
            detection_seconds=searched - start,
            correction_seconds=done - searched if correction is not None else 0.0,
        )
        return summary, correction is not None

    def _sync_database(self) -> None:
        """Bring the keyframe database to the map's keyframes: add the new, erase the culled."""
        m, db = self.map, self._database
        assert m is not None
        if db is None:  # not relocalising
            return
        held = set(db.keyframes.tolist())
        for keyframe in sorted(held - set(m.kf_id.tolist())):
            db.erase(keyframe)
        for k, keyframe in enumerate(m.kf_id.tolist()):
            if keyframe not in held:
                db.add(keyframe, m.kp_descriptor[m.keypoint_rows(k)])

    def _record_failure(self, frame: Frame) -> bool:
        """Count an untracked frame, which ends the velocity; return whether it loses the track.

        Lost once more than ``loss.relocalize_seconds`` have passed since the first failed frame, or
        for a map of ``loss.reset_keyframes`` keyframes or fewer or with ``relocalization.enabled``
        off, after ``loss.lost_frames`` failures in a row. :meth:`track` then closes the map.
        """
        assert self.map is not None
        self._failures += 1
        self._velocity = None
        if self._lost_since is None:
            self._lost_since = frame.timestamp
        options = self.options
        if not options.relocalization.enabled or self.map.keyframes <= options.loss.reset_keyframes:
            return self._failures >= options.loss.lost_frames
        return frame.timestamp - self._lost_since > options.loss.relocalize_seconds

    def _place(self, m: Map, frame: Frame) -> tuple[Tracked | None, FrameSummary, IntArray]:
        """Track the last frame, or recover by PnP, or relocalise.

        Return the frame, how it was placed, and the last stage's matches before refinement.
        """
        assert self._last_frame is not None
        assert self._last_tracked is not None
        options = self.options
        # No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM
        # tracks its reference keyframe by bag of words here; the lite pipeline's words only
        # relocalise.
        predicted = tracking.predict(self._last_tracked.pose, self._velocity)
        motion = tracking.track_last_frame(
            m, self.camera, frame, self._last_frame, self._last_tracked,
            predicted=predicted, mode=self.mode, options=options,
        )  # fmt: skip
        summary = FrameSummary(motion=motion.summary)
        if motion.tracked is not None:
            return motion.tracked, replace(summary, placed_by=Placement.MOTION), motion.matched
        last = self._last_tracked.points
        recovery = tracking.relocalize_locally(m, self.camera, frame, last, options)
        summary = replace(summary, pnp=recovery.summary)
        if recovery.tracked is not None:
            placed_by = Placement.LOCAL_PNP
            return recovery.tracked, replace(summary, placed_by=placed_by), recovery.matched
        if self._database is None:  # not relocalising
            return None, summary, recovery.matched
        found = relocalization.relocalize(m, self._database, self.camera, frame, options)
        summary = replace(summary, relocalization=found.summary)
        if found.tracked is None:
            return None, summary, found.matched
        placed_by = Placement.RELOCALIZATION
        return found.tracked, replace(summary, placed_by=placed_by), found.matched

    def _track(self, frame: Frame) -> _Step:
        """Track a frame against the map; insert a keyframe if one is needed (and time it)."""
        assert self.map is not None
        m, options = self.map, self.options
        placed, summary, matched = self._place(m, frame)
        if summary.placed_by in (Placement.LOCAL_PNP, Placement.RELOCALIZATION):
            self._since_recovery = 0
        elif self._since_recovery is not None:
            self._since_recovery += 1
        since = self._since_recovery
        recent = since is not None and since < options.tracking.max_frames
        wide = since is not None and since < options.tracking.recovery_window_frames
        tracked = None
        if placed is not None:
            local = tracking.track_local_map(
                m, self.camera, frame, placed, mode=self.mode, options=options,
                recovering=wide,
            )  # fmt: skip
            summary = replace(summary, local_map=local.summary(self._kf_base))
            matched = local.matched
            m = maps.count(m, local.visible, visible=np.ones(len(local.visible), dtype=np.int64))
            m = maps.count(m, local.found, found=np.ones(len(local.found), dtype=np.int64))
            needed = options.tracking.recovery_tracked if recent else options.tracking.min_tracked
            tracked = local.tracked if local.tracked.inliers >= needed else None
            reference = local.reference
        self.map = m
        if tracked is None:
            lost = self._record_failure(frame)
            return _Step(None, False, 0.0, summary, matched=matched, lost=lost)
        held = recent and m.keyframes > options.tracking.max_frames
        keyframe = not held and mapping.need_keyframe(
            m, self.camera, frame.depth, tracked, reference=reference, mode=self.mode,
            options=options,
        )  # fmt: skip
        map_seconds = loop_seconds = 0.0
        made = None
        corrected = False
        if keyframe:
            start = time.perf_counter()
            holding = () if self._pending is None else self._pending.held
            mapped = mapping.process(
                m, self.camera, frame, tracked, mode=self.mode, options=options, held=holding
            )
            self.map, made = mapped.map, mapped.made
            self._sync_database()
            map_seconds = time.perf_counter() - start
            summary = replace(summary, mapping=mapped.summary)
            # The frame's reference is its own keyframe, placed where the frame was.
            self._placements.append(
                _Placement(
                    frame.timestamp, self._map_id or 0, self._kf_base + int(self.map.kf_id[-1]),
                    np.eye(4), tracked.pose.copy(),
                )
            )  # fmt: skip
            looped, corrected = self._close_loops()
            if looped is not None:
                summary = replace(summary, loop=looped)
                loop_seconds = looped.detection_seconds + looped.correction_seconds
        else:
            self._place_frame(frame, tracked, int(m.kf_id[reference]))
        self._accept(frame, tracked)
        if corrected:  # the last frame, the new keyframe's own, moved with it
            assert self.map is not None
            assert self._last_tracked is not None
            self._last_tracked = replace(self._last_tracked, pose=self.map.kf_pose[-1].copy())
        return _Step(
            tracked, keyframe, map_seconds, summary, matched=matched, made=made,
            loop_seconds=loop_seconds,
        )  # fmt: skip
