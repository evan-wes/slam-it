"""The edge pipeline: initialise, track each frame, map on keyframes (Python's whole run).

:class:`Pipeline` holds the map and the tracking state and runs the stages per frame
(:mod:`slam_it_all.edge.initialization`, :mod:`slam_it_all.edge.tracking`,
:mod:`slam_it_all.edge.mapping`):

1. **Not initialised:** a monocular frame becomes the reference, and later frames are matched
   to it (:func:`slam_it_all.edge.initialization.initialize_monocular`); with too few matches the
   current frame becomes the reference, and a frame with too few features drops it (ORB-SLAM
   drops its initializer). A stereo or RGB-D frame starts the map from depth. Monocular frames
   are extracted with more features until then (:meth:`Pipeline.extraction`).
2. **Tracking:** predict by constant velocity (no motion when the last frame was not
   tracked) and track the last frame; if that fails, fall back to PnP against the local map
   (:func:`slam_it_all.edge.tracking.relocalize_locally`); then track the local map, its window
   widened just after a recovery (``recovery_local_window``). Fewer than ``min_tracked``
   inliers (``recovery_tracked`` within ``max_frames`` of a recovery) counts as a
   failure; after ``lost_frames`` failures in a row the track is lost, and the run ends
   there.
3. **Mapping:** a tracked frame that needs a keyframe
   (:func:`slam_it_all.edge.mapping.need_keyframe`; none within ``max_frames`` of a recovery once
   the map holds more than ``max_frames``) is inserted and mapped
   (:func:`slam_it_all.edge.mapping.process`).

Each frame's result carries how its stages went (:class:`FrameSummary`), which a run records
in its diagnostics, and what became of each of its keypoints (:class:`Fates`), which a run
records if asked (:mod:`slam_it_all.eval.runs`). After each frame the pipeline dates what the
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

from slam_it_all.edge import initialization, mapping, tracking
from slam_it_all.edge import map as maps
from slam_it_all.edge.frames import Camera, Frame, Tracked
from slam_it_all.edge.initialization import Initialization, InitSummary, Outcome
from slam_it_all.edge.map import Map, Removal
from slam_it_all.edge.mapping import MappingSummary
from slam_it_all.edge.options import MODES, Options
from slam_it_all.edge.tracking import LocalMapSummary, MotionSummary, PnpSummary
from slam_it_all.foundations import orb, se3
from slam_it_all.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


class State(enum.StrEnum):
    """Where the pipeline is."""

    NOT_INITIALIZED = "not_initialized"
    """No map yet."""
    TRACKING = "tracking"
    """Tracking frames against the map."""
    LOST = "lost"
    """The track was lost: the run ends."""


class Placement(enum.StrEnum):
    """How a frame was placed, before the local map refined it."""

    INITIALIZATION = "initialization"
    """It started the map."""
    MOTION = "motion"
    """Tracking the last frame, from the constant-velocity prediction."""
    LOCAL_PNP = "local_pnp"
    """The fallback: PnP against the local map."""


class Fate(enum.IntEnum):
    """What became of a keypoint on its frame, with the codes a run records.

    The first that applies, in the order made, tracked, rejected: a keypoint tracked to a point
    that mapping then culls may have a new point made of it.
    """

    UNMATCHED = 0
    """Nothing: no stage matched it, and no point was made of it."""
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
        The id of the point concerned: the one made of it, the one it was tracked to, or the
        one it was matched to and rejected from; -1 if unmatched.
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
class FrameSummary:
    """How the stages went on one frame, as a run records it; None where a stage did not run.

    Attributes
    ----------
    placed_by : Placement or None
        How the frame was placed; None if it was not. A frame placed may still fail the local
        map.
    init : slam_it_all.edge.initialization.InitSummary or None
        For a frame before the map existed.
    motion : slam_it_all.edge.tracking.MotionSummary or None
        Tracking the last frame.
    pnp : slam_it_all.edge.tracking.PnpSummary or None
        The PnP fallback, run only when tracking the last frame failed.
    local_map : slam_it_all.edge.tracking.LocalMapSummary or None
        Tracking the local map, run on a placed frame.
    mapping : slam_it_all.edge.mapping.MappingSummary or None
        Local mapping, on a keyframe after the first.
    """

    placed_by: Placement | None = None
    init: InitSummary | None = None
    motion: MotionSummary | None = None
    pnp: PnpSummary | None = None
    local_map: LocalMapSummary | None = None
    mapping: MappingSummary | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values, the keys a run's diagnostics add.

        Returns
        -------
        dict
            ``placed_by`` (by name), ``init``, ``motion``, ``pnp``, ``local_map`` and
            ``mapping``, each an object or None.
        """
        return {
            "placed_by": None if self.placed_by is None else self.placed_by.value,
            "init": _record(self.init),
            "motion": _record(self.motion),
            "pnp": _record(self.pnp),
            "local_map": _record(self.local_map),
            "mapping": _record(self.mapping),
        }


@dataclass(frozen=True)
class FrameResult:
    """What tracking one frame gave.

    Attributes
    ----------
    timestamp : float
        The frame's time.
    state : State
        The pipeline's state after the frame.
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
        The time mapping it took as a keyframe (:func:`slam_it_all.edge.mapping.process`), 0 for
        any other frame: what ORB-SLAM's local-mapping thread does.
    summary : FrameSummary
        How its stages went.
    fates : Fates
        What became of each of its keypoints.
    """

    timestamp: float
    state: State
    pose: Array | None
    tracked: int
    keyframe: bool
    track_seconds: float
    map_seconds: float
    summary: FrameSummary
    fates: Fates


@dataclass(frozen=True)
class Lifetimes:
    """Every point and keyframe a run made, by id, where each last was, and its life.

    A life is three numbers: the frame that made it (frames counted from 0, one per
    :meth:`Pipeline.track`), the frame that removed it or -1, and why (a
    :class:`slam_it_all.edge.map.Removal` code) or -1.

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
    """

    points: Array
    point_lives: NDArray[np.int64]
    keyframe_times: Array
    keyframe_poses: Array
    keyframe_lives: NDArray[np.int64]


@dataclass(frozen=True)
class _Step:
    """A frame handled: the frame, if tracked, and what came of it.

    ``matched`` holds the last tracking stage's matches before refinement and ``made`` the
    points made of each keypoint, each None where no stage gave them.
    """

    tracked: Tracked | None
    keyframe: bool
    map_seconds: float
    summary: FrameSummary
    matched: IntArray | None = None
    made: IntArray | None = None

    def fates(self, size: int) -> Fates:
        """Return what became of each of the frame's `size` keypoints (:class:`Fate`)."""
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
                point[ids >= 0] = ids[ids >= 0]
        return Fates(fate=fate, point=point)


@dataclass
class Pipeline:
    """The edge pipeline for one sequence (see the module docstring).

    Attributes
    ----------
    mode : {"monocular", "stereo", "rgbd"}
        The sensor mode.
    camera : slam_it_all.edge.frames.Camera
        The camera.
    options : Options
        The settings.
    state : State
        Where the pipeline is.
    map : Map or None
        The map, once initialised.
    """

    mode: str
    camera: Camera
    options: Options = field(default_factory=Options)
    state: State = field(default=State.NOT_INITIALIZED, init=False)
    map: Map | None = field(default=None, init=False)
    _last_frame: Frame | None = field(default=None, init=False, repr=False)
    _last_tracked: Tracked | None = field(default=None, init=False, repr=False)
    _velocity: Array | None = field(default=None, init=False, repr=False)
    _reference: Frame | None = field(default=None, init=False, repr=False)
    _previous: Array | None = field(default=None, init=False, repr=False)
    _failures: int = field(default=0, init=False, repr=False)
    _since_recovery: int | None = field(default=None, init=False, repr=False)
    _frames: int = field(default=0, init=False, repr=False)
    _point_lives: list[list[int]] = field(default_factory=list, init=False, repr=False)
    _keyframe_lives: list[list[int]] = field(default_factory=list, init=False, repr=False)
    _gone_points: dict[int, Array] = field(default_factory=dict, init=False, repr=False)
    _gone_keyframes: dict[int, tuple[float, Array]] = field(
        default_factory=dict, init=False, repr=False
    )

    def __post_init__(self) -> None:
        """Check the mode.

        Raises
        ------
        slam_it_all.foundations.errors.InvalidInputError
            If the mode is unknown, or a stereo or RGB-D camera has no baseline.
        """
        if self.mode not in MODES:
            raise InvalidInputError(f"mode: expected one of {MODES}, got {self.mode!r}")
        if self.mode != "monocular" and not self.camera.baseline > 0.0:
            raise InvalidInputError(f"camera.baseline: {self.mode} needs a positive baseline")

    def extraction(self) -> orb.Options:
        """Return the extraction settings the next frame should be built with.

        Returns
        -------
        slam_it_all.foundations.orb.Options
            ``init_feature_factor`` times the features for a monocular frame before the map
            exists, the usual ones otherwise.
        """
        initializing = self.mode == "monocular" and self.state is State.NOT_INITIALIZED
        return self.options.extraction(initializing=initializing)

    def track(self, frame: Frame) -> FrameResult:
        """Track one frame (and map, if it becomes a keyframe).

        Parameters
        ----------
        frame : slam_it_all.edge.frames.Frame
            The next frame, built with :meth:`extraction`'s settings.

        Returns
        -------
        FrameResult
            Its pose, or why there is none.
        """
        start = time.perf_counter()
        step = _Step(tracked=None, keyframe=False, map_seconds=0.0, summary=FrameSummary())
        if self.state is State.NOT_INITIALIZED:
            step = self._initialize(frame)
        elif self.state is State.TRACKING:
            step = self._track(frame)
        self._date()
        track_seconds = time.perf_counter() - start - step.map_seconds
        tracked = step.tracked
        return FrameResult(
            timestamp=frame.timestamp,
            state=self.state,
            pose=None if tracked is None else tracked.pose.copy(),
            tracked=0 if tracked is None else tracked.inliers,
            keyframe=step.keyframe,
            track_seconds=track_seconds,
            map_seconds=step.map_seconds,
            summary=step.summary,
            fates=step.fates(frame.size),
        )

    def lifetimes(self) -> Lifetimes:
        """Return every point and keyframe the run has made, with their lives.

        Returns
        -------
        Lifetimes
            The points and keyframes by id: those in the map where the map has them, the
            removed ones where they last were.
        """
        m = self.map
        points = np.zeros((len(self._point_lives), 3))
        times = np.zeros(len(self._keyframe_lives))
        poses = np.tile(np.eye(4), (len(self._keyframe_lives), 1, 1))
        if m is not None:
            points[m.pt_id] = m.pt_xyz
            times[m.kf_id] = m.kf_time
            poses[m.kf_id] = m.kf_pose
        for point_id, xyz in self._gone_points.items():
            points[point_id] = xyz
        for keyframe_id, (seconds, pose) in self._gone_keyframes.items():
            times[keyframe_id], poses[keyframe_id] = seconds, pose
        return Lifetimes(
            points=points,
            point_lives=np.array(self._point_lives, dtype=np.int64).reshape(-1, 3),
            keyframe_times=times,
            keyframe_poses=poses,
            keyframe_lives=np.array(self._keyframe_lives, dtype=np.int64).reshape(-1, 3),
        )

    def _date(self) -> None:
        """Date what this frame made and removed, and empty the map's archive of removals."""
        frame = self._frames
        self._frames += 1
        m = self.map
        if m is None:
            return
        made = m.next_pt_id - len(self._point_lives)
        self._point_lives.extend([frame, -1, -1] for _ in range(made))
        made = m.next_kf_id - len(self._keyframe_lives)
        self._keyframe_lives.extend([frame, -1, -1] for _ in range(made))
        for point_id, xyz, how in zip(m.gone_pt_id, m.gone_pt_xyz, m.gone_pt_how, strict=True):
            self._point_lives[int(point_id)][1:] = [frame, int(how)]
            self._gone_points[int(point_id)] = xyz
        for keyframe_id, seconds, pose in zip(
            m.gone_kf_id, m.gone_kf_time, m.gone_kf_pose, strict=True
        ):
            self._keyframe_lives[int(keyframe_id)][1:] = [frame, int(Removal.CULLED)]
            self._gone_keyframes[int(keyframe_id)] = (float(seconds), pose)
        if len(m.gone_pt_id) or len(m.gone_kf_id):
            self.map = maps.clear_gone(m)

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
        self.state = State.TRACKING
        self._accept(frame, tracked)
        summary = replace(summary, placed_by=Placement.INITIALIZATION)
        # Every point the frame observes was made of it.
        return _Step(tracked, keyframe=True, map_seconds=0.0, summary=summary, made=tracked.points)

    def _initialize_monocular(self, frame: Frame) -> Initialization:
        """Pair a monocular frame with the reference, or make it the reference."""
        if frame.size <= self.options.min_init_features:
            self._reference = None  # too few features: ORB-SLAM drops its initializer
            return Initialization(started=None, summary=InitSummary(Outcome.TOO_FEW_FEATURES))
        if self._reference is None or self._previous is None:
            self._reference, self._previous = frame, frame.features.keypoints.xy.copy()
            return Initialization(started=None, summary=InitSummary(Outcome.REFERENCE_SET))
        matched = initialization.match_for_initialization(
            self._reference, frame, self._previous, self.options
        )
        if len(matched.pairs) < self.options.init_min_matches:
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
        self._failures = 0

    def _record_failure(self) -> None:
        """Count an untracked frame: it ends the velocity, and enough in a row lose the track."""
        self._failures += 1
        self._velocity = None
        if self._failures >= self.options.lost_frames:
            self.state = State.LOST

    def _place(self, m: Map, frame: Frame) -> tuple[Tracked | None, FrameSummary, IntArray]:
        """Track the last frame, or recover by PnP.

        Return the frame, how it was placed, and the last stage's matches before refinement.
        """
        assert self._last_frame is not None
        assert self._last_tracked is not None
        options = self.options
        # No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM
        # tracks its reference keyframe by bag of words here; the lite pipeline has none.
        predicted = tracking.predict(self._last_tracked.pose, self._velocity)
        motion = tracking.track_last_frame(
            m, self.camera, frame, self._last_frame, self._last_tracked,
            predicted=predicted, mode=self.mode, options=options,
        )  # fmt: skip
        summary = FrameSummary(motion=motion.summary)
        if motion.tracked is not None:
            return motion.tracked, replace(summary, placed_by=Placement.MOTION), motion.matched
        recovery = tracking.relocalize_locally(m, self.camera, frame, options)
        summary = replace(summary, pnp=recovery.summary)
        if recovery.tracked is None:
            return None, summary, recovery.matched
        placed_by = Placement.LOCAL_PNP
        return recovery.tracked, replace(summary, placed_by=placed_by), recovery.matched

    def _track(self, frame: Frame) -> _Step:
        """Track a frame against the map; insert a keyframe if one is needed (and time it)."""
        assert self.map is not None
        m, options = self.map, self.options
        placed, summary, matched = self._place(m, frame)
        if summary.placed_by is Placement.LOCAL_PNP:
            self._since_recovery = 0
        elif self._since_recovery is not None:
            self._since_recovery += 1
        since = self._since_recovery
        recent = since is not None and since < options.max_frames
        wide = since is not None and since < options.recovery_window_frames
        tracked = None
        if placed is not None:
            local = tracking.track_local_map(
                m, self.camera, frame, placed, mode=self.mode, options=options,
                recovering=wide,
            )  # fmt: skip
            summary = replace(summary, local_map=local.summary())
            matched = local.matched
            m = maps.count(m, local.visible, visible=np.ones(len(local.visible), dtype=np.int64))
            m = maps.count(m, local.found, found=np.ones(len(local.found), dtype=np.int64))
            needed = options.recovery_tracked if recent else options.min_tracked
            tracked = local.tracked if local.tracked.inliers >= needed else None
        self.map = m
        if tracked is None:
            self._record_failure()
            return _Step(None, keyframe=False, map_seconds=0.0, summary=summary, matched=matched)
        held = recent and m.keyframes > options.max_frames
        keyframe = not held and mapping.need_keyframe(
            m, self.camera, frame.depth, tracked, mode=self.mode, options=options
        )
        map_seconds = 0.0
        made = None
        if keyframe:
            start = time.perf_counter()
            mapped = mapping.process(
                m, self.camera, frame, tracked, mode=self.mode, options=options
            )
            self.map, made = mapped.map, mapped.made
            map_seconds = time.perf_counter() - start
            summary = replace(summary, mapping=mapped.summary)
        self._accept(frame, tracked)
        return _Step(tracked, keyframe, map_seconds, summary, matched=matched, made=made)
