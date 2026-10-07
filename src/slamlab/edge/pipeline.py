"""The edge pipeline: initialise, track each frame, map on keyframes (Python's whole run).

:class:`Pipeline` holds the map and the tracking state and runs the stages per frame
(:mod:`slamlab.edge.initialization`, :mod:`slamlab.edge.tracking`, :mod:`slamlab.edge.mapping`):

1. **Not initialised:** a monocular frame becomes the reference, and later frames are matched
   to it (:func:`slamlab.edge.initialization.initialize_monocular`); with too few matches the
   current frame becomes the reference, and a frame with too few features drops it (ORB-SLAM
   drops its initializer). A stereo or RGB-D frame starts the map from depth. Monocular frames
   are extracted with more features until then (:meth:`Pipeline.extraction`).
2. **Tracking:** predict by constant velocity (no motion when the last frame was not
   tracked) and track the last frame; if that fails, fall back to PnP against the local map
   (:func:`slamlab.edge.tracking.relocalize_locally`); then track the local map, its window
   widened just after a recovery. Fewer than ``min_tracked`` inliers (:data:`RECOVERY_TRACKED`
   within ``max_frames`` of a recovery) counts as a failure; after ``lost_frames`` failures
   in a row the track is lost, and the run ends there.
3. **Mapping:** a tracked frame that needs a keyframe (:func:`slamlab.edge.mapping.need_keyframe`;
   none within ``max_frames`` of a recovery once the map holds more than ``max_frames``) is
   inserted and mapped (:func:`slamlab.edge.mapping.process`).

The C++ and Rust pipelines run in their native programs; this one runs in-process.
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from slamlab.edge import initialization, mapping, tracking
from slamlab.edge import map as maps
from slamlab.edge.frames import Camera, Frame, Tracked
from slamlab.edge.map import Map
from slamlab.edge.options import MODES, Options
from slamlab.foundations import orb, se3
from slamlab.foundations.errors import InvalidInputError

Array = NDArray[np.float64]

RECOVERY_WIDEN = 5.0
"""On a recovery's frame and the next, the local map is searched in this many times the
window (``SearchLocalPoints``' ``th = 5``)."""

RECOVERY_WIDE_FRAMES = 2
"""The frames, counting the recovery's, searched that wide."""

RECOVERY_TRACKED = 50
"""Within ``max_frames`` of a recovery, a frame needs this many inliers
(``TrackLocalMap``)."""


class State(enum.StrEnum):
    """Where the pipeline is."""

    NOT_INITIALIZED = "not_initialized"
    """No map yet."""
    TRACKING = "tracking"
    """Tracking frames against the map."""
    LOST = "lost"
    """The track was lost: the run ends."""


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
    seconds : float
        The time tracking (and mapping) it took.
    """

    timestamp: float
    state: State
    pose: Array | None
    tracked: int
    keyframe: bool
    seconds: float


@dataclass
class Pipeline:
    """The edge pipeline for one sequence (see the module docstring).

    Attributes
    ----------
    mode : {"monocular", "stereo", "rgbd"}
        The sensor mode.
    camera : slamlab.edge.frames.Camera
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
    state: State = State.NOT_INITIALIZED
    map: Map | None = None
    _last_frame: Frame | None = field(default=None, repr=False)
    _last_tracked: Tracked | None = field(default=None, repr=False)
    _velocity: Array | None = field(default=None, repr=False)
    _reference: Frame | None = field(default=None, repr=False)
    _failures: int = field(default=0, repr=False)
    _since_recovery: int | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        """Check the mode.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If the mode is unknown, or a stereo or RGB-D camera has no baseline.
        """
        if self.mode not in MODES:
            raise InvalidInputError(f"mode: expected one of {MODES}, got {self.mode!r}")
        if self.mode != "monocular" and not self.camera.baseline > 0.0:
            raise InvalidInputError(f"camera.baseline: {self.mode} needs a positive baseline")

    @property
    def last_tracked(self) -> Tracked | None:
        """The last tracked frame's pose and matches, if any."""
        return self._last_tracked

    def extraction(self) -> orb.Options:
        """Return the extraction settings the next frame should be built with.

        Returns
        -------
        slamlab.foundations.orb.Options
            ``init_feature_factor`` times the features for a monocular frame before the map
            exists, the usual ones otherwise.
        """
        initializing = self.mode == "monocular" and self.state is State.NOT_INITIALIZED
        return self.options.extraction(initializing=initializing)

    def track(self, frame: Frame) -> FrameResult:
        """Track one frame (and map, if it becomes a keyframe).

        Parameters
        ----------
        frame : slamlab.edge.frames.Frame
            The next frame, built with :meth:`extraction`'s settings.

        Returns
        -------
        FrameResult
            Its pose, or why there is none.
        """
        start = time.perf_counter()
        keyframe = False
        tracked: Tracked | None = None
        if self.state is State.NOT_INITIALIZED:
            tracked = self._initialize(frame)
            keyframe = tracked is not None
        elif self.state is State.TRACKING:
            tracked, keyframe = self._track(frame)
        return FrameResult(
            timestamp=frame.timestamp,
            state=self.state,
            pose=None if tracked is None else tracked.pose.copy(),
            tracked=0 if tracked is None else tracked.inliers,
            keyframe=keyframe,
            seconds=time.perf_counter() - start,
        )

    def _initialize(self, frame: Frame) -> Tracked | None:
        """Try to start the map with this frame."""
        if self.mode == "monocular":
            started = self._initialize_monocular(frame)
        else:
            started = initialization.initialize_from_depth(frame, self.options)
        if started is None:
            return None
        self.map, tracked = started
        self.state = State.TRACKING
        self._accept(frame, tracked)
        return tracked

    def _initialize_monocular(self, frame: Frame) -> tuple[Map, Tracked] | None:
        """Pair a monocular frame with the reference, or make it the reference."""
        if frame.size < self.options.min_init_features:
            self._reference = None  # too few features: ORB-SLAM drops its initializer
            return None
        if self._reference is None:
            self._reference = frame
            return None
        matched = initialization.match_for_initialization(self._reference, frame, self.options)
        if len(matched.pairs) < self.options.init_min_matches:
            self._reference = frame  # the views no longer overlap enough: start again
            return None
        # None keeps the reference: a later frame may have the parallax.
        return initialization.initialize_monocular(
            self.camera, self._reference, frame, self.options
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

    def _place(self, m: Map, frame: Frame) -> tuple[Tracked | None, bool]:
        """Track the last frame, or recover by PnP; return the frame and whether it recovered."""
        assert self._last_frame is not None
        assert self._last_tracked is not None
        options = self.options
        # No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM
        # tracks its reference keyframe by bag of words here; the lite pipeline has none.
        predicted = tracking.predict(self._last_tracked.pose, self._velocity)
        tracked = tracking.track_last_frame(
            m, self.camera, frame, self._last_frame, self._last_tracked,
            predicted=predicted, mode=self.mode, options=options,
        )  # fmt: skip
        if tracked is not None:
            return tracked, False
        return tracking.relocalize_locally(m, self.camera, frame, options), True

    def _track(self, frame: Frame) -> tuple[Tracked | None, bool]:
        """Track a frame against the map; insert a keyframe if one is needed."""
        assert self.map is not None
        m, options = self.map, self.options
        placed, recovered = self._place(m, frame)
        if recovered and placed is not None:
            self._since_recovery = 0
        elif self._since_recovery is not None:
            self._since_recovery += 1
        since = self._since_recovery
        recent = since is not None and since < options.max_frames
        wide = since is not None and since < RECOVERY_WIDE_FRAMES
        tracked = None
        if placed is not None:
            local = tracking.track_local_map(
                m, self.camera, frame, placed, mode=self.mode, options=options,
                widen=RECOVERY_WIDEN if wide else 1.0,
            )  # fmt: skip
            m = maps.count(m, local.visible, visible=np.ones(len(local.visible), dtype=np.int64))
            m = maps.count(m, local.found, found=np.ones(len(local.found), dtype=np.int64))
            needed = RECOVERY_TRACKED if recent else options.min_tracked
            tracked = local.tracked if local.tracked.inliers >= needed else None
        self.map = m
        if tracked is None:
            self._record_failure()
            return None, False
        held = recent and m.keyframes > options.max_frames
        keyframe = not held and mapping.need_keyframe(
            m, self.camera, frame.depth, tracked, mode=self.mode, options=options
        )
        if keyframe:
            self.map = mapping.process(
                m, self.camera, frame, tracked, mode=self.mode, options=options
            )
        self._accept(frame, tracked)
        return tracked, keyframe
