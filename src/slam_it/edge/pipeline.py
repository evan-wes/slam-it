"""The edge pipeline: initialise, track each frame, map on keyframes (Python's whole run).

:class:`Pipeline` runs the stages per frame (:mod:`slam_it.edge.initialization`,
:mod:`slam_it.edge.tracking`, :mod:`slam_it.edge.mapping`, :mod:`slam_it.edge.loops`):

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

**The steps and their state.** Tracking (steps 1 to 3, and the keyframe decision), local mapping
(step 4) and loop closing (step 5) are ORB-SLAM's three threads. Each runs here as a function
over state of its own, which :meth:`Pipeline.track` calls in turn, in one thread: tracking's
(:class:`TrackingState`: the last frame, the motion, the loss's counters, the monocular
reference) and loop closing's (:class:`LoopState`: the loop being confirmed). Mapping keeps no
state between keyframes of its own: its map is the owner's, and the keyframes it spares are
those the loop being confirmed holds, which loop closing gives it.

**The map's owner** (:class:`MapOwner`) holds the map being tracked and its keyframe database,
and every change to them goes through it: tracking's counts of the frames each point was seen
and found in, a mapping step's result, a correction, the archive of removals emptied once dated.
A correction holds the map through the owner's handshake (:meth:`MapOwner.stopped`, ORB-SLAM's
``RequestStop``, its global BA's abort and ``Release``), which is nothing in one thread.

**The keyframe database** (:class:`slam_it.edge.database.Database`), kept whenever relocalisation
or loop closing is on (B3's review panel, H7: ORB-SLAM always keeps its ``KeyFrameDatabase``),
holds the words of the map being tracked, with the pipeline's vocabulary: a map's
first keyframes enter it when the map starts, a keyframe made by mapping when it is mapped
(timed as mapping, as ORB-SLAM computes a keyframe's words in its local-mapping thread), and a
culled keyframe leaves it. A map closed takes its database with it.

**The run's records** are the atlas's (:class:`slam_it.edge.atlas.Atlas`): the maps' numbers and
the run's ids, each point's and keyframe's life, and each tracked frame's place, from which the
final trajectory is made (:meth:`Pipeline.lifetimes`, :meth:`Pipeline.trajectory`). After each
frame the atlas dates what the frame made and removed.

**In threads of their own** (``threaded``, milestone 2B's B3b; the online-SLAM design,
section 12), tracking runs in the caller's thread, and local mapping, loop closing and each
global BA in one of their own (:mod:`slam_it.edge.threads`), as ORB-SLAM's do:

- **Tracking** reads only the map's published view (:attr:`MapOwner.view`), once a frame, and
  takes no lock: the view carries the corrections applied to its map, stamped by the writer that
  published it, and the frame's step is given that one view. Its counts of the points seen and
  found go to mapping as messages, applied before its next keyframe. Python's view is its
  immutable map, published by reference; the native pipelines publish a copy of the part
  tracking's next frame may read (:func:`slam_it.edge.covisibility.view_keyframes`,
  :func:`slam_it.edge.map.sub_map`). Relocalisation reads the database with mapping held off,
  against the view published last, which the database matches, and the frame goes on with that
  view. A frame that wants a keyframe asks for one as ORB-SLAM's ``NeedNewKeyFrame`` does: none
  while mapping is stopped or being stopped; taken at once with mapping idle; busy, the local BA
  is interrupted, and a stereo or RGB-D keyframe is queued while fewer than
  ``mapping.max_queued`` wait, a monocular one refused. After a correction, the last frame moves
  with the keyframe it was tracked against.
- **Local mapping** maps each keyframe in turn, the one writer of the map while it does,
  interrupted as ORB-SLAM's ``LocalMapping::Run`` is (:class:`slam_it.edge.mapping.Interrupts`),
  publishes the map once the keyframe's points are made and again when done, and gives the
  keyframe to loop closing. A keyframe tracked on a view older than a correction (queued while
  it ran, or asked for just after) is placed by its reference keyframe as the map now has it,
  ORB-SLAM's ``UpdateLastFrame`` rule (:meth:`slam_it.edge.atlas.Atlas.pose_by`; the
  online-SLAM design, section 12).
- **Loop closing** searches each keyframe with mapping held off; a loop found is corrected with
  mapping stopped (:meth:`MapOwner.stopped`), without global BA, which then solves a copy of the
  map in a thread of its own. A new loop supersedes a global BA still running
  (:class:`slam_it.edge.threads.Generation`): it stops at its next iteration, its result
  dropped; one that finishes is carried onto the map as it has become
  (:func:`slam_it.edge.loops.propagate_global_ba`), mapping stopped.
- **A map lost** waits for local mapping and loop closing to finish the keyframes given, and
  supersedes a global BA running, before it closes.

A frame's result is then what tracking made of it; what the threads finish later comes as
:class:`Completion` records (:meth:`Pipeline.completed`), each naming its frame, and
:meth:`Pipeline.finish` ends the run, waiting for the threads; :meth:`Pipeline.close` ends them
after a failure, and a pipeline dropped without either ends them too. Inline, the default, the steps
run in turn in the caller's thread and a run repeats exactly; the golden records and the stage
tests run so.

Each frame's result carries how its stages went (:class:`FrameSummary`), which a run records
in its diagnostics, and what became of each of its keypoints (:class:`Fates`), which a run
records if asked (:mod:`slam_it.eval.runs`).

The C++ and Rust pipelines run in their native programs; this one runs in-process.
"""

from __future__ import annotations

import enum
import threading
import time
import weakref
from collections.abc import Iterator
from contextlib import AbstractContextManager, contextmanager, nullcontext
from dataclasses import dataclass, field, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it.edge import initialization, loops, mapping, relocalization, tracking
from slam_it.edge import map as maps
from slam_it.edge.atlas import Atlas, Lifetimes, Trajectory
from slam_it.edge.database import Database
from slam_it.edge.frames import Camera, Frame, Tracked
from slam_it.edge.initialization import Initialization, InitSummary, Outcome
from slam_it.edge.loops import CorrectionSummary, DetectionSummary, Pending
from slam_it.edge.map import Map
from slam_it.edge.mapping import Mapped, MappingSummary, Schedule
from slam_it.edge.options import MODES, Options
from slam_it.edge.relocalization import RelocalizationSummary
from slam_it.edge.threads import Generation, Worker, Writer
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

    Its time is the frame's ``loop_seconds`` (:class:`FrameResult`).

    Attributes
    ----------
    detection : slam_it.edge.loops.DetectionSummary
        The search.
    correction : slam_it.edge.loops.CorrectionSummary or None
        The correction, when a loop closed.
    """

    detection: DetectionSummary
    correction: CorrectionSummary | None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``detection``, and ``correction`` (or None).
        """
        return {"detection": self.detection.as_dict(), "correction": _record(self.correction)}


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
    wait_seconds : float
        The time tracking waited for the map, part of ``track_seconds``: 0 inline; threaded,
        relocalisation's wait for mapping to let the map go, and a lost map's for the threads.
    asked : bool
        Whether it wanted a keyframe (:func:`slam_it.edge.mapping.need_keyframe`).
    refused : bool
        Whether it wanted one and mapping could not take it: stopped, or busy and its queue
        full (a monocular frame's queue being none). Never inline.
    queue : int
        The keyframes waiting for mapping when it asked; 0 inline, and when it did not ask.
    schedule : slam_it.edge.mapping.Schedule or None
        How mapping scheduled its keyframe's steps, inline; threaded, it comes later
        (:class:`Completion`).
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
    wait_seconds: float = 0.0
    asked: bool = False
    refused: bool = False
    queue: int = 0
    schedule: Schedule | None = None


@dataclass(frozen=True)
class Completion:
    """What a thread of its own finished after its frame's result was given (threaded runs).

    A run adds its ``entries`` to frame ``frame``'s diagnostics: a keyframe's mapping
    (``map_seconds``, ``mapping``, ``schedule``) and loop closing (``loop_seconds``, ``loop``)
    on the keyframe's frame; a global BA's (``global_ba``, ``global_ba_seconds``) on the frame
    being tracked when it ended, or past the last frame when the run's end waited for it.

    Attributes
    ----------
    frame : int
        The frame, counted from 0, one per :meth:`Pipeline.track`.
    entries : dict
        Its diagnostics' keys, as JSON values.
    made : ndarray of int64, shape (N,) or None
        The keyframe's keypoints' points made by mapping, by the run's ids, -1 where none:
        their fate is :attr:`Fate.MADE`. None but for a keyframe's mapping.
    """

    frame: int
    entries: dict[str, Any]
    made: IntArray | None = None


@dataclass
class TrackingState:
    """Tracking's own state from frame to frame (ORB-SLAM's ``Tracking``'s).

    A map closed takes it with it: the next map starts from a fresh one.

    Attributes
    ----------
    last_frame : slam_it.edge.frames.Frame or None
        The last frame tracked.
    last_tracked : slam_it.edge.frames.Tracked or None
        Its pose and the points it tracked.
    velocity : ndarray of float64, shape (4, 4) or None
        The motion from the frame before it to it, ``T_last_previous``; None unless both were
        tracked, one after the other.
    reference : slam_it.edge.frames.Frame or None
        A monocular map's first view, waiting for a second (ORB-SLAM's initializer).
    previous : ndarray of float64, shape (N, 2) or None
        Where each of the reference's keypoints was last matched, which the next search starts
        from (``SearchForInitialization``'s ``vbPrevMatched``).
    failures : int
        The frames not tracked since the last one tracked.
    since_recovery : int or None
        The frames tracked since the last placed by the PnP fallback or relocalisation; None if
        none was.
    lost_since : float or None
        The time of the first frame not tracked since the last one tracked; None while tracking.
    since_keyframe : int
        The frames since the last that became a keyframe (ORB-SLAM's ``mnLastKeyFrameId``).
    anchor : int or None
        Threaded: the keyframe the last frame was tracked against, by the map's id.
    anchor_relative : ndarray of float64, shape (4, 4) or None
        Threaded: the last frame's pose relative to it, ``T_last_anchor``.
    corrections : int
        Threaded: the corrections the view the last frame was placed on carried
        (:attr:`View.corrections`).
    """

    last_frame: Frame | None = None
    last_tracked: Tracked | None = None
    velocity: Array | None = None
    reference: Frame | None = None
    previous: Array | None = None
    failures: int = 0
    since_recovery: int | None = None
    lost_since: float | None = None
    since_keyframe: int = 0
    anchor: int | None = None
    anchor_relative: Array | None = None
    corrections: int = 0

    def accept(self, frame: Frame, tracked: Tracked) -> None:
        """Make a tracked frame the last one; the velocity only spans consecutive frames.

        Parameters
        ----------
        frame : slam_it.edge.frames.Frame
            The frame.
        tracked : slam_it.edge.frames.Tracked
            Its pose and points.
        """
        last = self.last_tracked
        if last is not None and self.failures == 0:
            self.velocity = tracked.pose @ se3.inverse(last.pose)
        else:
            self.velocity = None
        self.last_frame, self.last_tracked = frame, tracked
        self.failures, self.lost_since = 0, None

    def fail(self, frame: Frame, keyframes: int, options: Options) -> bool:
        """Count an untracked frame, which ends the velocity; return whether it loses the track.

        Lost once more than ``loss.relocalize_seconds`` have passed since the first failed frame,
        or for a map of ``loss.reset_keyframes`` keyframes or fewer or with
        ``relocalization.enabled`` off, after ``loss.lost_frames`` failures in a row.

        Parameters
        ----------
        frame : slam_it.edge.frames.Frame
            The frame not tracked.
        keyframes : int
            The map's keyframes.
        options : Options
            The settings.

        Returns
        -------
        bool
            Whether the track is lost on this frame.
        """
        self.failures += 1
        self.velocity = None
        if self.lost_since is None:
            self.lost_since = frame.timestamp
        if not options.relocalization.enabled or keyframes <= options.loss.reset_keyframes:
            return self.failures >= options.loss.lost_frames
        return frame.timestamp - self.lost_since > options.loss.relocalize_seconds

    def move_last(self, pose: Array) -> None:
        """Move the last frame tracked to `pose`, its keyframe's, after a correction moved it.

        Parameters
        ----------
        pose : ndarray of float64, shape (4, 4)
            Its new ``T_cam_world``.
        """
        assert self.last_tracked is not None
        self.last_tracked = replace(self.last_tracked, pose=pose.copy())

    def anchor_to(self, view: Map, row: int, corrections: int) -> None:
        """Keep the last frame's pose relative to the keyframe it was tracked against (threaded).

        Parameters
        ----------
        view : Map
            The map tracking read.
        row : int
            The keyframe's row in it.
        corrections : int
            The corrections the view carried (:attr:`View.corrections`): the last frame's pose
            is in that map's world.
        """
        assert self.last_tracked is not None
        self.anchor = int(view.kf_id[row])
        self.anchor_relative = self.last_tracked.pose @ se3.inverse(view.kf_pose[row])
        self.corrections = corrections

    def follow(self, view: Map, corrections: int) -> None:
        """Move the last frame with its anchor keyframe, if the map was corrected since (threaded).

        ORB-SLAM's ``UpdateLastFrame`` places the last frame by its reference keyframe; here,
        only after a correction, as inline only the keyframe's own frame moves.

        Parameters
        ----------
        view : Map
            The map's view now.
        corrections : int
            The corrections that view carries (:attr:`View.corrections`), read with it.
        """
        if corrections == self.corrections:
            return
        self.corrections = corrections
        if self.last_tracked is None or self.anchor is None or self.anchor_relative is None:
            return
        row = int(view.keyframe_rows(np.array([self.anchor], dtype=np.int64))[0])
        if row >= 0:
            self.move_last(self.anchor_relative @ view.kf_pose[row])


@dataclass
class LoopState:
    """Loop closing's own state from keyframe to keyframe (ORB-SLAM3's ``LoopClosing``'s).

    A map closed takes it with it.

    Attributes
    ----------
    pending : slam_it.edge.loops.Pending or None
        The loop being confirmed in the map, if any.
    """

    pending: Pending | None = None

    def held(self) -> ArrayLike:
        """Return the keyframes the loop being confirmed holds, which mapping's culling spares.

        Returns
        -------
        array_like of int
            Their ids in the map; none when no loop is being confirmed.
        """
        return () if self.pending is None else self.pending.held


@dataclass(frozen=True)
class View:
    """What tracking reads of the map on a frame (:attr:`MapOwner.view`).

    A writer publishes the map and the count of corrections applied to it together, inside its
    turn, where the count cannot move; so a view read once is a map and the corrections it
    carries, never one without the other (B3b's review panel, A1).

    Attributes
    ----------
    map : Map
        The map as published: Python's view is its immutable map, by reference.
    corrections : int
        The corrections applied to it (:attr:`MapOwner.corrections`).
    """

    map: Map
    corrections: int


@dataclass
class MapOwner:
    """The map being tracked and its keyframe database: every change to them goes through here.

    Inline, the steps change the map in turn and the owner holds it between them; the view is
    the map. Threaded (``writer`` given), one thread writes it at a time
    (:class:`slam_it.edge.threads.Writer`), tracking reads the view the writers publish, and
    tracking's counts wait for local mapping to apply them (:meth:`apply_counts`).

    Attributes
    ----------
    vocabulary : slam_it.foundations.vocabulary.Vocabulary or None
        The database's words' vocabulary; None when neither relocalising nor closing loops, and
        there is no database.
    writer : slam_it.edge.threads.Writer or None
        The handshake between the threads; None inline.
    corrections : int
        The corrections applied: loops corrected, and global BAs carried onto the map.
    """

    vocabulary: Vocabulary | None = field(default=None, repr=False)
    writer: Writer | None = field(default=None, repr=False)
    corrections: int = field(default=0, init=False)
    _map: Map | None = field(default=None, init=False, repr=False)
    _view: View | None = field(default=None, init=False, repr=False)
    _database: Database | None = field(default=None, init=False, repr=False)
    _counts: list[tuple[IntArray, IntArray]] = field(default_factory=list, init=False, repr=False)
    _counting: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    @property
    def map(self) -> Map | None:
        """The map being tracked; None until one starts, and from a lost one to the next."""
        return self._map

    @property
    def view(self) -> View | None:
        """The map last published, with its corrections: what tracking reads (inline, the map)."""
        return self._view

    @property
    def database(self) -> Database | None:
        """Its keyframe database; None with neither relocalisation nor loop closing, or no map."""
        return self._database

    def read(self) -> Map:
        """Return the map being tracked.

        Returns
        -------
        Map
            The map; one is being tracked whenever a step reads it.
        """
        assert self._map is not None
        return self._map

    def start(self, m: Map) -> None:
        """Take `m`, a map just started, and start its database with its keyframes.

        Parameters
        ----------
        m : Map
            The new map.
        """
        self._map, self._view = m, View(m, self.corrections)
        if self.vocabulary is not None:
            self._database = Database(self.vocabulary)
            self._sync_database()

    def count(self, visible: IntArray, found: IntArray) -> None:
        """Count a frame tracked against the local map: the points seen, and found, in it.

        Threaded, the counts wait for local mapping (:meth:`apply_counts`).

        Parameters
        ----------
        visible : ndarray of int64, shape (V,)
            The points the frame's view held (by the map's ids).
        found : ndarray of int64, shape (F,)
            The points it tracked.
        """
        if self.writer is not None:
            with self._counting:
                self._counts.append((visible.copy(), found.copy()))
            return
        m = maps.count(self.read(), visible, visible=np.ones(len(visible), dtype=np.int64))
        self._set(maps.count(m, found, found=np.ones(len(found), dtype=np.int64)))

    def apply_counts(self) -> None:
        """Apply the counts tracking sent since the last (threaded; local mapping's)."""
        with self._counting:
            counts, self._counts = self._counts, []
        for visible, found in counts:
            m = maps.count(self.read(), visible, visible=np.ones(len(visible), dtype=np.int64))
            self._map = maps.count(m, found, found=np.ones(len(found), dtype=np.int64))

    def apply_mapping(self, m: Map) -> None:
        """Take a mapping step's map, bring the database to its keyframes, and publish it.

        Parameters
        ----------
        m : Map
            The map after the step.
        """
        self._map = m
        self._sync_database()
        self._view = View(m, self.corrections)

    def apply_correction(self, m: Map) -> None:
        """Take a corrected map, bring the database to its keyframes, and publish it, counted.

        The correction is counted before the view is published, which carries the new count.

        Parameters
        ----------
        m : Map
            The map after a loop's correction, or a global BA carried onto it.
        """
        self._map = m
        self._sync_database()
        self.corrections += 1
        self._view = View(m, self.corrections)

    def publish(self, m: Map) -> None:
        """Publish a map for tracking to read, while its writer goes on (threaded).

        Parameters
        ----------
        m : Map
            The map so far.
        """
        self._view = View(m, self.corrections)

    def clear_gone(self) -> None:
        """Empty the map's archive of removals, once the atlas has dated them."""
        m = self._map
        if m is not None and (len(m.gone_pt_id) or len(m.gone_kf_id)):
            self._set(maps.clear_gone(m))

    def close(self) -> Map:
        """Give up the map, lost: the database, the view and the counts waiting go with it.

        The next map numbers its points from 0 again, so tracking's counts for this one must
        not reach it.

        Returns
        -------
        Map
            The map.
        """
        m = self.read()
        self._map, self._view, self._database = None, None, None
        with self._counting:
            self._counts = []
        return m

    def reading(self) -> AbstractContextManager[None]:
        """Hold the map whole, with local mapping held off: nothing to hold inline.

        Returns
        -------
        contextlib.AbstractContextManager
            The hold.
        """
        return nullcontext() if self.writer is None else self.writer.held()

    def writing(self) -> AbstractContextManager[None]:
        """Hold the map for local mapping, in turn: nothing to hold inline.

        Returns
        -------
        contextlib.AbstractContextManager
            The hold.
        """
        return nullcontext() if self.writer is None else self.writer.mapping()

    @contextmanager
    def stopped(self) -> Iterator[Map]:
        """Hold the map for a correction: mapping stopped, global BA aborted, then released.

        ORB-SLAM's loop closing stops local mapping (``RequestStop``), aborts a global BA that
        is running (``mbStopGBA``), changes the map, and releases mapping (``Release``). Inline
        nothing runs beside the correction, so there is nothing to stop or abort. Threaded, the
        writer stops mapping (:meth:`slam_it.edge.threads.Writer.stopped`); a global BA running
        is superseded by the caller's new generation. Loop corrections, global BAs' results and
        milestone 2B's B4's merges all enter here.

        Yields
        ------
        Map
            The map, to be corrected and given back through :meth:`apply_correction`.
        """
        with nullcontext() if self.writer is None else self.writer.stopped():
            yield self.read()

    def _set(self, m: Map) -> None:
        """Take a map changed in place of a step: inline, the view is the map."""
        self._map = m
        if self.writer is None:
            self._view = View(m, self.corrections)

    def _sync_database(self) -> None:
        """Bring the keyframe database to the map's keyframes: add the new, erase the culled."""
        m, db = self.read(), self._database
        if db is None:  # not relocalising
            return
        held = set(db.keyframes.tolist())
        for keyframe in sorted(held - set(m.kf_id.tolist())):
            db.erase(keyframe)
        for k, keyframe in enumerate(m.kf_id.tolist()):
            if keyframe not in held:
                db.add(keyframe, m.kp_descriptor[m.keypoint_rows(k)])


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
    wait_seconds: float = 0.0
    asked: bool = False
    refused: bool = False
    queue: int = 0
    schedule: Schedule | None = None

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


@dataclass(frozen=True)
class _Tracking:
    """Tracking's result on a frame: the frame tracked or None, how, and whether it asks a keyframe.

    ``matched`` holds the last stage's matches before refinement; ``reference`` is the frame's
    reference keyframe's row in ``view.map``, ``view`` the view tracking read, -1 when not
    tracked; ``lost``
    says the frame lost the track; ``wait_seconds``, ``asked``, ``refused`` and ``queue`` are
    :class:`FrameResult`'s.
    """

    view: View
    tracked: Tracked | None
    summary: FrameSummary
    matched: IntArray
    reference: int = -1
    keyframe: bool = False
    lost: bool = False
    wait_seconds: float = 0.0
    asked: bool = False
    refused: bool = False
    queue: int = 0


class _Mapping(Protocol):
    """Local mapping, as tracking sees it when it wants a keyframe (``NeedNewKeyFrame``)."""

    def idle(self) -> bool:
        """Return whether mapping takes a keyframe at once (``AcceptKeyFrames``)."""
        ...

    def stopping(self) -> bool:
        """Return whether mapping is stopped or being stopped."""
        ...

    def waiting(self) -> int:
        """Return the keyframes waiting for mapping (``KeyframesInQueue``)."""
        ...

    def interrupt(self) -> None:
        """Interrupt mapping's local BA (``InterruptBA``)."""
        ...


class _Inline:
    """Local mapping run inline: always idle, never stopped, nothing waiting."""

    def idle(self) -> bool:
        """Return True: inline, mapping runs as soon as asked."""
        return True

    def stopping(self) -> bool:
        """Return False: nothing stops mapping inline."""
        return False

    def waiting(self) -> int:
        """Return 0: nothing queues inline."""
        return 0

    def interrupt(self) -> None:
        """Do nothing: inline, no local BA runs while tracking does."""


def _initialize_monocular(
    state: TrackingState, camera: Camera, frame: Frame, options: Options
) -> Initialization:
    """Pair a monocular frame with the reference, or make it the reference."""
    if frame.size <= options.initialization.min_features:
        state.reference = None  # too few features: ORB-SLAM drops its initializer
        return Initialization(started=None, summary=InitSummary(Outcome.TOO_FEW_FEATURES))
    if state.reference is None or state.previous is None:
        state.reference, state.previous = frame, frame.features.keypoints.xy.copy()
        return Initialization(started=None, summary=InitSummary(Outcome.REFERENCE_SET))
    matched = initialization.match_for_initialization(
        state.reference, frame, state.previous, options
    )
    if len(matched.pairs) < options.initialization.min_matches:
        # The views no longer overlap enough: start again from this frame.
        state.reference, state.previous = frame, frame.features.keypoints.xy.copy()
        replaced = InitSummary(Outcome.REFERENCE_REPLACED, matches=len(matched.pairs))
        return Initialization(started=None, summary=replaced)
    # The search follows each matched keypoint (SearchForInitialization's vbPrevMatched).
    state.previous[matched.pairs[:, 0]] = frame.features.keypoints.xy[matched.pairs[:, 1]]
    # None keeps the reference: a later frame may have the parallax.
    return initialization.initialize_monocular(camera, state.reference, frame, matched, options)


def _place(  # noqa: PLR0913 -- tracking's state, the map's view and owner, the frame, the settings
    state: TrackingState,
    view: View,
    owner: MapOwner,
    camera: Camera,
    frame: Frame,
    *,
    mode: str,
    options: Options,
) -> tuple[Tracked | None, FrameSummary, IntArray, float, View]:
    """Track the last frame against `view`, the frame's view, or recover by PnP, or relocalise.

    Relocalisation reads the database with mapping held off, against the view published last,
    which the database matches. Return the frame, how it was placed, the last stage's matches
    before refinement, the time spent waiting for the map (0 inline), and the view the frame
    goes on with.
    """
    assert state.last_frame is not None
    assert state.last_tracked is not None
    m = view.map
    # No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM
    # tracks its reference keyframe by bag of words here; the lite pipeline's words only
    # relocalise.
    predicted = tracking.predict(state.last_tracked.pose, state.velocity)
    motion = tracking.track_last_frame(
        m, camera, frame, state.last_frame, state.last_tracked,
        predicted=predicted, mode=mode, options=options,
    )  # fmt: skip
    summary = FrameSummary(motion=motion.summary)
    if motion.tracked is not None:
        placed_by = Placement.MOTION
        return motion.tracked, replace(summary, placed_by=placed_by), motion.matched, 0.0, view
    last = state.last_tracked.points
    recovery = tracking.relocalize_locally(m, camera, frame, last, options)
    summary = replace(summary, pnp=recovery.summary)
    if recovery.tracked is not None:
        placed_by = Placement.LOCAL_PNP
        return recovery.tracked, replace(summary, placed_by=placed_by), recovery.matched, 0.0, view
    if owner.database is None or not options.relocalization.enabled:
        return None, summary, recovery.matched, 0.0, view
    asked = time.perf_counter()
    with owner.reading():
        held = time.perf_counter()
        db, latest = owner.database, owner.view
        assert db is not None
        assert latest is not None
        view = latest
        found = relocalization.relocalize(view.map, db, camera, frame, options)
    waited = 0.0 if owner.writer is None else held - asked
    summary = replace(summary, relocalization=found.summary)
    if found.tracked is None:
        return None, summary, found.matched, waited, view
    placed_by = Placement.RELOCALIZATION
    return found.tracked, replace(summary, placed_by=placed_by), found.matched, waited, view


def _track_frame(  # noqa: PLR0913 -- tracking's state, the map's owner and view, the frame, the settings
    state: TrackingState,
    owner: MapOwner,
    view: View,
    camera: Camera,
    frame: Frame,
    *,
    mode: str,
    options: Options,
    keyframe_base: int,
    mapper: _Mapping,
) -> _Tracking:
    """Tracking's step: place a frame against its view, track the local map, ask a keyframe.

    `view` is the map's view, read once for the frame. The frame's counts go to the map through
    `owner`; a frame not tracked is counted as a failure. `keyframe_base` is the run's id of the
    map's keyframe 0, for the summary; `mapper` says whether local mapping takes a keyframe (the
    module docstring).
    """
    state.since_keyframe += 1
    placed, summary, matched, waited, view = _place(
        state, view, owner, camera, frame, mode=mode, options=options
    )
    m = view.map
    if summary.placed_by in (Placement.LOCAL_PNP, Placement.RELOCALIZATION):
        state.since_recovery = 0
    elif state.since_recovery is not None:
        state.since_recovery += 1
    since = state.since_recovery
    recent = since is not None and since < options.tracking.max_frames
    wide = since is not None and since < options.tracking.recovery_window_frames
    tracked = None
    reference = -1
    if placed is not None:
        local = tracking.track_local_map(
            m, camera, frame, placed, mode=mode, options=options, recovering=wide
        )
        summary = replace(summary, local_map=local.summary(keyframe_base))
        matched = local.matched
        owner.count(local.visible, local.found)
        needed = options.tracking.recovery_tracked if recent else options.tracking.min_tracked
        tracked = local.tracked if local.tracked.inliers >= needed else None
        reference = local.reference
    if tracked is None:
        lost = state.fail(frame, m.keyframes, options)
        return _Tracking(view, None, summary, matched, lost=lost, wait_seconds=waited)
    if recent and m.keyframes > options.tracking.max_frames:  # none just after a recovery
        return _Tracking(view, tracked, summary, matched, reference=reference, wait_seconds=waited)
    idle = mapper.idle()
    asked = mapping.need_keyframe(
        m, camera, frame.depth, tracked, reference=reference, mode=mode, options=options,
        idle=idle, since_keyframe=state.since_keyframe,
    )  # fmt: skip
    keyframe = refused = False
    queue = 0
    if asked:
        queue = mapper.waiting()
        if mapper.stopping():
            refused = True
        elif idle:
            keyframe = True
        else:
            mapper.interrupt()
            keyframe = mode != "monocular" and queue < options.mapping.max_queued
            refused = not keyframe
    if keyframe:
        state.since_keyframe = 0
    return _Tracking(
        view, tracked, summary, matched, reference=reference, keyframe=keyframe,
        wait_seconds=waited, asked=asked, refused=refused, queue=queue,
    )  # fmt: skip


def _map_keyframe(  # noqa: PLR0913 -- the map's owner, the keyframe, the settings, what is held
    owner: MapOwner,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    *,
    mode: str,
    options: Options,
    held: ArrayLike,
) -> Mapped:
    """Local mapping's step: insert a tracked frame as a keyframe and map around it.

    The map goes back to `owner`; culling spares the `held` keyframes.
    """
    mapped = mapping.process(
        owner.read(), camera, frame, tracked, mode=mode, options=options, held=held
    )
    owner.apply_mapping(mapped.map)
    return mapped


def _close_loops(
    state: LoopState, owner: MapOwner, camera: Camera, *, mode: str, options: Options
) -> tuple[LoopSummary | None, bool]:
    """Loop closing's step: search the newest keyframe for a loop, and correct one that closes.

    Return how it went (None when loop closing is off, there is no database, or the keyframe
    is the map's first), and whether a loop was corrected.
    """
    m, db = owner.read(), owner.database
    keyframe = int(m.kf_id[-1])
    if db is None or not options.loops.enabled or keyframe == 0:
        return None, False
    found = loops.detect(m, db, camera, keyframe, state.pending, mode=mode, options=options)
    state.pending = found.pending
    correction = None
    if found.loop is not None:
        with owner.stopped() as held:
            fixed = loops.correct(held, camera, found.loop, mode=mode, options=options)
            owner.apply_correction(fixed.map)
        correction = fixed.summary
    summary = LoopSummary(detection=found.summary, correction=correction)
    return summary, correction is not None


@dataclass(frozen=True)
class _Request:
    """A keyframe tracking asked local mapping for: the frame, its number, and how it tracked.

    ``reference`` is the frame's reference keyframe, by the map's id, ``relative`` its pose
    relative to it (``T_frame_reference``), and ``corrections`` the corrections the view it was
    tracked on carried (:attr:`View.corrections`): mapping places it by that keyframe if the map
    was corrected since.
    """

    index: int
    frame: Frame
    tracked: Tracked
    reference: int
    relative: Array
    corrections: int


def _still_mapped(tracked: Tracked, m: Map) -> Tracked:
    """Return a frame as tracked, its points no longer in `m` dropped (``ProcessNewKeyFrame``).

    Threaded, mapping may cull or fuse a point a keyframe tracked before it inserts the
    keyframe; ORB-SLAM skips the points gone bad.
    """
    kept = tracked.points.copy()
    seen = kept >= 0
    kept[seen] = np.where(m.point_rows(kept[seen]) >= 0, kept[seen], -1)
    return replace(tracked, points=kept)


class _Interrupts:
    """What local mapping's thread is asked between its steps (:class:`mapping.Interrupts`)."""

    def __init__(self, threads: _Threads) -> None:
        """Ask `threads`."""
        self._threads = threads

    def waiting(self) -> bool:
        """Return whether another keyframe waits."""
        return self._threads.waiting() > 0

    def stopping(self) -> bool:
        """Return whether a stop is requested."""
        return self._threads.stopping()

    def aborted(self) -> bool:
        """Return whether the local BA is to stop."""
        return self._threads.writer.abort.is_set()

    def publish(self, m: Map) -> None:
        """Publish the map so far for tracking."""
        self._threads.owner.publish(m)


class _Threads:
    """Local mapping, loop closing and the global BAs, each in a thread (the module docstring).

    Tracking asks it whether mapping takes a keyframe (:class:`_Mapping`), and gives it the
    keyframes; the threads give back :class:`Completion` records.
    """

    def __init__(self, pipeline: Pipeline, writer: Writer) -> None:
        """Start the threads for `pipeline`, its writer `writer`."""
        self.mode, self.camera, self.options = pipeline.mode, pipeline.camera, pipeline.options
        self.owner, self.atlas, self.closing = pipeline.owner, pipeline.atlas, pipeline.closing
        self.writer = writer
        self.generation = Generation()
        self._inline_ba_off = self.options.with_settings({"loops.global_ba": False})
        self._lock = threading.Lock()
        self._completions: list[Completion] = []
        self._global: list[threading.Thread] = []
        self._errors: list[BaseException] = []
        self.loop_closing: Worker[tuple[int, int]] = Worker("loop closing", self._close_loops)
        self.mapping: Worker[_Request] = Worker("local mapping", self._map)

    def idle(self) -> bool:
        """Return whether local mapping takes a keyframe at once: none in hand, none waiting.

        Counted by the queue, the keyframe in hand from when it is taken until it is mapped, as
        the native cores count it (``unfinished``; B3b's review panel, H4).
        """
        return self.mapping.queue.unfinished() == 0

    def stopping(self) -> bool:
        """Return whether local mapping is stopped or being stopped."""
        return self.writer.stopping()

    def waiting(self) -> int:
        """Return the keyframes waiting for local mapping."""
        return self.mapping.queue.waiting()

    def interrupt(self) -> None:
        """Interrupt local mapping's local BA."""
        self.writer.abort.set()

    def request(self, request: _Request) -> None:
        """Queue a keyframe for local mapping, aborting its local BA (``InsertKeyFrame``)."""
        self.mapping.queue.put(request)
        self.writer.abort.set()

    def completed(self) -> list[Completion]:
        """Return the completions since the last call, in the order they came."""
        with self._lock:
            out, self._completions = self._completions, []
        return out

    def check(self) -> None:
        """Raise the error a thread failed with, if one did.

        Raises
        ------
        BaseException
            The error.
        """
        self.mapping.check()
        self.loop_closing.check()
        with self._lock:
            if self._errors:
                raise self._errors[0]

    def settle(self, *, apply: bool) -> None:
        """Wait for mapping and loop closing to handle every keyframe given, and the global BAs.

        Parameters
        ----------
        apply : bool
            Whether a global BA still running applies its result when it ends (the run's end);
            if not (a map lost), it is superseded.

        Raises
        ------
        BaseException
            The error a thread failed with, if one did.
        """
        self.mapping.queue.drain()
        self.loop_closing.queue.drain()
        if not apply:
            self.generation.next()
        with self._lock:
            running, self._global = self._global, []
        for thread in running:
            thread.join()
        self.check()

    def stop(self) -> None:
        """End the threads; :meth:`settle` first."""
        self.mapping.stop()
        self.loop_closing.stop()

    def _complete(self, completion: Completion) -> None:
        """Keep a completion for the run."""
        with self._lock:
            self._completions.append(completion)

    def _placed(self, request: _Request) -> Tracked:
        """Return a keyframe as tracked, placed by its reference keyframe if the map moved since.

        A correction (a loop's, or a global BA's carried onto the map) since the frame was
        tracked moved the map from under its pose; the keyframe is placed where its reference
        keyframe now is, ``T_frame_reference T_reference_world`` (ORB-SLAM's ``UpdateLastFrame``
        rule; B3b's review panel, A2 and L1), through the reference's culled ancestors. If that
        chain ends at no keyframe of the map, its pose as tracked is kept, as the trajectory
        keeps such a frame's. Mapping's turn held.
        """
        if request.corrections == self.owner.corrections:
            return request.tracked
        pose = self.atlas.pose_by(self.owner.read(), request.reference, request.relative)
        return request.tracked if pose is None else replace(request.tracked, pose=pose)

    def _map(self, request: _Request) -> None:
        """Local mapping's thread: map a keyframe, the one writer while it does."""
        with self.owner.writing():
            self.writer.abort.clear()
            self.owner.apply_counts()
            start = time.perf_counter()
            tracked = _still_mapped(self._placed(request), self.owner.read())
            mapped = mapping.process(
                self.owner.read(), self.camera, request.frame, tracked,
                mode=self.mode, options=self.options, held=self.closing.held(),
                interrupts=_Interrupts(self),
            )  # fmt: skip
            self.owner.apply_mapping(mapped.map)
            seconds = time.perf_counter() - start
            m = self.owner.read()
            self.atlas.resolve(request.index, m)
            self.atlas.record(m)
            self.owner.clear_gone()
            keyframe, base = int(m.kf_id[-1]), self.atlas.point_base
        entries = {
            "map_seconds": seconds,
            "mapping": mapped.summary.as_dict(),
            "schedule": mapped.schedule.as_dict(),
        }
        made = np.where(mapped.made >= 0, base + mapped.made, -1)
        self._complete(Completion(request.index, entries, made=made))
        self.loop_closing.queue.put((request.index, keyframe))

    def _close_loops(self, item: tuple[int, int]) -> None:
        """Loop closing's thread: search a keyframe for a loop, and correct one that closes."""
        index, keyframe = item
        if self.owner.database is None or not self.options.loops.enabled or keyframe == 0:
            return
        start = time.perf_counter()
        copy, generation = None, 0
        with self.owner.reading():
            m, db = self.owner.read(), self.owner.database
            assert db is not None
            if keyframe not in m.kf_id:  # culled since it was mapped
                return
            found = loops.detect(
                m, db, self.camera, keyframe, self.closing.pending, mode=self.mode,
                options=self.options,
            )  # fmt: skip
            self.closing.pending = found.pending
            correction = None
            if found.loop is not None:
                with self.owner.stopped() as held:
                    generation = self.generation.next()
                    fixed = loops.correct(
                        held, self.camera, found.loop, mode=self.mode, options=self._inline_ba_off
                    )
                    self.owner.apply_correction(fixed.map)
                    self.atlas.record(self.owner.read())
                    self.owner.clear_gone()
                    correction = fixed.summary
                    if self.options.loops.global_ba:
                        copy = self.owner.read()
        summary = LoopSummary(detection=found.summary, correction=correction)
        seconds = time.perf_counter() - start
        self._complete(Completion(index, {"loop_seconds": seconds, "loop": summary.as_dict()}))
        if copy is not None:
            thread = threading.Thread(
                target=self._global_ba, args=(copy, generation), name="global BA", daemon=True
            )
            with self._lock:
                self._global.append(thread)
            thread.start()

    def _global_ba(self, copy: Map, generation: int) -> None:
        """Solve a copy in a global BA's thread, and carry the result onto the map if still due."""
        try:
            start = time.perf_counter()
            solved, solve = loops.global_bundle_adjust(
                copy, self.camera, self.options,
                stop=lambda: not self.generation.current(generation),
            )  # fmt: skip
            moved = None
            with self.owner.stopped() as now:
                if self.generation.current(generation):
                    moved = loops.propagate_global_ba(now, solved)
                    self.owner.apply_correction(moved.map)
                    self.atlas.record(self.owner.read())
                    self.owner.clear_gone()
                frame = self.atlas.frame
            summary = loops.GlobalSummary(generation, moved is not None, solve, moved)
            seconds = time.perf_counter() - start
            entries = {"global_ba": summary.as_dict(), "global_ba_seconds": seconds}
            self._complete(Completion(frame, entries))
        except BaseException as error:  # kept, and raised in tracking's thread
            with self._lock:
                self._errors.append(error)


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
        ``relocalization.enabled`` or ``loops.enabled`` on, refused with both off.
    threaded : bool
        Whether the steps run in threads of their own (the module docstring); default, inline.
    state : State
        Where the pipeline is: never :attr:`State.LOST`, which a lost frame's result has.
    tracking : TrackingState
        Tracking's own state.
    closing : LoopState
        Loop closing's own state.
    owner : MapOwner
        The map being tracked and its database.
    atlas : slam_it.edge.atlas.Atlas
        The run's records.
    """

    mode: str
    camera: Camera
    options: Options = field(default_factory=Options)
    vocabulary: Vocabulary | None = field(default=None, repr=False)
    threaded: bool = False
    state: State = field(default=State.NOT_INITIALIZED, init=False)
    tracking: TrackingState = field(default_factory=TrackingState, init=False, repr=False)
    closing: LoopState = field(default_factory=LoopState, init=False, repr=False)
    owner: MapOwner = field(init=False, repr=False)
    atlas: Atlas = field(init=False, repr=False)
    _threads: _Threads | None = field(default=None, init=False, repr=False)
    _finished: bool = field(default=False, init=False, repr=False)
    _finalizer: weakref.finalize[[], Pipeline] | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        """Check the mode, and start the owner, the atlas and, threaded, the threads.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If the mode is unknown, a stereo or RGB-D camera has no baseline, or a vocabulary
            is missing while relocalising or closing loops, or given while doing neither.
        """
        if self.mode not in MODES:
            raise InvalidInputError(f"mode: expected one of {MODES}, got {self.mode!r}")
        if self.mode != "monocular" and not self.camera.baseline > 0.0:
            raise InvalidInputError(f"camera.baseline: {self.mode} needs a positive baseline")
        needed = self.options.keeps_keyframe_database()
        if needed and self.vocabulary is None:
            raise InvalidInputError("vocabulary: relocalization.enabled or loops.enabled needs one")
        if not needed and self.vocabulary is not None:
            raise InvalidInputError(
                "vocabulary: given, but relocalization.enabled and loops.enabled are both off"
            )
        writer = Writer() if self.threaded else None
        self.owner = MapOwner(self.vocabulary, writer)
        self.atlas = Atlas(self.options)
        if writer is not None:
            self._threads = _Threads(self, writer)
            # Dropped without finish() or close(), the threads end rather than wait on their
            # queues: they hold nothing of the pipeline, so it can be collected. Not at exit,
            # where the threads, daemons, end with the interpreter.
            self._finalizer = weakref.finalize(self, self._threads.stop)
            self._finalizer.atexit = False

    @property
    def map(self) -> Map | None:
        """The current map; None until one starts, and from a lost one to the next."""
        return self.owner.map

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

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If the run was finished (:meth:`finish`).
        BaseException
            Threaded, the error a thread failed with, if one did.
        """
        if self._finished:
            raise InvalidInputError("track: the run is finished")
        start = time.perf_counter()
        tracking_map = self.state is State.TRACKING
        step = self._track(frame) if tracking_map else self._initialize(frame)
        threads = self._threads
        waited = step.wait_seconds
        if threads is None:
            self.atlas.date(self.owner.map)
            self.owner.clear_gone()
        else:
            threads.check()
            if not tracking_map and self.owner.map is not None:  # a map started on it
                self.atlas.record(self.owner.map)
                self.owner.clear_gone()
            if step.lost and self.owner.map is not None:  # the threads end the map's work first
                settling = time.perf_counter()
                threads.settle(apply=False)
                waited += time.perf_counter() - settling
            self.atlas.tick()
        state = State.LOST if step.lost else self.state
        map_id, summary = self.atlas.map_id, step.summary
        fates = step.fates(frame.size, self.atlas.point_base)
        if step.lost and self.owner.map is not None:  # a lost frame was tracking a map
            summary = replace(summary, lost=self._close())
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
            wait_seconds=waited,
            asked=step.asked,
            refused=step.refused,
            queue=step.queue,
            schedule=step.schedule,
        )

    def finish(self) -> None:
        """End the run: threaded, wait for the threads to finish their work, then end them.

        Local mapping and loop closing handle every keyframe given, and each global BA running
        ends, the latest carried onto the map; then the threads end. Inline, nothing waits.
        No frame is tracked after.

        Raises
        ------
        BaseException
            Threaded, the error a thread failed with, if one did.
        """
        if self._threads is not None and not self._finished:
            self._finished = True
            self.atlas.end()
            try:
                self._threads.settle(apply=True)
            finally:  # a thread's failure still ends the threads (B3b's review panel, H5)
                self._end_threads()
        self._finished = True

    def close(self) -> None:
        """End the threads without waiting for a global BA or raising what they failed with.

        For a run that failed, or one not wanted: local mapping and loop closing end once the
        keyframes queued are handled, and a global BA running is superseded, so it stops at its
        next iteration. No frame is tracked after. Inline, or after :meth:`finish`, nothing to do.
        """
        if self._threads is not None and not self._finished:
            self._threads.generation.next()
            self._end_threads()
        self._finished = True

    def _end_threads(self) -> None:
        """End the threads now (threaded), which a dropped pipeline then need not."""
        assert self._threads is not None
        if self._finalizer is not None:
            self._finalizer.detach()
        self._threads.stop()

    def completed(self) -> list[Completion]:
        """Return what the threads finished since the last call (threaded; none inline).

        Returns
        -------
        list of Completion
            The completions, in the order they came.
        """
        return [] if self._threads is None else self._threads.completed()

    def lifetimes(self) -> Lifetimes:
        """Return every point and keyframe the run has made, with their lives.

        Returns
        -------
        slam_it.edge.atlas.Lifetimes
            The points and keyframes by the run's id: those in the current map where it has
            them, the others where they last were (:meth:`slam_it.edge.atlas.Atlas.lifetimes`).
        """
        with self.owner.reading():
            return self.atlas.lifetimes(self.owner.map)

    def trajectory(self) -> Trajectory:
        """Return the run's trajectory so far: each tracked frame's final pose, and as tracked.

        Returns
        -------
        slam_it.edge.atlas.Trajectory
            The frames tracked, but a reset map's, each placed by its reference keyframe's last
            pose (:meth:`slam_it.edge.atlas.Atlas.trajectory`).
        """
        with self.owner.reading():
            return self.atlas.trajectory(self.owner.map)

    def _close(self) -> LostSummary:
        """Close the map the track was lost in, on this frame: keep it, or reset it if small.

        The tracking and loop-closing state go with the map, so the next frame starts a new one.
        """
        m = self.owner.close()
        reset = m.keyframes <= self.options.loss.reset_keyframes
        self.atlas.close(m, reset=reset)
        self.tracking = TrackingState()
        self.closing.pending = None  # shared with loop closing's thread, settled by now
        self.state = State.NOT_INITIALIZED
        return LostSummary(keyframes=m.keyframes, points=m.points, reset=reset)

    def _initialize(self, frame: Frame) -> _Step:
        """Try to start the map with this frame."""
        if self.mode == "monocular":
            attempt = _initialize_monocular(self.tracking, self.camera, frame, self.options)
        else:
            attempt = initialization.initialize_from_depth(frame, self.options)
        summary = FrameSummary(init=attempt.summary)
        if attempt.started is None:
            return _Step(tracked=None, keyframe=False, map_seconds=0.0, summary=summary)
        m, tracked = attempt.started
        self.owner.start(m)
        self.atlas.start()
        self.state = State.TRACKING
        self.atlas.place_frame(m, frame.timestamp, int(m.kf_id[-1]), tracked.pose)
        self.tracking.accept(frame, tracked)
        if self._threads is not None:
            view = self.owner.view  # the map's first, of the keyframe it started on
            assert view is not None
            self.tracking.anchor_to(view.map, view.map.keyframes - 1, view.corrections)
        summary = replace(summary, placed_by=Placement.INITIALIZATION)
        # Every point the frame observes was made of it.
        return _Step(tracked, keyframe=True, map_seconds=0.0, summary=summary, made=tracked.points)

    def _track(self, frame: Frame) -> _Step:
        """Run the steps on a frame: track it, and map and close loops if it becomes a keyframe.

        Threaded, a keyframe goes to local mapping's thread instead (:meth:`_hand_over`).
        """
        threads = self._threads
        view = self.owner.view  # read once for the frame: its map and the corrections it carries
        assert view is not None
        if threads is not None:
            self.tracking.follow(view.map, view.corrections)
        found = _track_frame(
            self.tracking, self.owner, view, self.camera, frame, mode=self.mode,
            options=self.options, keyframe_base=self.atlas.keyframe_base,
            mapper=_Inline() if threads is None else threads,
        )  # fmt: skip
        if found.tracked is None:
            return _Step(
                None, False, 0.0, found.summary, matched=found.matched, lost=found.lost,
                wait_seconds=found.wait_seconds,
            )  # fmt: skip
        if threads is not None:
            return self._hand_over(threads, frame, found)
        tracked, summary = found.tracked, found.summary
        map_seconds = loop_seconds = 0.0
        made = None
        schedule = None
        corrected = False
        if found.keyframe:
            start = time.perf_counter()
            mapped = _map_keyframe(
                self.owner, self.camera, frame, tracked, mode=self.mode, options=self.options,
                held=self.closing.held(),
            )  # fmt: skip
            made, schedule = mapped.made, mapped.schedule
            map_seconds = time.perf_counter() - start
            summary = replace(summary, mapping=mapped.summary)
            # The frame's reference is its own keyframe, placed where the frame was.
            self.atlas.place_keyframe(self.owner.read(), frame.timestamp, tracked.pose)
            searching = time.perf_counter()
            looped, corrected = _close_loops(
                self.closing, self.owner, self.camera, mode=self.mode, options=self.options
            )
            if looped is not None:
                summary = replace(summary, loop=looped)
                loop_seconds = time.perf_counter() - searching
        else:
            m = self.owner.read()
            self.atlas.place_frame(m, frame.timestamp, int(m.kf_id[found.reference]), tracked.pose)
        self.tracking.accept(frame, tracked)
        if corrected:  # the last frame, the new keyframe's own, moved with it
            self.tracking.move_last(self.owner.read().kf_pose[-1])
        return _Step(
            tracked, found.keyframe, map_seconds, summary, matched=found.matched, made=made,
            loop_seconds=loop_seconds, asked=found.asked, schedule=schedule,
        )  # fmt: skip

    def _hand_over(self, threads: _Threads, frame: Frame, found: _Tracking) -> _Step:
        """Threaded: place a tracked frame, and give local mapping its keyframe if it is one."""
        tracked = found.tracked
        assert tracked is not None
        view = found.view.map  # the map the frame was tracked in, its reference's row in it
        if found.keyframe:
            index = self.atlas.frames
            self.atlas.place_pending(index, frame.timestamp, tracked.pose)
            reference = int(view.kf_id[found.reference])
            relative = tracked.pose @ se3.inverse(view.kf_pose[found.reference])
            corrections = found.view.corrections
            threads.request(_Request(index, frame, tracked, reference, relative, corrections))
        else:
            reference = int(view.kf_id[found.reference])
            self.atlas.place_frame(view, frame.timestamp, reference, tracked.pose)
        self.tracking.accept(frame, tracked)
        self.tracking.anchor_to(view, found.reference, found.view.corrections)
        return _Step(
            tracked, found.keyframe, 0.0, found.summary, matched=found.matched,
            wait_seconds=found.wait_seconds, asked=found.asked, refused=found.refused,
            queue=found.queue,
        )  # fmt: skip
