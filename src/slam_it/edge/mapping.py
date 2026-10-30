"""Local mapping: grow and clean the map around each new keyframe (ORB-SLAM's ``LocalMapping``).

After tracking decides a frame becomes a keyframe (:func:`need_keyframe`), :func:`process`
runs ORB-SLAM's local mapping steps in order, single-threaded:

1. **Insert the keyframe** with the points it tracked and its features' vocabulary nodes,
   quantised now (:attr:`slam_it.edge.map.Map.kp_node`, ``ComputeBoW`` in
   ``ProcessNewKeyFrame``); a stereo or RGB-D keyframe also creates points from its close depth
   (:func:`create_points_from_depth`, ``CreateNewKeyFrame``).
2. **Cull new points** (:func:`cull_points`, ``MapPointCulling``).
3. **Triangulate new points** between the keyframe and its best links
   (:func:`triangulate_new_points`, ``CreateNewMapPoints``).
4. **Fuse duplicates** (:func:`fuse_points`, ``SearchInNeighbors``).
5. **Local BA** over the keyframe and its links (:func:`slam_it.edge.adjust.bundle_adjust`),
   the keyframes that see their points and the map's first keyframe fixed, once the map holds
   ``adjustment.min_keyframes`` (``KeyFramesInMap() > 2``); skipped when none of them is fixed,
   as ORB-SLAM3 skips it.
6. **Cull redundant keyframes** (:func:`cull_keyframes`, ``KeyFrameCulling``), sparing those
   with a loop edge and those loop detection holds (:func:`slam_it.edge.covisibility.cull`).

:func:`process` returns the map with how each step went (:class:`MappingSummary`), which a
run records per keyframe.

**In a thread of its own** (milestone 2B's B3b), local mapping is asked between its steps what
ORB-SLAM's ``LocalMapping::Run`` asks (:class:`Interrupts`): fusion runs only while no keyframe
waits (``CheckNewKeyFrames``); local BA and keyframe culling only while none waits and no stop
is requested (``stopRequested``); local BA stops early when aborted (``mbAbortBA``,
:func:`slam_it.edge.adjust.bundle_adjust`); and the map is published for tracking once the
keyframe's points are made. How each was scheduled is kept (:class:`Schedule`). Tracking asks
for a keyframe as ORB-SLAM's ``NeedNewKeyFrame`` does whether mapping is idle or busy
(:func:`need_keyframe`). The C++ and Rust cores are interrupted the same way, by their own
threads; their bindings do not cross Python's callables, so a compiled backend given
``interrupts`` is refused.

A keyframe's neighbours are its links in the covisibility graph
(:mod:`slam_it.edge.covisibility`), as ORB-SLAM's are. Matching for triangulation pairs only
keypoints under the same vocabulary node (:attr:`slam_it.edge.map.Map.kp_node`), as ORB-SLAM3's
``SearchForTriangulation`` walks the two keyframes' ``FeatureVector`` node by node
(:func:`match_for_triangulation`, which names its three departures).
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.backends import Crossing
from slam_it.edge import adjust, covisibility, residuals
from slam_it.edge import map as maps
from slam_it.edge.frames import Camera, Frame, Tracked, as_pose, checked_tracked
from slam_it.edge.map import Map, Removal
from slam_it.edge.options import Options, check_mode
from slam_it.foundations import matching, triangulation, vocabulary
from slam_it.foundations.arrays import as_count, as_flag, as_real
from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.vocabulary import Vocabulary

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


class Interrupts(Protocol):
    """What local mapping's own thread is asked between its steps (the module docstring)."""

    def waiting(self) -> bool:
        """Return whether another keyframe waits to be mapped (``CheckNewKeyFrames``)."""
        ...

    def stopping(self) -> bool:
        """Return whether a stop of local mapping is requested (``stopRequested``)."""
        ...

    def aborted(self) -> bool:
        """Return whether the local BA is to stop now (``mbAbortBA``)."""
        ...

    def publish(self, m: Map) -> None:
        """Publish the map so far for tracking, the keyframe's new points made."""
        ...


class Scheduled(enum.StrEnum):
    """How a step local mapping schedules went (:class:`Schedule`)."""

    RAN = "ran"
    """It ran (and may have found nothing to do)."""
    SKIPPED = "skipped"
    """It was passed over: a keyframe waited, or a stop was requested."""
    ABORTED = "aborted"
    """It started and was stopped early."""


@dataclass(frozen=True)
class Schedule:
    """How local mapping scheduled a keyframe's steps (the module docstring).

    Attributes
    ----------
    fusion : Scheduled
        Fusing duplicates with the neighbours (``SearchInNeighbors``).
    adjustment : Scheduled
        The local BA.
    culling : Scheduled
        Culling redundant keyframes.
    """

    fusion: Scheduled
    adjustment: Scheduled
    culling: Scheduled

    def as_dict(self) -> dict[str, str]:
        """Return the schedule as JSON values.

        Returns
        -------
        dict
            ``fusion``, ``adjustment`` and ``culling``, each by name.
        """
        return {
            "fusion": self.fusion.value,
            "adjustment": self.adjustment.value,
            "culling": self.culling.value,
        }


@dataclass(frozen=True)
class MappingSummary:
    """How local mapping went on a keyframe, as a run records it.

    The point counts are the map's before and after each step.

    Attributes
    ----------
    from_depth : int
        Points made from the keyframe's close depth (0 in monocular).
    culled_points : int
        Recent points culled.
    triangulated : int
        Points triangulated with the keyframe's best links.
    fused : int
        Points fused into another.
    solves : tuple of slam_it.edge.adjust.Solve
        The local bundle adjustment's solves; empty if it did not run.
    free : int
        The keyframes it adjusted: the keyframe and its links, but the map's first (0 if it did
        not run).
    fixed : int
        The keyframes it held fixed: the map's first among those, and the others that observe
        their points.
    outliers : int
        The observations it dropped.
    weak_points : int
        Points erased because the observations dropped, by the adjustment or with a culled
        keyframe, left them too weakly observed.
    culled_keyframes : int
        Redundant keyframes culled.
    """

    from_depth: int
    culled_points: int
    triangulated: int
    fused: int
    solves: tuple[adjust.Solve, ...]
    free: int
    fixed: int
    outliers: int
    weak_points: int
    culled_keyframes: int

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            The counts, with the solves as ``ba``, in the steps' order.
        """
        return {
            "from_depth": self.from_depth,
            "culled_points": self.culled_points,
            "triangulated": self.triangulated,
            "fused": self.fused,
            "ba": [solve.as_dict() for solve in self.solves],
            "free": self.free,
            "fixed": self.fixed,
            "outliers": self.outliers,
            "weak_points": self.weak_points,
            "culled_keyframes": self.culled_keyframes,
        }


@dataclass(frozen=True)
class Mapped:
    """A keyframe mapped: the map after local mapping, and how it went.

    Attributes
    ----------
    map : Map
        The map.
    summary : MappingSummary
        How each step went.
    made : ndarray of int64, shape (N,)
        The id of the point made for each of the keyframe's keypoints (from its depth, or
        triangulated), -1 where none: as made, though fusion or culling may since have removed
        it.
    schedule : Schedule
        How its steps were scheduled.
    """

    map: Map
    summary: MappingSummary
    made: IntArray
    schedule: Schedule


_COUNTS = (
    "from_depth",
    "culled_points",
    "triangulated",
    "fused",
    "free",
    "fixed",
    "outliers",
    "weak_points",
    "culled_keyframes",
)
""":class:`MappingSummary`'s counts, as they cross the bindings."""


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.mapping`` submodule provides.

    The map crosses as :meth:`Map.as_dict`, a frame as
    :meth:`slam_it.edge.frames.Frame.as_dict`, a tracked frame as :meth:`Tracked.as_dict`, the
    mode by name, the vocabulary as :func:`slam_it.foundations.vocabulary.fields` gives it. A
    mapped keyframe crosses back as a dict of ``map``, the summary's counts, ``solves`` (as
    :func:`slam_it.edge.adjust.decode_solves` reads them), ``made`` and ``schedule`` (each step's
    :class:`Scheduled` by name).
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def need_keyframe(  # noqa: PLR0913 -- the stage's state, as the reference takes it
        self,
        *,
        m: Crossing,
        camera: Crossing,
        depth: NDArray[np.float64],
        tracked: Crossing,
        reference: int,
        mode: str,
        options: Crossing,
        idle: bool,
        since_keyframe: int | None,
        keyframes: int | None,
    ) -> bool:
        """Decide whether a tracked frame becomes a keyframe."""
        ...

    def create_points_from_depth(
        self, m: Crossing, camera: Crossing, options: Crossing
    ) -> Crossing:
        """Create points from the newest keyframe's close depth."""
        ...

    def cull_points(self, m: Crossing, mode: str, options: Crossing) -> Crossing:
        """Cull recently created points tracking rarely finds."""
        ...

    def match_for_triangulation(
        self, m: Crossing, camera: Crossing, k1: int, k2: int, options: Crossing
    ) -> NDArray[np.int64]:
        """Match two keyframes' free keypoints for triangulation."""
        ...

    def triangulate_new_points(
        self, m: Crossing, camera: Crossing, mode: str, options: Crossing
    ) -> Crossing:
        """Triangulate the newest keyframe's free keypoints against its best links'."""
        ...

    def fuse_targets(self, m: Crossing, mode: str, options: Crossing) -> NDArray[np.int64]:
        """Return the ids of the keyframes the newest keyframe's points are fused into."""
        ...

    def fuse_points(self, m: Crossing, camera: Crossing, mode: str, options: Crossing) -> Crossing:
        """Fuse the newest keyframe's points with its neighbours'."""
        ...

    def local_ba_keyframes(self, m: Crossing, options: Crossing) -> NDArray[np.int64]:
        """Return the rows of the keyframes local BA adjusts."""
        ...

    def cull_keyframes(
        self,
        m: Crossing,
        camera: Crossing,
        mode: str,
        options: Crossing,
        held: NDArray[np.int64],
    ) -> Crossing:
        """Cull the newest keyframe's redundant links."""
        ...

    def process(  # noqa: PLR0913 -- the stage's state, as the reference takes it
        self,
        *,
        m: Crossing,
        camera: Crossing,
        frame: Crossing,
        tracked: Crossing,
        mode: str,
        options: Crossing,
        vocabulary: Crossing,
        held: NDArray[np.int64],
    ) -> Crossing:
        """Insert a keyframe and run local mapping on it."""
        ...


def _arguments(m: Map, camera: Camera | None, options: Options, **items: object) -> Crossing:
    """Return a compiled step's arguments: the map, the camera, the settings, and the rest."""
    out: Crossing = {"m": m.as_dict(), "options": options.as_dict(), **items}
    if camera is not None:
        out["camera"] = camera.as_dict()
    return out


def _decode_mapped(raw: Crossing, codes: backends.Codes) -> Mapped:
    """Rebuild :class:`Mapped` from a compiled backend's dict."""
    counts = {name: int(raw[name]) for name in _COUNTS}
    return Mapped(
        map=maps.decode(raw["map"], codes),
        summary=MappingSummary(solves=adjust.decode_solves(raw["solves"], codes), **counts),
        made=backends.checked_array(raw["made"], np.int64, 1, "made ids"),
        schedule=Schedule(**{step: Scheduled(name) for step, name in raw["schedule"].items()}),
    )


def _check_newest(m: Map) -> None:
    """Refuse a map without keyframes (the steps work on its newest)."""
    if m.keyframes == 0:
        raise InvalidInputError("m: expected a map with at least one keyframe")


def close_depth(camera: Camera, options: Options) -> float:
    """Return the depth below which a point is close: ``stereo.close_depth`` baselines.

    ORB-SLAM's ``mThDepth``.

    Parameters
    ----------
    camera : slam_it.edge.frames.Camera
        The camera (its baseline).
    options : Options
        The factor.

    Returns
    -------
    float
        The depth, in metres (0 for monocular).
    """
    return options.stereo.close_depth * camera.baseline


def _world(pose: Array, local: Array) -> Array:
    """Move camera-frame points into the world: ``R^T (X - t)``."""
    return np.asarray((local - pose[:3, 3]) @ pose[:3, :3], dtype=np.float64)


def _keypoint_indices(m: Map, k: int) -> IntArray:
    """Return a keyframe's keypoint rows."""
    return np.arange(m.kf_start[k], m.kf_start[k + 1], dtype=np.int64)


def _made_since(m: Map, first: int) -> IntArray:
    """Return the points from id `first` on that the newest keyframe observes, by keypoint."""
    ids = m.kp_point[_keypoint_indices(m, m.keyframes - 1)]
    return np.where(ids >= first, ids, -1)


def _from_depth(m: Map, k: int, rows: IntArray) -> Array:
    """Return the world points keypoints' depths give (``UnprojectStereo``)."""
    bearings = m.kp_bearing[rows]
    return _world(m.kf_pose[k], bearings * (m.kp_depth[rows] / bearings[:, 2])[:, None])


def _need_keyframe(  # noqa: PLR0913 -- the stage's state
    m: Map,
    camera: Camera,
    depth: Array,
    tracked: Tracked,
    *,
    reference: int,
    mode: str,
    options: Options,
    idle: bool = True,
    since_keyframe: int | None = None,
    keyframes: int | None = None,
) -> bool:
    """Decide whether a tracked frame becomes a keyframe (reference)."""
    total = m.keyframes if keyframes is None else keyframes
    early = total <= 2  # NeedNewKeyFrame's nKFs <= 2
    min_observations = (
        options.keyframes.ref_min_observations_early
        if early
        else options.keyframes.ref_min_observations
    )
    seen = m.kp_point[m.keypoint_rows(reference)]
    rows = m.point_rows(seen[seen >= 0])
    weights = maps.observation_weight(m, rows[rows >= 0])
    tracked_reference = int(np.count_nonzero(weights >= min_observations))
    need_close = False
    if mode != "monocular":
        close = (depth > 0.0) & (depth < close_depth(camera, options))
        tracked_close = int(np.count_nonzero(close & (tracked.points >= 0)))
        untracked_close = int(np.count_nonzero(close & (tracked.points < 0)))
        need_close = (
            tracked_close < options.keyframes.close_tracked
            and untracked_close > options.keyframes.close_untracked
        )
    ratio = options.keyframes.first_ref_ratio if total < 2 else options.keyframes.ref_ratio
    if mode == "monocular":
        ratio = options.keyframes.mono_ref_ratio
    weak = tracked.inliers < tracked_reference * ratio or need_close
    wanted = bool(weak and tracked.inliers > options.keyframes.min_inliers)  # c2
    if idle:  # c1b: no frames need pass, with mapping idle
        return wanted
    late = since_keyframe is not None and since_keyframe >= options.tracking.max_frames  # c1a
    poor = mode != "monocular" and (  # c1c
        tracked.inliers < tracked_reference * options.keyframes.busy_ref_ratio or need_close
    )
    return wanted and (late or poor)


def need_keyframe(  # noqa: PLR0913 -- the stage's state, and the backend
    m: Map,
    camera: Camera,
    depth: ArrayLike,
    tracked: Tracked,
    *,
    reference: int,
    mode: str,
    options: Options,
    idle: bool = True,
    since_keyframe: int | None = None,
    keyframes: int | None = None,
    backend: str = "python",
) -> bool:
    """Decide whether a tracked frame becomes a keyframe (``NeedNewKeyFrame``).

    With mapping idle, ORB-SLAM's conditions reduce to: the frame tracks fewer than
    ``keyframes.ref_ratio`` (``keyframes.mono_ref_ratio``; ``keyframes.first_ref_ratio`` with one
    keyframe) of the points the frame's reference keyframe (the one sharing the most of its points,
    :func:`slam_it.edge.covisibility.local_map`) sees from two keyframes (three once there are more
    than two), or, with depth, fewer than ``keyframes.close_tracked`` close points while more than
    ``keyframes.close_untracked`` close ones go untracked; and it tracks more than
    ``keyframes.min_inliers``. With mapping busy, those hold and also: ``tracking.max_frames``
    frames have passed since the last keyframe, or, with depth, the frame tracks fewer than
    ``keyframes.busy_ref_ratio`` of the reference's points, or the close points ask for one. Whether
    mapping then takes the keyframe is the pipeline's (:mod:`slam_it.edge.pipeline`).

    Parameters
    ----------
    m : Map
        The map, with at least one keyframe.
    camera : slam_it.edge.frames.Camera
        The camera.
    depth : array_like, shape (N,)
        The frame's keypoints' depths (NaN where unknown).
    tracked : slam_it.edge.frames.Tracked
        Its pose and matches.
    reference : int
        Its reference keyframe's row (:attr:`slam_it.edge.tracking.LocalTracking.reference`).
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The ratios and counts.
    idle : bool, optional
        Whether local mapping is idle (default: it is, as when it runs inline).
    since_keyframe : int, optional
        The frames since the last keyframe, read only with mapping busy (default: unknown, as
        if too few).
    keyframes : int, optional
        The whole map's keyframes, which the rules count, when `m` is a view of it
        (:func:`slam_it.edge.map.sub_map`; default: `m`'s own).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    bool
        Whether to insert a keyframe.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, the map has no keyframe, the depths are infinite, the matches
        are not one per depth, `idle` is not a boolean, `since_keyframe` is not an integer of
        0 or more, or `keyframes` is not an integer of at least `m`'s keyframes.
    """
    check_mode(mode)
    _check_newest(m)
    idle = as_flag(idle, "idle")
    most = int(np.iinfo(np.int64).max)
    if since_keyframe is not None:
        since_keyframe = as_count(since_keyframe, "since_keyframe", 0, most)
    if keyframes is not None:  # the whole map holds every keyframe of its view
        keyframes = as_count(keyframes, "keyframes", m.keyframes, most)
    depths = as_real(depth, "depth", allow_nan=True).reshape(-1)
    ids = np.asarray(tracked.points)
    if ids.shape != depths.shape or not np.issubdtype(ids.dtype, np.integer):
        raise InvalidInputError("tracked.points: expected one point id (or -1) per depth")
    checked = Tracked(pose=as_pose(tracked.pose, "tracked.pose"), points=ids.astype(np.int64))
    row = maps.as_keyframe_row(m, reference, "reference")
    return backends.call(
        backend,
        __name__,
        "need_keyframe",
        reference=lambda: _need_keyframe(
            m,
            camera,
            depths,
            checked,
            reference=row,
            mode=mode,
            options=options,
            idle=idle,
            since_keyframe=since_keyframe,
            keyframes=keyframes,
        ),
        arguments=_arguments(
            m,
            camera,
            options,
            depth=depths,
            tracked=checked.as_dict(),
            reference=row,
            mode=mode,
            idle=idle,
            since_keyframe=since_keyframe,
            keyframes=keyframes,
        ),
        decode=lambda raw, codes: bool(raw),
    )


def _create_points_from_depth(m: Map, camera: Camera, options: Options) -> Map:
    """Create points from the newest keyframe's close depth (reference)."""
    k = m.keyframes - 1
    rows = _keypoint_indices(m, k)
    depth = m.kp_depth[rows]
    usable = (depth > 0.0) & (m.kp_bearing[rows, 2] > 0.0)
    order = rows[usable][np.argsort(depth[usable], kind="stable")]
    limit = close_depth(camera, options)
    chosen = []
    for count, row in enumerate(order, start=1):
        if m.kp_point[row] < 0:
            chosen.append(row)
        if m.kp_depth[row] > limit and count > options.mapping.min_new_points:
            break
    if not chosen:
        return m
    new = np.array(chosen, dtype=np.int64)
    return maps.add_points(m, _from_depth(m, k, new), new[:, None])


def create_points_from_depth(
    m: Map, camera: Camera, options: Options, *, backend: str = "python"
) -> Map:
    """Create points from the newest keyframe's close depth (``CreateNewKeyFrame``).

    Keypoints with depth, nearest first: each one not observing a point creates one, until
    the keyframe holds at least ``mapping.min_new_points`` (counting those it tracked) and the next
    is no longer close.

    Parameters
    ----------
    m : Map
        The map, its newest keyframe stereo or RGB-D.
    camera : slam_it.edge.frames.Camera
        The camera.
    options : Options
        The counts and close depth.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with the new points.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the map has no keyframe.
    """
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "create_points_from_depth",
        reference=lambda: _create_points_from_depth(m, camera, options),
        arguments=_arguments(m, camera, options),
        decode=maps.decode,
    )


def _cull_points(m: Map, mode: str, options: Options) -> Map:
    """Cull recently created points tracking rarely finds (reference)."""
    age = m.kf_id[-1] - m.pt_first_kf
    recent = np.flatnonzero(age <= options.mapping.recent_point_keyframes)
    if len(recent) == 0:
        return m
    minimum = (
        options.mapping.min_point_observations
        if mode == "monocular"
        else options.mapping.min_point_observations_depth
    )
    ratio = m.pt_found[recent] / m.pt_visible[recent]
    weight = maps.observation_weight(m, recent)
    erase = np.zeros(m.points, dtype=bool)
    erase[recent] = (ratio < options.mapping.found_ratio) | (
        (age[recent] >= options.mapping.point_cull_age) & (weight <= minimum)
    )
    return maps.erase_points(m, erase, Removal.CULLED)


def cull_points(m: Map, mode: str, options: Options, *, backend: str = "python") -> Map:
    """Cull recently created points that tracking rarely finds (``MapPointCulling``).

    A point created at most ``mapping.recent_point_keyframes`` keyframes ago goes when found in
    under ``mapping.found_ratio`` of the frames that had it in view, or when, two or more keyframes
    after its creation, it is observed with weight ``mapping.min_point_observations`` or less
    (``mapping.min_point_observations_depth`` with depth).

    Parameters
    ----------
    m : Map
        The map, its newest keyframe the current one.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The ratio and minimums.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map without the culled points.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, or the map has no keyframe.
    """
    check_mode(mode)
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "cull_points",
        reference=lambda: _cull_points(m, mode, options),
        arguments=_arguments(m, None, options, mode=mode),
        decode=maps.decode,
    )


def _relative(m: Map, k1: int, k2: int) -> tuple[Array, Array]:
    """Return ``T_2_1``'s rotation and translation: ``R2 R1^T`` and ``t2 - R t1``."""
    pose1, pose2 = m.kf_pose[k1], m.kf_pose[k2]
    rotation = pose2[:3, :3] @ pose1[:3, :3].T
    return rotation, pose2[:3, 3] - rotation @ pose1[:3, 3]


def _free_rows(m: Map, k: int) -> IntArray:
    """Return a keyframe's keypoints observing no point, with a bearing in front."""
    rows = _keypoint_indices(m, k)
    free: IntArray = rows[(m.kp_point[rows] < 0) & (m.kp_bearing[rows, 2] > 0.0)]
    return free


def _match_for_triangulation(
    m: Map, camera: Camera, k1: int, k2: int, options: Options
) -> IntArray:
    """Match two keyframes' free keypoints for triangulation (reference)."""
    rows1, rows2 = _free_rows(m, k1), _free_rows(m, k2)
    if len(rows1) == 0 or len(rows2) == 0:
        return np.zeros((0, 2), dtype=np.int64)
    rotation, t = _relative(m, k1, k2)
    skew = np.array([[0.0, -t[2], t[1]], [t[2], 0.0, -t[0]], [-t[1], t[0], 0.0]])
    essential = skew @ rotation
    x1 = m.kp_bearing[rows1] / m.kp_bearing[rows1, 2:]
    x2 = m.kp_bearing[rows2] / m.kp_bearing[rows2, 2:]
    lines = x1 @ essential.T  # each k1 keypoint's epipolar line in k2, normalised plane
    numerator = lines @ x2.T
    distance2 = numerator**2 / (lines[:, :1] ** 2 + lines[:, 1:2] ** 2) * camera.focal**2
    band = (
        distance2 < options.mapping.epipolar_chi2 * residuals.sigma2(m, m.kp_level[rows2])[None, :]
    )
    if t[2] != 0.0:  # camera 1's centre in camera 2 is t: the epipole
        epipole = t[:2] / t[2]
        near = np.sum((x2[:, :2] - epipole) ** 2, axis=1) * camera.focal**2 < (
            options.mapping.min_epipole_distance2
            * m.scale_factor ** m.kp_level[rows2].astype(np.float64)
        )
        no_depth = np.isnan(m.kp_depth[rows1])[:, None] & np.isnan(m.kp_depth[rows2])[None, :]
        band &= ~(no_depth & near[None, :])
    same_node = m.kp_node[rows1][:, None] == m.kp_node[rows2][None, :]
    hamming = matching.distances(m.kp_descriptor[rows1], m.kp_descriptor[rows2])
    hamming = np.where(
        same_node & band & (hamming <= options.matching.th_low), hamming, np.iinfo(np.int64).max
    )
    # The nearest candidate, the last among equals: ORB-SLAM3's ``dist > bestDist`` skips only a
    # farther one.
    best = hamming.shape[1] - 1 - np.argmin(hamming[:, ::-1], axis=1)
    best_distance = hamming[np.arange(len(rows1)), best]
    claims: dict[int, tuple[int, int]] = {}
    for i in np.flatnonzero(best_distance <= options.matching.th_low):
        j, d = int(best[i]), int(best_distance[i])
        if j not in claims or d < claims[j][0]:
            claims[j] = (d, int(i))
    pairs = sorted((i, j) for j, (_, i) in claims.items())
    return np.array([[rows1[i], rows2[j]] for i, j in pairs], dtype=np.int64).reshape(-1, 2)


def match_for_triangulation(  # noqa: PLR0913 -- the map, its camera, two keyframes, the settings
    m: Map, camera: Camera, k1: int, k2: int, options: Options, *, backend: str = "python"
) -> IntArray:
    """Match two keyframes' free keypoints for triangulation (``SearchForTriangulation``).

    For each free keypoint of `k1` (one observing no point, with a bearing in front), the nearest
    free keypoint of `k2` under the same vocabulary node (:attr:`slam_it.edge.map.Map.kp_node`;
    ORB-SLAM3 walks the two keyframes' ``FeatureVector`` node by node) within
    ``matching.th_low`` (ties: the last, as ORB-SLAM3's ``dist > bestDist`` keeps) that lies
    within the epipolar band (``mapping.epipolar_chi2`` of its level's variance, in pixels at the
    focal length) and, when neither has depth, not near the epipole
    (``mapping.min_epipole_distance2``). One match per keypoint of `k2`: the nearest keeps it
    (ties: the lower `k1` keypoint).

    Three departures from ORB-SLAM3 (the online-SLAM design's section 11, L2; the first two
    decided by evan-wes on 2026-10-28, the third named by the 0.3.18.0 review panel, L12):

    - A `k2` keypoint is matched once, as the map holds one point per keypoint. ORB-SLAM3 v1.0
      tests ``vbMatched2`` but never sets it, so two `k1` keypoints taking one `k2` keypoint make
      two points that both record that observation while the keyframe keeps only the later
      (``LocalMapping.cc:698-702``), the earlier's left dangling. Its measurements were
      inconclusive.
    - A monocular keyframe triangulates against 20 neighbours, ORB-SLAM2's count, where
      ORB-SLAM3's ``CreateNewMapPoints`` takes 30 (``mapping.mono_triangulation_neighbours``):
      not measured on the test set.
    - :func:`triangulate_new_points` matches against every neighbour, where ORB-SLAM stops after
      the first once a new keyframe waits for mapping (``if(i>0 && CheckNewKeyFrames())
      return;``, ``LocalMapping.cc:436-437``).

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    k1, k2 : int
        The two keyframes' rows.
    options : Options
        The epipolar band and the epipole's distance.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (Q, 2)
        Keypoint rows ``(in k1, in k2)``, by `k1` keypoint.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a keyframe row is out of range, or the two are the same.
    """
    first = maps.as_keyframe_row(m, k1, "k1")
    second = maps.as_keyframe_row(m, k2, "k2")
    if first == second:
        raise InvalidInputError("k1, k2: expected two different keyframes")
    return backends.call(
        backend,
        __name__,
        "match_for_triangulation",
        reference=lambda: _match_for_triangulation(m, camera, first, second, options),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "k1": first,
            "k2": second,
            "options": options.as_dict(),
        },
        decode=lambda raw, codes: backends.checked_array(raw, np.int64, 2, "triangulation pairs"),
    )


def _stereo_cos(m: Map, camera: Camera, rows: IntArray) -> Array:
    """Return the parallax cosine keypoints' depths give: ``cos(2 atan2(b / 2, z))``."""
    return np.asarray(np.cos(2.0 * np.arctan2(camera.baseline / 2.0, m.kp_depth[rows])))


def _triangulate(  # noqa: PLR0913 -- the two keyframes, their matches, the settings
    m: Map, camera: Camera, k1: int, k2: int, pairs: IntArray, *, options: Options
) -> tuple[Array, Mask]:
    """Triangulate matched keypoints; return the points and which pass every check."""
    rows1, rows2 = pairs[:, 0], pairs[:, 1]
    rays1 = m.kp_bearing[rows1] @ m.kf_pose[k1][:3, :3]  # world directions
    rays2 = m.kp_bearing[rows2] @ m.kf_pose[k2][:3, :3]
    cos_rays = np.sum(rays1 * rays2, axis=1)
    has1, has2 = ~np.isnan(m.kp_depth[rows1]), ~np.isnan(m.kp_depth[rows2])
    # ORB-SLAM takes the depth parallax of the first keypoint with depth only; the other's
    # stands at cos_rays + 1, above any ray parallax.
    cos1 = np.where(has1, _stereo_cos(m, camera, rows1), cos_rays + 1.0)
    cos2 = np.where(~has1 & has2, _stereo_cos(m, camera, rows2), cos_rays + 1.0)
    cos_stereo = np.minimum(cos1, cos2)
    from_rays = (
        (cos_rays < cos_stereo)
        & (cos_rays > 0.0)
        & (has1 | has2 | (cos_rays < options.mapping.max_parallax_cos))
    )
    from_depth1 = ~from_rays & has1 & (cos1 < cos2)
    from_depth2 = ~from_rays & ~from_depth1 & has2 & (cos2 < cos1)
    xyz = np.full((len(pairs), 3), np.nan)
    if np.any(from_rays):
        rotation, t = _relative(m, k1, k2)
        relative = np.eye(4)
        relative[:3, :3], relative[:3, 3] = rotation, t
        found = triangulation.two_view(
            relative, m.kp_bearing[rows1[from_rays]], m.kp_bearing[rows2[from_rays]]
        )
        xyz[from_rays] = _world(m.kf_pose[k1], found.points)
    if np.any(from_depth1):
        xyz[from_depth1] = _from_depth(m, k1, rows1[from_depth1])
    if np.any(from_depth2):
        xyz[from_depth2] = _from_depth(m, k2, rows2[from_depth2])
    keep = ~np.isnan(xyz[:, 0])
    filled = np.where(keep[:, None], xyz, 0.0)
    for k, rows in ((k1, rows1), (k2, rows2)):
        error = residuals.chi2(
            camera,
            maps.to_camera(m.kf_pose[k], filled),
            m.kp_xy[rows],
            m.kp_right[rows],
            residuals.sigma2(m, m.kp_level[rows]),
        )
        keep &= error < residuals.thresholds(m.kp_right[rows], options)
    centres = m.centres()
    distance1 = np.linalg.norm(filled - centres[k1], axis=1)
    distance2 = np.linalg.norm(filled - centres[k2], axis=1)
    keep &= (distance1 > 0.0) & (distance2 > 0.0)
    ratio_distance = distance2 / np.where(distance1 > 0.0, distance1, 1.0)
    ratio_octave = m.scale_factor ** (m.kp_level[rows1] - m.kp_level[rows2]).astype(np.float64)
    factor = options.mapping.scale_ratio * m.scale_factor
    keep &= ~((ratio_distance * factor < ratio_octave) | (ratio_distance > ratio_octave * factor))
    return filled, keep


def _median_depth(m: Map, k: int) -> float | None:
    """Return a keyframe's median scene depth, the lower middle (``ComputeSceneMedianDepth``)."""
    seen = m.kp_point[m.keypoint_rows(k)]
    rows = m.point_rows(seen[seen >= 0])
    rows = rows[rows >= 0]
    if len(rows) == 0:
        return None
    depths = np.sort(maps.to_camera(m.kf_pose[k], m.pt_xyz[rows])[:, 2])
    return float(depths[(len(depths) - 1) // 2])


def _triangulate_new_points(m: Map, camera: Camera, mode: str, options: Options) -> Map:
    """Triangulate the newest keyframe's free keypoints against its best links (reference)."""
    k1 = m.keyframes - 1
    neighbours = (
        options.mapping.mono_triangulation_neighbours
        if mode == "monocular"
        else options.mapping.triangulation_neighbours
    )
    others = covisibility.links(m, k1, options)[0][:neighbours]
    centres = m.centres()
    for k2 in (int(k) for k in others):
        baseline = float(np.linalg.norm(centres[k2] - centres[k1]))
        if mode == "monocular":
            median = _median_depth(m, k2)
            if median is None or baseline / median < options.mapping.min_baseline_depth:
                continue
        elif baseline < camera.baseline:
            continue
        pairs = _match_for_triangulation(m, camera, k1, k2, options)
        if len(pairs) == 0:
            continue
        xyz, keep = _triangulate(m, camera, k1, k2, pairs, options=options)
        if np.any(keep):
            m = maps.add_points(m, xyz[keep], pairs[keep])
    return m


def triangulate_new_points(
    m: Map, camera: Camera, mode: str, options: Options, *, backend: str = "python"
) -> Map:
    """Triangulate the newest keyframe's free keypoints against its best links.

    ORB-SLAM's ``CreateNewMapPoints``. For each neighbour (the keyframe's best links, at most
    ``mapping.triangulation_neighbours``, ``mapping.mono_triangulation_neighbours`` for
    monocular): skip it if the baseline is too short (shorter than the stereo baseline, or under
    ``mapping.min_baseline_depth`` of its median depth for monocular);
    match (:func:`match_for_triangulation`); for each match take the point from the rays
    (:func:`slam_it.foundations.triangulation.two_view`) if their parallax beats what depth
    gives, else from the keypoint with the better depth; keep it if it is in front of both,
    passes the chi-square test in both, and its distances agree with the keypoints' levels
    (``mapping.scale_ratio``).

    Every neighbour is matched: ORB-SLAM stops after the first once a new keyframe waits for
    mapping (``LocalMapping.cc:436-437``), a departure :func:`match_for_triangulation` names
    with the others.

    Parameters
    ----------
    m : Map
        The map, its newest keyframe the current one.
    camera : slam_it.edge.frames.Camera
        The camera.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The thresholds.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with the new points.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, or the map has no keyframe.
    """
    check_mode(mode)
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "triangulate_new_points",
        reference=lambda: _triangulate_new_points(m, camera, mode, options),
        arguments=_arguments(m, camera, options, mode=mode),
        decode=maps.decode,
    )


def _fuse_target(m: Map, camera: Camera, k: int, p: int, options: Options) -> int | None:
    """Return the keypoint row of keyframe `k` that point row `p` fuses into, if any (``Fuse``)."""
    pose = m.kf_pose[k]
    xyz = m.pt_xyz[p]
    local = maps.to_camera(pose, xyz[None])
    if not local[0, 2] > 0.0:
        return None
    pixel = camera.project(local)[0]
    offset = xyz - maps.camera_centre(pose)
    distance = float(np.linalg.norm(offset))
    if not (
        camera.inside(pixel[None])[0]
        and maps.in_distance_range(m, np.array([p]), np.array([distance]), options)[0]
        and maps.viewing_cos(m, np.array([p]), offset[None], np.array([distance]))[0]
        >= options.tracking.view_cos_limit
    ):
        return None
    level = int(maps.predict_level(m, np.array([p]), np.array([distance]))[0])
    rows = _keypoint_indices(m, k)
    near = matching.features_in_area(
        m.kp_xy[rows],
        m.kp_level[rows],
        pixel,
        options.mapping.fuse_window * m.scale_factor**level,
        min_level=level - 1,
        max_level=level,
    )
    candidates = rows[near]
    error = residuals.chi2(
        camera,
        np.repeat(local, len(candidates), axis=0),
        m.kp_xy[candidates],
        m.kp_right[candidates],
        residuals.sigma2(m, m.kp_level[candidates]),
    )
    candidates = candidates[error < residuals.thresholds(m.kp_right[candidates], options)]
    if len(candidates) == 0:
        return None
    hamming = matching.distances(m.pt_descriptor[p : p + 1], m.kp_descriptor[candidates])[0]
    best = int(np.argmin(hamming))
    return int(candidates[best]) if hamming[best] <= options.matching.th_low else None


def _fuse_one(m: Map, camera: Camera, k: int, point_id: int, options: Options) -> Map:
    """Fuse one point into a keyframe: a free keypoint observes it, a held one merges two."""
    p = int(m.point_rows(np.array([point_id]))[0])
    if p < 0 or np.any(m.kp_point[_keypoint_indices(m, k)] == point_id):
        return m  # merged away, or already in the keyframe
    row = _fuse_target(m, camera, k, p, options)
    if row is None:
        return m
    held = int(m.kp_point[row])
    if held < 0:
        kp_point = m.kp_point.copy()
        kp_point[row] = point_id
        return replace(m, kp_point=kp_point)
    q = int(m.point_rows(np.array([held]))[0])
    weights = maps.observation_weight(m, np.array([q, p]))
    if weights[0] > weights[1]:
        return maps.replace_point(m, lose=point_id, keep=held)
    return maps.replace_point(m, lose=held, keep=point_id)


def _fuse_targets(m: Map, mode: str, options: Options) -> IntArray:
    """Return the newest keyframe's fusion targets' ids, as ``SearchInNeighbors`` takes them.

    Its best links, then each one's own best in turn, leaving out the newest keyframe and
    repeats: ORB-SLAM3's order. ORB-SLAM2 took each link's own best right after it, and could
    take a keyframe twice (it did not mark those).
    """
    newest = m.keyframes - 1
    settings = options.mapping
    n = settings.mono_fuse_neighbours if mode == "monocular" else settings.fuse_neighbours
    targets = [int(r) for r in covisibility.links(m, newest, options)[0][:n]]
    for row in list(targets):
        second = covisibility.links(m, row, options)[0][: settings.fuse_second_neighbours]
        targets += [int(r) for r in second if int(r) != newest and int(r) not in targets]
    return m.kf_id[np.array(targets, dtype=np.int64)]


def fuse_targets(m: Map, mode: str, options: Options, *, backend: str = "python") -> IntArray:
    """Return the keyframes the newest keyframe's points are fused into (``SearchInNeighbors``).

    Its best ``mapping.fuse_neighbours`` links (``mapping.mono_fuse_neighbours`` in monocular),
    then each one's best ``mapping.fuse_second_neighbours`` in turn, leaving out the newest
    keyframe and repeats: ORB-SLAM3's order, with ORB-SLAM2's counts. ORB-SLAM2 took each link's
    own best right after it, and could take a keyframe twice. :func:`fuse_points` fuses into
    them.

    Parameters
    ----------
    m : Map
        The map, with at least one keyframe.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The counts and ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (T,)
        The targets' ids, in order.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, or the map has no keyframe.
    """
    check_mode(mode)
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "fuse_targets",
        reference=lambda: _fuse_targets(m, mode, options),
        arguments=_arguments(m, None, options, mode=mode),
        decode=lambda raw, codes: backends.checked_array(raw, np.int64, 1, "fusion targets"),
    )


def _local_ba_keyframes(m: Map, options: Options) -> IntArray:
    """Return the newest keyframe's row and its links', ascending (reference)."""
    newest = m.keyframes - 1
    return np.sort(np.append(covisibility.links(m, newest, options)[0], newest)).astype(np.int64)


def local_ba_keyframes(m: Map, options: Options, *, backend: str = "python") -> IntArray:
    """Return the keyframes local BA adjusts: the newest and every one linked to it.

    ORB-SLAM's ``LocalBundleAdjustment`` takes the new keyframe and its covisible keyframes
    (:func:`slam_it.edge.covisibility.links`); :func:`process` adjusts them, holding the map's
    first keyframe and every other that observes their points fixed
    (:func:`slam_it.edge.adjust.bundle_adjust`).

    Parameters
    ----------
    m : Map
        The map, with at least one keyframe.
    options : Options
        ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (L,)
        Their rows, ascending.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the map has no keyframe.
    """
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "local_ba_keyframes",
        reference=lambda: _local_ba_keyframes(m, options),
        arguments=_arguments(m, None, options),
        decode=lambda raw, codes: backends.checked_array(raw, np.int64, 1, "local BA's keyframes"),
    )


def _fuse_points(m: Map, camera: Camera, mode: str, options: Options) -> Map:
    """Fuse the newest keyframe's points with its neighbours' (reference)."""
    newest_id = int(m.kf_id[-1])
    target_ids = _fuse_targets(m, mode, options)
    own = m.pt_id[m.observed_by(np.array([m.keyframes - 1]))]
    for target in target_ids:
        k = int(m.keyframe_rows(np.array([target]))[0])
        for point_id in own:
            m = _fuse_one(m, camera, k, int(point_id), options)
    newest = m.keyframes - 1
    rows = m.keyframe_rows(target_ids)
    theirs = m.pt_id[m.observed_by(rows[rows >= 0])]
    for point_id in theirs:
        m = _fuse_one(m, camera, newest, int(point_id), options)
    assert int(m.kf_id[-1]) == newest_id
    return maps.refresh_points(m, m.observed_by(np.array([newest])))


def fuse_points(
    m: Map, camera: Camera, mode: str, options: Options, *, backend: str = "python"
) -> Map:
    """Fuse the newest keyframe's points with its neighbours' (``SearchInNeighbors``).

    The neighbours are its best ``mapping.fuse_neighbours`` links (``mapping.mono_fuse_neighbours``
    in monocular), then each one's best ``mapping.fuse_second_neighbours``, leaving out the newest
    keyframe and repeats
    (:func:`slam_it.edge.covisibility.links`). The newest keyframe's points are fused into each
    neighbour, then all the neighbours' points into the newest (``Fuse``): each point not already in
    the keyframe, in front, inside the image, in its distance range
    (:func:`slam_it.edge.map.in_distance_range`) and seen within ``tracking.view_cos_limit`` of its
    viewing direction, is compared with the keyframe's keypoints within ``mapping.fuse_window``
    pixels at its predicted level (on that level or the one below) that pass the chi-square test;
    the nearest within ``matching.th_low`` takes it: a free keypoint observes it, one observing
    another point merges the two (the one with the lower observation weight replaced). The newest
    keyframe's points are refreshed at the end.

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The neighbours and thresholds.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map after fusing.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the map has no keyframe.
    """
    check_mode(mode)
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "fuse_points",
        reference=lambda: _fuse_points(m, camera, mode, options),
        arguments=_arguments(m, camera, options, mode=mode),
        decode=maps.decode,
    )


def _redundant(m: Map, k: int, rows: IntArray, threshold: int) -> int:
    """Count a keyframe's points seen by enough other keyframes at its level or finer."""
    keyframe_of = m.keypoint_keyframe()
    point_rows = m.point_rows(m.kp_point[rows])
    weights = maps.observation_weight(m, point_rows)
    redundant = 0
    for row, p, weight in zip(rows, point_rows, weights, strict=True):
        if weight <= threshold:
            continue
        others = np.flatnonzero((m.kp_point == m.pt_id[p]) & (keyframe_of != k))
        if np.count_nonzero(m.kp_level[others] <= m.kp_level[row] + 1) >= threshold:
            redundant += 1
    return redundant


def _cull_keyframes(m: Map, camera: Camera, mode: str, options: Options, held: IntArray) -> Map:
    """Cull the newest keyframe's redundant links (reference)."""
    limit = close_depth(camera, options)
    for kf_id in m.kf_id[covisibility.links(m, m.keyframes - 1, options)[0]]:
        if kf_id == 0:
            continue
        k = int(m.keyframe_rows(np.array([kf_id]))[0])
        rows = _keypoint_indices(m, k)
        rows = rows[m.kp_point[rows] >= 0]
        if mode != "monocular":
            depth = m.kp_depth[rows]
            rows = rows[(depth >= 0.0) & (depth <= limit)]  # NaN: no depth, not counted
        if len(rows) and _redundant(m, k, rows, options.mapping.redundant_observers) > (
            options.mapping.redundant_fraction * len(rows)
        ):
            m = covisibility.cull(m, k, options, held=held)
    return m


def cull_keyframes(  # noqa: PLR0913 -- the map, its camera, the mode, the settings, the held
    m: Map,
    camera: Camera,
    mode: str,
    options: Options,
    *,
    held: ArrayLike = (),
    backend: str = "python",
) -> Map:
    """Cull the newest keyframe's redundant links (``KeyFrameCulling``).

    Each keyframe linked to the newest (:func:`slam_it.edge.covisibility.links`, the most
    shared first), but the map's first (id 0), is culled (:func:`slam_it.edge.covisibility.cull`)
    when more than ``mapping.redundant_fraction`` of its points (its close ones with depth, for
    stereo and RGB-D) are observed with weight above ``mapping.redundant_observers`` and seen by
    at least ``mapping.redundant_observers`` other keyframes at its level, a finer one, or the next
    coarser; ``cull`` spares one with a loop edge, and one of `held`.

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The fraction and count.
    held : array_like of int, optional
        The ids of the keyframes loop detection holds (default: none).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map without the redundant keyframes.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, the map has no keyframe, or `held` is not ids.
    """
    check_mode(mode)
    _check_newest(m)
    kept = covisibility.as_held(held)
    return backends.call(
        backend,
        __name__,
        "cull_keyframes",
        reference=lambda: _cull_keyframes(m, camera, mode, options, kept),
        arguments=_arguments(m, camera, options, mode=mode, held=kept),
        decode=maps.decode,
    )


def _process(  # noqa: PLR0913 -- the stage's state
    m: Map,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    *,
    mode: str,
    options: Options,
    words: Vocabulary,
    held: IntArray,
    interrupts: Interrupts | None = None,
) -> Mapped:
    """Insert a keyframe and run local mapping on it (reference)."""
    nodes = vocabulary.quantize(words, frame.features.descriptors)[1]
    m = maps.insert_keyframe(m, frame, tracked.pose, tracked.points, nodes)
    # Its parent, as ORB-SLAM's first UpdateConnections sets it (after fusion if it shares none).
    m = covisibility.adopt(m, m.keyframes - 1)
    start, first = m.points, m.next_pt_id
    if mode != "monocular":
        m = _create_points_from_depth(m, camera, options)
    with_depth, from_depth = m.points, _made_since(m, first)
    m = _cull_points(m, mode, options)
    culled = m.points
    m = _triangulate_new_points(m, camera, mode, options)
    triangulated = m.points
    # Taken before fusion, which may replace a new point; a depth point culled since stays made.
    made = np.where(from_depth >= 0, from_depth, _made_since(m, first))
    if interrupts is not None:
        interrupts.publish(m)
    fusion = Scheduled.RAN if interrupts is None or not interrupts.waiting() else Scheduled.SKIPPED
    if fusion is Scheduled.RAN:
        m = covisibility.adopt(_fuse_points(m, camera, mode, options), m.keyframes - 1)
    else:
        m = covisibility.adopt(m, m.keyframes - 1)
    fused = m.points
    quiet = interrupts is None or not (interrupts.waiting() or interrupts.stopping())
    adjustment = culling = Scheduled.RAN if quiet else Scheduled.SKIPPED
    solves: tuple[adjust.Solve, ...] = ()
    outliers = 0
    free = fixed = 0
    if quiet and m.keyframes >= options.adjustment.min_keyframes:
        local = _local_ba_keyframes(m, options)
        abort = None if interrupts is None else interrupts.aborted
        adjusted = adjust.bundle_adjust(m, camera, local, m.kf_id == 0, options, abort=abort)
        m, solves, outliers = adjusted.map, adjusted.solves, adjusted.outliers
        free, fixed = adjusted.free, adjusted.fixed
        if adjusted.aborted:
            adjustment = Scheduled.ABORTED
    keyframes = m.keyframes
    if quiet:
        m = _cull_keyframes(m, camera, mode, options, held)
    summary = MappingSummary(
        from_depth=with_depth - start,
        culled_points=with_depth - culled,
        triangulated=triangulated - culled,
        fused=triangulated - fused,
        solves=solves,
        free=free,
        fixed=fixed,
        outliers=outliers,
        weak_points=fused - m.points,
        culled_keyframes=keyframes - m.keyframes,
    )
    return Mapped(map=m, summary=summary, made=made, schedule=Schedule(fusion, adjustment, culling))


def process(  # noqa: PLR0913 -- the stage's state, and the backend
    m: Map,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    *,
    mode: str,
    options: Options,
    words: Vocabulary,
    held: ArrayLike = (),
    interrupts: Interrupts | None = None,
    backend: str = "python",
) -> Mapped:
    """Insert a keyframe and run local mapping on it (the module docstring's six steps).

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    frame : slam_it.edge.frames.Frame
        The new keyframe's frame.
    tracked : slam_it.edge.frames.Tracked
        Its pose and matches.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The settings.
    words : slam_it.foundations.vocabulary.Vocabulary
        The vocabulary the keyframe's features are quantised by (its nodes, step 1).
    held : array_like of int, optional
        The ids of the keyframes loop detection holds, which culling spares (default: none).
    interrupts : Interrupts, optional
        What local mapping's thread is asked between the steps (the module docstring); default:
        none, every step run, as inline.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Mapped
        The map after local mapping, how each step went and was scheduled, and the points made
        for the keyframe's keypoints.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, the matches are not one per keypoint, `held` is not ids, or
        `interrupts` is given to a compiled backend.
    """
    check_mode(mode)
    if interrupts is not None and backend != "python":
        raise InvalidInputError("interrupts: a compiled backend's own threads interrupt it")
    checked = checked_tracked(tracked, frame)
    kept = covisibility.as_held(held)
    return backends.call(
        backend,
        __name__,
        "process",
        reference=lambda: _process(
            m,
            camera,
            frame,
            checked,
            mode=mode,
            options=options,
            words=words,
            held=kept,
            interrupts=interrupts,
        ),
        arguments=_arguments(
            m,
            camera,
            options,
            frame=frame.as_dict(),
            tracked=checked.as_dict(),
            mode=mode,
            vocabulary=vocabulary.fields(words),
            held=kept,
        ),
        decode=_decode_mapped,
    )
