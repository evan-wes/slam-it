"""The map: keyframes, their keypoints and the map points, as one struct of arrays.

A :class:`Map` holds three tables, each a set of arrays row for row:

- **keyframes** (K rows): an id, a time, the pose ``T_cam_world``, and where its keypoints
  start in the keypoint table (``kf_start``, K + 1 offsets);
- **keypoints** (T rows, keyframe after keyframe): what tracking and mapping read of each
  keyframe's features, and the id of the map point each observes (-1 for none);
- **points** (P rows): an id, the position in the world, and what matching a point needs:
  its distinctive descriptor, its mean viewing direction and the distances over which its
  scale stays invariant (ORB-SLAM's ``MapPoint``), and how often it was visible and found.

Points and keyframes are referred to by **id**, never by row: an id is given once, in
increasing order, so the id columns stay sorted and :meth:`Map.point_rows` finds rows by
binary search. Erasing a point therefore only clears the keypoints that observed it; erasing
a keyframe removes its rows. Every operation returns a new map and leaves its input as it was.

**A point's descriptor, normal and distances** follow ORB-SLAM's ``MapPoint``:

- the descriptor is the observation whose median Hamming distance to the others is least
  (``ComputeDistinctiveDescriptors``; ties: the earliest);
- the normal is the mean of the unit directions from each observing keyframe's centre to the
  point, normalised (ORB-SLAM keeps it unnormalised);
- with ``d`` the distance from the reference keyframe's centre and ``l`` the level of the
  keypoint observing it there, the point stays matchable from ``d_min = d_max / s^(L - 1)``
  to ``d_max = d s^l`` (``UpdateNormalAndDepth``), and :func:`predict_level` predicts the
  level it appears at from a distance (``PredictScale``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.edge.frames import Frame
from slamlab.edge.options import Options
from slamlab.foundations import matching, orb
from slamlab.foundations.arrays import as_count, as_mask, as_real, holds_integers
from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.orb import DESCRIPTOR_BYTES

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


@dataclass(frozen=True)
class Map:
    """Keyframes, their keypoints and the map points (see the module docstring).

    Attributes
    ----------
    levels : int
        The pyramid's levels.
    scale_factor : float
        Its scale factor.
    kf_id : ndarray of int64, shape (K,)
        Each keyframe's id, ascending.
    kf_time : ndarray of float64, shape (K,)
        Its time, in seconds.
    kf_pose : ndarray of float64, shape (K, 4, 4)
        Its pose ``T_cam_world``.
    kf_start : ndarray of int64, shape (K + 1,)
        Where each keyframe's keypoints start in the keypoint table; the last is T.
    kp_xy : ndarray of float64, shape (T, 2)
        Each keypoint's pixel.
    kp_level : ndarray of int64, shape (T,)
        Its pyramid level.
    kp_angle : ndarray of int64, shape (T,)
        Its orientation bin.
    kp_descriptor : ndarray of uint8, shape (T, DESCRIPTOR_BYTES)
        Its descriptor.
    kp_bearing : ndarray of float64, shape (T, 3)
        Its unit bearing (NaN where it has none).
    kp_depth : ndarray of float64, shape (T,)
        Its depth in metres (NaN for none).
    kp_right : ndarray of float64, shape (T,)
        Its right-image column (NaN for none).
    kp_point : ndarray of int64, shape (T,)
        The id of the map point it observes, or -1.
    pt_id : ndarray of int64, shape (P,)
        Each point's id, ascending.
    pt_xyz : ndarray of float64, shape (P, 3)
        Its position in the world.
    pt_descriptor : ndarray of uint8, shape (P, DESCRIPTOR_BYTES)
        Its distinctive descriptor.
    pt_normal : ndarray of float64, shape (P, 3)
        Its mean viewing direction, a unit vector (from the cameras to the point).
    pt_min_distance, pt_max_distance : ndarray of float64, shape (P,)
        The distances over which it is matchable.
    pt_visible, pt_found : ndarray of int64, shape (P,)
        How many frames had it in view, and how many matched it.
    pt_first_kf : ndarray of int64, shape (P,)
        The id of the keyframe that created it.
    pt_reference_kf : ndarray of int64, shape (P,)
        The id of its reference keyframe (where its distances are measured from).
    next_kf_id, next_pt_id : int
        The next ids to give.
    """

    levels: int
    scale_factor: float
    kf_id: IntArray
    kf_time: Array
    kf_pose: Array
    kf_start: IntArray
    kp_xy: Array
    kp_level: IntArray
    kp_angle: IntArray
    kp_descriptor: NDArray[np.uint8]
    kp_bearing: Array
    kp_depth: Array
    kp_right: Array
    kp_point: IntArray
    pt_id: IntArray
    pt_xyz: Array
    pt_descriptor: NDArray[np.uint8]
    pt_normal: Array
    pt_min_distance: Array
    pt_max_distance: Array
    pt_visible: IntArray
    pt_found: IntArray
    pt_first_kf: IntArray
    pt_reference_kf: IntArray
    next_kf_id: int
    next_pt_id: int

    def __post_init__(self) -> None:
        """Check the tables' shapes, types and links (not copied: maps are built by this module).

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If an array has the wrong type or length, the keypoint offsets do not run from 0
            to the keypoint count, an id column is not strictly ascending or reaches its next
            id, or a keypoint observes a point that is not in the map.
        """
        _check(self)

    @property
    def keyframes(self) -> int:
        """The number of keyframes."""
        return len(self.kf_id)

    @property
    def points(self) -> int:
        """The number of map points."""
        return len(self.pt_id)

    def keypoint_rows(self, k: int) -> slice:
        """Return the rows of keyframe `k`'s keypoints.

        Parameters
        ----------
        k : int
            The keyframe's row.

        Returns
        -------
        slice
            Its rows in the keypoint table.
        """
        return slice(int(self.kf_start[k]), int(self.kf_start[k + 1]))

    def keypoint_keyframe(self) -> IntArray:
        """Each keypoint's keyframe row.

        Returns
        -------
        ndarray of int64, shape (T,)
            The rows.
        """
        return np.repeat(np.arange(self.keyframes), np.diff(self.kf_start)).astype(np.int64)

    def point_rows(self, ids: IntArray) -> IntArray:
        """Find points by id.

        Parameters
        ----------
        ids : ndarray of int64
            Point ids (-1 allowed).

        Returns
        -------
        ndarray of int64
            Each one's row, -1 where it is -1 or no longer in the map.
        """
        return _rows(self.pt_id, ids)

    def keyframe_rows(self, ids: IntArray) -> IntArray:
        """Find keyframes by id.

        Parameters
        ----------
        ids : ndarray of int64
            Keyframe ids.

        Returns
        -------
        ndarray of int64
            Each one's row, -1 where it is no longer in the map.
        """
        return _rows(self.kf_id, ids)

    def centres(self) -> Array:
        """Each keyframe's camera centre in the world.

        Returns
        -------
        ndarray of float64, shape (K, 3)
            ``-Rᵀ t`` for each pose.
        """
        rotation, translation = self.kf_pose[:, :3, :3], self.kf_pose[:, :3, 3]
        return np.asarray(-np.einsum("kji,kj->ki", rotation, translation))

    def window(self, size: int) -> IntArray:
        """Return the sliding window: the rows of the last `size` keyframes.

        Parameters
        ----------
        size : int
            The window's size.

        Returns
        -------
        ndarray of int64
            Their rows, oldest first.
        """
        return np.arange(max(0, self.keyframes - size), self.keyframes, dtype=np.int64)

    def observed_by(self, keyframes: IntArray) -> IntArray:
        """Return the points the given keyframes observe.

        Parameters
        ----------
        keyframes : ndarray of int64
            Keyframe rows.

        Returns
        -------
        ndarray of int64
            The points' rows, ascending, each once.
        """
        rows = [self.kp_point[self.keypoint_rows(int(k))] for k in keyframes]
        ids = np.concatenate(rows) if rows else np.zeros(0, dtype=np.int64)
        found = self.point_rows(np.unique(ids[ids >= 0]))
        return np.sort(found[found >= 0])

    def as_dict(self) -> dict[str, Any]:
        """Return every field, as the map crosses the bindings.

        Returns
        -------
        dict
            Field name to array (or scalar).
        """
        return {
            item.name: (np.ascontiguousarray(value) if isinstance(value, np.ndarray) else value)
            for item in fields(self)
            for value in [getattr(self, item.name)]
        }


def _rows(ids: IntArray, wanted: IntArray) -> IntArray:
    """Find ids in an ascending id column: each one's row, or -1."""
    wanted = np.asarray(wanted, dtype=np.int64)
    at = np.searchsorted(ids, wanted)
    inside = at < len(ids)
    found = np.zeros(wanted.shape, dtype=bool)
    found[inside] = ids[at[inside]] == wanted[inside]
    return np.where(found & (wanted >= 0), at, -1).astype(np.int64)


def empty(levels: int, scale_factor: float) -> Map:
    """Return a map with no keyframes and no points.

    Parameters
    ----------
    levels : int
        The pyramid's levels.
    scale_factor : float
        Its scale factor.

    Returns
    -------
    Map
        The empty map.
    """
    no_int = np.zeros(0, dtype=np.int64)
    no_real = np.zeros(0)
    return Map(
        levels=int(levels),
        scale_factor=float(scale_factor),
        kf_id=no_int,
        kf_time=no_real,
        kf_pose=np.zeros((0, 4, 4)),
        kf_start=np.zeros(1, dtype=np.int64),
        kp_xy=np.zeros((0, 2)),
        kp_level=no_int,
        kp_angle=no_int,
        kp_descriptor=np.zeros((0, DESCRIPTOR_BYTES), dtype=np.uint8),
        kp_bearing=np.zeros((0, 3)),
        kp_depth=no_real,
        kp_right=no_real,
        kp_point=no_int,
        pt_id=no_int,
        pt_xyz=np.zeros((0, 3)),
        pt_descriptor=np.zeros((0, DESCRIPTOR_BYTES), dtype=np.uint8),
        pt_normal=np.zeros((0, 3)),
        pt_min_distance=no_real,
        pt_max_distance=no_real,
        pt_visible=no_int,
        pt_found=no_int,
        pt_first_kf=no_int,
        pt_reference_kf=no_int,
        next_kf_id=0,
        next_pt_id=0,
    )


def _predict_level(m: Map, rows: IntArray, distance: Array) -> IntArray:
    """Predict points' levels from their distances (reference)."""
    ratio = m.pt_max_distance[rows] / distance
    level = np.ceil(np.log(ratio) / math.log(m.scale_factor))
    return np.clip(level, 0, m.levels - 1).astype(np.int64)


def _insert_keyframe(m: Map, frame: Frame, pose: Array, points: IntArray) -> Map:
    """Add a keyframe (reference)."""
    keypoints = frame.features.keypoints
    total = int(m.kf_start[-1])
    grown = replace(
        m,
        kf_id=np.append(m.kf_id, m.next_kf_id),
        kf_time=np.append(m.kf_time, frame.timestamp),
        kf_pose=np.concatenate([m.kf_pose, np.asarray(pose, dtype=np.float64)[None]]),
        kf_start=np.append(m.kf_start, total + frame.size),
        kp_xy=np.concatenate([m.kp_xy, keypoints.xy]),
        kp_level=np.concatenate([m.kp_level, keypoints.level]),
        kp_angle=np.concatenate([m.kp_angle, keypoints.angle]),
        kp_descriptor=np.concatenate([m.kp_descriptor, frame.features.descriptors]),
        kp_bearing=np.concatenate([m.kp_bearing, frame.bearings]),
        kp_depth=np.concatenate([m.kp_depth, frame.depth]),
        kp_right=np.concatenate([m.kp_right, frame.right]),
        kp_point=np.concatenate([m.kp_point, np.asarray(points, dtype=np.int64)]),
        next_kf_id=m.next_kf_id + 1,
    )
    observed = grown.point_rows(np.unique(points[points >= 0]))
    return _refresh_points(grown, observed[observed >= 0])


def _add_points(m: Map, xyz: Array, observers: IntArray) -> Map:
    """Create map points (reference)."""
    q = len(xyz)
    if q == 0:
        return m
    ids = np.arange(m.next_pt_id, m.next_pt_id + q, dtype=np.int64)
    kp_point = m.kp_point.copy()
    for i in range(q):
        rows = observers[i][observers[i] >= 0]
        kp_point[rows] = ids[i]
    keyframe_of = m.keypoint_keyframe()
    last = np.array([observers[i][observers[i] >= 0].max() for i in range(q)])
    creator = m.kf_id[keyframe_of[last]]
    grown = replace(
        m,
        kp_point=kp_point,
        pt_id=np.concatenate([m.pt_id, ids]),
        pt_xyz=np.concatenate([m.pt_xyz, np.asarray(xyz, dtype=np.float64)]),
        pt_descriptor=np.concatenate(
            [m.pt_descriptor, np.zeros((q, DESCRIPTOR_BYTES), dtype=np.uint8)]
        ),
        pt_normal=np.concatenate([m.pt_normal, np.zeros((q, 3))]),
        pt_min_distance=np.concatenate([m.pt_min_distance, np.zeros(q)]),
        pt_max_distance=np.concatenate([m.pt_max_distance, np.zeros(q)]),
        pt_visible=np.concatenate([m.pt_visible, np.ones(q, dtype=np.int64)]),
        pt_found=np.concatenate([m.pt_found, np.ones(q, dtype=np.int64)]),
        pt_first_kf=np.concatenate([m.pt_first_kf, creator]),
        pt_reference_kf=np.concatenate([m.pt_reference_kf, creator]),
        next_pt_id=m.next_pt_id + q,
    )
    return _refresh_points(grown, np.arange(m.points, m.points + q, dtype=np.int64))


def _observations(m: Map) -> dict[int, IntArray]:
    """Each point id's observing keypoint rows, ascending."""
    rows = np.flatnonzero(m.kp_point >= 0)
    ids = m.kp_point[rows]
    order = np.argsort(ids, kind="stable")
    rows, ids = rows[order], ids[order]
    starts = np.flatnonzero(np.r_[True, ids[1:] != ids[:-1]]) if len(ids) else np.zeros(0, int)
    ends = np.r_[starts[1:], len(ids)]
    return {int(ids[s]): rows[s:e] for s, e in zip(starts, ends, strict=True)}


def _distinctive_descriptor(descriptors: NDArray[np.uint8]) -> int:
    """Pick the most central descriptor (reference)."""
    distances = np.sort(matching.distances(descriptors, descriptors), axis=1)
    median = distances[:, (len(descriptors) - 1) // 2]
    return int(np.argmin(median))


def _refresh_points(m: Map, rows: IntArray) -> Map:
    """Recompute points' descriptors, normals and distances (reference)."""
    if len(rows) == 0:
        return m
    observations = _observations(m)
    keyframe_of = m.keypoint_keyframe()
    centres = m.centres()
    descriptor = m.pt_descriptor.copy()
    normal = m.pt_normal.copy()
    near, far = m.pt_min_distance.copy(), m.pt_max_distance.copy()
    reference = m.pt_reference_kf.copy()
    for p in rows:
        seen = observations[int(m.pt_id[p])]
        descriptor[p] = m.kp_descriptor[seen[_distinctive_descriptor(m.kp_descriptor[seen])]]
        directions = m.pt_xyz[p] - centres[keyframe_of[seen]]
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        mean = directions.mean(axis=0)
        normal[p] = mean / np.linalg.norm(mean)
        observers = m.kf_id[keyframe_of[seen]]
        if reference[p] not in observers:
            reference[p] = observers.min()
        at = seen[observers == reference[p]][0]
        distance = float(np.linalg.norm(m.pt_xyz[p] - centres[keyframe_of[at]]))
        far[p] = distance * m.scale_factor ** int(m.kp_level[at])
        near[p] = far[p] / m.scale_factor ** (m.levels - 1)
    return replace(
        m,
        pt_descriptor=descriptor,
        pt_normal=normal,
        pt_min_distance=near,
        pt_max_distance=far,
        pt_reference_kf=reference,
    )


def _erase_points(m: Map, erase: Mask) -> Map:
    """Remove points (reference)."""
    if not np.any(erase):
        return m
    keep = ~erase
    gone = np.isin(m.kp_point, m.pt_id[erase])
    return replace(
        m,
        kp_point=np.where(gone, -1, m.kp_point),
        **{
            item.name: getattr(m, item.name)[keep]
            for item in fields(m)
            if item.name.startswith("pt_")
        },
    )


def _observation_weight(m: Map, rows: IntArray) -> IntArray:
    """Weigh points' observations (reference)."""
    observing = m.kp_point >= 0
    weight = np.where(np.isnan(m.kp_right), 1, 2)[observing]
    counts = dict(
        zip(*np.unique(np.repeat(m.kp_point[observing], weight), return_counts=True), strict=True)
    )
    return np.array([counts.get(int(m.pt_id[p]), 0) for p in rows], dtype=np.int64)


def _forget_observations(m: Map, rows: IntArray) -> Map:
    """Remove keypoints' observations (reference)."""
    touched = np.unique(m.kp_point[rows][m.kp_point[rows] >= 0])
    if len(touched) == 0:
        return m
    kp_point = m.kp_point.copy()
    kp_point[rows] = -1
    cleared = replace(m, kp_point=kp_point)
    points = cleared.point_rows(touched)
    points = points[points >= 0]
    weak = np.zeros(cleared.points, dtype=bool)
    weak[points] = _observation_weight(cleared, points) <= 2
    remaining = points[~weak[points]]
    return _erase_points(_refresh_points(cleared, remaining), weak)


def _erase_keyframe(m: Map, k: int) -> Map:
    """Remove a keyframe (reference)."""
    rows = np.arange(m.kf_start[k], m.kf_start[k + 1])
    forgotten = _forget_observations(m, rows)
    keep_rows = np.ones(len(forgotten.kp_point), dtype=bool)
    keep_rows[rows] = False
    counts = np.diff(forgotten.kf_start)
    counts = np.delete(counts, k)
    keep_kf = np.ones(forgotten.keyframes, dtype=bool)
    keep_kf[k] = False
    return replace(
        forgotten,
        kf_id=forgotten.kf_id[keep_kf],
        kf_time=forgotten.kf_time[keep_kf],
        kf_pose=forgotten.kf_pose[keep_kf],
        kf_start=np.concatenate([[0], np.cumsum(counts)]).astype(np.int64),
        **{
            item.name: getattr(forgotten, item.name)[keep_rows]
            for item in fields(forgotten)
            if item.name.startswith("kp_")
        },
    )


def _count(m: Map, ids: IntArray, visible: IntArray, found: IntArray) -> Map:
    """Add to points' visible and found counts (reference)."""
    rows = m.point_rows(ids)
    ok = rows >= 0
    seen, matched = m.pt_visible.copy(), m.pt_found.copy()
    np.add.at(seen, rows[ok], visible[ok])
    np.add.at(matched, rows[ok], found[ok])
    return replace(m, pt_visible=seen, pt_found=matched)


# --- checks and dispatch ------------------------------------------------------------------

_SHAPES: dict[str, tuple[str, type[np.generic], tuple[int, ...]]] = {
    "kf_id": ("K", np.int64, ()),
    "kf_time": ("K", np.float64, ()),
    "kf_pose": ("K", np.float64, (4, 4)),
    "kp_xy": ("T", np.float64, (2,)),
    "kp_level": ("T", np.int64, ()),
    "kp_angle": ("T", np.int64, ()),
    "kp_descriptor": ("T", np.uint8, (DESCRIPTOR_BYTES,)),
    "kp_bearing": ("T", np.float64, (3,)),
    "kp_depth": ("T", np.float64, ()),
    "kp_right": ("T", np.float64, ()),
    "kp_point": ("T", np.int64, ()),
    "pt_id": ("P", np.int64, ()),
    "pt_xyz": ("P", np.float64, (3,)),
    "pt_descriptor": ("P", np.uint8, (DESCRIPTOR_BYTES,)),
    "pt_normal": ("P", np.float64, (3,)),
    "pt_min_distance": ("P", np.float64, ()),
    "pt_max_distance": ("P", np.float64, ()),
    "pt_visible": ("P", np.int64, ()),
    "pt_found": ("P", np.int64, ()),
    "pt_first_kf": ("P", np.int64, ()),
    "pt_reference_kf": ("P", np.int64, ()),
}
"""Each table array's row count (keyframes, keypoints or points), dtype and item shape."""


def _check(m: Map) -> None:
    """Check a map's structure (see :meth:`Map.__post_init__`)."""
    as_count(m.levels, "map.levels", 1, orb.MAX_LEVELS)
    if not (np.isfinite(m.scale_factor) and m.scale_factor > 1.0):
        raise InvalidInputError("map.scale_factor: must be finite and above 1")
    starts = np.asarray(m.kf_start)
    if starts.dtype != np.int64 or starts.ndim != 1 or len(starts) < 1 or starts[0] != 0:
        raise InvalidInputError("map.kf_start: expected int64 offsets starting at 0")
    if np.any(np.diff(starts) < 0):
        raise InvalidInputError("map.kf_start: offsets must not decrease")
    counts = {"K": len(starts) - 1, "T": int(starts[-1]), "P": len(np.asarray(m.pt_id))}
    for name, (rows, dtype, item) in _SHAPES.items():
        value = np.asarray(getattr(m, name))
        if value.dtype != dtype or value.shape != (counts[rows], *item):
            raise InvalidInputError(
                f"map.{name}: expected {np.dtype(dtype).name} {(counts[rows], *item)}, "
                f"got {value.dtype} {value.shape}"
            )
    for name, last in (("kf_id", m.next_kf_id), ("pt_id", m.next_pt_id)):
        ids = np.asarray(getattr(m, name))
        if np.any(np.diff(ids) <= 0) or (len(ids) and (ids[0] < 0 or ids[-1] >= last)):
            raise InvalidInputError(f"map.{name}: ids must be ascending, from 0, below the next id")
    observed = np.asarray(m.kp_point)
    observed = observed[observed >= 0]
    if np.any(np.asarray(m.kp_point) < -1) or np.any(_rows(m.pt_id, observed) < 0):
        raise InvalidInputError("map.kp_point: every observed point must be in the map")


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.map`` submodule provides.

    Each takes the map as a dict of :class:`Map`'s fields and returns the new one the same
    way; a frame crosses as :func:`frame_dict` builds it.
    """

    def predict_level(
        self, m: dict[str, Any], rows: NDArray[np.int64], distance: NDArray[np.float64]
    ) -> NDArray[np.int64]:
        """Predict points' levels from their distances."""
        ...

    def insert_keyframe(
        self,
        m: dict[str, Any],
        frame: dict[str, Any],
        pose: NDArray[np.float64],
        points: NDArray[np.int64],
    ) -> dict[str, Any]:
        """Add a keyframe."""
        ...

    def add_points(
        self, m: dict[str, Any], xyz: NDArray[np.float64], observers: NDArray[np.int64]
    ) -> dict[str, Any]:
        """Create map points."""
        ...

    def distinctive_descriptor(self, descriptors: NDArray[np.uint8]) -> int:
        """Pick the most central descriptor."""
        ...

    def refresh_points(self, m: dict[str, Any], rows: NDArray[np.int64]) -> dict[str, Any]:
        """Recompute points' descriptors, normals and distances."""
        ...

    def erase_points(self, m: dict[str, Any], erase: NDArray[np.bool_]) -> dict[str, Any]:
        """Remove points."""
        ...

    def observation_weight(self, m: dict[str, Any], rows: NDArray[np.int64]) -> NDArray[np.int64]:
        """Weigh points' observations."""
        ...

    def forget_observations(self, m: dict[str, Any], rows: NDArray[np.int64]) -> dict[str, Any]:
        """Remove keypoints' observations."""
        ...

    def erase_keyframe(self, m: dict[str, Any], k: int) -> dict[str, Any]:
        """Remove a keyframe."""
        ...

    def count(
        self,
        m: dict[str, Any],
        ids: NDArray[np.int64],
        visible: NDArray[np.int64],
        found: NDArray[np.int64],
    ) -> dict[str, Any]:
        """Add to points' visible and found counts."""
        ...


def frame_dict(frame: Frame) -> dict[str, Any]:
    """Return a frame as it crosses the bindings: its features' fields and its own arrays.

    Parameters
    ----------
    frame : slamlab.edge.frames.Frame
        The frame.

    Returns
    -------
    dict
        ``timestamp``, the keypoints' ``xy``, ``level``, ``level_xy``, ``response`` and
        ``angle``, ``descriptors``, ``scale_factor``, ``levels``, and ``bearings``,
        ``depth`` and ``right``.
    """
    keypoints = frame.features.keypoints
    return {
        "timestamp": float(frame.timestamp),
        "xy": keypoints.xy,
        "level": keypoints.level,
        "level_xy": keypoints.level_xy,
        "response": keypoints.response,
        "angle": keypoints.angle,
        "descriptors": frame.features.descriptors,
        "scale_factor": float(frame.features.scale_factor),
        "levels": int(frame.features.levels),
        "bearings": frame.bearings,
        "depth": frame.depth,
        "right": frame.right,
    }


def decode(raw: dict[str, Any], codes: backends.Codes) -> Map:
    """Rebuild a map from a compiled backend's dict, its arrays checked (never cast).

    Parameters
    ----------
    raw : dict
        The map's fields, as :meth:`Map.as_dict` gives them.
    codes : dict
        The backend module's code tables (the map crosses none).

    Returns
    -------
    Map
        The map, checked.
    """
    del codes  # the map crosses no codes
    arrays = {
        name: backends.checked_array(raw[name], dtype, 1 + len(item), f"map.{name}")
        for name, (_, dtype, item) in _SHAPES.items()
    }
    arrays["kf_start"] = backends.checked_array(raw["kf_start"], np.int64, 1, "map.kf_start")
    return Map(
        levels=int(raw["levels"]),
        scale_factor=float(raw["scale_factor"]),
        next_kf_id=int(raw["next_kf_id"]),
        next_pt_id=int(raw["next_pt_id"]),
        **arrays,
    )


def _decode_int64(raw: Any, codes: backends.Codes) -> NDArray[np.int64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's int64 vector, checked (never cast)."""
    del codes
    out: NDArray[np.int64] = backends.checked_array(raw, np.int64, 1, "map result")
    return out


def as_keyframe_row(m: Map, k: object, name: str = "k") -> int:
    """Check a keyframe row: an integer in ``[0, K)`` (a boolean refused).

    Parameters
    ----------
    m : Map
        The map.
    k : object
        The row.
    name : str, optional
        The argument's name, for messages.

    Returns
    -------
    int
        The row.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If it is not an integer in range.
    """
    return as_count(k, name, 0, m.keyframes - 1)


def as_point_rows(m: Map, rows: ArrayLike, name: str = "rows") -> IntArray:
    """Check point rows: a vector of integers in ``[0, P)`` (booleans refused).

    Parameters
    ----------
    m : Map
        The map.
    rows : array_like of int
        The rows.
    name : str, optional
        The argument's name, for messages.

    Returns
    -------
    ndarray of int64
        The rows, contiguous.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If they are not integers, or one is out of range.
    """
    return _as_rows(rows, m.points, name)


def _as_rows(rows: ArrayLike, bound: int, name: str) -> IntArray:
    """Check rows: a vector of integers in ``[0, bound)`` (booleans refused)."""
    arr = np.asarray(rows)
    if arr.size == 0:
        return np.zeros(0, dtype=np.int64)
    if arr.ndim != 1 or not holds_integers(arr):
        raise InvalidInputError(
            f"{name}: expected a vector of integers, got {arr.dtype} {arr.shape}"
        )
    if arr.min() < 0 or arr.max() >= bound:
        raise InvalidInputError(f"{name}: rows must lie in [0, {bound})")
    return np.ascontiguousarray(arr, dtype=np.int64)


def _as_ids(ids: ArrayLike, name: str) -> IntArray:
    """Check ids: a vector of integers, -1 or more."""
    arr = np.asarray(ids)
    if arr.size == 0:
        return np.zeros(0, dtype=np.int64)
    if arr.ndim != 1 or not holds_integers(arr) or arr.min() < -1:
        raise InvalidInputError(f"{name}: expected a vector of ids (-1 or more)")
    return np.ascontiguousarray(arr, dtype=np.int64)


def in_distance_range(
    m: Map, rows: NDArray[np.int64], distance: NDArray[np.float64], options: Options
) -> NDArray[np.bool_]:
    """Return which points lie within the distances they are matchable from.

    From ``min_distance_margin`` of a point's minimum distance to ``max_distance_margin`` of
    its maximum (``isInFrustum``'s and ``Fuse``'s 0.8 and 1.2). Unchecked: `rows` are the
    map's point rows and `distance` holds one distance per row.

    Parameters
    ----------
    m : Map
        The map.
    rows : ndarray of int64, shape (N,)
        The points' rows.
    distance : ndarray of float64, shape (N,)
        Each point's distance from the camera centre.
    options : slamlab.edge.options.Options
        The margins.

    Returns
    -------
    ndarray of bool, shape (N,)
        Which are in range.
    """
    near = distance >= options.min_distance_margin * m.pt_min_distance[rows]
    return np.asarray(near & (distance <= options.max_distance_margin * m.pt_max_distance[rows]))


def predict_level(
    m: Map, rows: ArrayLike, distance: ArrayLike, *, backend: str = "python"
) -> IntArray:
    """Predict the pyramid level points appear at from a distance (``PredictScale``).

    Parameters
    ----------
    m : Map
        The map.
    rows : array_like of int
        The points' rows.
    distance : array_like of float
        Their distances from the camera, finite and positive.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64
        ``ceil(log(d_max / d) / log s)``, clamped to the pyramid.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a row is out of range, or a distance is not finite and positive, or there is not
        one per row.
    """
    chosen = as_point_rows(m, rows)
    distances = as_real(distance, "distance")
    if distances.shape != chosen.shape or np.any(distances <= 0.0):
        raise InvalidInputError("distance: expected one positive distance per row")
    return backends.call(
        backend,
        __name__,
        "predict_level",
        reference=lambda: _predict_level(m, chosen, distances),
        arguments={"m": m.as_dict(), "rows": chosen, "distance": distances},
        decode=_decode_int64,
    )


def insert_keyframe(
    m: Map, frame: Frame, pose: ArrayLike, points: ArrayLike, *, backend: str = "python"
) -> Map:
    """Add a keyframe: the frame, its pose, and the points its keypoints observe.

    The observed points' descriptors, normals and distances are refreshed.

    Parameters
    ----------
    m : Map
        The map.
    frame : slamlab.edge.frames.Frame
        The frame (its pyramid must be the map's).
    pose : array_like, shape (4, 4)
        Its pose ``T_cam_world``.
    points : array_like of int, shape (N,)
        The id of the point each keypoint observes (each in the map, each at most once), or
        -1.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with the keyframe last.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the frame's pyramid differs from the map's, the pose is not finite, or the point
        ids are not one per keypoint, in the map, and distinct.
    """
    if frame.features.levels != m.levels or frame.features.scale_factor != m.scale_factor:
        raise InvalidInputError("frame: its pyramid must be the map's")
    matrix = as_real(pose, "pose")
    if matrix.shape != (4, 4):
        raise InvalidInputError(f"pose: expected shape (4, 4), got {matrix.shape}")
    ids = _as_ids(points, "points")
    if ids.shape != (frame.size,):
        raise InvalidInputError(f"points: expected {frame.size} ids, got {ids.shape}")
    observed = ids[ids >= 0]
    if len(np.unique(observed)) != len(observed) or np.any(m.point_rows(observed) < 0):
        raise InvalidInputError("points: each must be in the map, and observed once")
    return backends.call(
        backend,
        __name__,
        "insert_keyframe",
        reference=lambda: _insert_keyframe(m, frame, matrix, ids),
        arguments={"m": m.as_dict(), "frame": frame_dict(frame), "pose": matrix, "points": ids},
        decode=decode,
    )


def add_points(m: Map, xyz: ArrayLike, observers: ArrayLike, *, backend: str = "python") -> Map:
    """Create map points, each observed by keyframe keypoints.

    A new point's reference and creating keyframe is its last observer's; it counts as
    visible and found once (as ORB-SLAM's ``MapPoint`` starts).

    Parameters
    ----------
    m : Map
        The map.
    xyz : array_like, shape (Q, 3)
        The points in the world, finite.
    observers : array_like of int, shape (Q, V)
        Each point's observing keypoint rows, -1 padded, at least one: every row free
        (observing no point), used once, and each point's from different keyframes.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with the points last, their descriptors, normals and distances set.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the points are not finite, or the observers break a rule above.
    """
    points = as_real(xyz, "xyz").reshape(-1, 3) if np.size(xyz) else np.zeros((0, 3))
    rows = np.asarray(observers)
    if rows.size == 0 and len(points) == 0:
        rows = np.zeros((0, 1), dtype=np.int64)
    if rows.ndim != 2 or rows.shape[0] != len(points) or not holds_integers(rows):
        raise InvalidInputError("observers: expected one row of keypoint rows per point")
    rows = np.ascontiguousarray(rows, dtype=np.int64)
    used = rows[rows >= 0]
    keyframe_of = m.keypoint_keyframe()
    if (
        np.any(rows < -1)
        or np.any(used >= len(m.kp_point))
        or np.any(np.count_nonzero(rows >= 0, axis=1) == 0)
        or len(np.unique(used)) != len(used)
        or np.any(m.kp_point[used] >= 0)
        or any(len(np.unique(keyframe_of[r[r >= 0]])) != np.count_nonzero(r >= 0) for r in rows)
    ):
        raise InvalidInputError(
            "observers: each point needs free keypoint rows, used once, from different keyframes"
        )
    return backends.call(
        backend,
        __name__,
        "add_points",
        reference=lambda: _add_points(m, points, rows),
        arguments={"m": m.as_dict(), "xyz": points, "observers": rows},
        decode=decode,
    )


def distinctive_descriptor(descriptors: ArrayLike, *, backend: str = "python") -> int:
    """Pick the descriptor with the least median distance to the others.

    Parameters
    ----------
    descriptors : array_like of uint8, shape (n, DESCRIPTOR_BYTES)
        A point's observations' descriptors, n >= 1.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    int
        The chosen row (ties: the earliest). The median of n sorted distances is the
        ``(n - 1) // 2``-th, ORB-SLAM's ``vDists[0.5 * (N - 1)]``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the descriptors are not an (n, DESCRIPTOR_BYTES) uint8 array with n >= 1.
    """
    rows = np.asarray(descriptors)
    if (
        rows.dtype != np.uint8
        or rows.ndim != 2
        or rows.shape[1] != DESCRIPTOR_BYTES
        or len(rows) < 1
    ):
        raise InvalidInputError(
            f"descriptors: expected an (n, {DESCRIPTOR_BYTES}) uint8 array, n >= 1"
        )
    chosen = np.ascontiguousarray(rows)
    return backends.call(
        backend,
        __name__,
        "distinctive_descriptor",
        reference=lambda: _distinctive_descriptor(chosen),
        arguments={"descriptors": chosen},
        decode=lambda raw, codes: int(raw),
    )


def refresh_points(m: Map, rows: ArrayLike, *, backend: str = "python") -> Map:
    """Recompute points' descriptors, normals and distances from their observations.

    A point whose reference keyframe no longer observes it takes its earliest remaining
    observer (the lowest keyframe id) as the reference (ORB-SLAM takes another observer too).

    Parameters
    ----------
    m : Map
        The map.
    rows : array_like of int
        The points' rows; each must have at least one observation.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with those points refreshed.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a row is out of range or its point has no observation.
    """
    chosen = as_point_rows(m, rows)
    if np.any(~np.isin(m.pt_id[chosen], m.kp_point)):
        raise InvalidInputError("rows: every point must have an observation")
    return backends.call(
        backend,
        __name__,
        "refresh_points",
        reference=lambda: _refresh_points(m, chosen),
        arguments={"m": m.as_dict(), "rows": chosen},
        decode=decode,
    )


def erase_points(m: Map, erase: ArrayLike, *, backend: str = "python") -> Map:
    """Remove points; the keypoints that observed them observe nothing.

    Parameters
    ----------
    m : Map
        The map.
    erase : array_like of bool, shape (P,)
        Which points to remove.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map without them.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `erase` is not one boolean per point.
    """
    mask = as_mask(erase, (m.points,), "erase")
    return backends.call(
        backend,
        __name__,
        "erase_points",
        reference=lambda: _erase_points(m, mask),
        arguments={"m": m.as_dict(), "erase": mask},
        decode=decode,
    )


def observation_weight(m: Map, rows: ArrayLike, *, backend: str = "python") -> IntArray:
    """Each point's observation weight: 1 per keypoint, 2 for one with depth (ORB-SLAM's).

    Parameters
    ----------
    m : Map
        The map.
    rows : array_like of int
        The points' rows.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64
        The weights.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a row is out of range.
    """
    chosen = as_point_rows(m, rows)
    return backends.call(
        backend,
        __name__,
        "observation_weight",
        reference=lambda: _observation_weight(m, chosen),
        arguments={"m": m.as_dict(), "rows": chosen},
        decode=_decode_int64,
    )


def forget_observations(m: Map, rows: ArrayLike, *, backend: str = "python") -> Map:
    """Remove keypoints' observations; points left observed with weight 2 or less go too.

    As ORB-SLAM's ``EraseObservation``: a point that only one keyframe sees (once, or twice
    with depth) is erased. The others are refreshed.

    Parameters
    ----------
    m : Map
        The map.
    rows : array_like of int
        Keypoint rows whose observations to remove.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a row is out of range.
    """
    chosen = _as_rows(rows, len(m.kp_point), "rows")
    return backends.call(
        backend,
        __name__,
        "forget_observations",
        reference=lambda: _forget_observations(m, chosen),
        arguments={"m": m.as_dict(), "rows": chosen},
        decode=decode,
    )


def erase_keyframe(m: Map, k: int, *, backend: str = "python") -> Map:
    """Remove a keyframe and its keypoints; its points lose an observation each.

    Parameters
    ----------
    m : Map
        The map.
    k : int
        The keyframe's row.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map without it.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `k` is not a keyframe's row.
    """
    if m.keyframes == 0:
        raise InvalidInputError("k: the map has no keyframes")
    row = as_count(k, "k", 0, m.keyframes - 1)
    return backends.call(
        backend,
        __name__,
        "erase_keyframe",
        reference=lambda: _erase_keyframe(m, row),
        arguments={"m": m.as_dict(), "k": row},
        decode=decode,
    )


def count(
    m: Map,
    ids: ArrayLike,
    visible: ArrayLike | None = None,
    found: ArrayLike | None = None,
    *,
    backend: str = "python",
) -> Map:
    """Add to points' visible and found counts.

    Parameters
    ----------
    m : Map
        The map.
    ids : array_like of int
        Point ids (those no longer in the map are skipped; repeats add again).
    visible, found : array_like of int, optional
        Each id's non-negative increments (default: none).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with the counts added.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the ids are malformed, or an increment is negative or not one per id.
    """
    chosen = _as_ids(ids, "ids")
    increments = []
    for name, value in (("visible", visible), ("found", found)):
        if value is None:
            increments.append(np.zeros(len(chosen), dtype=np.int64))
            continue
        arr = _as_ids(value, name)
        if arr.shape != chosen.shape or np.any(arr < 0):
            raise InvalidInputError(f"{name}: expected one non-negative increment per id")
        increments.append(arr)
    seen, matched = increments
    return backends.call(
        backend,
        __name__,
        "count",
        reference=lambda: _count(m, chosen, seen, matched),
        arguments={"m": m.as_dict(), "ids": chosen, "visible": seen, "found": matched},
        decode=decode,
    )
