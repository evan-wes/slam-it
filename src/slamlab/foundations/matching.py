"""Matching binary descriptors: Hamming distances, and ORB-SLAM's filtered nearest neighbours.

A descriptor is a row of :data:`slamlab.foundations.orb.DESCRIPTOR_BYTES` bytes; the
distance between two is the number of bits in which they differ (their XOR's population
count).

:func:`match` pairs each descriptor of the first set with its nearest in the second, then
filters as ORB-SLAM's matcher does (Mur-Artal et al., T-RO 2015):

- **Distance cap:** the nearest must be within ``max_distance`` (ORB-SLAM's ``TH_LOW``, 50).
- **Ratio test** (Lowe, IJCV 2004): the nearest must be clearly nearer than the second
  nearest, ``best < ratio * second`` (when the second set has a single descriptor there is no
  second nearest, and the test passes).
- **One match per second-set descriptor:** where several first-set descriptors chose the
  same one, the nearest keeps it (ties: the lowest first-set index).
- **Rotation consistency** (optional): a true match's orientation difference is the same
  across the image for a rigid motion, so the differences are binned in a 30-bin histogram
  and only matches in its three highest bins are kept; the second and third are dropped when
  they hold under a tenth of the highest (ORB-SLAM's ``ComputeThreeMaxima``). With ORB's
  orientation bins of 12 degrees, a match's histogram bin is exactly
  ``(angle1 - angle2) mod 30``. (ORB-SLAM's matcher bins at 30 degrees, ``round(rot / 30)``
  over its float angles, 2.5 times wider; on TUM pairs the recall of true matches is about the
  same either way.)

Nearest neighbours tie to the lowest second-set index. Every quantity is an integer except
the ratio test's one float64 product, so the three languages agree exactly.

**Guided matching** (:func:`search_by_projection`, ORB-SLAM's ``SearchByProjection``) is
what a tracker does once it can predict where its map points appear: each map point's
descriptor is compared only with the frame's keypoints near its projection, so far fewer
pairs are tried and a repeated texture elsewhere in the image cannot steal the match. For
each projection:

- **The window** (:func:`features_in_area`, ORB-SLAM's ``GetFeaturesInArea``): keypoints with
  ``|x - u| < r`` and ``|y - v| < r`` around the projection ``(u, v)``, on the levels it
  allows, and not already taken (matched by an earlier search). With a predicted right-image
  column (stereo and RGB-D), a keypoint with one too must lie within ``r`` of it as well.
- **The nearest** by Hamming distance, within ``max_distance`` (ORB-SLAM's ``TH_HIGH``, 100).
- **The ratio test, on one level:** rejected when the second nearest lies on the nearest's
  level and ``best > ratio * second`` (ORB-SLAM's local-map search; ``ratio`` 1 disables it).
- **One projection per keypoint:** where several chose the same keypoint, the nearest keeps
  it (ties: the lowest projection index); ORB-SLAM gives it to the first in its loop instead.
- **Rotation consistency** (optional), as for :func:`match`, between each projection's
  ``angle`` (the bin of the observation its descriptor came from) and the keypoint's.

The nearest and the second nearest are the first two candidates by ``(distance, index)``,
which is what ORB-SLAM's running update over ascending indices finds. The compiled cores
bucket the frame's keypoints into a :data:`GRID_COLS` x :data:`GRID_ROWS` grid over their
bounding box and read only the cells a window covers; the grid's cell maps are monotone, so
it finds exactly the keypoints this module's brute force finds.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, NamedTuple, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations.arrays import as_count, as_flag, as_number, as_real, holds_integers
from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.orb import ANGLE_BINS, DESCRIPTOR_BYTES, MAX_LEVELS, Features

MAX_DESCRIPTORS = 1 << 20
"""The most descriptors a set may hold."""

MAX_DISTANCE = 8 * DESCRIPTOR_BYTES
"""The largest Hamming distance: every bit differs."""

SECONDARY_FRACTION = 0.1
"""The rotation histogram's second and third bins are kept only if they hold at least this
fraction of the highest (ORB-SLAM's ``ComputeThreeMaxima``)."""

GRID_COLS = 64
"""The compiled cores' feature grid: cells across the keypoints' bounding box (ORB-SLAM's
``FRAME_GRID_COLS``). Results do not depend on it."""

GRID_ROWS = 48
"""Cells down it (ORB-SLAM's ``FRAME_GRID_ROWS``)."""

_POPCOUNT = np.array([bin(b).count("1") for b in range(256)], dtype=np.int64)
"""Set bits per byte value."""

_CHUNK = 256
"""First-set rows per block of the reference's distance computation (bounds its memory)."""


@dataclass(frozen=True)
class Options:
    """Matching filters.

    Attributes
    ----------
    max_distance : int
        The largest Hamming distance a match may have, from 0 to :data:`MAX_DISTANCE`
        (ORB-SLAM's 50).
    ratio : float
        The ratio test's bound, in (0, 1]: the nearest must be nearer than ``ratio`` times
        the second nearest. 1 keeps every nearest that is not tied.
    check_orientation : bool
        Keep only matches in the rotation histogram's three highest bins.
    """

    max_distance: int = 50
    ratio: float = 0.8
    check_orientation: bool = True

    def __post_init__(self) -> None:
        """Check the settings' types and ranges.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting has the wrong type or is out of range.
        """
        set_ = object.__setattr__  # frozen: store the checked, normalised values
        set_(self, "max_distance", as_count(self.max_distance, "max_distance", 0, MAX_DISTANCE))
        ratio = as_number(self.ratio, "ratio")
        if not (math.isfinite(ratio) and 0.0 < ratio <= 1.0):
            raise InvalidInputError(f"ratio: must be in (0, 1], got {ratio}")
        set_(self, "ratio", ratio)
        set_(self, "check_orientation", as_flag(self.check_orientation, "check_orientation"))


@dataclass(frozen=True)
class Matches:
    """Matched pairs, ordered by their first-set index.

    Attributes
    ----------
    pairs : ndarray of int64, shape (K, 2)
        Each match's ``(first-set index, second-set index)``.
    distances : ndarray of int64, shape (K,)
        Each match's Hamming distance.
    """

    pairs: NDArray[np.int64]
    distances: NDArray[np.int64]


@dataclass(frozen=True)
class SearchOptions:
    """The filters of a search by projection (defaults: ORB-SLAM's search from the last frame).

    Attributes
    ----------
    max_distance : int
        The largest Hamming distance a match may have, from 0 to :data:`MAX_DISTANCE`
        (ORB-SLAM's ``TH_HIGH``, 100).
    ratio : float
        The one-level ratio test's bound, in (0, 1]; 1 disables it (ORB-SLAM's local-map
        search uses 0.8).
    check_orientation : bool
        Keep only matches in the rotation histogram's three highest bins (ORB-SLAM checks
        when tracking the last frame, not the local map).
    """

    max_distance: int = 100
    ratio: float = 1.0
    check_orientation: bool = True

    def __post_init__(self) -> None:
        """Check the settings' types and ranges.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting has the wrong type or is out of range.
        """
        checked = Options(self.max_distance, self.ratio, self.check_orientation)
        set_ = object.__setattr__  # frozen: store the checked, normalised values
        set_(self, "max_distance", checked.max_distance)
        set_(self, "ratio", checked.ratio)
        set_(self, "check_orientation", checked.check_orientation)


@dataclass(frozen=True)
class Projections:
    """Map points projected into a frame: where to look for each, and what to look for.

    Built checked: every array is validated and stored as a new C-contiguous copy.

    Attributes
    ----------
    xy : ndarray of float64, shape (M, 2)
        Each projection, in the full image's pixels: finite.
    radius : ndarray of float64, shape (M,)
        The half-width of each one's square search window, in pixels: finite and positive.
    min_level, max_level : ndarray of int64, shape (M,)
        The pyramid levels a match may lie on, inclusive (an empty range finds nothing).
    descriptors : ndarray of uint8, shape (M, DESCRIPTOR_BYTES)
        Each map point's descriptor.
    angle : ndarray of int64, shape (M,)
        The orientation bin, in ``[0, ANGLE_BINS)``, of the observation each descriptor came
        from (the rotation check compares it with the keypoint's).
    right : ndarray of float64, shape (M,)
        Each one's predicted right-image column, ``u - f b / z`` (stereo and RGB-D); NaN where
        there is none.
    """

    xy: NDArray[np.float64]
    radius: NDArray[np.float64]
    min_level: NDArray[np.int64]
    max_level: NDArray[np.int64]
    descriptors: NDArray[np.uint8]
    angle: NDArray[np.int64]
    right: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Check every array, one row per projection.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If an array is malformed, a position or radius is not finite, a radius is not
            positive, or a right column is infinite.
        """
        set_ = object.__setattr__  # frozen: store the checked copies
        descriptors = _as_descriptors(self.descriptors, "projections.descriptors")
        m = len(descriptors)
        xy = as_real(self.xy, "projections.xy")
        if xy.shape != (m, 2):
            raise InvalidInputError(f"projections.xy: expected shape ({m}, 2), got {xy.shape}")
        radius = _as_vector(self.radius, m, "projections.radius")
        if np.any(radius <= 0.0):
            raise InvalidInputError("projections.radius: must be positive")
        set_(self, "xy", xy)
        set_(self, "radius", radius)
        set_(self, "min_level", _as_integers(self.min_level, m, "projections.min_level"))
        set_(self, "max_level", _as_integers(self.max_level, m, "projections.max_level"))
        set_(self, "descriptors", descriptors)
        set_(self, "angle", _as_angles(self.angle, m, "projections.angle"))
        set_(self, "right", _as_vector(self.right, m, "projections.right", allow_nan=True))


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.matching`` submodule provides."""

    def distances(
        self, descriptors1: NDArray[np.uint8], descriptors2: NDArray[np.uint8]
    ) -> NDArray[np.int64]:
        """Return every pair's Hamming distance."""
        ...

    def match(
        self,
        descriptors1: NDArray[np.uint8],
        descriptors2: NDArray[np.uint8],
        angles1: NDArray[np.int64],
        angles2: NDArray[np.int64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Match two descriptor sets; return a dict of :class:`Matches`' fields."""
        ...

    def features_in_area(  # noqa: PLR0913 -- the keypoints, then the window
        self,
        xy: NDArray[np.float64],
        level: NDArray[np.int64],
        center: tuple[float, float],
        radius: float,
        *,
        min_level: int,
        max_level: int,
    ) -> NDArray[np.int64]:
        """Return the indices of the keypoints in a window, ascending."""
        ...

    def search_by_projection(
        self, projections: dict[str, Any], keypoints: dict[str, Any], options: dict[str, Any]
    ) -> dict[str, Any]:
        """Match projected map points to a frame's keypoints; a dict of :class:`Matches`' fields.

        ``projections`` holds :class:`Projections`' fields; ``keypoints`` the frame's ``xy``,
        ``level``, ``angle``, ``descriptors``, ``right`` (NaN for none) and ``taken``, and
        the pyramid's ``levels``.
        """
        ...


def _distances(
    descriptors1: NDArray[np.uint8], descriptors2: NDArray[np.uint8]
) -> NDArray[np.int64]:
    """Every pair's Hamming distance (reference)."""
    out = np.empty((len(descriptors1), len(descriptors2)), dtype=np.int64)
    for start in range(0, len(descriptors1), _CHUNK):
        block = descriptors1[start : start + _CHUNK, None, :] ^ descriptors2[None, :, :]
        out[start : start + _CHUNK] = _POPCOUNT[block].sum(axis=-1)
    return out


def _three_maxima(counts: NDArray[np.int64]) -> list[int]:
    """Return the rotation histogram's kept bins: its highest three, the lower two if large."""
    top: list[tuple[int, int]] = []  # (count, bin), highest first; ties keep the first bin
    for k, count in enumerate(counts.tolist()):
        place = next((i for i, (c, _) in enumerate(top) if count > c), len(top))
        top.insert(place, (count, k))
        del top[3:]
    highest = top[0][0]
    kept = [top[0][1]]
    for count, k in top[1:]:
        if count < SECONDARY_FRACTION * highest:
            break
        kept.append(k)
    return kept


def _nearest_two(
    descriptors1: NDArray[np.uint8], descriptors2: NDArray[np.uint8]
) -> tuple[NDArray[np.int64], NDArray[np.int64], NDArray[np.int64]]:
    """Each first-set row's nearest index and distance, and its second-nearest distance.

    Ties go to the lowest index; the second nearest equals the nearest on a tie. Rows are
    taken a block at a time, so memory stays linear in the second set.
    """
    n = len(descriptors1)
    best = np.empty(n, dtype=np.int64)
    best_distance = np.empty(n, dtype=np.int64)
    second = np.full(n, np.iinfo(np.int64).max, dtype=np.int64)
    rows = np.arange(min(n, _CHUNK))
    for start in range(0, n, _CHUNK):
        block = _distances(descriptors1[start : start + _CHUNK], descriptors2)
        index = np.argmin(block, axis=1)
        here = rows[: len(block)]
        best[start : start + _CHUNK] = index
        best_distance[start : start + _CHUNK] = block[here, index]
        if block.shape[1] > 1:
            block[here, index] = np.iinfo(np.int64).max
            second[start : start + _CHUNK] = block.min(axis=1)
    return best, best_distance, second


def _match(
    descriptors1: NDArray[np.uint8],
    descriptors2: NDArray[np.uint8],
    angles1: NDArray[np.int64],
    angles2: NDArray[np.int64],
    options: Options,
) -> Matches:
    """Match two descriptor sets (reference)."""
    if len(descriptors1) == 0 or len(descriptors2) == 0:
        return Matches(np.zeros((0, 2), dtype=np.int64), np.zeros(0, dtype=np.int64))
    best, best_distance, second = _nearest_two(descriptors1, descriptors2)
    if len(descriptors2) > 1:
        passes = best_distance < options.ratio * second.astype(np.float64)
    else:
        passes = np.ones(len(descriptors1), dtype=bool)
    candidates = np.flatnonzero(passes & (best_distance <= options.max_distance))
    # One match per second-set descriptor: the nearest (ties: the lowest first-set index).
    order = np.lexsort((candidates, best_distance[candidates], best[candidates]))
    ranked = candidates[order]
    first_of_each = np.ones(len(ranked), dtype=bool)
    first_of_each[1:] = best[ranked][1:] != best[ranked][:-1]
    kept = np.sort(ranked[first_of_each])
    if options.check_orientation and len(kept):
        rotation = (angles1[kept] - angles2[best[kept]]) % ANGLE_BINS
        counts = np.bincount(rotation, minlength=ANGLE_BINS).astype(np.int64)
        kept = kept[np.isin(rotation, _three_maxima(counts))]
    return Matches(
        pairs=np.column_stack([kept, best[kept]]).astype(np.int64).reshape(-1, 2),
        distances=best_distance[kept].astype(np.int64),
    )


class _Window(NamedTuple):
    """A search window: a square around a centre, on a range of levels."""

    u: float
    v: float
    radius: float
    min_level: int
    max_level: int


def _in_window(
    xy: NDArray[np.float64], level: NDArray[np.int64], window: _Window
) -> NDArray[np.bool_]:
    """Which keypoints lie in a window: near its centre on both axes, on its levels."""
    return np.asarray(
        (np.abs(xy[:, 0] - window.u) < window.radius)
        & (np.abs(xy[:, 1] - window.v) < window.radius)
        & (level >= window.min_level)
        & (level <= window.max_level)
    )


def _features_in_area(
    xy: NDArray[np.float64], level: NDArray[np.int64], window: _Window
) -> NDArray[np.int64]:
    """Return the indices of the keypoints in a window (reference)."""
    return np.flatnonzero(_in_window(xy, level, window)).astype(np.int64)


@dataclass(frozen=True)
class _Frame:
    """A frame's keypoints, checked, as a search by projection reads them."""

    xy: NDArray[np.float64]
    level: NDArray[np.int64]
    angle: NDArray[np.int64]
    descriptors: NDArray[np.uint8]
    right: NDArray[np.float64]
    taken: NDArray[np.bool_]
    levels: int


def _search_by_projection(
    projections: Projections, frame: _Frame, options: SearchOptions
) -> Matches:
    """Match projected map points to a frame's keypoints (reference)."""
    claim: dict[int, tuple[int, int]] = {}  # keypoint: (distance, projection)
    free = ~frame.taken
    for i in range(len(projections.xy)):
        radius = float(projections.radius[i])
        window = _Window(
            float(projections.xy[i, 0]),
            float(projections.xy[i, 1]),
            radius,
            int(projections.min_level[i]),
            int(projections.max_level[i]),
        )
        candidates = _in_window(frame.xy, frame.level, window) & free
        if not np.isnan(projections.right[i]):
            off = np.abs(frame.right - projections.right[i]) > radius  # False where NaN
            candidates &= ~off
        index = np.flatnonzero(candidates)
        if len(index) == 0:
            continue
        distance = _POPCOUNT[frame.descriptors[index] ^ projections.descriptors[i]].sum(axis=1)
        order = np.lexsort((index, distance))  # by distance, then index
        best, best_k = int(distance[order[0]]), int(index[order[0]])
        if best > options.max_distance:
            continue
        if len(order) > 1:
            second, second_k = float(distance[order[1]]), int(index[order[1]])
            if frame.level[second_k] == frame.level[best_k] and best > options.ratio * second:
                continue
        held = claim.get(best_k)
        if held is None or best < held[0]:  # ties: the earlier projection keeps it
            claim[best_k] = (best, i)
    kept = sorted((i, k, d) for k, (d, i) in claim.items())
    pairs = np.array([[i, k] for i, k, _ in kept], dtype=np.int64).reshape(-1, 2)
    distances = np.array([d for _, _, d in kept], dtype=np.int64)
    if options.check_orientation and kept:
        rotation = (projections.angle[pairs[:, 0]] - frame.angle[pairs[:, 1]]) % ANGLE_BINS
        counts = np.bincount(rotation, minlength=ANGLE_BINS).astype(np.int64)
        keep = np.isin(rotation, _three_maxima(counts))
        pairs, distances = pairs[keep], distances[keep]
    return Matches(pairs=pairs, distances=distances)


def _as_vector(
    value: ArrayLike, count: int, name: str, *, allow_nan: bool = False
) -> NDArray[np.float64]:
    """Check `count` real numbers: finite, or NaN too where allowed (never infinite)."""
    arr = as_real(value, name, allow_nan=allow_nan)
    if arr.shape != (count,):
        raise InvalidInputError(f"{name}: expected shape ({count},), got {arr.shape}")
    return arr


def _as_integers(value: ArrayLike, count: int, name: str) -> NDArray[np.int64]:
    """Check `count` integers (booleans refused); an empty list stands for none."""
    arr = np.asarray(value)
    if arr.size == 0 and count == 0:
        return np.zeros(0, dtype=np.int64)  # an empty list is float64 to numpy
    if arr.shape != (count,) or not holds_integers(arr):
        raise InvalidInputError(f"{name}: expected {count} integers, got {arr.dtype} {arr.shape}")
    return np.ascontiguousarray(arr, dtype=np.int64).copy()


def _as_frame(features: Features, right: ArrayLike | None, taken: ArrayLike | None) -> _Frame:
    """Check a frame's features and its per-keypoint extras."""
    keypoints = features.keypoints
    descriptors = _as_descriptors(features.descriptors, "features.descriptors")
    n = len(descriptors)
    xy = as_real(keypoints.xy, "features.keypoints.xy")
    if xy.shape != (n, 2):
        raise InvalidInputError(f"features.keypoints.xy: expected shape ({n}, 2), got {xy.shape}")
    levels = as_count(features.levels, "features.levels", 1, MAX_LEVELS)
    level = _as_integers(keypoints.level, n, "features.keypoints.level")
    if n and (level.min() < 0 or level.max() >= levels):
        raise InvalidInputError(f"features.keypoints.level: levels must lie in [0, {levels})")
    flags = np.zeros(n, dtype=bool) if taken is None else np.asarray(taken)
    if flags.shape != (n,) or flags.dtype != np.bool_:
        raise InvalidInputError(
            f"taken: expected booleans of shape ({n},), got {flags.dtype} {flags.shape}"
        )
    columns = np.full(n, np.nan) if right is None else _as_vector(right, n, "right", allow_nan=True)
    return _Frame(
        xy=xy,
        level=level,
        angle=_as_angles(keypoints.angle, n, "features.keypoints.angle"),
        descriptors=descriptors,
        right=columns,
        taken=np.ascontiguousarray(flags).copy(),
        levels=levels,
    )


def _as_descriptors(value: ArrayLike, name: str) -> NDArray[np.uint8]:
    """Check a descriptor set: an (N, DESCRIPTOR_BYTES) uint8 array."""
    arr = np.asarray(value)
    if arr.dtype != np.uint8 or arr.ndim != 2 or arr.shape[1] != DESCRIPTOR_BYTES:
        raise InvalidInputError(
            f"{name}: expected an (N, {DESCRIPTOR_BYTES}) uint8 array, got {arr.dtype} {arr.shape}"
        )
    if len(arr) > MAX_DESCRIPTORS:
        raise InvalidInputError(f"{name}: at most {MAX_DESCRIPTORS} descriptors, got {len(arr)}")
    return np.ascontiguousarray(arr).copy()


def _as_angles(value: ArrayLike, count: int, name: str) -> NDArray[np.int64]:
    """Check orientation bins: `count` integers in ``[0, ANGLE_BINS)`` (booleans refused)."""
    arr = _as_integers(value, count, name)
    if count and (arr.min() < 0 or arr.max() >= ANGLE_BINS):
        raise InvalidInputError(f"{name}: bins must lie in [0, {ANGLE_BINS})")
    return arr


def _decode_distances(raw: Any, codes: backends.Codes) -> NDArray[np.int64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's distance matrix, checked (never cast)."""
    del codes  # the matching module crosses no codes
    out: NDArray[np.int64] = backends.checked_array(raw, np.int64, 2, "matching distances")
    return out


def decode_matches(raw: dict[str, Any], codes: backends.Codes) -> Matches:
    """Rebuild :class:`Matches` from a compiled backend's dict, its arrays checked.

    Parameters
    ----------
    raw : dict
        ``pairs`` (M, 2) and ``distances`` (M,), int64.
    codes : dict
        The backend module's code tables (matches cross none).

    Returns
    -------
    Matches
        The matches.
    """
    del codes
    return Matches(
        pairs=backends.checked_array(raw["pairs"], np.int64, 2, "matching pairs"),
        distances=backends.checked_array(raw["distances"], np.int64, 1, "matching distances"),
    )


def distances(
    descriptors1: ArrayLike, descriptors2: ArrayLike, *, backend: str = "python"
) -> NDArray[np.int64]:
    """Return the Hamming distance between every pair of descriptors.

    The result holds N x M int64 values (8 bytes each), so two sets of
    :data:`MAX_DESCRIPTORS` would need 8 TiB; :func:`match` never forms it, taking the
    first set a block of rows at a time.

    Parameters
    ----------
    descriptors1 : array_like of uint8, shape (N, DESCRIPTOR_BYTES)
        The first set.
    descriptors2 : array_like of uint8, shape (M, DESCRIPTOR_BYTES)
        The second set.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (N, M)
        The number of differing bits, from 0 to :data:`MAX_DISTANCE`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a set is not an (N, DESCRIPTOR_BYTES) uint8 array of at most
        :data:`MAX_DESCRIPTORS` rows.
    """
    first = _as_descriptors(descriptors1, "descriptors1")
    second = _as_descriptors(descriptors2, "descriptors2")
    return backends.call(
        backend,
        __name__,
        "distances",
        reference=lambda: _distances(first, second),
        arguments={"descriptors1": first, "descriptors2": second},
        decode=_decode_distances,
    )


def match(  # noqa: PLR0913 -- two sets, their angles, the filters
    descriptors1: ArrayLike,
    descriptors2: ArrayLike,
    angles1: ArrayLike,
    angles2: ArrayLike,
    options: Options | None = None,
    *,
    backend: str = "python",
) -> Matches:
    """Match each descriptor of the first set to its nearest in the second, filtered.

    Parameters
    ----------
    descriptors1 : array_like of uint8, shape (N, DESCRIPTOR_BYTES)
        The first set.
    descriptors2 : array_like of uint8, shape (M, DESCRIPTOR_BYTES)
        The second set.
    angles1 : array_like of int, shape (N,)
        The first set's orientation bins (:attr:`slamlab.foundations.orb.Keypoints.angle`).
    angles2 : array_like of int, shape (M,)
        The second set's.
    options : Options, optional
        The filters (default: distance 50, ratio 0.8, rotation consistency on).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Matches
        The kept pairs, by first-set index; each second-set descriptor appears at most once.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a set is malformed, or the angles are not one bin in ``[0, ANGLE_BINS)`` per
        descriptor.
    """
    first = _as_descriptors(descriptors1, "descriptors1")
    second = _as_descriptors(descriptors2, "descriptors2")
    bins1 = _as_angles(angles1, len(first), "angles1")
    bins2 = _as_angles(angles2, len(second), "angles2")
    settings = Options() if options is None else options
    return backends.call(
        backend,
        __name__,
        "match",
        reference=lambda: _match(first, second, bins1, bins2, settings),
        arguments={
            "descriptors1": first,
            "descriptors2": second,
            "angles1": bins1,
            "angles2": bins2,
            "options": asdict(settings),
        },
        decode=decode_matches,
    )


def _decode_indices(raw: Any, codes: backends.Codes) -> NDArray[np.int64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's keypoint indices, checked (never cast)."""
    del codes
    out: NDArray[np.int64] = backends.checked_array(raw, np.int64, 1, "features_in_area indices")
    return out


def features_in_area(  # noqa: PLR0913 -- the keypoints, then the window
    xy: ArrayLike,
    level: ArrayLike,
    center: ArrayLike,
    radius: float,
    *,
    min_level: int = 0,
    max_level: int = MAX_LEVELS - 1,
    backend: str = "python",
) -> NDArray[np.int64]:
    """Return the keypoints in a square window, on a range of levels (``GetFeaturesInArea``).

    Parameters
    ----------
    xy : array_like of float, shape (N, 2)
        The keypoints, in the full image's pixels.
    level : array_like of int, shape (N,)
        Their pyramid levels, non-negative.
    center : array_like of float, shape (2,)
        The window's centre ``(u, v)``.
    radius : float
        Its half-width ``r``, positive: a keypoint is in it when ``|x - u| < r`` and
        ``|y - v| < r``.
    min_level, max_level : int, optional
        The levels it covers, inclusive (default: every level; an empty range finds nothing).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (K,)
        The keypoints' indices, ascending.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the keypoints are malformed or not finite, there are more than
        :data:`MAX_DESCRIPTORS`, or the window's centre or radius is not finite, or the
        radius is not positive.
    """
    positions = as_real(xy, "xy")
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise InvalidInputError(f"xy: expected shape (N, 2), got {positions.shape}")
    n = len(positions)
    if n > MAX_DESCRIPTORS:
        raise InvalidInputError(f"xy: at most {MAX_DESCRIPTORS} keypoints, got {n}")
    levels = _as_integers(level, n, "level")
    if n and levels.min() < 0:
        raise InvalidInputError("level: levels must be non-negative")
    middle = as_real(center, "center")
    if middle.shape != (2,):
        raise InvalidInputError(f"center: expected shape (2,), got {middle.shape}")
    half = as_number(radius, "radius")
    if not (math.isfinite(half) and half > 0.0):
        raise InvalidInputError(f"radius: must be finite and positive, got {half}")
    low = _as_level_bound(min_level, "min_level")
    high = _as_level_bound(max_level, "max_level")
    window = _Window(float(middle[0]), float(middle[1]), half, low, high)
    return backends.call(
        backend,
        __name__,
        "features_in_area",
        reference=lambda: _features_in_area(positions, levels, window),
        arguments={
            "xy": positions,
            "level": levels,
            "center": (window.u, window.v),
            "radius": half,
            "min_level": low,
            "max_level": high,
        },
        decode=_decode_indices,
    )


def _as_level_bound(value: object, name: str) -> int:
    """Check a level range's bound: any int64, inside the pyramid or not."""
    bounds = np.iinfo(np.int64)
    return as_count(value, name, int(bounds.min), int(bounds.max))


def search_by_projection(  # noqa: PLR0913 -- the projections, the frame and its extras
    projections: Projections,
    features: Features,
    options: SearchOptions | None = None,
    *,
    right: ArrayLike | None = None,
    taken: ArrayLike | None = None,
    backend: str = "python",
) -> Matches:
    """Match map points projected into a frame to its keypoints (``SearchByProjection``).

    Parameters
    ----------
    projections : Projections
        Where each map point is predicted to appear, the window and levels to search, and
        its descriptor.
    features : slamlab.foundations.orb.Features
        The frame's features (their positions, levels, angles and descriptors are read).
    options : SearchOptions, optional
        The filters (default: distance 100, no ratio test, rotation consistency on).
    right : array_like of float, shape (N,), optional
        Each keypoint's right-image column (stereo, or RGB-D's virtual one); NaN for none.
        Default: none.
    taken : array_like of bool, shape (N,), optional
        Keypoints already matched, never candidates. Default: none.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Matches
        ``(projection, keypoint)`` pairs, by projection; each keypoint appears at most once.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the features are malformed (a position not finite, a level outside the pyramid,
        an angle outside ``[0, ANGLE_BINS)``), `right` or `taken` does not have one entry per
        keypoint, or either set has more than :data:`MAX_DESCRIPTORS` rows.
    """
    frame = _as_frame(features, right, taken)
    settings = SearchOptions() if options is None else options
    return backends.call(
        backend,
        __name__,
        "search_by_projection",
        reference=lambda: _search_by_projection(projections, frame, settings),
        arguments={
            "projections": asdict(projections),
            "keypoints": asdict(frame),
            "options": asdict(settings),
        },
        decode=decode_matches,
    )
