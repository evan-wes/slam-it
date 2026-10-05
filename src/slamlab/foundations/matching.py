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
  ``(angle1 - angle2) mod 30``.

Nearest neighbours tie to the lowest second-set index. Every quantity is an integer except
the ratio test's one float64 product, so the three languages agree exactly.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.orb import ANGLE_BINS, DESCRIPTOR_BYTES

MAX_DESCRIPTORS = 1 << 20
"""The most descriptors a set may hold."""

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
        The largest Hamming distance a match may have, from 0 to 256 (ORB-SLAM's 50).
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
        """Check the settings.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting is out of range.
        """
        if not 0 <= self.max_distance <= 8 * DESCRIPTOR_BYTES:
            raise InvalidInputError(
                f"max_distance: must be from 0 to {8 * DESCRIPTOR_BYTES}, got {self.max_distance}"
            )
        if not (math.isfinite(self.ratio) and 0.0 < self.ratio <= 1.0):
            raise InvalidInputError(f"ratio: must be in (0, 1], got {self.ratio}")


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
        if count < 0.1 * highest:
            break
        kept.append(k)
    return kept


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
    distances = _distances(descriptors1, descriptors2)
    best = np.argmin(distances, axis=1)  # ties: the lowest second-set index
    best_distance = distances[np.arange(len(distances)), best]
    if distances.shape[1] > 1:
        masked = distances.copy()
        masked[np.arange(len(masked)), best] = np.iinfo(np.int64).max
        second = masked.min(axis=1).astype(np.float64)
        passes = best_distance < options.ratio * second
    else:
        passes = np.ones(len(distances), dtype=bool)
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
    """Check orientation bins: `count` integers in ``[0, ANGLE_BINS)``."""
    arr = np.asarray(value)
    if arr.shape != (count,) or not np.can_cast(arr.dtype, np.int64):
        raise InvalidInputError(f"{name}: expected {count} integers, got {arr.dtype} {arr.shape}")
    if count and (arr.min() < 0 or arr.max() >= ANGLE_BINS):
        raise InvalidInputError(f"{name}: bins must lie in [0, {ANGLE_BINS})")
    return arr.astype(np.int64)


def _decode_distances(raw: Any, codes: backends.Codes) -> NDArray[np.int64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's distances, checked to be int64 (never cast)."""
    del codes  # the matching module crosses no codes
    out = np.asarray(raw)
    if out.dtype != np.int64:
        raise RuntimeError(f"matching: backend returned {out.dtype}, not int64")
    return out


def _decode_matches(raw: dict[str, Any], codes: backends.Codes) -> Matches:
    """Rebuild :class:`Matches` from a compiled backend's dict."""
    return Matches(
        pairs=_decode_distances(raw["pairs"], codes).reshape(-1, 2),
        distances=_decode_distances(raw["distances"], codes),
    )


def distances(
    descriptors1: ArrayLike, descriptors2: ArrayLike, *, backend: str = "python"
) -> NDArray[np.int64]:
    """Return the Hamming distance between every pair of descriptors.

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
        The number of differing bits, from 0 to 256.

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
        decode=_decode_matches,
    )
