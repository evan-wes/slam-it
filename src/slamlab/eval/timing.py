"""Timestamps: checking that they ascend, and finding the nearest.

Every reader and metric in :mod:`slamlab.eval` pairs samples by time: a frame with its
ground-truth pose (:mod:`slamlab.eval.tum`, :mod:`slamlab.eval.euroc`), a run's poses with the
ground truth's (:func:`slamlab.eval.metrics.associate`), a pose with the one an interval later
(:func:`slamlab.eval.metrics.relative_error`). Each relies on sorted times, which a binary
search needs; this module holds the check and the search they share.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab.foundations.arrays import as_real
from slamlab.foundations.errors import InvalidInputError


def as_times(values: ArrayLike, name: str, *, strict: bool = True) -> NDArray[np.float64]:
    """Check timestamps: a finite 1D array that ascends.

    Parameters
    ----------
    values : array_like, shape (N,)
        The times, in seconds.
    name : str
        Their name, for messages.
    strict : bool, optional
        Require every time to exceed the one before (default); otherwise equal times may
        repeat.

    Returns
    -------
    ndarray of float64, shape (N,)
        The times.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `values` is not a finite 1D array, or it does not ascend.
    """
    times = as_real(values, name)
    if times.ndim != 1:
        raise InvalidInputError(f"{name}: expected a 1D array, got shape {times.shape}")
    steps = np.diff(times)
    descending = steps <= 0.0 if strict else steps < 0.0
    if np.any(descending):
        order = "strictly " if strict else ""
        raise InvalidInputError(f"{name}: must ascend {order}(sort them first)")
    return times


def nearest(times: NDArray[Any], stamps: ArrayLike) -> NDArray[np.int64]:
    """Return, for each stamp, the index of the nearest of `times` (ties: the earlier).

    Parameters
    ----------
    times : ndarray, shape (N,)
        Ascending times, N >= 1: seconds, or integer nanoseconds (compared exactly).
    stamps : array_like, shape (M,)
        The times to look up, in the same unit, in any order.

    Returns
    -------
    ndarray of int64, shape (M,)
        Indices into `times`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `times` is empty, not 1D, or does not ascend.
    """
    if times.ndim != 1 or len(times) == 0 or np.any(np.diff(times) < 0):
        raise InvalidInputError("times: expected a non-empty, ascending 1D array")
    lookups = np.asarray(stamps)
    if len(times) == 1:
        return np.zeros(lookups.shape, dtype=np.int64)
    after = np.clip(np.searchsorted(times, lookups), 1, len(times) - 1)
    before = after - 1
    earlier = np.abs(times[before] - lookups) <= np.abs(times[after] - lookups)
    return np.asarray(np.where(earlier, before, after), dtype=np.int64)
