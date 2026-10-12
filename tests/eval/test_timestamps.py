"""Timestamps: the ascending check and the nearest-time search the eval modules share."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from slam_it.eval import timestamps
from slam_it.foundations.errors import InvalidInputError


def test_as_times_accepts_ascending_times() -> None:
    """Strictly ascending times pass; repeats pass only when not strict."""
    np.testing.assert_array_equal(timestamps.as_times([0, 0.5, 2], "t"), [0.0, 0.5, 2.0])
    np.testing.assert_array_equal(timestamps.as_times([1.0, 1.0], "t", strict=False), [1.0, 1.0])
    assert timestamps.as_times([], "t").shape == (0,)


@pytest.mark.parametrize(
    ("values", "message"),
    [
        ([[0.0, 1.0]], "expected a 1D array"),
        ([0.0, np.nan], "expected finite values"),
        ([0.0, 2.0, 1.0], "must ascend strictly"),
        ([1.0, 1.0], "must ascend strictly"),
    ],
)
def test_as_times_rejects(values: list[Any], message: str) -> None:
    """Non-1D, non-finite, descending or (when strict) repeated times are refused."""
    with pytest.raises(InvalidInputError, match=message):
        timestamps.as_times(values, "t")


def test_nearest_finds_the_closest_time() -> None:
    """Each stamp takes its nearest time; a tie takes the earlier; the ends clamp."""
    times = np.array([0.0, 1.0, 2.0, 4.0])
    found = timestamps.nearest(times, [-5.0, 0.4, 0.5, 2.9, 3.0, 3.1, 9.0])
    np.testing.assert_array_equal(found, [0, 0, 0, 2, 2, 3, 3])
    np.testing.assert_array_equal(timestamps.nearest(times[:1], [-1.0, 7.0]), [0, 0])


def test_nearest_compares_nanoseconds_exactly() -> None:
    """Integer nanosecond times one apart are told apart (float64 seconds could not)."""
    start = 1403636579763555584
    times = np.array([start, start + 2], dtype=np.int64)
    np.testing.assert_array_equal(
        timestamps.nearest(times, np.array([start + 1, start + 2])), [0, 1]
    )


@pytest.mark.parametrize("times", [np.zeros(0), np.array([1.0, 0.0]), np.zeros((2, 2))])
def test_nearest_rejects(times: np.ndarray) -> None:
    """Empty, descending or non-1D times are refused."""
    with pytest.raises(InvalidInputError, match="non-empty, ascending"):
        timestamps.nearest(times, [0.0])


@pytest.mark.parametrize(
    ("times", "stamps"),
    [
        (np.array([1, 5, 9], np.uint64), np.array([4, 6], np.uint64)),  # would wrap
        (np.array([1, 5, 9], np.int64), np.array([4.0, 6.0])),
        (np.array([1.0, 5.0], np.float32), np.array([4.0], np.float32)),
    ],
)
def test_nearest_rejects_other_dtypes(times: np.ndarray, stamps: np.ndarray) -> None:
    """Only float64 or int64, the same for both: unsigned differences would wrap silently."""
    with pytest.raises(InvalidInputError, match="expected one dtype"):
        timestamps.nearest(times, stamps)
