"""Tests for slam_it_all.eval.reprojection: in-front masks and RMS error, by hand."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from slam_it_all.eval.reprojection import in_front, rms
from slam_it_all.foundations import ba
from slam_it_all.foundations.errors import InvalidInputError


def _problem() -> ba.BAProblem:
    """Two points seen by one identity camera (f = 100, centre 0): pixels 100 x/z."""
    return ba.BAProblem.create(
        "pinhole",
        [100.0, 100.0, 0.0, 0.0],
        np.eye(4)[None],
        [[0.0, 0.0, 2.0], [1.0, 0.0, 4.0]],
        [[3.0, 4.0], [25.0, 0.0]],
        [0, 0],
        [0, 1],
    )


def test_rms_by_hand() -> None:
    """Errors (3, 4) and (0, 0) give RMS sqrt((25 + 0) / 2)."""
    problem = _problem()
    np.testing.assert_allclose(rms(problem, np.array([True, True])), np.sqrt(12.5), rtol=1e-15)
    np.testing.assert_allclose(rms(problem, np.array([False, True])), 0.0, atol=1e-15)


def test_in_front_and_rejections() -> None:
    """A point behind the camera is flagged, and rms refuses to measure it or nothing."""
    problem = replace(_problem(), points=np.array([[0.0, 0.0, 2.0], [1.0, 0.0, -4.0]]))
    np.testing.assert_array_equal(in_front(problem), [True, False])
    with pytest.raises(InvalidInputError, match="behind"):
        rms(problem, np.array([True, True]))
    with pytest.raises(InvalidInputError, match="no observations"):
        rms(problem, np.array([False, False]))
    with pytest.raises(InvalidInputError, match="used"):
        rms(problem, np.array([True]))
