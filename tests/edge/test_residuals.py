"""Observation errors' tests: every backend against the reference, and the test's rules."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import ArrayLike

from slam_it import backends
from slam_it.edge import residuals
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import Truth
from tests.edge.support import CAMERA, OPTIONS, assert_close


def _observations(truth: Truth) -> tuple[np.ndarray, ...]:
    """Every observation of the truth map: points in their cameras, pixels, columns, variances."""
    m = truth.map
    rows = np.flatnonzero(m.kp_point >= 0)
    pose = m.kf_pose[m.keypoint_keyframe()[rows]]
    xyz = m.pt_xyz[m.point_rows(m.kp_point[rows])]
    local = np.einsum("mij,mj->mi", pose[:, :3, :3], xyz) + pose[:, :3, 3]
    local[::17, 2] *= -1.0  # some behind the camera
    variance = residuals.sigma2(m, m.kp_level[rows])
    return local, m.kp_xy[rows], m.kp_right[rows], variance


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_chi2_matches_the_reference(backend: str, truth: Truth) -> None:
    """Every backend's errors equal the reference's, infinities included."""
    chosen = require_function(backend, residuals.__name__, "chi2")
    observations = _observations(truth)
    expected = residuals.chi2(CAMERA, *observations)
    got = residuals.chi2(CAMERA, *observations, backend=chosen)
    assert_close(got, expected)
    assert np.isinf(expected).sum() == len(expected[::17])


def test_chi2_weighs_both_terms() -> None:
    """The pixel term and, where there is one, the right column's, over the variance."""
    local = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 2.0], [0.0, 0.0, 0.0]])
    centre = CAMERA.project(local[:1])[0]
    xy = np.stack([centre + np.array([3.0, 4.0]), centre, centre])
    predicted = centre[0] - CAMERA.focal * CAMERA.baseline / 2.0
    errors = residuals.chi2(CAMERA, local, xy, [np.nan, predicted - 2.0, np.nan], [1.0, 4.0, 1.0])
    np.testing.assert_allclose(errors[:2], [25.0, 1.0])
    assert np.isinf(errors[2])  # on the camera's plane: not in front


def test_thresholds_follow_the_right_column(truth: Truth) -> None:
    """2 dof without a right column, 3 with; level l's variance is s^(2 l)."""
    limits = residuals.thresholds(np.array([np.nan, 10.0]), OPTIONS)
    np.testing.assert_array_equal(limits, [OPTIONS.chi2_mono, OPTIONS.chi2_stereo])
    np.testing.assert_allclose(residuals.sigma2(truth.map, np.array([0, 2])), [1.0, 1.2**4])


@pytest.mark.parametrize(
    ("local", "right", "variance", "message"),
    [
        ([[0.0, 0.0, np.nan]], [np.nan], [1.0], "local"),
        ([[0.0, 0.0, 1.0]], [np.inf], [1.0], "right"),
        ([[0.0, 0.0, 1.0]], [np.nan], [0.0], "variance: must be positive"),
        ([[0.0, 0.0, 1.0]], [np.nan, np.nan], [1.0], "one row per observation"),
    ],
)
def test_chi2_rejects_invalid_input(
    local: ArrayLike, right: ArrayLike, variance: ArrayLike, message: str
) -> None:
    """Values must be finite (NaN columns aside), one per observation, variances positive."""
    with pytest.raises(InvalidInputError, match=message):
        residuals.chi2(CAMERA, local, [[0.0, 0.0]], right, variance)
