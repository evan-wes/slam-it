"""Camera-model tests: parity with the mpmath fixture, round trips, and failure modes."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import camera
from slamlab.foundations.errors import NotConvergedError
from tests.conftest import check_parity, load_fixture, parity_params, protocol_members

CAMERA = load_fixture("foundations/camera.json")
RADTAN = np.array([520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172])


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    assert set(CAMERA["functions"]) == protocol_members(camera)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(CAMERA))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend matches the mpmath oracle under the function's tolerance policy."""
    check_parity(camera, CAMERA, function, case, backend)


def _points(n: int, seed: int = 0) -> np.ndarray[Any, np.dtype[np.float64]]:
    """Random points in front of the camera, inside a typical field of view."""
    rng = np.random.default_rng(seed)
    return np.column_stack(
        [rng.uniform(-1.0, 1.0, n), rng.uniform(-0.8, 0.8, n), rng.uniform(1.0, 5.0, n)]
    )


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize(("model", "intrinsics"), [("pinhole", RADTAN[:4]), ("radtan", RADTAN)])
def test_unproject_inverts_project(
    backend: str, model: str, intrinsics: NDArray[np.float64]
) -> None:
    """Unprojecting a projected point gives the unit ray through it."""
    points = _points(300)
    pixels = camera.project(model, intrinsics, points, backend=backend)
    rays = points / np.linalg.norm(points, axis=-1, keepdims=True)
    np.testing.assert_allclose(
        camera.unproject(model, intrinsics, pixels, backend=backend), rays, rtol=0, atol=1e-14
    )


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_jacobians_match_finite_differences(backend: str) -> None:
    """The analytic Jacobians agree with central differences in every input."""
    points = _points(20, seed=1)
    d_point, d_intrinsics = camera.project_jacobians("radtan", RADTAN, points, backend=backend)
    step = 1e-6
    for k in range(3):
        delta = np.zeros(3)
        delta[k] = step
        numeric = (
            camera.project("radtan", RADTAN, points + delta)
            - camera.project("radtan", RADTAN, points - delta)
        ) / (2 * step)
        np.testing.assert_allclose(d_point[..., k], numeric, rtol=1e-7, atol=1e-6)
    for k in range(RADTAN.size):
        delta = np.zeros(RADTAN.size)
        delta[k] = step * max(1.0, abs(RADTAN[k]))
        numeric = (
            camera.project("radtan", RADTAN + delta, points)
            - camera.project("radtan", RADTAN - delta, points)
        ) / (2 * delta[k])
        np.testing.assert_allclose(d_intrinsics[..., k], numeric, rtol=1e-7, atol=1e-6)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_undistortion_outside_the_invertible_region_is_reported(backend: str) -> None:
    """A pixel the distortion cannot reach raises NotConvergedError, not a wrong ray."""
    barrel = [500.0, 500.0, 320.0, 240.0, -0.9, 0.0, 0.0, 0.0, 0.0]
    with pytest.raises(NotConvergedError):
        camera.unproject("radtan", barrel, [[2000.0, 1500.0]], backend=backend)


def test_pinhole_is_radtan_without_distortion() -> None:
    """With zero distortion coefficients, radtan projects exactly like pinhole."""
    points = _points(50)
    zero = np.concatenate([RADTAN[:4], np.zeros(5)])
    np.testing.assert_array_equal(
        camera.project("radtan", zero, points), camera.project("pinhole", RADTAN[:4], points)
    )
