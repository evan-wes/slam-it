"""Triangulation tests: parity with the mpmath fixture, optimality, hard cases, validation."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import se3, triangulation
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import assert_matches_fixture, load_fixture, require_function
from tests.foundations.scenes import POSE
from tests.foundations.scenes import bearings as _bearings
from tests.foundations.scenes import in_camera as _in_camera
from tests.foundations.scenes import scene as _scene

FIXTURE = load_fixture("foundations/triangulation.json")


def _packed(result: triangulation.Triangulation) -> NDArray[np.float64]:
    """Pack a result as the fixture stores it: point, in_front, parallax."""
    return np.column_stack([result.points, result.in_front, result.parallax])


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", ["two_view", "n_view"])
def test_matches_fixture(backend: str, function: str) -> None:
    """Each backend matches the 50-digit oracle on every case."""
    spec: dict[str, Any] = FIXTURE["functions"][function]
    chosen = require_function(backend, triangulation.__name__, function)
    for case in spec["cases"]:
        result = getattr(triangulation, function)(**case["input"], backend=chosen)
        assert_matches_fixture(_packed(result), case["expected"], spec["tolerance"])


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_two_view_is_optimal(backend: str) -> None:
    """No nearby point has a lower reprojection error than niter2's (Lindstrom's optimality)."""
    rng = np.random.default_rng(1)
    points = _scene(20)
    x1 = points / points[:, 2:3]
    in2 = _in_camera(POSE, points)
    x2 = in2 / in2[:, 2:3]
    x1[:, :2] += rng.normal(scale=1e-3, size=(20, 2))
    x2[:, :2] += rng.normal(scale=1e-3, size=(20, 2))
    result = triangulation.two_view(POSE, _bearings(x1), _bearings(x2), backend=backend)

    def cost(point: NDArray[np.float64], i: int) -> float:
        q = POSE[:3, :3] @ point + POSE[:3, 3]
        r1 = point[:2] / point[2] - x1[i, :2]
        r2 = q[:2] / q[2] - x2[i, :2]
        return float(r1 @ r1 + r2 @ r2)

    for i in range(20):
        best = cost(result.points[i], i)
        for step in rng.normal(scale=1e-4, size=(50, 3)):
            assert cost(result.points[i] + step, i) >= best * (1.0 - 1e-9)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_points_on_the_baseline_are_at_infinity(backend: str) -> None:
    """A point on the line through both centres has parallel rays: NaN, behind, no parallax."""
    pose = se3.exp([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])  # camera 2 one metre forward
    points = np.array([[0.0, 0.0, 5.0], [1.0, 0.5, 5.0]])
    result = triangulation.two_view(
        pose, _bearings(points), _bearings(_in_camera(pose, points)), backend=backend
    )
    assert np.all(np.isnan(result.points[0]))
    assert not result.in_front[0]
    assert result.parallax[0] == 0.0
    np.testing.assert_allclose(result.points[1], points[1], rtol=0, atol=1e-12)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_n_view_parallel_rays_are_at_infinity(backend: str) -> None:
    """Identical rays from two centres along them meet at infinity in the DLT too."""
    poses = np.stack([np.eye(4), se3.exp([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])])
    rays = np.tile(np.array([0.0, 0.0, 1.0]), (1, 2, 1))
    result = triangulation.n_view(poses, rays, backend=backend)
    assert np.all(np.isnan(result.points))
    assert not result.in_front[0]


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_n_view_recovers_points_from_partial_tracks(backend: str) -> None:
    """Noise-free tracks of different lengths recover their points; two views equal two_view."""
    points = _scene(25, seed=2)
    poses = np.stack([np.eye(4), POSE, se3.exp([-0.4, 0.2, 0.1, -0.05, 0.08, 0.0])])
    rays = np.stack([_bearings(_in_camera(p, points)) for p in poses], axis=1)
    observed = np.ones((25, 3), dtype=bool)
    observed[::2, 2] = False
    result = triangulation.n_view(poses, rays, observed, backend=backend)
    np.testing.assert_allclose(result.points, points, rtol=0, atol=1e-12)
    assert result.in_front.all()
    pair = triangulation.two_view(POSE, rays[:, 0], rays[:, 1], backend=backend)
    np.testing.assert_allclose(pair.points, points, rtol=0, atol=1e-12)


def test_parallax_is_the_angle_between_centres() -> None:
    """A point straight ahead of two cameras 2 m apart, 1 m away: 2 atan(1) = 90 degrees."""
    poses = np.stack(
        [se3.exp([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), se3.exp([-1.0, 0.0, 0.0] + [0.0] * 3)]
    )
    point = np.array([0.0, 0.0, 1.0])
    rays = np.stack([_bearings(_in_camera(p, point[None]))[0] for p in poses])[None]
    result = triangulation.n_view(poses, rays)
    assert result.parallax[0] == pytest.approx(np.pi / 2, abs=1e-14)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"relative_pose": np.eye(4)}, "zero translation"),
        ({"relative_pose": np.stack([POSE, POSE])}, "one"),
        ({"bearings1": np.ones((3, 3))}, "unit"),
        ({"bearings1": _bearings(np.array([[0.0, 0.0, -1.0]] * 3))}, "positive z"),
        ({"bearings2": _bearings(np.ones((2, 3)))}, "equal"),
    ],
)
def test_two_view_rejects_invalid_input(kwargs: dict[str, Any], message: str) -> None:
    """Bad poses and bearings are rejected before any backend runs."""
    args: dict[str, Any] = {
        "relative_pose": POSE,
        "bearings1": _bearings(np.ones((3, 3))),
        "bearings2": _bearings(np.ones((3, 3))),
    }
    args.update(kwargs)
    with pytest.raises(InvalidInputError, match=message):
        triangulation.two_view(**args)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"observed": np.array([[True, False]])}, "two observing"),
        ({"bearings": np.ones((1, 2, 3))}, "unit"),
        ({"bearings": _bearings(np.ones((1, 3, 3)))}, r"\(P, 2, 3\)"),
        ({"observed": np.ones((1, 2), dtype=int)}, "booleans"),
        ({"poses": np.eye(4)}, r"\(V, 4, 4\)"),
    ],
)
def test_n_view_rejects_invalid_input(kwargs: dict[str, Any], message: str) -> None:
    """Bad shapes, masks and bearings are rejected before any backend runs."""
    args: dict[str, Any] = {
        "poses": np.stack([np.eye(4), POSE]),
        "bearings": _bearings(np.ones((1, 2, 3))),
    }
    args.update(kwargs)
    with pytest.raises(InvalidInputError, match=message):
        triangulation.n_view(**args)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS - {"python"}))
def test_core_rejects_what_the_entry_point_rejects(backend: str) -> None:
    """The C++ core checks its own input: a non-unit bearing fails there too."""
    impl = backends.implementation(backend, triangulation.__name__)
    with pytest.raises(InvalidInputError, match="unit"):
        impl.two_view(POSE, np.ones((2, 3)), np.ones((2, 3)))
    with pytest.raises(InvalidInputError, match="two observing"):
        impl.n_view(np.stack([np.eye(4), POSE]), np.ones((1, 2, 3)), np.array([[True, False]]))
