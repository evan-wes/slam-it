"""P3P tests: the true pose among the solutions, the solvers and backends agree, PoseLib."""

from __future__ import annotations

import numpy as np
import pytest

from slam_it_all import backends
from slam_it_all.foundations import p3p
from slam_it_all.foundations.errors import InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

SAMPLES = 200
"""Random exact samples per property test."""


def _distance(found: np.ndarray, expected: np.ndarray) -> float:
    """Return the closest solution's max-abs distance to `expected`."""
    return float(min((np.abs(f - expected).max() for f in found), default=np.inf))


def _sample(seed: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return three world points, their exact bearings, and the true ``T_cam_world``."""
    return sc.posed(np.random.default_rng(seed), 3)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_matches_fixture(backend: str) -> None:
    """Each backend finds the 50-digit true pose among its solutions."""
    chosen = require_function(backend, p3p.__name__, "p3p")
    spec = load_fixture("foundations/p3p.json")["functions"]["p3p"]
    for case in spec["cases"]:
        found = p3p.p3p(**case["input"], backend=chosen)
        for expected in case["expected"]:
            assert _distance(found, np.asarray(expected)) <= spec["tolerance"]["atol"], case["name"]


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("solver", p3p.SOLVERS)
def test_the_true_pose_is_among_the_solutions(backend: str, solver: str) -> None:
    """On exact data one solution is the true pose.

    Every solution is a rigid motion that puts each point at a positive depth along its bearing.
    """
    chosen = require_function(backend, p3p.__name__, "p3p")
    for seed in range(SAMPLES):
        world, rays, truth = _sample(seed)
        found = p3p.p3p(world, rays, solver=solver, backend=chosen)
        assert 1 <= len(found) <= 4
        assert _distance(found, truth) < 1e-9
        for pose in found:
            np.testing.assert_allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3), atol=1e-9)
            assert np.linalg.det(pose[:3, :3]) > 0.0
            in_camera = sc.in_camera(pose, world)
            assert np.all(np.einsum("ij,ij->i", in_camera, rays) > 0.0)
            np.testing.assert_allclose(sc.bearings(in_camera), rays, atol=1e-9)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_ding_and_lambda_twist_find_the_same_poses(backend: str) -> None:
    """Two routes through the same conic pencil: the same solutions, in any order."""
    chosen = require_function(backend, p3p.__name__, "p3p")
    for seed in range(SAMPLES):
        world, rays, _ = _sample(seed)
        ding = p3p.p3p(world, rays, solver="ding", backend=chosen)
        twist = p3p.p3p(world, rays, solver="lambda_twist", backend=chosen)
        assert len(ding) == len(twist)
        for pose in ding:
            assert _distance(twist, pose) < 1e-9


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize("solver", p3p.SOLVERS)
def test_backends_agree(backend: str, solver: str) -> None:
    """Every compiled core returns the reference's solutions, in the same order."""
    chosen = require_function(backend, p3p.__name__, "p3p")
    for seed in range(SAMPLES):
        world, rays, _ = _sample(seed)
        reference = p3p.p3p(world, rays, solver=solver)
        compiled = p3p.p3p(world, rays, solver=solver, backend=chosen)
        np.testing.assert_allclose(compiled, reference, atol=1e-10)


def test_poselib_solutions_are_among_ours() -> None:
    """Every pose PoseLib's P3P returns is one of ours, and the counts agree."""
    poselib = pytest.importorskip("poselib")
    for seed in range(SAMPLES):
        world, rays, _ = _sample(seed)
        ours = p3p.p3p(world, rays)
        theirs = poselib.p3p(rays, world)
        assert len(theirs) == len(ours)
        for pose in theirs:
            matrix = np.eye(4)
            matrix[:3, :3], matrix[:3, 3] = pose.R, pose.t
            assert _distance(ours, matrix) < 1e-9


def test_the_outside_cylinder_case_needs_both_lines(monkeypatch: pytest.MonkeyPatch) -> None:
    """The fixture's sample 1e-8 outside the cylinder still guards Ding's second line.

    PoseLib's shortcut skips the second line when the cubic has a single real root. On this
    sample the float64 cubic has one (its discriminant is positive) and the first line alone
    misses the true pose, so a solver with the shortcut fails the fixture. If a change to the
    sample or the solver loses either property, this test says so.
    """
    spec = load_fixture("foundations/p3p.json")["functions"]["p3p"]
    case = next(
        c for c in spec["cases"] if c["name"] == "camera 1e-08 outside the danger cylinder, ding"
    )
    truth = np.asarray(case["expected"][0])
    discriminants: list[float] = []
    root = p3p.cubic_root

    def recording(c2: float, c1: float, c0: float) -> float:
        p, q = c1 - c2 * c2 / 3.0, (2.0 * c2**3 - 9.0 * c2 * c1) / 27.0 + c0
        discriminants.append(q * q / 4.0 + p**3 / 27.0)
        return root(c2, c1, c0)

    monkeypatch.setattr(p3p, "cubic_root", recording)
    line_pair = p3p._line_pair

    def first_line(conic: np.ndarray) -> tuple[np.ndarray, ...] | None:
        lines = line_pair(conic)
        return None if lines is None else lines[:1]

    monkeypatch.setattr(p3p, "_line_pair", first_line)
    first_line_only = p3p.p3p(case["input"]["points"], case["input"]["bearings"], solver="ding")
    assert discriminants == [pytest.approx(discriminants[0])]
    assert discriminants[0] > 0.0
    assert _distance(first_line_only, truth) > 1e-2


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("solver", p3p.SOLVERS)
def test_collinear_points_give_no_solution(backend: str, solver: str) -> None:
    """Three points on a line leave the rotation about it free: no pose is returned."""
    chosen = require_function(backend, p3p.__name__, "p3p")
    world = np.array([[0.0, 0.0, 5.0], [1.0, 0.5, 6.0], [2.0, 1.0, 7.0]])
    found = p3p.p3p(world, sc.bearings(world), solver=solver, backend=chosen)
    assert found.shape == (0, 4, 4)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("solver", p3p.SOLVERS)
def test_coinciding_bearings_give_no_false_pose(backend: str, solver: str) -> None:
    """Keypoints detected at one pixel, matched to different points: a sample with no pose.

    Three coinciding bearings see three points that are not collinear: no depths satisfy the
    distance equations, so nothing is returned (the papers' closed form returned a matrix far
    from a rotation here). Two coinciding bearings must not fail on a rounding-negative scale.
    """
    chosen = require_function(backend, p3p.__name__, "p3p")
    # A sample PnP RANSAC drew on TUM fr1/xyz (points from depth): Ding's solver
    # returned two poses for it before the distance check.
    world = np.array(
        [
            [0.43377713272907875, 0.03152780954753013, 1.0192],
            [0.4391719361395056, -0.0018883592376698233, 0.9992],
            [0.43332093334344846, 0.023175747951696066, 1.0112],
        ]
    )
    ray = np.array([-0.26834769777954354, -0.18065317409694256, 0.9462314430334253])
    assert p3p.p3p(world, np.tile(ray, (3, 1)), solver=solver, backend=chosen).shape == (0, 4, 4)
    for seed in range(20):
        points, rays, _ = sc.posed(np.random.default_rng(seed), 3)
        rays[1] = rays[0]
        for pose in p3p.p3p(points, rays, solver=solver, backend=chosen):
            np.testing.assert_allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3), atol=1e-12)


@pytest.mark.parametrize(
    ("roots", "atol"),
    [
        ([2.0, -1.0, 0.5], 1e-12),
        ([-4.0], 1e-12),
        ([1e-3], 1e-12),
        ([5.0, 1.0, 0.9], 1e-12),
        # A double root is determined only to about the square root of the rounding error.
        ([3.0, 3.0, -1.0], 1e-7),
    ],
)
def test_cubic_root_is_the_largest_real_root(roots: list[float], atol: float) -> None:
    """The largest real root: of three, or the only one (beside a complex pair)."""
    if len(roots) == 1:
        poly = np.polynomial.polynomial.polymul(
            np.polynomial.polynomial.polyfromroots(roots), [2.0, -1.0, 1.0]
        )
    else:
        poly = np.polynomial.polynomial.polyfromroots(roots)
    c0, c1, c2, _ = poly / poly[3]
    assert p3p.cubic_root(c2, c1, c0) == pytest.approx(max(roots), abs=atol)


def test_quadratic_roots() -> None:
    """Two roots without cancellation; a double root despite rounding; a linear equation."""
    big, small = p3p.quadratic_roots(1.0, -1e8, 1.0)
    assert big == pytest.approx(1e8)
    assert small == pytest.approx(1e-8, rel=1e-15)
    assert p3p.quadratic_roots(1.0, -2.0, 1.0 + 1e-15) == pytest.approx([1.0])
    assert p3p.quadratic_roots(1.0, 0.0, 1.0) == []
    assert p3p.quadratic_roots(0.0, 2.0, -1.0) == [0.5]


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_the_compiled_solver_codes_match(backend: str) -> None:
    """The solver codes Python sends are each compiled core's."""
    chosen = require_function(backend, p3p.__name__, "p3p")
    codes = backends.implementation(chosen, p3p.__name__).codes()
    assert tuple(codes["solver"]) == p3p.SOLVERS


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_rejects_invalid_input(backend: str) -> None:
    """Other than three finite points and unit bearings, or an unknown solver, is rejected."""
    chosen = require_function(backend, p3p.__name__, "p3p")
    world, rays, _ = _sample(0)
    with pytest.raises(InvalidInputError, match="solver"):
        p3p.p3p(world, rays, solver="grunert", backend=chosen)
    with pytest.raises(InvalidInputError, match="exactly 3"):
        p3p.p3p(np.vstack([world, world[:1]]), np.vstack([rays, rays[:1]]), backend=chosen)
    with pytest.raises(InvalidInputError, match="unit"):
        p3p.p3p(world, 2.0 * rays, backend=chosen)
    with pytest.raises(InvalidInputError, match="finite"):
        p3p.p3p(np.where(np.eye(3, dtype=bool), np.nan, world), rays, backend=chosen)
