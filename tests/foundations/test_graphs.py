"""Factor-graph tests: the factors' Jacobians, and each solve recovering a known answer."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import gtsam
import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.foundations import camera, graphs, se3
from slam_it.foundations.errors import InvalidInputError, NotConvergedError
from tests.conftest import assert_matches_fixture, load_fixture, parity_params, require_function

Array = NDArray[np.float64]

PINHOLE = ("pinhole", np.array([520.0, 515.0, 320.0, 240.0]))
RADTAN = (
    "radtan",
    np.array([517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633]),
)
CAMERAS = {"pinhole": PINHOLE, "radtan": RADTAN}
BF = 40.0
"""A stereo rig's baseline times its focal length, in pixel-metres."""

STEP = 1e-6
"""The central differences' step."""


def _numeric(function: Callable[[Array], Array], dimension: int) -> Array:
    """Differentiate `function` at 0 by central differences, one column per coordinate."""
    columns = []
    for k in range(dimension):
        delta = np.zeros(dimension)
        delta[k] = STEP
        columns.append((function(delta) - function(-delta)) / (2.0 * STEP))
    return np.stack(columns, axis=-1)


def _pose(seed: int) -> Array:
    """Return a camera-to-world pose near the origin, looking down +z at the scene."""
    rng = np.random.default_rng(seed)
    return se3.exp(np.concatenate([rng.normal(scale=0.3, size=3), rng.normal(scale=0.1, size=3)]))


def _similarity(rotation: Array, translation: Array, scale: float) -> Array:
    """Return ``[[s R, t], [0, 1]]``."""
    out = np.eye(4)
    out[:3, :3] = scale * rotation
    out[:3, 3] = translation
    return out


def test_a_similarity_crosses_to_gtsam_and_back() -> None:
    """``[[s R, t], [0, 1]]`` maps p to s R p + t, as its GTSAM ``Similarity3`` does."""
    matrix = _similarity(
        se3.exp([0, 0, 0, 0.3, -0.2, 0.5])[:3, :3], np.array([1.0, -2.0, 0.5]), 1.7
    )
    value = graphs._to_similarity3(matrix)
    point = np.array([0.3, -0.4, 2.0])
    np.testing.assert_allclose(value.transformFrom(point), matrix[:3, :3] @ point + matrix[:3, 3])
    np.testing.assert_allclose(graphs._from_similarity3(value), matrix, rtol=0, atol=1e-14)


@pytest.mark.parametrize("model", CAMERAS)
@pytest.mark.parametrize("stereo", [False, True])
def test_the_projection_factor_s_jacobians_are_its_derivatives(model: str, stereo: bool) -> None:
    """On the pose's tangent, perturbed on the right as GTSAM retracts it, and on the point."""
    name, intrinsics = CAMERAS[model]
    pose = _pose(1)
    point = pose[:3, :3] @ np.array([0.2, -0.1, 2.5]) + pose[:3, 3]
    measured = np.array([300.0, 250.0, 290.0]) if stereo else np.array([300.0, 250.0])
    projected = graphs.projection_residual(pose, point, measured, name, intrinsics, BF)
    assert projected.in_front

    def on_pose(delta: Array) -> Array:
        moved = np.asarray(gtsam.Pose3(pose).retract(delta).matrix())
        return graphs.projection_residual(moved, point, measured, name, intrinsics, BF).residual

    def on_point(delta: Array) -> Array:
        return graphs.projection_residual(
            pose, point + delta, measured, name, intrinsics, BF
        ).residual

    np.testing.assert_allclose(projected.d_pose, _numeric(on_pose, 6), rtol=1e-6, atol=1e-5)
    np.testing.assert_allclose(projected.d_point, _numeric(on_point, 3), rtol=1e-6, atol=1e-5)


@pytest.mark.parametrize("model", CAMERAS)
@pytest.mark.parametrize("fixed_scale", [False, True])
def test_the_similarity_factors_jacobians_are_their_derivatives(
    model: str, fixed_scale: bool
) -> None:
    """Both directions, on the similarity's tangent (or the rigid motion's), on the right."""
    name, intrinsics = CAMERAS[model]
    scale = 1.0 if fixed_scale else 1.3
    motion = _similarity(
        se3.exp([0, 0, 0, 0.05, -0.1, 0.02])[:3, :3], np.array([0.1, -0.05, 0.2]), scale
    )
    point2 = np.array([0.3, -0.2, 2.4])
    point1 = motion[:3, :3] @ point2 + motion[:3, 3] + np.array([0.01, -0.02, 0.03])
    pixel1, pixel2 = np.array([350.0, 220.0]), np.array([330.0, 230.0])
    both = graphs.similarity_residuals(
        motion, point1, point2, pixel1, pixel2, name, intrinsics, fixed_scale
    )
    assert both.in_front1
    assert both.in_front2

    def moved(delta: Array) -> Array:
        if fixed_scale:
            return np.asarray(gtsam.Pose3(motion).retract(delta).matrix())
        return graphs._from_similarity3(graphs._to_similarity3(motion).retract(delta))

    def side(first: bool) -> Callable[[Array], Array]:
        def residual(delta: Array) -> Array:
            at = graphs.similarity_residuals(
                moved(delta), point1, point2, pixel1, pixel2, name, intrinsics, fixed_scale
            )
            return at.residual1 if first else at.residual2

        return residual

    dimension = graphs.POSE_DIM if fixed_scale else graphs.SIMILARITY_DIM
    np.testing.assert_allclose(
        both.d_similarity1, _numeric(side(True), dimension), rtol=1e-6, atol=1e-5
    )
    np.testing.assert_allclose(
        both.d_similarity2, _numeric(side(False), dimension), rtol=1e-6, atol=1e-5
    )


def test_a_point_behind_its_camera_costs_a_constant() -> None:
    """In BA, twice the focal length in each row with zero Jacobians, GTSAM's own factors' cost.

    In a refinement, a carried point behind its camera contributes nothing, and its chi2 is
    infinite.
    """
    name, intrinsics = PINHOLE
    behind = graphs.projection_residual(
        np.eye(4), np.array([0.0, 0.0, -1.0]), np.array([1.0, 2.0, 3.0]), name, intrinsics, BF
    )
    assert not behind.in_front
    np.testing.assert_array_equal(behind.residual, np.full(3, 2.0 * intrinsics[0]))
    for block in (behind.d_pose, behind.d_point):
        assert not block.any()
    both = graphs.similarity_residuals(
        np.eye(4), np.array([0.0, 0.0, 2.0]), np.array([0.0, 0.0, -2.0]), np.zeros(2),
        np.zeros(2), name, intrinsics, False,
    )  # fmt: skip
    assert (both.in_front1, both.in_front2) == (False, True)
    assert not both.residual1.any()
    assert not both.d_similarity1.any()


def _ring(count: int, similarity: bool) -> tuple[Array, NDArray[np.int64], Array]:
    """Return a ring of camera-to-world poses, its edges round it, and each edge's relative.

    With `similarity`, each pose also carries a scale, as a monocular map's drift does.
    """
    poses = []
    for k in range(count):
        angle = 2.0 * np.pi * k / count
        pose = se3.exp([3.0 * np.cos(angle), 3.0 * np.sin(angle), 0.0, 0.0, 0.0, angle])
        scale = 1.0 + 0.1 * k if similarity else 1.0
        poses.append(_similarity(pose[:3, :3], pose[:3, 3], scale))
    truth = np.stack(poses)
    edges = np.array([(k, (k + 1) % count) for k in range(count)], dtype=np.int64)
    relatives = np.stack([np.linalg.inv(truth[i]) @ truth[j] for i, j in edges])
    return truth, edges, relatives


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("similarity", [False, True])
def test_a_pose_graph_closes_its_ring(backend: str, similarity: bool) -> None:
    """From poses that drift, the ring's own relatives bring every pose back; the fixed holds."""
    chosen = require_function(backend, graphs.__name__, "optimize_pose_graph")
    truth, edges, relatives = _ring(8, similarity)
    start = truth.copy()
    for k in range(1, 8):  # drift growing round the ring
        start[k] = start[k] @ se3.exp([0.05 * k, -0.03 * k, 0.02, 0.01 * k, 0.0, -0.02 * k])
        if similarity:
            start[k][:3, :3] *= 1.0 + 0.02 * k
    result = graphs.optimize_pose_graph(
        start, edges, relatives, [0], similarity=similarity,
        options=graphs.Options(max_iterations=30), backend=chosen,
    )  # fmt: skip
    np.testing.assert_array_equal(result.poses[0], truth[0])
    np.testing.assert_allclose(result.poses, truth, rtol=0, atol=1e-8)
    assert result.summary.final_error < 1e-15 < result.summary.initial_error


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("similarity", [False, True])
def test_held_poses_are_constants_and_an_edge_between_two_costs_a_constant(
    backend: str, similarity: bool
) -> None:
    """Held poses are constants: each comes back bit for bit, the free ones to the truth.

    An edge with a held end pulls its free end alone, from either side; one joining two held
    poses costs a constant, counted in the errors; with every pose held, nothing is solved.
    """
    chosen = require_function(backend, graphs.__name__, "optimize_pose_graph")
    truth, _, _ = _ring(4, similarity)
    edges = np.array([[0, 1], [1, 2], [3, 1], [2, 3]])  # both held; held first; second; free
    relatives = np.stack([np.linalg.inv(truth[i]) @ truth[j] for i, j in edges])
    relatives[0] = relatives[0] @ se3.exp([0.1, 0.0, 0.0, 0.0, 0.0, 0.05])  # costs a constant
    start = truth.copy()
    start[2] = start[2] @ se3.exp([0.05, -0.03, 0.02, 0.01, 0.0, -0.02])
    start[3] = start[3] @ se3.exp([-0.04, 0.02, 0.01, 0.0, 0.02, 0.01])

    def solve(poses: Array, fixed: list[int]) -> graphs.PoseGraph:
        return graphs.optimize_pose_graph(
            poses, edges, relatives, fixed, similarity=similarity,
            options=graphs.Options(max_iterations=30), backend=chosen,
        )  # fmt: skip

    result, every = solve(start, [0, 1]), solve(truth, [0, 1, 2, 3])
    np.testing.assert_array_equal(result.poses[:2], start[:2])
    np.testing.assert_allclose(result.poses[2:], truth[2:], rtol=0, atol=1e-8)
    np.testing.assert_array_equal(every.poses, truth)
    held = every.summary
    assert (held.iterations, held.termination, held.converged) == (0, "function", True)
    assert held.final_error == held.initial_error > 1e-4
    assert result.summary.final_error == pytest.approx(held.initial_error, rel=1e-9)
    assert result.summary.initial_error > result.summary.final_error


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_an_edge_between_held_poses_costs_its_error_inside_gtsam_too(backend: str) -> None:
    """A free pose in no edge: the one iteration lowers nothing, and the solve converges.

    GTSAM's own search over dampings stops when a failed step changes the error by less than its
    relative tolerance times the error, which must hold the held edge's (0.3.17.2's review panel,
    L1): left out, the search went on to the damping's limit, not converged.
    """
    chosen = require_function(backend, graphs.__name__, "optimize_pose_graph")
    truth = np.stack(
        [se3.exp([1.0 * k, 0.2 * np.sin(k), 0, 0, 0, 2 * np.pi * k / 3]) for k in range(3)]
    )
    relative = np.linalg.inv(truth[0]) @ truth[1] @ se3.exp([0.1, 0, 0, 0, 0, 0.05])
    result = graphs.optimize_pose_graph(
        truth, [[0, 1]], relative[None], [0, 1], similarity=False,
        options=graphs.Options(max_iterations=10), backend=chosen,
    )  # fmt: skip
    summary = result.summary
    assert (summary.iterations, summary.termination, summary.converged) == (1, "function", True)
    assert summary.final_error == summary.initial_error > 1e-3


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("fixed_scale", [False, True])
def test_a_similarity_is_refined_its_outlier_dropped_and_refined_again(
    backend: str, fixed_scale: bool
) -> None:
    """ORB-SLAM's two rounds: a wrong match stands out, and without it the motion is exact.

    The first round's Huber loss keeps the wrong match from pulling the motion far; the second,
    on the inliers, has none, as ORB-SLAM's.
    """
    chosen = require_function(backend, graphs.__name__, "refine_similarity")
    name, intrinsics = PINHOLE
    rng = np.random.default_rng(3)
    scale = 1.0 if fixed_scale else 0.8
    rotation = se3.exp([0, 0, 0, 0.05, 0.1, -0.03])[:3, :3]
    truth = _similarity(rotation, np.array([0.2, -0.1, 0.1]), scale)
    points2 = np.column_stack(
        [rng.uniform(-1, 1, 40), rng.uniform(-1, 1, 40), rng.uniform(2, 5, 40)]
    )
    points1 = points2 @ truth[:3, :3].T + truth[:3, 3]
    pixels1 = camera.project(name, intrinsics, points1)
    pixels2 = camera.project(name, intrinsics, points2)
    pixels1[0] += [40.0, -35.0]  # one wrong match

    def refine(
        start: Array, keep: NDArray[np.bool_], huber: float | None
    ) -> graphs.SimilarityRefinement:
        return graphs.refine_similarity(
            start, points1[keep], points2[keep], pixels1[keep], pixels2[keep],
            np.ones((int(keep.sum()), 2)), model=name, intrinsics=intrinsics,
            fixed_scale=fixed_scale, huber=huber, options=graphs.Options(max_iterations=20),
            backend=chosen,
        )  # fmt: skip

    start = truth @ se3.exp([0.02, -0.02, 0.01, 0.01, -0.01, 0.02])
    first = refine(start, np.ones(40, bool), np.sqrt(10.0))
    np.testing.assert_allclose(first.similarity, truth, rtol=0, atol=1e-2)
    assert first.chi2[0, 0] > 1000.0
    keep = np.all(first.chi2 <= 10.0, axis=1)
    assert keep.tolist() == [False] + [True] * 39
    second = refine(first.similarity, keep, None)
    np.testing.assert_allclose(second.similarity, truth, rtol=0, atol=1e-12)
    assert second.summary.final_error < 1e-20  # exact data: down to rounding
    assert np.all(second.chi2 < 1e-12)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_match_image_2_does_not_observe_counts_its_image_1_error_alone(backend: str) -> None:
    """A NaN row of pixels2: no image-2 error, its chi2 NaN; the motion still found."""
    chosen = require_function(backend, graphs.__name__, "refine_similarity")
    name, intrinsics = PINHOLE
    rng = np.random.default_rng(4)
    truth = _similarity(
        se3.exp([0, 0, 0, 0.02, -0.05, 0.03])[:3, :3], np.array([0.1, 0.0, -0.1]), 1.2
    )
    points2 = np.column_stack(
        [rng.uniform(-1, 1, 30), rng.uniform(-1, 1, 30), rng.uniform(2, 5, 30)]
    )
    points1 = points2 @ truth[:3, :3].T + truth[:3, 3]
    pixels1 = camera.project(name, intrinsics, points1)
    pixels2 = camera.project(name, intrinsics, points2)
    pixels2[::3] = np.nan  # a third of the matches unseen in image 2
    result = graphs.refine_similarity(
        truth @ se3.exp([0.01, 0.0, -0.01, 0.01, 0.01, 0.0]), points1, points2, pixels1, pixels2,
        np.ones((30, 2)), model=name, intrinsics=intrinsics, fixed_scale=False,
        huber=np.sqrt(10.0), options=graphs.Options(max_iterations=20), backend=chosen,
    )  # fmt: skip
    np.testing.assert_allclose(result.similarity, truth, rtol=0, atol=1e-10)
    assert np.isnan(result.chi2[::3, 1]).all()
    assert np.isfinite(result.chi2[:, 0]).all()


def _scene(seed: int) -> tuple[Array, Array]:
    """Five cameras along x looking down +z, and 60 points in front of them."""
    rng = np.random.default_rng(seed)
    poses = np.stack([se3.exp([0.4 * k, 0.0, 0.0, 0.0, 0.05 * k, 0.0]) for k in range(5)])
    points = np.column_stack(
        [rng.uniform(-1, 3, 60), rng.uniform(-1, 1, 60), rng.uniform(3, 6, 60)]
    )
    return poses, points


def _observed(
    name: str, intrinsics: Array, poses: Array, points: Array, *, stereo: bool
) -> tuple[list[int], list[int], Array, Array]:
    """Return every camera's view of every point: cameras, points, pixels and right columns.

    With `stereo`, every second point has its right column; NaN otherwise.
    """
    cameras, ids, pixels, right = [], [], [], []
    for c, pose in enumerate(poses):
        local = (points - pose[:3, 3]) @ pose[:3, :3]
        seen = camera.project(name, intrinsics, local)
        for p in range(points.shape[0]):
            cameras.append(c)
            ids.append(p)
            pixels.append(seen[p])
            right.append(seen[p, 0] - BF / local[p, 2] if stereo and p % 2 else np.nan)
    return cameras, ids, np.array(pixels), np.array(right)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("lens", [RADTAN, PINHOLE], ids=["radtan", "pinhole"])
@pytest.mark.parametrize("held", [1, 3, 5], ids=["one-held", "three-held", "every-one-held"])
def test_bundle_adjustment_recovers_a_stereo_scene(
    backend: str, lens: tuple[str, Array], held: int
) -> None:
    """Stereo observations fix the scale: perturbed cameras and points return to the truth.

    Through radtan, Python's factors are its own; through a pinhole camera, GTSAM's. The held
    cameras are constants, through every factor a held camera's observation makes (monocular and
    stereo, pinhole and radtan): each comes back bit for bit as given, with one, three or all
    five held, when the points alone are adjusted.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    name, intrinsics = lens
    poses, points = _scene(5)
    cameras, ids, pixels, right = _observed(name, intrinsics, poses, points, stereo=True)
    rng = np.random.default_rng(6)
    start_poses = np.stack(
        [poses[0]] + [pose @ se3.exp(rng.normal(scale=0.02, size=6)) for pose in poses[1:]]
    )
    start_poses[:held] = poses[:held]
    start_points = points + rng.normal(scale=0.05, size=points.shape)
    result = graphs.bundle_adjust(
        start_poses, start_points, cameras, ids, pixels, right, np.ones(len(cameras)),
        model=name, intrinsics=intrinsics, bf=BF, fixed=list(range(held)),
        options=graphs.Options(max_iterations=20), backend=chosen,
    )  # fmt: skip
    np.testing.assert_array_equal(result.poses[:held], poses[:held])
    np.testing.assert_allclose(result.poses, poses, rtol=0, atol=1e-7)
    np.testing.assert_allclose(result.points, points, rtol=0, atol=1e-6)
    assert result.summary.converged or result.summary.final_error < 1e-12


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("lens", [RADTAN, PINHOLE], ids=["radtan", "pinhole"])
@pytest.mark.parametrize("right", [np.nan, 300.0], ids=["mono", "stereo"])
def test_bundle_adjustment_charges_a_point_behind_its_camera(
    backend: str, lens: tuple[str, Array], right: float
) -> None:
    """A point behind its camera costs ``(2 f_x)²`` a row, half summed: GTSAM's own factors' cost.

    With no gradient there, the point stays where it is; through radtan the factor is each
    language's own, through a pinhole camera Python's is GTSAM's.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    name, intrinsics = lens
    rows = 2 if np.isnan(right) else 3
    result = graphs.bundle_adjust(
        np.stack([np.eye(4), np.eye(4)]), np.array([[0.1, 0.2, -1.0]]), [0], [0],
        [[300.0, 200.0]], [right], [1.0], model=name, intrinsics=intrinsics, bf=BF,
        fixed=[0, 1], options=graphs.Options(max_iterations=5), backend=chosen,
    )  # fmt: skip
    cost = 0.5 * rows * (2.0 * intrinsics[0]) ** 2
    assert result.summary.initial_error == pytest.approx(cost, rel=1e-12)
    assert result.summary.final_error == pytest.approx(cost, rel=1e-12)
    np.testing.assert_array_equal(result.points, [[0.1, 0.2, -1.0]])


def test_a_held_stereo_camera_charges_a_point_behind_it_as_gtsam_s_stereo_factor() -> None:
    """Python's held stereo factor copies GTSAM's cheirality convention: pinned to the factor.

    At a point behind the camera, its error, its right-hand side and its Jacobian on the point
    equal ``GenericStereoFactor3D``'s with the pose a variable (0.3.17.2's review panel, L7).
    """
    build = graphs._pinhole_factors(PINHOLE[1], BF)
    pose = se3.exp([0.1, 0.2, 0.3, 0.0, 0.1, 0.0])
    noise = gtsam.noiseModel.Isotropic.Precision(3, 0.7)
    measured = np.array([300.0, 290.0, 250.0])
    held, free = build(measured, noise, pose, 1), build(measured, noise, 0, 1)
    values = gtsam.Values()
    values.insert(0, gtsam.Pose3(pose))
    values.insert(1, pose[:3, :3] @ np.array([0.1, -0.2, -1.0]) + pose[:3, 3])  # behind it
    assert held.error(values) == free.error(values) > 0.0
    a_held, b_held = held.linearize(values).jacobian()
    a_free, b_free = free.linearize(values).jacobian()
    np.testing.assert_array_equal(b_held, b_free)
    np.testing.assert_array_equal(a_held, a_free[:, graphs.POSE_DIM :])


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_huber_loss_holds_bundle_adjustment_against_an_outlier(backend: str) -> None:
    """One observation 40 px off: with Huber's loss the cameras move a tenth as far.

    Each compiled backend's solve matches Python's to the solves' convergence.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    name, intrinsics = PINHOLE
    poses, points = _scene(7)
    cameras, ids, pixels, right = _observed(name, intrinsics, poses, points, stereo=False)
    pixels[2 * points.shape[0]] += 40.0  # camera 2's view of point 0

    def solve(huber: Array | None, which: str) -> graphs.Adjustment:
        return graphs.bundle_adjust(
            poses, points, cameras, ids, pixels, right, np.ones(len(cameras)), model=name,
            intrinsics=intrinsics, bf=BF, fixed=[0, 1], huber=huber,
            options=graphs.Options(max_iterations=20), backend=which,
        )  # fmt: skip

    robust = np.full(len(cameras), np.sqrt(5.991))
    plain, held = solve(None, chosen), solve(robust, chosen)
    assert np.abs(held.poses - poses).max() < np.abs(plain.poses - poses).max() / 10
    reference = solve(robust, "python")
    np.testing.assert_allclose(held.poses, reference.poses, rtol=0, atol=1e-7)
    np.testing.assert_allclose(held.points, reference.points, rtol=0, atol=1e-7)
    assert held.summary.iterations == reference.summary.iterations


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_the_huber_loss_acts_on_a_stereo_observation_s_whitened_norm(backend: str) -> None:
    """A known answer: one stereo observation, its whitened error (3, 4, 0), k = √7.815.

    The loss acts on the error's norm, as g2o's on chi² (GTSAM's ``Block`` reweighting):
    ``k·5 - k²/2`` = 10.0702, where acting on each component would give 11.7537 and no loss
    12.5. The camera, at the origin, is held; the point, 2 m ahead, is seen 3 px right and 4 px
    down of where it projects, its right column where it projects.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    k = np.sqrt(7.815)
    fx, fy, cx, cy = 500.0, 500.0, 320.0, 240.0
    bf = 50.0
    point = np.array([[0.0, 0.0, 2.0]])
    right = cx - bf / point[0, 2]  # where the point projects in the right image
    result = graphs.bundle_adjust(
        np.eye(4)[None], point, [0], [0], np.array([[cx + 3.0, cy + 4.0]]), np.array([right]),
        np.ones(1), model="pinhole", intrinsics=np.array([fx, fy, cx, cy]), bf=bf, fixed=[0],
        huber=np.array([k]), options=graphs.Options(max_iterations=1), backend=chosen,
    )  # fmt: skip
    assert result.summary.initial_error == pytest.approx(k * 5.0 - k * k / 2.0, rel=1e-12)
    assert result.summary.initial_error == pytest.approx(10.0702, abs=5e-5)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_solve_stops_by_its_iterations_its_tolerances_or_its_damping(backend: str) -> None:
    """Capped; converged by the relative tolerance; and, at a minimum, no step lowers the error."""
    chosen = require_function(backend, graphs.__name__, "optimize_pose_graph")
    truth, edges, relatives = _ring(6, False)
    relatives[0] = relatives[0] @ se3.exp([0.1, 0.0, 0.0, 0.0, 0.0, 0.05])  # inconsistent: no zero
    start = np.stack([pose @ se3.exp([0.1, 0.1, 0, 0, 0, 0.1]) for pose in truth])
    start[0] = truth[0]

    def solve(poses: Array, options: graphs.Options) -> graphs.PoseGraph:
        return graphs.optimize_pose_graph(
            poses, edges, relatives, [0], similarity=False, options=options, backend=chosen
        )

    capped = solve(start, graphs.Options(max_iterations=1))
    assert (capped.summary.iterations, capped.summary.termination) == (1, "max_iterations")
    assert not capped.summary.converged
    tolerant = solve(start, graphs.Options(max_iterations=50, relative_tolerance=1e-6))
    assert tolerant.summary.termination == graphs.Termination.FUNCTION
    assert tolerant.summary.converged
    at_minimum = solve(tolerant.poses, graphs.Options(max_iterations=50, relative_tolerance=0.0))
    assert at_minimum.summary.termination == graphs.Termination.DAMPING_LIMIT
    # Rounding may still lower it a little before no step can.
    assert at_minimum.summary.final_error == pytest.approx(
        at_minimum.summary.initial_error, rel=1e-9
    )


HELD_PAIR = np.stack([np.eye(4), se3.exp([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])])
"""Two held cameras 0.3 m apart along x, looking down +z: a point alone is solved."""

FAR_POINT = (np.array([0.2, -0.1, 4.0]), np.array([0.2, -0.1, 12.0]))
"""A point 4 m ahead and a start three times as far: GTSAM's first trial steps fail there."""


def _seen_from_held_pair() -> Array:
    """Return where the held pair's cameras see FAR_POINT's truth."""
    name, intrinsics = PINHOLE
    truth = FAR_POINT[0]
    return np.stack(
        [
            camera.project(name, intrinsics, ((truth - pose[:3, 3]) @ pose[:3, :3])[None])[0]
            for pose in HELD_PAIR
        ]
    )


def _point_residuals(point: Array) -> tuple[Array, Array]:
    """Return the held pair's residuals at `point` (unit information) and their Jacobian."""
    rows, blocks = [], []
    for pose, seen in zip(HELD_PAIR, _seen_from_held_pair(), strict=True):
        at = graphs.projection_residual(pose, point, seen, *PINHOLE, BF)
        rows.append(at.residual)
        blocks.append(at.d_point)
    return np.concatenate(rows), np.vstack(blocks)


def _gtsam_iteration(
    point: Array, damping: float, factor: float, *, nielsen: bool, limit: float
) -> tuple[Array, float, float]:
    """One of GTSAM's Levenberg-Marquardt iterations on the held pair's point, emulated.

    ``LevenbergMarquardtOptimizer::tryLambda``'s search: each trial solves ``(H + λI) δ = -g``,
    and is accepted when the error falls with a gain ratio over ``minModelFidelity`` (1e-3); a
    failed trial multiplies ``λ`` by the factor, which Nielsen's schedule then doubles; an
    accepted one divides ``λ`` by the factor, or with Nielsen's schedule multiplies it by
    ``max(1/3, 1 - (2ρ - 1)³)`` and resets the factor to 2. Returns the point, ``λ`` and the
    factor after the iteration.
    """
    residual, jacobian = _point_residuals(point)
    error = 0.5 * residual @ residual
    hessian, gradient = jacobian.T @ jacobian, jacobian.T @ residual
    while damping < limit:
        step = -np.linalg.solve(hessian + damping * np.eye(3), gradient)
        linear = residual + jacobian @ step
        predicted = error - 0.5 * linear @ linear
        moved, _ = _point_residuals(point + step)
        ratio = float((error - 0.5 * moved @ moved) / predicted)
        if ratio > 1e-3:
            if nielsen:
                return point + step, damping * max(1 / 3, 1 - (2 * ratio - 1) ** 3), 2.0
            return point + step, damping / factor, factor
        damping *= factor
        factor *= 2.0 if nielsen else 1.0
    return point, damping, factor


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("scaled", [True, False], ids=["scaled", "absolute"])
def test_the_damping_starts_and_moves_as_its_schedule_says(backend: str, scaled: bool) -> None:
    """Scaled: ``λ`` from ``τ`` times the largest diagonal entry, by Nielsen's schedule; or GTSAM's.

    One point, two held cameras, the point started three times too far: each iteration's result,
    one to four, is the emulated search's (:func:`_gtsam_iteration`). The scaled damping's first
    iteration fails three trials, at ``λ0``, ``2 λ0`` and ``8 λ0``, before ``64 λ0`` steps, and its
    later ones follow the gain ratio; GTSAM's own starts at ``τ`` and moves by 10. The field the
    other damping reads is set far off (1.0), so each solve shows it reads its own alone.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    start = FAR_POINT[1]
    tau = 1e-5
    _, jacobian = _point_residuals(start)
    largest = float(np.max(np.diag(jacobian.T @ jacobian)))
    damping, factor = (tau * largest, 2.0) if scaled else (tau, graphs.ABSOLUTE_DAMPING_FACTOR)
    limit = graphs.SCALED_DAMPING_LIMIT if scaled else graphs.ABSOLUTE_DAMPING_LIMIT
    expected = start
    for iterations in range(1, 5):
        expected, damping, factor = _gtsam_iteration(
            expected, damping, factor, nielsen=scaled, limit=limit
        )
        result = graphs.bundle_adjust(
            HELD_PAIR, start[None], [0, 1], [0, 0], _seen_from_held_pair(), [np.nan, np.nan],
            [1.0, 1.0], model=PINHOLE[0], intrinsics=PINHOLE[1], bf=BF, fixed=[0, 1],
            options=graphs.Options(
                max_iterations=iterations,
                initial_damping=1.0 if scaled else tau,
                damping_scale=tau if scaled else 1.0,
                scaled_damping=scaled,
            ),
            backend=chosen,
        )  # fmt: skip
        np.testing.assert_allclose(result.points[0], expected, rtol=0, atol=1e-9)
        assert result.summary.iterations == iterations


def _stereo_start() -> dict[str, Any]:
    """Return the stereo scene's bundle from a perturbed start, its pixels noisy."""
    name, intrinsics = PINHOLE
    poses, points = _scene(5)
    cameras, ids, pixels, right = _observed(name, intrinsics, poses, points, stereo=True)
    rng = np.random.default_rng(6)
    start = np.stack(
        [poses[0]] + [pose @ se3.exp(rng.normal(scale=0.02, size=6)) for pose in poses[1:]]
    )
    return {
        "poses": start, "points": points + rng.normal(scale=0.05, size=points.shape),
        "cameras": cameras, "point_ids": ids,
        "pixels": pixels + rng.normal(scale=0.5, size=pixels.shape), "right": right,
        "information": np.ones(len(cameras)), "model": name, "intrinsics": intrinsics, "bf": BF,
        "fixed": [0],
    }  # fmt: skip


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_small_steps_in_a_row_end_a_solve_and_a_large_one_resets_their_count(
    backend: str,
) -> None:
    """ORB-SLAM3's stop: ``small_steps`` decreases in a row under ``small_decrease`` end it.

    A heavily damped start (``τ`` = 3000) makes the decreases grow from small, through large,
    then shrink as the solve converges. Each iteration's error is read from solves capped there;
    ``small_decrease`` is set between the second decrease and the third, so the first two count
    as small and the third resets them: the solve ends at the third small decrease in a row after
    it, converged (:attr:`~slam_it.foundations.graphs.Termination.SMALL_STEPS`), its error the
    capped solve's; without the reset it would end at the first small decrease after the third.
    With every decrease small, it ends after three iterations.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    bundle = _stereo_start()

    def solve(**changes: Any) -> graphs.Adjustment:  # noqa: ANN401 -- Options' fields
        options = graphs.Options(scaled_damping=True, damping_scale=3000.0, **changes)
        return graphs.bundle_adjust(**bundle, options=options, backend=chosen)

    errors = [solve(max_iterations=1).summary.initial_error]
    errors += [solve(max_iterations=k).summary.final_error for k in range(1, 31)]
    decrease = [(errors[k - 1] - errors[k]) / errors[k - 1] for k in range(1, len(errors))]
    threshold = float(np.sqrt(decrease[1] * decrease[2]))
    small = [d < threshold for d in decrease]
    assert small[:3] == [True, True, False]  # two small, then one that resets them
    ends = next(k for k in range(3, len(small)) if all(small[k - 2 : k + 1])) + 1
    without_reset = next(k for k in range(3, len(small)) if small[k]) + 1
    assert ends > without_reset  # the reset decides where the solve ends
    stopped = solve(max_iterations=30, small_steps=3, small_decrease=threshold)
    assert (stopped.summary.iterations, stopped.summary.termination) == (ends, "small_steps")
    assert stopped.summary.converged
    assert stopped.summary.final_error == pytest.approx(errors[ends], rel=1e-12)
    three = solve(max_iterations=30, small_steps=3, small_decrease=0.9)
    assert (three.summary.iterations, three.summary.termination) == (3, "small_steps")


def _depths_of(
    poses: NDArray[np.float64], points: NDArray[np.float64], bundle: dict[str, Any]
) -> NDArray[np.float64]:
    """Return each of a bundle's observations' depth, its point along its camera's optical axis."""
    cameras, ids = np.array(bundle["cameras"]), np.array(bundle["point_ids"])
    return np.asarray(
        np.einsum("mi,mi->m", poses[cameras, :3, 2], points[ids] - poses[cameras, :3, 3]),
        dtype=np.float64,
    )


def _near_and_behind() -> tuple[dict[str, Any], NDArray[np.bool_]]:
    """Return a bundle with observations near and behind their cameras, and which are kept.

    The stereo scene's five cameras and 60 points, monocular, camera 0 held, a sixth camera 2 m
    ahead of them, and two more points: point 60, 1 mm in front of camera 5 (under 1e-3 of the
    median depth) and seen also by cameras 0 and 4, 2 m from them; point 61, 1 m behind camera 0,
    seen by it, camera 1 and camera 5, behind each. Every observation of point 61 and camera 5
    is left out, so neither is solved.
    """
    name, intrinsics = PINHOLE
    poses, points = _scene(5)
    poses = np.concatenate([poses, se3.exp([0.8, 0.0, 2.0, 0.0, 0.0, 0.0])[None]])
    near = poses[5][:3, :3] @ np.array([0.0, 0.0, 1e-3]) + poses[5][:3, 3]
    behind = poses[0][:3, :3] @ np.array([0.1, 0.0, -1.0]) + poses[0][:3, 3]
    points = np.concatenate([points, near[None], behind[None]])
    cameras, ids, pixels, _ = _observed(name, intrinsics, poses[:5], points[:60], stereo=False)
    extra = [(5, 60), (0, 60), (4, 60), (0, 61), (1, 61), (5, 61)]
    cameras += [c for c, _ in extra]
    ids += [p for _, p in extra]
    pixels = np.concatenate([pixels, np.full((len(extra), 2), 300.0)])
    kept = np.ones(len(cameras), dtype=bool)
    kept[[len(cameras) - 6, len(cameras) - 3, len(cameras) - 2, len(cameras) - 1]] = False
    rng = np.random.default_rng(8)
    bundle: dict[str, Any] = {
        "poses": np.stack([poses[0]] + [p @ se3.exp(rng.normal(0, 0.01, 6)) for p in poses[1:]]),
        "points": points, "cameras": cameras, "point_ids": ids, "pixels": pixels,
        "right": np.full(len(cameras), np.nan), "information": np.ones(len(cameras)),
        "model": name, "intrinsics": intrinsics, "bf": BF, "fixed": [0],
    }  # fmt: skip
    return bundle, kept


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_the_near_filter_leaves_out_exactly_the_near_and_behind_observations(
    backend: str,
) -> None:
    """With ``near``, the solve's starting error is the kept observations' alone, and counted.

    The summary counts the observations left out, and the kept ones whose point ends behind its
    camera or below the limit, as recounted here from the result. The point and the camera every
    one of whose observations is left out come back bit for bit as given, as does the held
    camera; without ``near`` every observation counts, and the observations of the point behind
    its cameras end behind them.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    bundle, kept = _near_and_behind()
    costs = np.array(
        [
            0.5
            * np.sum(
                graphs.projection_residual(
                    bundle["poses"][c], bundle["points"][p], bundle["pixels"][m], *PINHOLE, BF
                ).residual
                ** 2
            )
            for m, (c, p) in enumerate(zip(bundle["cameras"], bundle["point_ids"], strict=True))
        ]
    )
    options = graphs.Options(max_iterations=5)
    filtered = graphs.bundle_adjust(**bundle, near=1e-3, options=options, backend=chosen)
    assert filtered.summary.initial_error == pytest.approx(float(costs[kept].sum()), rel=1e-12)
    assert filtered.summary.left_out == int(np.count_nonzero(~kept))
    start = _depths_of(bundle["poses"], bundle["points"], bundle)
    limit = 1e-3 * float(np.sort(start)[len(start) // 2])
    end = _depths_of(filtered.poses, filtered.points, bundle)
    assert filtered.summary.ended_near == int(np.count_nonzero(kept & ((end <= 0) | (end < limit))))
    np.testing.assert_array_equal(filtered.points[61], bundle["points"][61])
    np.testing.assert_array_equal(filtered.poses[[0, 5]], bundle["poses"][[0, 5]])
    assert not np.array_equal(filtered.points[60], bundle["points"][60])  # seen by 0 and 4 still
    every = graphs.bundle_adjust(**bundle, options=options, backend=chosen)
    assert every.summary.initial_error == pytest.approx(float(costs.sum()), rel=1e-12)
    assert every.summary.left_out == 0
    behind = _depths_of(every.poses, every.points, bundle) <= 0
    assert every.summary.ended_near == int(np.count_nonzero(behind)) >= 3  # point 61's three


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_bundle_whose_observations_are_all_left_out_comes_back_as_given(backend: str) -> None:
    """Every point behind its cameras: nothing is solved, 0 iterations, converged, no error."""
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    poses = np.stack([np.eye(4), se3.exp([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])])
    points = np.array([[0.1, 0.2, -1.0], [0.0, -0.1, -2.0]])
    result = graphs.bundle_adjust(
        poses, points, [0, 1, 1], [0, 0, 1], np.full((3, 2), 300.0), np.full(3, np.nan),
        np.ones(3), model=PINHOLE[0], intrinsics=PINHOLE[1], bf=BF, fixed=[0], near=1e-3,
        backend=chosen,
    )  # fmt: skip
    np.testing.assert_array_equal(result.poses, poses)
    np.testing.assert_array_equal(result.points, points)
    summary = result.summary
    assert (summary.iterations, summary.termination, summary.converged) == (0, "function", True)
    assert (summary.initial_error, summary.final_error) == (0.0, 0.0)
    assert (summary.left_out, summary.ended_near) == (3, 0)


@pytest.mark.parametrize("near", [-1e-3, np.inf, np.nan])
def test_a_negative_or_infinite_near_fraction_is_refused(near: float) -> None:
    """``near`` is a fraction of the median depth, finite and non-negative."""
    with pytest.raises(InvalidInputError, match="near"):
        graphs.bundle_adjust(
            HELD_PAIR, FAR_POINT[1][None], [0, 1], [0, 0], _seen_from_held_pair(),
            [np.nan, np.nan], [1.0, 1.0], model=PINHOLE[0], intrinsics=PINHOLE[1], bf=BF,
            fixed=[0, 1], near=near,
        )  # fmt: skip


def test_bad_graphs_are_refused() -> None:
    """Every input is checked before GTSAM sees it."""
    truth, edges, relatives = _ring(4, False)
    with pytest.raises(InvalidInputError, match="at least one pose held fixed"):
        graphs.optimize_pose_graph(truth, edges, relatives, [], similarity=False)
    with pytest.raises(InvalidInputError, match="joins a pose to itself"):
        graphs.optimize_pose_graph(truth, [[1, 1]], relatives[:1], [0], similarity=False)
    with pytest.raises(InvalidInputError, match=r"indices in \[0, 4\)"):
        graphs.optimize_pose_graph(truth, [[0, 4]], relatives[:1], [0], similarity=False)
    scaled = truth.copy()
    scaled[1][:3, :3] *= 2.0
    with pytest.raises(InvalidInputError, match=r"poses\[1\]"):
        graphs.optimize_pose_graph(scaled, edges, relatives, [0], similarity=False)
    with pytest.raises(InvalidInputError, match="max_iterations"):
        graphs.Options(max_iterations=0)
    poses, points = _scene(1)
    with pytest.raises(InvalidInputError, match="every point must be observed"):
        graphs.bundle_adjust(
            poses, points, [0], [0], [[1.0, 2.0]], [np.nan], [1.0], model="pinhole",
            intrinsics=PINHOLE[1], bf=BF, fixed=[0],
        )  # fmt: skip
    with pytest.raises(InvalidInputError, match="bf"):
        graphs.bundle_adjust(
            poses, points[:1], [0], [0], [[1.0, 2.0]], [3.0], [1.0], model="pinhole",
            intrinsics=PINHOLE[1], bf=0.0, fixed=[0],
        )  # fmt: skip


def _matches(count: int = 12) -> tuple[Array, Array, Array, Array, Array]:
    """Return a similarity and `count` matches it carries exactly: points1, points2, pixels."""
    name, intrinsics = PINHOLE
    rng = np.random.default_rng(11)
    truth = _similarity(np.eye(3), np.array([0.1, 0.0, -0.1]), 1.1)
    points2 = np.column_stack(
        [rng.uniform(-1, 1, count), rng.uniform(-1, 1, count), rng.uniform(2, 5, count)]
    )
    points1 = points2 @ truth[:3, :3].T + truth[:3, 3]
    return (
        truth,
        points1,
        points2,
        camera.project(name, intrinsics, points1),
        camera.project(name, intrinsics, points2),
    )


@pytest.mark.parametrize("bad", [[np.nan, 210.0], [np.inf, 210.0], [300.0, -np.inf]])
def test_a_half_unseen_or_infinite_pixels2_row_is_refused(bad: list[float]) -> None:
    """A row of pixels2 is finite, or wholly NaN (unseen); anything else is refused (H1, L1).

    Python's boundary: the check runs before any backend is called, so each core's own check is
    tested natively (``test_bad_input`` in ``cpp/tests/foundations/test_graphs.cpp``,
    ``bad_graphs_are_refused`` in ``graphs.rs``; B3's review panel, run 2, H21).
    """
    truth, points1, points2, pixels1, pixels2 = _matches()
    pixels2[1] = bad
    with pytest.raises(InvalidInputError, match="pixels2"):
        graphs.refine_similarity(
            truth, points1, points2, pixels1, pixels2, np.ones((12, 2)), model=PINHOLE[0],
            intrinsics=PINHOLE[1], fixed_scale=False, huber=None,
        )  # fmt: skip


def _refine_with(**changes: object) -> None:
    """Refine the exact matches with one argument replaced."""
    truth, points1, points2, pixels1, pixels2 = _matches()
    arguments: dict[str, object] = {
        "similarity": truth, "points1": points1, "points2": points2, "pixels1": pixels1,
        "pixels2": pixels2, "information": np.ones((12, 2)), "model": PINHOLE[0],
        "intrinsics": PINHOLE[1], "fixed_scale": False, "huber": None,
    }  # fmt: skip
    arguments.update(changes)
    graphs.refine_similarity(**arguments)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("changes", "named"),
    [
        ({"points1": np.zeros((0, 3)), "points2": np.zeros((0, 3)), "pixels1": np.zeros((0, 2)),
          "pixels2": np.zeros((0, 2)), "information": np.zeros((0, 2))}, "matches"),
        ({"points1": np.ones((11, 3))}, "matches"),
        ({"information": np.zeros((12, 2))}, "information"),
        ({"huber": 0.0}, "huber"),
        ({"huber": np.nan}, "huber"),
        ({"similarity": -np.eye(4)}, "similarity"),
        ({"intrinsics": np.array([520.0, 515.0])}, "intrinsics"),
    ],
)  # fmt: skip
def test_bad_refinements_are_refused(changes: dict[str, object], named: str) -> None:
    """Each bad argument of a refinement is refused by name (B3's review panel, H4)."""
    with pytest.raises(InvalidInputError, match=named):
        _refine_with(**changes)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("max_iterations", 0),
        ("max_iterations", True),
        ("initial_damping", 0.0),
        ("initial_damping", np.inf),
        ("damping_scale", 0.0),
        ("damping_scale", np.nan),
        ("relative_tolerance", -1.0),
        ("absolute_tolerance", np.nan),
        ("scaled_damping", 1),
        ("small_steps", -1),
        ("small_steps", 2.0),
        ("small_decrease", 0.0),
        ("small_decrease", np.inf),
    ],
)
def test_bad_options_are_refused(field: str, value: object) -> None:
    """Each of a solve's options is checked, and named, when made."""
    with pytest.raises(InvalidInputError, match=field):
        graphs.Options(**{field: value})  # type: ignore[arg-type]


def test_a_failure_inside_gtsam_is_not_converged() -> None:
    """GTSAM's own errors leave a solve as NotConvergedError, as the compiled cores report them."""
    graph = gtsam.NonlinearFactorGraph()
    unit = gtsam.noiseModel.Unit.Create(graphs.POSE_DIM)
    graph.add(gtsam.BetweenFactorPose3(0, 5, gtsam.Pose3(), unit))  # key 5 has no value
    values = gtsam.Values()
    values.insert(0, gtsam.Pose3())
    with pytest.raises(NotConvergedError, match="GTSAM failed"):
        graphs._solve(graph, values, graphs.Options())


GRAPHS = load_fixture("foundations/graphs.json")
"""Rung 0's graphs fixture: the Python reference's results on noisy data (generate_graphs.py)."""

INDEX_INPUTS = frozenset({"edges", "fixed", "cameras", "point_ids"})
"""The fixture's inputs that are indices, read as int64."""


def _inputs(raw: dict[str, object]) -> dict[str, object]:
    """Return a fixture case's inputs as the functions take them: lists as arrays, null as NaN."""
    out: dict[str, object] = {}
    for key, value in raw.items():
        if key == "options":
            out[key] = graphs.Options(**value)  # type: ignore[arg-type]
        elif isinstance(value, list):
            array = np.array(value, dtype=np.float64)
            out[key] = array.astype(np.int64) if key in INDEX_INPUTS else array
        else:
            out[key] = value
    return out


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(GRAPHS))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Every backend reproduces the fixture: its values, errors, iterations and termination (A2).

    Each field under its policy (``generate_graphs.TOLERANCES``; B3's review panel, run 2, H22).
    A null (NaN) chi-square, a match image 2 does not observe, compares equal to NaN.
    """
    chosen = require_function(backend, graphs.__name__, function)
    got = getattr(graphs, function)(**_inputs(case["input"]), backend=chosen)
    expected, tolerance = case["expected"], GRAPHS["tolerance"]
    for field in expected.keys() - {"errors", "codes"}:
        values = np.array(expected[field], dtype=np.float64)
        assert_matches_fixture(getattr(got, field), values, tolerance[field])
    summary = got.summary
    errors = [summary.initial_error, summary.final_error]
    assert_matches_fixture(errors, expected["errors"], tolerance["errors"])
    codes = {
        "iterations": summary.iterations,
        "termination": str(summary.termination),
        "left_out": summary.left_out,
        "ended_near": summary.ended_near,
    }
    assert_matches_fixture(codes, expected["codes"], tolerance["codes"])
