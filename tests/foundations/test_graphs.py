"""Factor-graph tests: the factors' Jacobians, and each solve recovering a known answer."""

from __future__ import annotations

from collections.abc import Callable

import gtsam
import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.foundations import camera, graphs, se3
from slam_it.foundations.errors import InvalidInputError, NotConvergedError
from tests.conftest import load_fixture, require_function

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


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("lens", [RADTAN, PINHOLE], ids=["radtan", "pinhole"])
def test_bundle_adjustment_recovers_a_stereo_scene(backend: str, lens: tuple[str, Array]) -> None:
    """Stereo observations fix the scale: perturbed cameras and points return to the truth.

    Through radtan, Python's factors are its own; through a pinhole camera, GTSAM's.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    name, intrinsics = lens
    poses, points = _scene(5)
    cameras, ids, pixels, right = [], [], [], []
    for c, pose in enumerate(poses):
        local = (points - pose[:3, 3]) @ pose[:3, :3]
        seen = camera.project(name, intrinsics, local)
        for p in range(points.shape[0]):
            cameras.append(c)
            ids.append(p)
            pixels.append(seen[p])
            right.append(seen[p, 0] - BF / local[p, 2] if p % 2 else np.nan)
    rng = np.random.default_rng(6)
    start_poses = np.stack(
        [poses[0]] + [pose @ se3.exp(rng.normal(scale=0.02, size=6)) for pose in poses[1:]]
    )
    start_points = points + rng.normal(scale=0.05, size=points.shape)
    result = graphs.bundle_adjust(
        start_poses, start_points, cameras, ids, pixels, right, np.ones(len(cameras)),
        model=name, intrinsics=intrinsics, bf=BF, fixed=[0],
        options=graphs.Options(max_iterations=20), backend=chosen,
    )  # fmt: skip
    np.testing.assert_array_equal(result.poses[0], poses[0])
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


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("bad", [[np.nan, 210.0], [np.inf, 210.0], [300.0, -np.inf]])
def test_a_half_unseen_or_infinite_pixels2_row_is_refused(backend: str, bad: list[float]) -> None:
    """A row of pixels2 is finite, or wholly NaN (unseen); anything else is refused (H1, L1)."""
    chosen = require_function(backend, graphs.__name__, "refine_similarity")
    truth, points1, points2, pixels1, pixels2 = _matches()
    pixels2[1] = bad
    with pytest.raises(InvalidInputError, match="pixels2"):
        graphs.refine_similarity(
            truth, points1, points2, pixels1, pixels2, np.ones((12, 2)), model=PINHOLE[0],
            intrinsics=PINHOLE[1], fixed_scale=False, huber=None, backend=chosen,
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
        ("relative_tolerance", -1.0),
        ("absolute_tolerance", np.nan),
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
        if isinstance(value, list):
            array = np.array(value, dtype=np.float64)
            out[key] = array.astype(np.int64) if key in INDEX_INPUTS else array
        else:
            out[key] = value
    return out


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", ["optimize_pose_graph", "refine_similarity", "bundle_adjust"])
def test_matches_fixture(backend: str, function: str) -> None:
    """Every backend reproduces the fixture: its values, errors, iterations and termination (A2)."""
    chosen = require_function(backend, graphs.__name__, function)
    tolerance = GRAPHS["tolerance"]
    for case in GRAPHS["functions"][function]["cases"]:
        got = getattr(graphs, function)(**_inputs(case["input"]), backend=chosen)
        for field, expected in case["expected"].items():
            np.testing.assert_allclose(
                getattr(got, field), np.array(expected, dtype=np.float64), rtol=0,
                atol=tolerance["value_atol"], equal_nan=True, err_msg=f"{case['name']}: {field}",
            )  # fmt: skip
        summary = case["summary"]
        assert got.summary.iterations == summary["iterations"], case["name"]
        assert str(got.summary.termination) == summary["termination"], case["name"]
        for key in ("initial_error", "final_error"):
            assert np.isclose(
                getattr(got.summary, key), summary[key], rtol=tolerance["error_rtol"], atol=1e-12
            ), f"{case['name']}: {key}"
