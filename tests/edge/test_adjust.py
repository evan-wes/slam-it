"""Bundle adjustment's tests: every backend against the reference, and its rules.

On the room's map: what the adjustment holds fixed, recovers and forgets.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import adjust
from slam_it.edge.map import Map
from slam_it.foundations import nls
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import Truth
from tests.edge.support import (
    CAMERA,
    OPTIONS,
    SOLVER_TOLERANCE,
    assert_maps_match,
    assert_records_match,
)


def _perturbed(m: Map, seed: int = 1) -> Map:
    """Move every keyframe but the first by about a centimetre, and every point by half."""
    rng = np.random.default_rng(seed)
    poses = m.kf_pose.copy()
    poses[1:, :3, 3] += rng.normal(0.0, 0.01, (m.keyframes - 1, 3))
    return replace(m, kf_pose=poses, pt_xyz=m.pt_xyz + rng.normal(0.0, 0.005, m.pt_xyz.shape))


def _first_fixed(m: Map) -> np.ndarray:
    """Hold the first keyframe fixed (the gauge)."""
    fixed = np.zeros(m.keyframes, dtype=bool)
    fixed[0] = True
    return fixed


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("window", [None, 2])
def test_matches_the_reference(backend: str, window: int | None, truth: Truth) -> None:
    """Every backend adjusts the whole map, or its last keyframes, as the reference does."""
    chosen = require_function(backend, adjust.__name__, "bundle_adjust")
    m = _perturbed(truth.map)
    keyframes = np.arange(m.keyframes)[-window:] if window else np.arange(m.keyframes)
    expected = adjust.bundle_adjust(m, CAMERA, keyframes, _first_fixed(m), OPTIONS)
    got = adjust.bundle_adjust(m, CAMERA, keyframes, _first_fixed(m), OPTIONS, backend=chosen)
    assert_maps_match(got.map, expected.map, SOLVER_TOLERANCE)
    assert got.outliers == expected.outliers
    assert_records_match(
        [solve.as_dict() for solve in got.solves], [solve.as_dict() for solve in expected.solves]
    )


def test_converges_from_noise_to_the_noiseless_solution(truth: Truth) -> None:
    """From the truth or from noise, the adjustment reaches the same poses and points."""
    m = truth.map
    fixed = _first_fixed(m)
    every = np.arange(m.keyframes)
    clean = adjust.bundle_adjust(m, CAMERA, every, fixed, OPTIONS).map
    noisy = adjust.bundle_adjust(_perturbed(m), CAMERA, every, fixed, OPTIONS).map
    np.testing.assert_allclose(noisy.kf_pose, clean.kf_pose, rtol=0, atol=1e-5)
    ids = np.intersect1d(noisy.pt_id, clean.pt_id)
    assert len(ids) > 0.99 * m.points
    seen = np.bincount(m.point_rows(m.kp_point[m.kp_point >= 0]), minlength=m.points)
    moved = ids[seen[m.point_rows(ids)] >= 2]  # a point seen once is left out of the problem
    np.testing.assert_allclose(
        noisy.pt_xyz[noisy.point_rows(moved)], clean.pt_xyz[clean.point_rows(moved)], atol=1e-4
    )


def test_holds_fixed_and_unchosen_keyframes_exactly(truth: Truth) -> None:
    """The fixed keyframe and those outside the chosen ones keep their poses bit for bit."""
    m = _perturbed(truth.map)
    out = adjust.bundle_adjust(m, CAMERA, np.array([2, 3]), _first_fixed(m), OPTIONS).map
    np.testing.assert_array_equal(out.kf_pose[:2], m.kf_pose[:2])
    assert not np.array_equal(out.kf_pose[2:], m.kf_pose[2:])


def test_forgets_an_observation_moved_off_its_point(truth: Truth) -> None:
    """A keypoint shifted 40 pixels from its point fails the chi-square test and is forgotten."""
    m = truth.map
    last = np.arange(m.kf_start[-2], m.kf_start[-1])
    row = int(last[m.kp_point[last] >= 0][0])
    xy = m.kp_xy.copy()
    xy[row, 0] += 40.0
    adjusted = adjust.bundle_adjust(
        replace(m, kp_xy=xy), CAMERA, np.arange(m.keyframes), _first_fixed(m), OPTIONS
    )
    out = adjusted.map
    assert out.kp_point[row] == -1
    assert np.count_nonzero(out.kp_point >= 0) > 0.95 * np.count_nonzero(m.kp_point >= 0)
    assert adjusted.outliers == np.count_nonzero(m.kp_point >= 0) - np.count_nonzero(
        out.kp_point >= 0
    )


def test_records_its_two_solves(truth: Truth) -> None:
    """The adjustment reports both solves; from noise, each lowers the cost it started from."""
    m = _perturbed(truth.map)
    adjusted = adjust.bundle_adjust(m, CAMERA, np.arange(m.keyframes), _first_fixed(m), OPTIONS)
    assert len(adjusted.solves) == 2
    first, second = adjusted.solves
    assert first.final_cost < first.initial_cost
    assert second.final_cost <= second.initial_cost
    assert 0 < first.iterations <= OPTIONS.adjustment.iterations
    assert 0 < second.iterations <= OPTIONS.adjustment.refine_iterations
    assert set(first.as_dict()) == {"initial_cost", "final_cost", "iterations", "termination"}


def test_no_points_leave_the_map_unchanged(truth: Truth) -> None:
    """Keyframes observing nothing: nothing to adjust."""
    m = truth.map
    bare = replace(m, kp_point=np.full_like(m.kp_point, -1))
    adjusted = adjust.bundle_adjust(bare, CAMERA, np.arange(m.keyframes), _first_fixed(m), OPTIONS)
    assert adjusted.map is bare
    assert (adjusted.solves, adjusted.outliers) == ((), 0)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_no_keyframe_held_fixed_leaves_the_map_unchanged(backend: str, truth: Truth) -> None:
    """Every keyframe free, nothing gauges the problem: ORB-SLAM3 skips it, and so does this."""
    chosen = require_function(backend, adjust.__name__, "bundle_adjust")
    m = _perturbed(truth.map)
    free = np.zeros(m.keyframes, dtype=bool)
    adjusted = adjust.bundle_adjust(
        m, CAMERA, np.arange(m.keyframes), free, OPTIONS, backend=chosen
    )
    assert_maps_match(adjusted.map, m)
    assert (adjusted.solves, adjusted.outliers, adjusted.free, adjusted.fixed) == ((), 0, 0, 0)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rejects_malformed_arguments(backend: str, truth: Truth) -> None:
    """Rows out of range and flags that are not one per keyframe raise before any backend runs."""
    m = truth.map
    fixed = _first_fixed(m)
    calls: list[Callable[[], object]] = [
        lambda: adjust.bundle_adjust(m, CAMERA, [m.keyframes], fixed, OPTIONS, backend=backend),
        lambda: adjust.bundle_adjust(m, CAMERA, [0], [True], OPTIONS, backend=backend),
        lambda: adjust.bundle_adjust(m, CAMERA, [0.5], fixed, OPTIONS, backend=backend),
    ]
    for call in calls:
        with pytest.raises(InvalidInputError):
            call()


def test_an_abort_before_the_first_solve_leaves_the_map(truth: Truth) -> None:
    """Stopped before it starts, the adjustment solves nothing (ORB-SLAM3's early return)."""
    m = _perturbed(truth.map)
    every = np.arange(m.keyframes)
    adjusted = adjust.bundle_adjust(m, CAMERA, every, _first_fixed(m), OPTIONS, abort=lambda: True)
    assert adjusted.map is m
    assert (adjusted.solves, adjusted.outliers, adjusted.aborted) == ((), 0, True)


def test_an_abort_between_iterations_keeps_what_was_solved(truth: Truth) -> None:
    """Stopped after two iterations: the first solve ends there, written back; no second runs.

    The check is asked before the first solve, then before each of its iterations (g2o's
    force-stop), then before the refining solve.
    """
    m = _perturbed(truth.map)
    every = np.arange(m.keyframes)
    asked: list[bool] = []

    def abort() -> bool:
        asked.append(True)
        return len(asked) > 3  # before the solve, then two iterations, then stop

    stopped = adjust.bundle_adjust(m, CAMERA, every, _first_fixed(m), OPTIONS, abort=abort)
    whole = adjust.bundle_adjust(m, CAMERA, every, _first_fixed(m), OPTIONS)
    assert stopped.aborted
    assert not whole.aborted
    (first,) = stopped.solves
    assert (first.iterations, first.termination) == (2, nls.Termination.STOPPED)
    assert first.final_cost < first.initial_cost
    assert not np.array_equal(stopped.map.kf_pose, m.kf_pose)  # the first solve moved them
    assert not np.array_equal(stopped.map.kf_pose, whole.map.kf_pose)  # and no second did


def test_a_compiled_backend_refuses_an_abort(truth: Truth) -> None:
    """The bindings do not cross Python's abort: a compiled backend refuses it, not ignores it."""
    m = truth.map
    with pytest.raises(InvalidInputError, match="abort"):
        adjust.bundle_adjust(
            m, CAMERA, np.arange(m.keyframes), _first_fixed(m), OPTIONS,
            abort=lambda: False, backend="cpp",
        )  # fmt: skip
