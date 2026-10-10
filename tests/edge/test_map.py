"""The map's tests: every backend's operations against the Python reference, and their rules."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
from typing import Any

import numpy as np
import pytest

from slam_it_all import backends
from slam_it_all.edge import map as maps
from slam_it_all.edge.frames import Frame
from slam_it_all.edge.map import Map, Removal
from slam_it_all.foundations.errors import InvalidInputError
from slam_it_all.foundations.orb import DESCRIPTOR_BYTES
from tests.conftest import protocol_members, require_function
from tests.edge.conftest import Truth, _observe
from tests.edge.support import assert_maps_match

FUNCTIONS = sorted(protocol_members(maps))

_GONE = ("gone_pt_id", "gone_pt_xyz", "gone_pt_how", "gone_kf_id", "gone_kf_time", "gone_kf_pose")
"""The map's archive of removed points and keyframes."""
"""Every compiled map operation."""


def _free_rows(m: Map, k: int, count: int) -> np.ndarray:
    """Return the first `count` keypoint rows of keyframe `k` that observe no point."""
    rows = np.arange(m.kf_start[k], m.kf_start[k + 1])
    return rows[m.kp_point[rows] < 0][:count]


def _cases(truth: Truth) -> dict[str, list[tuple[tuple[object, ...], dict[str, object]]]]:
    """Each operation's calls on the session's map: positional and keyword arguments."""
    m = truth.map
    rng = np.random.default_rng(3)
    p = m.points
    rows = np.arange(p, dtype=np.int64)
    next_frame: Frame = truth.frames[-1]
    pose = truth.poses[-1]
    seen = _observe(m, next_frame, pose)
    free = [_free_rows(m, k, 20) for k in range(m.keyframes)]
    a, b = np.argsort([-len(f) for f in free], kind="stable")[:2]
    q = min(len(free[a]), len(free[b]))
    assert q >= 5, "the room's map should leave free keypoints in two keyframes"
    observers = np.stack([free[a][:q], free[b][:q]], axis=1)
    return {
        "predict_level": [((m, rows, rng.uniform(0.5, 8.0, p)), {})],
        "insert_keyframe": [
            ((m, next_frame, pose, seen), {}),
            ((m, next_frame, np.eye(4), np.full(next_frame.size, -1)), {}),
        ],
        "add_points": [
            ((m, rng.uniform(-2.0, 2.0, (q, 3)) + np.array([0.0, 0.0, 4.0]), observers), {}),
            ((m, np.array([[0.1, 0.2, 3.0]]), observers[:1, :1]), {}),
        ],
        "distinctive_descriptor": [
            ((rng.integers(0, 256, (n, DESCRIPTOR_BYTES), dtype=np.uint8),), {})
            for n in (1, 2, 3, 4, 7)
        ],
        "refresh_points": [((m, rows), {}), ((m, rows[::7]), {})],
        "erase_points": [
            ((m, rng.random(p) < 0.3, Removal.CULLED), {}),
            ((m, rng.random(p) < 0.1, Removal.FUSED), {}),
            ((m, np.zeros(p, dtype=bool), Removal.WEAK), {}),
        ],
        "observation_weight": [((m, rows), {})],
        "forget_observations": [
            ((m, rng.choice(len(m.kp_point), 300, replace=False)), {}),
            ((m, np.arange(m.kf_start[0], m.kf_start[1])), {}),
        ],
        "erase_keyframe": [((m, k), {}) for k in range(m.keyframes)],
        "count": [
            (
                (m, np.concatenate([m.pt_id[:30], m.pt_id[:5], [-1, m.next_pt_id + 7]])),
                {
                    "visible": rng.integers(0, 3, 37),
                    "found": rng.integers(0, 2, 37),
                },
            )
        ],
    }


@pytest.fixture(scope="module")
def cases(truth: Truth) -> dict[str, list[tuple[tuple[object, ...], dict[str, object]]]]:
    """Build the calls once."""
    return _cases(truth)


def test_every_operation_has_cases(cases: dict[str, object]) -> None:
    """Every compiled operation is exercised."""
    assert set(cases) == set(FUNCTIONS)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", FUNCTIONS)
def test_matches_the_reference(backend: str, function: str, cases: dict[str, list[Any]]) -> None:
    """Each backend's operation leaves the map as the Python reference does."""
    chosen = require_function(backend, maps.__name__, function)
    for args, kwargs in cases[function]:
        got = getattr(maps, function)(*args, **kwargs, backend=chosen)
        expected = getattr(maps, function)(*args, **kwargs)
        if isinstance(expected, Map):
            assert isinstance(got, Map)
            assert_maps_match(got, expected)
        else:
            np.testing.assert_array_equal(got, expected)


def test_the_room_map_holds_together(truth: Truth) -> None:
    """The truth map: keyframes and points by ascending id, every observation linked."""
    m = truth.map
    assert m.keyframes >= 2
    assert m.points > 200
    assert np.all(np.diff(m.kf_id) > 0)
    assert np.all(np.diff(m.pt_id) > 0)
    observed = m.kp_point[m.kp_point >= 0]
    assert np.all(np.isin(observed, m.pt_id))
    np.testing.assert_allclose(np.linalg.norm(m.pt_normal, axis=1), 1.0)
    assert np.all(m.pt_min_distance < m.pt_max_distance)


def test_forgetting_erases_points_left_weakly_observed(truth: Truth) -> None:
    """A point left observed by one keyframe (weight 2 or less) is erased; others refreshed."""
    m = truth.map
    weights = maps.observation_weight(m, np.arange(m.points))
    strong = int(np.argmax(weights))
    seen = np.flatnonzero(m.kp_point == m.pt_id[strong])
    forgotten = maps.forget_observations(m, seen[:1])
    assert m.pt_id[strong] in forgotten.pt_id
    gone = maps.forget_observations(m, seen)
    assert m.pt_id[strong] not in gone.pt_id
    assert not np.any(gone.kp_point == m.pt_id[strong])


def test_what_is_removed_is_kept_aside_until_cleared(truth: Truth) -> None:
    """What is removed is kept aside, as it last was, until the archive is cleared.

    Erased points keep their last positions and why they went; an erased keyframe its time and
    pose; the points it leaves too weakly observed go too.
    """
    m = truth.map
    erase = np.zeros(m.points, dtype=bool)
    erase[[3, 7]] = True
    fused = maps.erase_points(m, erase, Removal.FUSED)
    np.testing.assert_array_equal(fused.gone_pt_id, m.pt_id[[3, 7]])
    np.testing.assert_array_equal(fused.gone_pt_xyz, m.pt_xyz[[3, 7]])
    np.testing.assert_array_equal(fused.gone_pt_how, [Removal.FUSED, Removal.FUSED])
    culled = maps.erase_keyframe(fused, 1)
    np.testing.assert_array_equal(culled.gone_kf_id, [m.kf_id[1]])
    np.testing.assert_array_equal(culled.gone_kf_time, [m.kf_time[1]])
    np.testing.assert_array_equal(culled.gone_kf_pose, m.kf_pose[[1]])
    weak = culled.gone_pt_id[2:]
    assert len(weak) == fused.points - culled.points
    assert np.all(culled.gone_pt_how[2:] == Removal.WEAK)
    assert not np.any(np.isin(weak, culled.pt_id))
    cleared = maps.clear_gone(culled)
    assert len(cleared.gone_pt_id) == len(cleared.gone_kf_id) == 0
    assert_maps_match(replace(culled, **{f: getattr(cleared, f) for f in _GONE}), cleared)


def test_erasing_a_keyframe_moves_the_others_keypoints_up(truth: Truth) -> None:
    """Erasing a keyframe removes exactly its keypoint rows; the others keep their contents."""
    m = truth.map
    k = 0
    erased = maps.erase_keyframe(m, k)
    assert erased.keyframes == m.keyframes - 1
    np.testing.assert_array_equal(erased.kf_id, np.delete(m.kf_id, k))
    rest = np.arange(m.kf_start[k + 1], len(m.kp_point))
    np.testing.assert_array_equal(erased.kp_descriptor, m.kp_descriptor[rest])
    np.testing.assert_array_equal(np.diff(erased.kf_start), np.delete(np.diff(m.kf_start), k))


def test_a_point_loses_its_reference_to_its_earliest_observer(truth: Truth) -> None:
    """When the reference keyframe no longer observes a point, the lowest observer id takes over."""
    m = truth.map
    keyframe_of = m.keypoint_keyframe()
    for p in range(m.points):
        seen = np.flatnonzero(m.kp_point == m.pt_id[p])
        observers = m.kf_id[keyframe_of[seen]]
        if len(observers) >= 2:
            break
    else:
        pytest.skip("no point with two observers")
    moved = replace(m, pt_reference_kf=m.pt_reference_kf.copy())
    moved.pt_reference_kf[p] = m.next_kf_id + 5  # a keyframe that does not observe it
    refreshed = maps.refresh_points(moved, np.array([p]))
    assert refreshed.pt_reference_kf[p] == observers.min()


def test_distinctive_descriptor_ties_go_to_the_earliest() -> None:
    """Two descriptors: both have the same median distance, so the first wins."""
    pair = np.zeros((2, DESCRIPTOR_BYTES), dtype=np.uint8)
    pair[1, 0] = 0xFF
    assert maps.distinctive_descriptor(pair) == 0


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rejects_malformed_arguments(backend: str, truth: Truth) -> None:
    """Bad rows, ids and shapes raise InvalidInputError before any backend runs."""
    m = truth.map
    calls: list[Callable[[], object]] = [
        lambda: maps.predict_level(m, [m.points], [1.0], backend=backend),
        lambda: maps.predict_level(m, [0], [0.0], backend=backend),
        lambda: maps.refresh_points(m, [-1], backend=backend),
        lambda: maps.erase_points(
            m, np.zeros(m.points + 1, dtype=bool), Removal.CULLED, backend=backend
        ),
        lambda: maps.erase_points(m, np.zeros(m.points, bool), 1, backend=backend),  # type: ignore[arg-type]
        lambda: maps.forget_observations(m, [len(m.kp_point)], backend=backend),
        lambda: maps.erase_keyframe(m, m.keyframes, backend=backend),
        lambda: maps.count(m, [0], visible=[-1], backend=backend),
        lambda: maps.add_points(m, [[0.0, 0.0, np.inf]], [[0]], backend=backend),
        lambda: maps.add_points(m, [[0.0, 0.0, 1.0]], [[_observed_row(m)]], backend=backend),
        lambda: maps.distinctive_descriptor(
            np.zeros((0, DESCRIPTOR_BYTES), np.uint8), backend=backend
        ),
    ]
    for call in calls:
        with pytest.raises(InvalidInputError):
            call()


def _observed_row(m: Map) -> int:
    """Return a keypoint row that already observes a point."""
    return int(np.flatnonzero(m.kp_point >= 0)[0])


def test_a_map_checks_its_structure(truth: Truth) -> None:
    """A map with unsorted ids, bad offsets or a dangling observation is refused."""
    m = truth.map
    with pytest.raises(InvalidInputError, match="pt_id"):
        replace(m, pt_id=m.pt_id[::-1].copy())
    with pytest.raises(InvalidInputError, match="kf_start"):
        replace(m, kf_start=m.kf_start + 1)
    dangling = m.kp_point.copy()
    dangling[0] = m.next_pt_id + 3
    with pytest.raises(InvalidInputError, match="kp_point"):
        replace(m, kp_point=dangling)
    with pytest.raises(InvalidInputError, match="gone"):  # erased, yet still in the map
        replace(
            m,
            gone_pt_id=m.pt_id[:1].copy(),
            gone_pt_xyz=m.pt_xyz[:1].copy(),
            gone_pt_how=np.array([Removal.CULLED], dtype=np.int64),
        )
    gone = maps.erase_points(m, np.arange(m.points) == 0, Removal.CULLED)
    with pytest.raises(InvalidInputError, match="gone_pt_how"):
        replace(gone, gone_pt_how=np.array([7], dtype=np.int64))
