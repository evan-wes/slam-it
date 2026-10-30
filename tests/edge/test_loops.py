"""Loop closing's tests: its searches, its similarity, detection and correction, on the loop map.

The loop map (:func:`tests.edge.conftest.loop_map`) holds the loop room's keyframes, mapped from
the truth but each seeing only its window, then drifted round the loop: its end makes its own
points where its start's are, out of place by the drift, as a tracked map's would be.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import replace
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.edge import covisibility, loops
from slam_it.edge import map as maps
from slam_it.edge.database import Database
from slam_it.edge.loops import Outcome, Pending
from slam_it.edge.map import Map, Removal
from slam_it.edge.options import Options
from slam_it.foundations import alignment, graphs, matching, se3, so3
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.edge.conftest import Loop
from tests.edge.support import (
    CAMERA,
    GRAPH_TOLERANCE,
    OPTIONS,
    TOLERANCE,
    assert_close,
    assert_maps_match,
    assert_records_match,
)

LAST_CONFIRMED = -2
"""The loop map's keyframe where the loop closes at once: the second last (its last has two
of its links to confirm it, the others falling short)."""

MIDDLE = 14
"""A keyframe half way round, where nothing has been seen before."""


def _mode(loop: Loop) -> str:
    """Return the mode a loop map stands for."""
    return "monocular" if loop.scaled else "rgbd"


def _rows(m: Map, k: int) -> np.ndarray:
    """Return a keyframe's keypoint rows, as an array."""
    return np.arange(m.kf_start[k], m.kf_start[k + 1], dtype=np.int64)


def _position_errors(m: Map, poses: list[np.ndarray]) -> np.ndarray:
    """Return each keyframe's distance from its true place, in metres."""
    return np.array(
        [np.linalg.norm(maps.camera_centre(m.kf_pose[k]) - maps.camera_centre(poses[k]))
         for k in range(m.keyframes)]
    )  # fmt: skip


def _detect(
    loop: Loop,
    db: Database,
    row: int,
    pending: Pending | None = None,
    *,
    confirmations: int | None = None,
    min_keyframes: int | None = None,
) -> loops.Detection:
    """Detect at the keyframe of a row (negative from the end), a setting or two changed."""
    keyframe = int(loop.map.kf_id[row])
    changed = {
        f"loops.{name}": value
        for name, value in (("confirmations", confirmations), ("min_keyframes", min_keyframes))
        if value is not None
    }
    options = OPTIONS.with_settings(changed)
    return loops.detect(loop.map, db, CAMERA, keyframe, pending, mode=_mode(loop), options=options)


@pytest.fixture(scope="module", params=["rigid", "scaled"])
def loop(request: pytest.FixtureRequest) -> Loop:
    """Return the loop map, drifted rigidly (RGB-D) and in scale (monocular)."""
    chosen: Loop = request.getfixturevalue(f"loop_{request.param}")
    return chosen


# --- similarities -------------------------------------------------------------------------


def test_a_similarity_s_scale_and_rigid_part() -> None:
    """``[[s R, t], [0, 1]]`` has scale ``s`` and projects through ``[[R, t / s], [0, 1]]``."""
    rigid = se3.exp(np.array([0.1, -0.2, 0.3, 0.05, 0.1, -0.15]))
    scaled = rigid.copy()
    scaled[:3, :] *= 1.7
    assert loops.scale_of(scaled) == pytest.approx(1.7)
    np.testing.assert_allclose(loops.rigid_of(scaled), rigid, atol=1e-15)


def test_the_ransac_budget_is_orb_slam_s() -> None:
    """``SetRansacParameters(0.99, 15, 300)``: the bound for 15 inliers, at most 300, else 1."""
    assert loops.ransac_budget(15, OPTIONS) == 1
    assert loops.ransac_budget(10, OPTIONS) == 1
    assert loops.ransac_budget(30, OPTIONS) == 35  # ceil(log 0.01 / log(1 - 0.5³))
    assert loops.ransac_budget(1000, OPTIONS) == 300


# --- searches -----------------------------------------------------------------------------


def test_a_keyframe_s_own_points_are_found_through_its_own_pose(loop_rigid: Loop) -> None:
    """Searched through its own pose, most of a keyframe's points come back to their keypoints.

    Four in five at least: the rest go to a keypoint with a nearer descriptor in the window, or
    are seen too obliquely; none is matched twice.
    """
    m = loop_rigid.truth
    for k in (0, MIDDLE, m.keyframes - 1):
        rows = _rows(m, k)
        seen = m.kp_point[rows]
        found = loops.search_by_similarity(
            m, CAMERA, k, m.kf_pose[k], m.point_rows(seen[seen >= 0]),
            window=OPTIONS.loops.fine_window, distance=OPTIONS.loops.fine_distance,
            options=OPTIONS,
        )  # fmt: skip
        assert len(found) == len(rows)
        same = np.count_nonzero((found >= 0) & (found == seen))
        assert same >= 0.8 * np.count_nonzero(seen >= 0)
        matched = found[found >= 0]
        assert len(np.unique(matched)) == len(matched)


def test_a_search_s_distance_bounds_its_matches(loop_rigid: Loop) -> None:
    """At distance 0, every match's descriptor is its point's exactly."""
    m = loop_rigid.truth
    k = MIDDLE
    seen = m.kp_point[m.keypoint_rows(k)]
    found = loops.search_by_similarity(
        m, CAMERA, k, m.kf_pose[k], m.point_rows(seen[seen >= 0]), window=5.0, distance=0,
        options=OPTIONS,
    )  # fmt: skip
    rows = m.kf_start[k] + np.flatnonzero(found >= 0)
    assert len(rows) > 0
    points = m.point_rows(found[found >= 0])
    np.testing.assert_array_equal(
        matching.distances(m.pt_descriptor[points], m.kp_descriptor[rows]).diagonal(), 0
    )


def test_matches_by_word_are_where_the_keypoints_are(
    loop_rigid: Loop, loop_database: Database
) -> None:
    """Between neighbours, nine in ten matches by word put a point by its keypoint.

    The point matched in the second keyframe projects into the first, by the truth, within 3
    pixels at the keypoint's level; only keypoints that observe a point are matched.
    """
    m = loop_rigid.truth
    for k in (1, MIDDLE, m.keyframes - 1):
        found = loops.match_by_word(m, loop_database, k, k - 1, OPTIONS)
        rows = _rows(m, k)
        matched = found >= 0
        assert np.count_nonzero(matched) >= 20
        assert np.all(m.kp_point[rows[matched]] >= 0)
        local = maps.to_camera(m.kf_pose[k], m.pt_xyz[m.point_rows(found[matched])])
        gap = np.linalg.norm(CAMERA.project(local) - m.kp_xy[rows[matched]], axis=1)
        near = gap < 3.0 * m.scale_factor ** m.kp_level[rows[matched]]
        assert np.count_nonzero(near) >= 0.9 * np.count_nonzero(matched)


def test_a_group_s_points_are_its_keyframes_once_each(loop_rigid: Loop) -> None:
    """The matched keyframe's and its best links' points, each once, never a linked one's."""
    m = loop_rigid.truth
    rows = loops.group_points(m, 2, [], OPTIONS)
    assert len(np.unique(rows)) == len(rows)
    links = covisibility.links(m, 2, OPTIONS)[0][: OPTIONS.loops.neighbours]
    expected = set()
    for k in [*links.tolist(), 2]:
        seen = m.kp_point[m.keypoint_rows(k)]
        expected |= set(seen[seen >= 0].tolist())
    assert expected <= set(m.pt_id[rows].tolist())


# --- the similarity -----------------------------------------------------------------------


def _shared(m: Map) -> tuple[int, int, np.ndarray, np.ndarray]:
    """Return two neighbours half way round, the first's points, and those the second sees too.

    The neighbours' rows, the point id of each keypoint of the first (-1 for none), and a mask
    of its keypoints whose point the second also observes.
    """
    k1, k2 = MIDDLE, MIDDLE - 1
    own = m.kp_point[m.keypoint_rows(k1)]
    shared = np.isin(own, m.kp_point[m.keypoint_rows(k2)]) & (own >= 0)
    return k1, k2, own, shared


@pytest.mark.parametrize("fixed_scale", [True, False])
def test_the_refinement_pulls_a_similarity_back_to_the_truth(
    loop_rigid: Loop, fixed_scale: bool
) -> None:
    """Between neighbours matched by their shared points, a nudged start comes back.

    ``S_12 = T_1 T_2⁻¹``; the start is off by 2° and 5 cm, and by 5% in scale when free.
    """
    m = loop_rigid.truth
    k1, k2, own, shared = _shared(m)
    matched = np.where(shared, own, -1)
    truth = m.kf_pose[k1] @ se3.inverse(m.kf_pose[k2])
    start = truth.copy()
    start[:3, :3] = so3.exp([0.0, np.radians(2.0), 0.0]) @ start[:3, :3]
    start[:3, 3] += [0.05, 0.0, -0.03]
    if not fixed_scale:
        start[:3, :] *= 1.05
    refined = loops.refine(
        m, CAMERA, k1, k2, similarity=start, matched=matched, fixed_scale=fixed_scale,
        options=OPTIONS,
    )  # fmt: skip
    assert refined is not None
    assert refined.inliers >= 0.9 * np.count_nonzero(matched >= 0)
    np.testing.assert_allclose(refined.similarity, truth, atol=0.01)
    if fixed_scale:
        assert loops.scale_of(refined.similarity) == pytest.approx(1.0, abs=1e-12)


def test_the_refinement_gives_up_with_too_few_matches(loop_rigid: Loop) -> None:
    """Fewer than ``loops.min_kept`` matches surviving the first round, or none at all: None."""
    m = loop_rigid.truth
    k1, k2, own, shared = _shared(m)
    few = np.flatnonzero(shared)[: OPTIONS.loops.min_kept - 1]
    matched = np.full(len(own), -1, dtype=np.int64)
    matched[few] = own[few]
    truth = m.kf_pose[k1] @ se3.inverse(m.kf_pose[k2])
    for given in (matched, np.full(len(own), -1, dtype=np.int64)):
        refined = loops.refine(
            m, CAMERA, k1, k2, similarity=truth, matched=given, fixed_scale=True, options=OPTIONS
        )
        assert refined is None


# --- detection ----------------------------------------------------------------------------


def test_a_loop_closes_where_the_path_comes_back(loop: Loop, loop_database: Database) -> None:
    """The loop closes at once on the second last keyframe, matched to the loop's start.

    Its similarity places the keyframe within 3 cm of its true place, and recovers the drift's
    scale to within 5%; it is rigid in RGB-D.
    """
    found = _detect(loop, loop_database, LAST_CONFIRMED)
    assert found.summary.outcome is Outcome.CLOSED
    assert found.pending is None
    assert found.loop is not None
    k = loop.map.keyframes + LAST_CONFIRMED
    assert found.loop.current == int(loop.map.kf_id[k])
    assert found.loop.matched <= 4
    assert found.summary.confirmations == OPTIONS.loops.confirmations
    placed = loops.rigid_of(found.loop.similarity)
    assert np.linalg.norm(maps.camera_centre(placed) - maps.camera_centre(loop.poses[k])) < 0.03
    drift = loops.scale_of(loop.drift[k]) / loops.scale_of(loop.drift[found.loop.matched])
    scale = loops.scale_of(found.loop.similarity)
    if loop.scaled:
        assert scale == pytest.approx(drift, rel=0.05)
    else:
        assert scale == pytest.approx(1.0, abs=1e-12)
    tried = [c for c in found.summary.candidates if not c.near]
    assert any(c.confirmations == OPTIONS.loops.confirmations for c in tried)
    json.dumps(found.summary.as_dict(), allow_nan=False)


def test_nothing_is_found_half_way_round(loop: Loop, loop_database: Database) -> None:
    """Half way round, nothing has been seen before: no loop, none carried."""
    found = _detect(loop, loop_database, MIDDLE)
    assert found.summary.outcome is Outcome.NONE
    assert (found.pending, found.loop) == (None, None)
    assert found.summary.confirmations == 0


def test_a_small_map_is_not_searched(loop: Loop, loop_database: Database) -> None:
    """Below ``loops.min_keyframes`` keyframes, detection skips, carrying what it was given."""
    pending = Pending(
        matched=0, last=1, similarity=np.eye(4), confirmations=1, misses=0,
        points=np.zeros(0, dtype=np.int64), matched_points=np.zeros(0, dtype=np.int64),
    )  # fmt: skip
    found = _detect(loop, loop_database, -1, pending, min_keyframes=loop.map.keyframes + 1)
    assert found.summary.outcome is Outcome.SKIPPED
    assert found.pending is pending
    assert found.loop is None
    assert found.summary.confirmations == 1


def test_a_loop_short_of_confirmations_is_carried_until_it_closes(
    loop: Loop, loop_database: Database
) -> None:
    """With one confirmation more needed than its links give, the loop is carried, then closes.

    Started on the second last keyframe, held by it and the matched one, it is confirmed on the
    last, and closes there.
    """
    needed = OPTIONS.loops.confirmations + 1
    started = _detect(loop, loop_database, -2, confirmations=needed)
    assert started.summary.outcome is Outcome.STARTED
    assert started.loop is None
    pending = started.pending
    assert pending is not None
    assert pending.confirmations == OPTIONS.loops.confirmations
    assert pending.last == int(loop.map.kf_id[-2])
    np.testing.assert_array_equal(pending.held, sorted({pending.matched, pending.last}))
    closed = _detect(loop, loop_database, -1, pending, confirmations=needed)
    assert closed.summary.outcome is Outcome.CLOSED
    assert closed.summary.confirmation is not None
    assert closed.summary.confirmation.confirmed
    assert closed.summary.candidates == ()  # confirmed: no search
    assert closed.loop is not None
    assert (closed.loop.current, closed.loop.matched) == (int(loop.map.kf_id[-1]), pending.matched)


def test_a_loop_unconfirmed_too_often_is_dropped(loop: Loop, loop_database: Database) -> None:
    """Carried to a keyframe that does not see it, a loop misses; ``loops.misses`` drop it.

    The keyframe half way round, given a loop placing it at the start, cannot confirm it.
    """
    started = _detect(loop, loop_database, -2, confirmations=4)
    assert started.pending is not None
    stray = replace(started.pending, last=int(loop.map.kf_id[MIDDLE - 1]))
    missed = _detect(loop, loop_database, MIDDLE, stray, confirmations=4)
    assert missed.summary.outcome is Outcome.NONE
    assert missed.pending is not None
    assert missed.pending.misses == 1
    assert missed.summary.confirmation is not None
    assert not missed.summary.confirmation.confirmed
    dropped = _detect(loop, loop_database, MIDDLE, missed.pending, confirmations=4)
    assert dropped.summary.outcome is Outcome.DROPPED
    assert (dropped.pending, dropped.loop) == (None, None)


def test_detection_refuses_a_keyframe_outside_the_map(loop: Loop, loop_database: Database) -> None:
    """A keyframe the map does not hold: InvalidInputError."""
    with pytest.raises(InvalidInputError, match="not in the map"):
        loops.detect(
            loop.map, loop_database, CAMERA, loop.map.next_kf_id, None, mode=_mode(loop),
            options=OPTIONS,
        )  # fmt: skip


# --- correction ---------------------------------------------------------------------------


def _closed(loop: Loop, db: Database) -> loops.Loop:
    """Return the loop closed on the second last keyframe."""
    found = _detect(loop, db, LAST_CONFIRMED)
    assert found.loop is not None
    return found.loop


def test_the_essential_graph_takes_the_drift_out(loop: Loop, loop_database: Database) -> None:
    """Without global BA, the correction brings the loop's end back within 3 cm of its place.

    The drift had carried it 7 cm and more; the rest of the drift is spread round the loop,
    every keyframe's error at most half the largest before. The loop edge joins the two.
    """
    closed = _closed(loop, loop_database)
    before = _position_errors(loop.map, loop.poses)
    assert before[-4:].min() > 0.07
    fixed = loops.correct(
        loop.map, CAMERA, closed, mode=_mode(loop),
        options=OPTIONS.with_settings({"loops.global_ba": False}),
    )  # fmt: skip
    after = _position_errors(fixed.map, loop.poses)
    assert after[-4:].max() < 0.03
    assert after.max() < before.max() / 2
    m = fixed.map
    np.testing.assert_array_equal(m.loop_edges, [[closed.matched, closed.current]])
    s = fixed.summary
    assert (s.current, s.matched) == (closed.current, closed.matched)
    assert s.global_ba is None
    assert s.essential.final_error < s.essential.initial_error
    assert s.corrected == 1 + len(
        covisibility.links(loop.map, m.keyframes + LAST_CONFIRMED, OPTIONS)[0]
    )
    assert s.moved > 0
    assert s.replaced > 0
    assert s.fused > 0
    assert s.links > 0
    assert m.points == loop.map.points - len(m.gone_pt_id) + len(loop.map.gone_pt_id)
    json.dumps(s.as_dict(), allow_nan=False)


def test_global_ba_follows_the_correction(loop: Loop, loop_database: Database) -> None:
    """With global BA, every keyframe ends within 3.5 cm of its place, the first one held.

    Each point fused at the loop was made twice, from two keypoints a few pixels apart, so its
    observations disagree by as much; with no robust loss, global BA's fit moves the map a
    couple of centimetres for them.
    """
    closed = _closed(loop, loop_database)
    fixed = loops.correct(loop.map, CAMERA, closed, mode=_mode(loop), options=OPTIONS)
    assert fixed.summary.global_ba is not None
    after = _position_errors(fixed.map, loop.poses)
    assert after.max() < 0.035
    graph_only = loops.correct(
        loop.map, CAMERA, closed, mode=_mode(loop),
        options=OPTIONS.with_settings({"loops.global_ba": False}),
    )  # fmt: skip
    first = int(np.flatnonzero(fixed.map.kf_id == 0)[0])
    np.testing.assert_allclose(fixed.map.kf_pose[first], graph_only.map.kf_pose[first], atol=1e-12)


def test_global_ba_finds_an_exact_map_from_a_nudged_one(loop_rigid: Loop) -> None:
    """Every keyframe but the first nudged: global BA puts them back, the observations exact.

    Its damping, g2o's, scaled to the Hessian (:func:`slam_it.edge.loops.global_ba_options`),
    takes the last digits more slowly than GTSAM's own, whose ``λ`` fell to 1e-15 in 10
    iterations: after its 10, the cost is 8e-14 of its start and the poses within 2e-6 of the
    truth (since 0.3.18.0; within 1e-6 before).
    """
    exact = loop_rigid.truth
    rng = np.random.default_rng(0)
    poses = exact.kf_pose.copy()
    for k in range(1, exact.keyframes):
        poses[k] = (
            se3.exp(np.concatenate([rng.normal(0, 0.005, 3), rng.normal(0, 0.002, 3)])) @ poses[k]
        )
    adjusted, summary = loops.global_bundle_adjust(replace(exact, kf_pose=poses), CAMERA, OPTIONS)
    assert summary.final_error < 1e-6 * summary.initial_error
    np.testing.assert_allclose(adjusted.kf_pose, exact.kf_pose, atol=3e-6)
    np.testing.assert_allclose(adjusted.pt_xyz, exact.pt_xyz, atol=1e-5)


def test_a_global_ba_stops_between_iterations(loop_rigid: Loop) -> None:
    """A global BA superseded stops before its next iteration, as ORB-SLAM's mbStopGBA stops it."""
    exact = loop_rigid.truth
    poses = exact.kf_pose.copy()
    poses[1:] = se3.exp(np.array([0.004, -0.003, 0.002, 0.001, 0.002, -0.001])) @ poses[1:]
    asked: list[bool] = []

    def stop() -> bool:
        asked.append(True)
        return len(asked) > 1  # one iteration, then stop

    nudged = replace(exact, kf_pose=poses)
    _, summary = loops.global_bundle_adjust(nudged, CAMERA, OPTIONS, stop=stop)
    assert (summary.iterations, summary.termination) == (1, graphs.Termination.STOPPED)
    assert summary.final_error < summary.initial_error
    with pytest.raises(InvalidInputError, match="stop"):
        loops.global_bundle_adjust(nudged, CAMERA, OPTIONS, stop=stop, backend="cpp")


def test_the_essential_graph_keeps_a_consistent_map(loop_rigid: Loop) -> None:
    """Started where the poses were, with no new links: nothing moves."""
    m = loop_rigid.truth
    poses = list(m.kf_pose)
    optimised, edges, summary = loops.essential_graph(
        m, poses, poses, {}, (m.keyframes - 1, 0), fixed=[0], fixed_scale=True, options=OPTIONS
    )
    assert edges >= m.keyframes - 1  # the spanning tree at least
    assert summary.final_error < 1e-20
    np.testing.assert_allclose(np.array(optimised), m.kf_pose, atol=1e-9)


def test_points_are_fused_into_a_keyframe_through_its_pose(loop_rigid: Loop) -> None:
    """Points a keyframe lost track of come back to it, onto their keypoints.

    A third of its points that another keyframe also observes are taken from it, then fused
    back through its own pose. Two in five at least return (the rest fail the search's tests:
    their other observers predict another level, or a narrower range of distances), four in
    five of those to their own keypoints.
    """
    m = loop_rigid.truth
    k = MIDDLE
    rows = _rows(m, k)
    observers = np.bincount(m.point_rows(m.kp_point[m.kp_point >= 0]), minlength=m.points)
    seen = np.flatnonzero((m.kp_point[rows] >= 0) & (observers[m.point_rows(m.kp_point[rows])] > 1))
    dropped = seen[::3]
    lost = m.kp_point[rows[dropped]].copy()
    kp_point = m.kp_point.copy()
    kp_point[rows[dropped]] = -1
    stripped = maps.refresh_points(replace(m, kp_point=kp_point), np.arange(m.points))
    fused, count = loops.fuse_by_similarity(
        stripped, CAMERA, k, m.kf_pose[k], lost, options=OPTIONS
    )
    assert count >= 0.4 * len(lost)
    back = fused.kp_point[fused.keypoint_rows(k)]
    assert np.count_nonzero(np.isin(lost, back)) == count
    assert np.count_nonzero(back[dropped] == lost) >= 0.8 * count


def test_correction_refuses_a_keyframe_outside_the_map(loop_rigid: Loop) -> None:
    """A loop naming a keyframe the map does not hold: InvalidInputError."""
    m = loop_rigid.map
    stray = loops.Loop(
        current=m.next_kf_id, matched=0, similarity=np.eye(4),
        points=np.zeros(0, dtype=np.int64), matched_points=np.zeros(0, dtype=np.int64),
    )  # fmt: skip
    with pytest.raises(InvalidInputError, match="not in the map"):
        loops.correct(m, CAMERA, stray, mode="rgbd", options=OPTIONS)


def _stray_loop(m: Map, **changes: object) -> loops.Loop:
    """Return a well-formed loop from the second keyframe to the first, one field changed."""
    current = 1
    keypoints = int(m.kf_start[current + 1] - m.kf_start[current])
    fields: dict[str, object] = {
        "current": int(m.kf_id[current]), "matched": int(m.kf_id[0]), "similarity": np.eye(4),
        "points": np.zeros(0, dtype=np.int64),
        "matched_points": np.full(keypoints, -1, dtype=np.int64),
    }  # fmt: skip
    fields.update(changes)
    return loops.Loop(**fields)  # type: ignore[arg-type]


def _without_first(m: Map) -> Map:
    """Return the map with its first keyframe (id 0) erased, the second its children's parent."""
    second = int(m.kf_id[1])
    parents = np.where(m.kf_parent == 0, second, m.kf_parent)
    parents[1] = -1
    return maps.erase_keyframe(replace(m, kf_parent=parents), 0)


BAD_CALLS: dict[str, tuple[str, Callable[[Map, Database], object]]] = {
    "a similarity that is not one": (
        "similarity",
        lambda m, db: loops.search_by_similarity(
            m, CAMERA, 0, -np.eye(4), [], window=5.0, distance=50, options=OPTIONS
        ),
    ),
    "a window not finite": (
        "window",
        lambda m, db: loops.search_by_similarity(
            m, CAMERA, 0, np.eye(4), [], window=float("nan"), distance=50, options=OPTIONS
        ),
    ),
    "a window not positive": (
        "window",
        lambda m, db: loops.search_by_similarity(
            m, CAMERA, 0, np.eye(4), [], window=-8.0, distance=50, options=OPTIONS
        ),
    ),
    "a negative distance": (
        "distance",
        lambda m, db: loops.search_by_similarity(
            m, CAMERA, 0, np.eye(4), [], window=5.0, distance=-5, options=OPTIONS
        ),
    ),
    "a distance past a descriptor's bits": (
        "distance",
        lambda m, db: loops.search_by_similarity(
            m, CAMERA, 0, np.eye(4), [], window=5.0, distance=10**9, options=OPTIONS
        ),
    ),
    "a linked row not a keyframe's": (
        "linked",
        lambda m, db: loops.group_points(m, 0, [m.keyframes], OPTIONS),
    ),
    "a negative point id to fuse": (
        "point_ids",
        lambda m, db: loops.fuse_by_similarity(m, CAMERA, 0, np.eye(4), [-1], options=OPTIONS),
    ),
    "an unknown mode to detect in": (
        "mode",
        lambda m, db: loops.detect(m, db, CAMERA, int(m.kf_id[-1]), None, mode="lidar",
                                   options=OPTIONS),
    ),
    "a pending loop's similarity that is not one": (
        "pending.similarity",
        lambda m, db: loops.detect(
            m, db, CAMERA, int(m.kf_id[-1]),
            Pending(matched=0, last=1, similarity=np.zeros((4, 4)), points=np.zeros(0, np.int64),
                    matched_points=np.zeros(0, np.int64), confirmations=1, misses=0),
            mode="rgbd", options=OPTIONS,
        ),
    ),
    "a loop's similarity that is not one": (
        "loop.similarity",
        lambda m, db: loops.correct(
            m, CAMERA, _stray_loop(m, similarity=-np.eye(4)), mode="rgbd", options=OPTIONS
        ),
    ),
    "matched points not one per keypoint": (
        "loop.matched_points",
        lambda m, db: loops.correct(
            m, CAMERA, _stray_loop(m, matched_points=np.zeros(1, np.int64)), mode="rgbd",
            options=OPTIONS,
        ),
    ),
    "a loop point id below -1": (
        "loop.points",
        lambda m, db: loops.correct(
            m, CAMERA, _stray_loop(m, points=np.array([-2])), mode="rgbd", options=OPTIONS
        ),
    ),
    "a graph's start short of the keyframes": (
        "start",
        lambda m, db: loops.essential_graph(
            m, [np.eye(4)] * (m.keyframes - 1), [np.eye(4)] * m.keyframes, {}, (0, 1),
            fixed=[1], fixed_scale=True, options=OPTIONS,
        ),
    ),
    "a graph's link not a keyframe's row": (
        "loop_links",
        lambda m, db: loops.essential_graph(
            m, [np.eye(4)] * m.keyframes, [np.eye(4)] * m.keyframes, {0: {m.keyframes}}, (0, 1),
            fixed=[1], fixed_scale=True, options=OPTIONS,
        ),
    ),
    "a graph whose two ends are one keyframe": (
        "ends",
        lambda m, db: loops.essential_graph(
            m, [np.eye(4)] * m.keyframes, [np.eye(4)] * m.keyframes, {}, (1, 1),
            fixed=[1], fixed_scale=True, options=OPTIONS,
        ),
    ),
    "a global BA's map without its first keyframe": (
        "m: expected the map's first keyframe",
        lambda m, db: loops.global_bundle_adjust(_without_first(m), CAMERA, OPTIONS),
    ),
}  # fmt: skip
"""Each public function given one bad argument, and the name its refusal must give."""


@pytest.mark.parametrize("case", sorted(BAD_CALLS))
def test_bad_arguments_are_refused_by_name(
    case: str, loop_rigid: Loop, loop_database: Database
) -> None:
    """Each bad argument is refused with InvalidInputError, naming it (B3's review panel, H4)."""
    named, call = BAD_CALLS[case]
    with pytest.raises(InvalidInputError, match=named):
        call(loop_rigid.map, loop_database)


# --- every backend ------------------------------------------------------------------------


def _assert_same_ids(got: np.ndarray, expected: np.ndarray) -> None:
    """Check two id vectors are the same, type and all."""
    assert got.dtype == np.int64
    np.testing.assert_array_equal(got, expected)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_searches_as_python_does(
    backend: str, loop_rigid: Loop, loop_database: Database
) -> None:
    """The similarity search, the matches by word and a group's points: exactly Python's."""
    for name in ("search_by_similarity", "match_by_word", "group_points"):
        require_function(backend, loops.__name__, name)
    m = loop_rigid.map
    for k in (1, MIDDLE, m.keyframes - 1):
        linked = covisibility.links(m, k, OPTIONS)[0]
        points = loops.group_points(m, 2, linked, OPTIONS)
        _assert_same_ids(loops.group_points(m, 2, linked, OPTIONS, backend=backend), points)
        for window, distance in ((8.0, 75), (4.0, 30)):
            searched = [
                loops.search_by_similarity(
                    m, CAMERA, k, m.kf_pose[k], points, window=window, distance=distance,
                    options=OPTIONS, backend=chosen,
                )
                for chosen in ("python", backend)
            ]  # fmt: skip
            _assert_same_ids(searched[1], searched[0])
        by_word = [
            loops.match_by_word(m, loop_database, k, k - 1, OPTIONS, backend=b)
            for b in ("python", backend)
        ]
        _assert_same_ids(by_word[1], by_word[0])


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_refines_as_python_does(backend: str, loop_rigid: Loop) -> None:
    """The same inliers, the similarity to GTSAM's convergence; None where Python's is."""
    require_function(backend, loops.__name__, "refine")
    m = loop_rigid.truth
    k1, k2, own, shared = _shared(m)
    rigid = m.kf_pose[k1] @ se3.inverse(m.kf_pose[k2])
    scaled = rigid.copy()
    scaled[:3, :] *= 1.03
    few = np.where(np.cumsum(shared) < OPTIONS.loops.min_kept, np.where(shared, own, -1), -1)
    for matched in (np.where(shared, own, -1), few):
        for fixed_scale, start in ((True, rigid), (False, scaled)):
            expected, got = (
                loops.refine(
                    m, CAMERA, k1, k2, similarity=start, matched=matched,
                    fixed_scale=fixed_scale, options=OPTIONS, backend=chosen,
                )
                for chosen in ("python", backend)
            )  # fmt: skip
            if expected is None:
                assert got is None
                continue
            assert got is not None
            assert got.inliers == expected.inliers
            assert_close(got.similarity, expected.similarity, GRAPH_TOLERANCE)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_fuses_as_python_does(backend: str, loop_rigid: Loop) -> None:
    """The loop's start fused into its end, undrifted: the same map, exactly.

    The end's points where the start's are get replaced, its free keypoints given them.
    """
    require_function(backend, loops.__name__, "fuse_by_similarity")
    m = loop_rigid.truth
    start = m.kp_point[np.concatenate([_rows(m, k) for k in range(4)])]
    ids = np.unique(start[start >= 0])
    k = m.keyframes - 1
    expected, got = (
        loops.fuse_by_similarity(m, CAMERA, k, m.kf_pose[k], ids, options=OPTIONS, backend=chosen)
        for chosen in ("python", backend)
    )
    assert got[1] == expected[1] > 0
    assert_maps_match(got[0], expected[0], TOLERANCE)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_adjusts_the_whole_map_as_python_does(backend: str, loop: Loop) -> None:
    """Global BA on the drifted map: the same map to GTSAM's convergence, the same solve."""
    require_function(backend, loops.__name__, "global_bundle_adjust")
    expected, got = (
        loops.global_bundle_adjust(loop.map, CAMERA, OPTIONS, backend=chosen)
        for chosen in ("python", backend)
    )
    assert_maps_match(got[0], expected[0], GRAPH_TOLERANCE)
    assert (got[1].iterations, got[1].termination) == (
        expected[1].iterations,
        expected[1].termination,
    )
    assert got[1].final_error == pytest.approx(expected[1].final_error, rel=GRAPH_TOLERANCE["rtol"])


MOVED_GLOBAL_BA = {
    "loops.global_damping_scale": 1e-2,
    "loops.global_small_steps": 1,
    "loops.global_small_decrease": 0.5,
    "loops.min_depth_ratio": 0.9,
}
"""Global BA's settings off their defaults: a heavier first damping, a solve ended by its first
iteration's decrease under half, and the observations nearer than 0.9 of the median depth left
out."""


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_reads_global_ba_s_settings(backend: str, loop: Loop) -> None:
    """Off their defaults, global BA's settings change each backend's solve, as Python's.

    So each native reads them (review panel finding H21: a native that hard-coded them would
    solve alike either way).
    """
    require_function(backend, loops.__name__, "global_bundle_adjust")
    moved = OPTIONS.with_settings(MOVED_GLOBAL_BA)
    expected, got = (
        loops.global_bundle_adjust(loop.map, CAMERA, moved, backend=chosen)
        for chosen in ("python", backend)
    )
    assert_maps_match(got[0], expected[0], GRAPH_TOLERANCE)
    assert (got[1].iterations, got[1].termination) == (1, expected[1].termination)
    assert got[1].initial_error == pytest.approx(expected[1].initial_error, rel=1e-12)
    _, default = loops.global_bundle_adjust(loop.map, CAMERA, OPTIONS, backend=backend)
    assert got[1].initial_error < default.initial_error  # the near-plane filter left some out
    assert default.iterations > 1


SLIDE = load_fixture("edge/global_ba_slide.json")
"""A monocular global BA whose scale GTSAM's damping lets slide (generate_global_ba_slide.py)."""

SCALE_TOLERANCE = 0.01
"""How far global BA may change a monocular map's scale: the scale of the similarity that carries
its keyframes' centres onto where they started, within 1% of 1. On :data:`SLIDE` its own
correction of the map moves it by 0.3% (0.9968); GTSAM's damping slid it to 0.7322."""


def _slide_problem() -> dict[str, Any]:
    """Return :data:`SLIDE`'s bundle as ``graphs.bundle_adjust`` takes it."""
    problem = SLIDE["problem"]
    count = len(problem["cameras"])
    return {
        "poses": np.array(problem["poses"]), "points": np.array(problem["points"]),
        "cameras": np.array(problem["cameras"], dtype=np.int64),
        "point_ids": np.array(problem["point_ids"], dtype=np.int64),
        "pixels": np.array(problem["pixels"]), "right": np.full(count, np.nan),
        "information": np.array(problem["information"]), "model": SLIDE["camera"],
        "intrinsics": np.array(problem["intrinsics"]), "bf": 0.0,
        "fixed": np.array(problem["fixed"], dtype=np.int64),
    }  # fmt: skip


def _centres_scale(before: NDArray[np.float64], after: NDArray[np.float64]) -> float:
    """Return the scale of the similarity carrying the cameras' centres `before` onto `after`."""
    return alignment.align(before[:, :3, 3], after[:, :3, 3], with_scale=True).scale


def _shape_ratio(
    poses: NDArray[np.float64], points: NDArray[np.float64], bundle: dict[str, Any]
) -> float:
    """Return a map's gauge-free shape: its cameras' mean step over its observations' median depth.

    A similarity of the whole map (the monocular gauge) leaves it unchanged.
    """
    step = float(np.mean(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)))
    cameras, ids = bundle["cameras"], bundle["point_ids"]
    depth = np.einsum("mi,mi->m", poses[cameras, :3, 2], points[ids] - poses[cameras, :3, 3])
    return step / float(np.median(depth))


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_global_ba_holds_a_monocular_map_s_scale(backend: str) -> None:
    """Global BA's solve keeps a monocular map's scale where GTSAM's damping let it slide (L4).

    A monocular global BA holding one keyframe has a free scale, the exact gauge, which no
    Levenberg-Marquardt step moves along (each step is orthogonal to it); beside it lies a
    near-flat valley, the cameras and the well-constrained points rescaling while far, weakly
    constrained points barely move, at almost no cost. From GTSAM's absolute ``λ`` of 1e-5,
    divided by 10 a step, the solve drifts along that valley (the centres' scale 0.7322 here,
    0.5508 on the whole problem); from g2o's, scaled to the Hessian's largest diagonal entry,
    with ORB-SLAM3's stop and the near-plane filter
    (:func:`slam_it.edge.loops.global_ba_options`), the damping holds those near-null directions
    and the scale stays within :data:`SCALE_TOLERANCE` (0.3.17.2's review panel, L4; the
    0.3.18.0 panel's L2). Both make the same gauge-free correction (:func:`_shape_ratio` agrees
    within 1% and moves by over 10%) and end at the same cost: the difference is drift alone.
    """
    chosen = require_function(backend, graphs.__name__, "bundle_adjust")
    bundle = _slide_problem()
    settings = Options()
    held = graphs.bundle_adjust(
        **bundle, near=settings.loops.min_depth_ratio, options=loops.global_ba_options(settings),
        backend=chosen,
    )  # fmt: skip
    assert abs(_centres_scale(bundle["poses"], held.poses) - 1.0) < SCALE_TOLERANCE
    slid = graphs.bundle_adjust(
        **bundle, options=graphs.Options(max_iterations=settings.loops.global_iterations),
        backend=chosen,
    )  # fmt: skip
    # Far past the tolerance, so the test tells the two dampings apart (0.7322 measured).
    assert _centres_scale(bundle["poses"], slid.poses) < 1.0 - 20 * SCALE_TOLERANCE
    assert slid.summary.final_error == pytest.approx(held.summary.final_error, rel=1e-3)
    start = _shape_ratio(bundle["poses"], bundle["points"], bundle)
    corrected = _shape_ratio(held.poses, held.points, bundle)
    assert _shape_ratio(slid.poses, slid.points, bundle) == pytest.approx(corrected, rel=1e-2)
    assert abs(corrected / start - 1.0) > 0.1


def test_global_ba_damps_as_g2o_and_stops_as_orb_slam3_and_the_weld_does_neither() -> None:
    """Global BA's solve: g2o's damping, ORB-SLAM3's stop; a welding BA's: GTSAM's damping alone."""
    settings = Options()
    assert loops.global_ba_options(settings) == graphs.Options(
        max_iterations=10, scaled_damping=True, damping_scale=1e-5, small_steps=3,
        small_decrease=1e-3,
    )  # fmt: skip
    assert loops.weld_options(5) == graphs.Options(max_iterations=5)
    assert not loops.weld_options(5).scaled_damping
    assert loops.weld_options(5).small_steps == 0


def _assert_same_detection(got: loops.Detection, expected: loops.Detection) -> None:
    """Check two detections: the summaries' decisions exactly, the similarities to GTSAM's."""
    assert_records_match(got.summary.as_dict(), expected.summary.as_dict(), GRAPH_TOLERANCE)
    for a, b in ((got.pending, expected.pending), (got.loop, expected.loop)):
        assert (a is None) == (b is None)
        if a is None or b is None:
            continue
        assert_close(a.similarity, b.similarity, GRAPH_TOLERANCE)
        _assert_same_ids(a.points, b.points)
        _assert_same_ids(a.matched_points, b.matched_points)
        assert (
            replace(a, similarity=b.similarity, points=b.points, matched_points=b.matched_points)
            == b
        )


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_detects_as_python_does(
    backend: str, loop: Loop, loop_database: Database
) -> None:
    """Half way round, closing at once, started and then carried: as Python decides."""
    require_function(backend, loops.__name__, "detect")
    m = loop.map
    needed = OPTIONS.loops.confirmations + 1
    cases: list[tuple[int, Pending | None, Options]] = [
        (MIDDLE, None, OPTIONS),
        (LAST_CONFIRMED, None, OPTIONS),
        (-2, None, OPTIONS.with_settings({"loops.confirmations": needed})),
    ]
    started = _detect(loop, loop_database, -2, confirmations=needed).pending
    assert started is not None
    cases.append((-1, started, OPTIONS.with_settings({"loops.confirmations": needed})))
    for row, pending, options in cases:
        keyframe = int(m.kf_id[row])
        expected, got = (
            loops.detect(
                m, loop_database, CAMERA, keyframe, pending, mode=_mode(loop), options=options,
                backend=chosen,
            )
            for chosen in ("python", backend)
        )  # fmt: skip
        _assert_same_detection(got, expected)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize("global_ba", [False, True], ids=["graph only", "global BA"])
def test_every_backend_corrects_as_python_does(
    backend: str, global_ba: bool, loop: Loop, loop_database: Database
) -> None:
    """The same map after the correction, to GTSAM's convergence; the same counts exactly."""
    require_function(backend, loops.__name__, "correct")
    closed = _closed(loop, loop_database)
    options = OPTIONS.with_settings({"loops.global_ba": global_ba})
    expected, got = (
        loops.correct(loop.map, CAMERA, closed, mode=_mode(loop), options=options, backend=chosen)
        for chosen in ("python", backend)
    )
    assert_maps_match(got.map, expected.map, GRAPH_TOLERANCE)
    assert_records_match(got.summary.as_dict(), expected.summary.as_dict(), GRAPH_TOLERANCE)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_core_refuses_a_keyframe_outside_the_map(
    backend: str, loop_rigid: Loop, loop_database: Database
) -> None:
    """Python refuses it before any backend runs; each core refuses it too."""
    for name in ("detect", "correct"):
        require_function(backend, loops.__name__, name)
    compiled = backends.implementation(backend, loops.__name__)
    m = loop_rigid.map
    stray = m.next_kf_id
    with pytest.raises(ValueError, match="not in the map"):
        compiled.detect(
            m=m.as_dict(), database=loop_database.as_dict(), camera=CAMERA.as_dict(),
            keyframe=stray, pending=None, mode="rgbd", options=OPTIONS.as_dict(),
        )  # fmt: skip
    loop = {
        "current": stray, "matched": 0, "similarity": np.eye(4),
        "points": np.zeros(0, dtype=np.int64), "matched_points": np.zeros(0, dtype=np.int64),
    }  # fmt: skip
    with pytest.raises(ValueError, match="not in the map"):
        compiled.correct(
            m=m.as_dict(), camera=CAMERA.as_dict(), loop=loop, mode="rgbd",
            options=OPTIONS.as_dict(),
        )  # fmt: skip


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_the_compiled_codes_match(backend: str) -> None:
    """The outcome and termination codes Python decodes are each compiled core's, in order."""
    chosen = require_function(backend, loops.__name__, "codes")
    assert backends.implementation(chosen, loops.__name__).codes() == loops.codes()


def _moved_world(m: Map, motion: np.ndarray) -> Map:
    """Return `m` with its world moved by ``motion`` (``X' = motion X``)."""
    xyz = m.pt_xyz @ motion[:3, :3].T + motion[:3, 3]
    return replace(m, kf_pose=m.kf_pose @ se3.inverse(motion), pt_xyz=xyz)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_global_ba_carries_what_was_made_since(backend: str, loop_rigid: Loop) -> None:
    """Solved on an earlier copy, the result carries onto keyframes and points made since.

    The copy lacks the loop map's last keyframe and the points first seen there. Were the BA
    to move the copy's world rigidly, every keyframe and point of the map now moves the same
    way: the solved ones by their result, the newest keyframe with its parent, and its points
    with it, their reference keyframe. Every backend carries it alike.
    """
    now = loop_rigid.map
    newest = int(now.kf_id[-1])
    made_there = now.pt_id[now.pt_reference_kf == newest]
    earlier = maps.erase_keyframe(now, now.keyframes - 1)
    earlier = maps.erase_points(earlier, np.isin(earlier.pt_id, made_there), Removal.CULLED)
    motion = se3.exp(np.array([0.05, -0.02, 0.03, 0.01, 0.02, -0.015]))
    chosen = require_function(backend, loops.__name__, "propagate_global_ba")
    out = loops.propagate_global_ba(now, _moved_world(earlier, motion), backend=chosen)
    expected = _moved_world(now, motion)
    np.testing.assert_allclose(out.map.kf_pose, expected.kf_pose, atol=1e-12)
    np.testing.assert_allclose(out.map.pt_xyz, expected.pt_xyz, atol=1e-12)
    assert (out.adjusted, out.propagated) == (now.keyframes - 1, 1)
    assert out.points == earlier.points
    assert out.carried == now.points - earlier.points > 0


def test_a_global_ba_refuses_what_is_not_an_earlier_copy(loop_rigid: Loop) -> None:
    """A copy that gave more ids than the map has is none of its: refused, as is no map (H9).

    B3b's review panel, H9: a map gives its ids in order, so its earlier copy gave no more.
    """
    now = loop_rigid.map
    with pytest.raises(InvalidInputError, match="solved: not an earlier copy of now"):
        loops.propagate_global_ba(now, replace(now, next_pt_id=now.next_pt_id + 1))
    with pytest.raises(InvalidInputError, match="now, solved: expected keyframes"):
        loops.propagate_global_ba(now, maps.empty(now.levels, now.scale_factor))
