"""Loop closing's tests: its searches, its similarity, detection and correction, on the loop map.

The loop map (:func:`tests.edge.conftest.loop_map`) holds the loop room's keyframes, mapped from
the truth but each seeing only its window, then drifted round the loop: its end makes its own
points where its start's are, out of place by the drift, as a tracked map's would be.
"""

from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import pytest

from slam_it.edge import covisibility, loops
from slam_it.edge import map as maps
from slam_it.edge.database import Database
from slam_it.edge.loops import Outcome, Pending
from slam_it.edge.map import Map
from slam_it.foundations import matching, se3, so3
from slam_it.foundations.errors import InvalidInputError
from tests.edge.conftest import Loop
from tests.edge.support import CAMERA, OPTIONS

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
    rows = loops.group_points(m, 2, set(), OPTIONS)
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
    """Every keyframe but the first nudged: global BA puts them back, the observations exact."""
    exact = loop_rigid.truth
    rng = np.random.default_rng(0)
    poses = exact.kf_pose.copy()
    for k in range(1, exact.keyframes):
        poses[k] = (
            se3.exp(np.concatenate([rng.normal(0, 0.005, 3), rng.normal(0, 0.002, 3)])) @ poses[k]
        )
    adjusted, summary = loops.global_bundle_adjust(replace(exact, kf_pose=poses), CAMERA, OPTIONS)
    assert summary.final_error < 1e-6 * summary.initial_error
    np.testing.assert_allclose(adjusted.kf_pose, exact.kf_pose, atol=1e-6)
    np.testing.assert_allclose(adjusted.pt_xyz, exact.pt_xyz, atol=1e-5)


def test_the_essential_graph_keeps_a_consistent_map(loop_rigid: Loop) -> None:
    """Started where the poses were, with no new links: nothing moves."""
    m = loop_rigid.truth
    poses = list(m.kf_pose)
    optimised, edges, summary = loops.essential_graph(
        m, poses, poses, {}, (m.keyframes - 1, 0), fixed_scale=True, options=OPTIONS
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
