"""Merging maps' tests: a merge's candidates, its detection, the merge, relocalisation anywhere.

The two maps (:class:`tests.edge.conftest.TwoMaps`) hold the loop room's keyframes, the first
half the older, matched map in the truth's world, the rest the active map in a world of its own
(scaled too, in monocular), whose last keyframes come back over where the first map began. All
of it is Python's reference alone until milestone 2B's B4 ports merging to C++ and Rust.
"""

from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import pytest

from slam_it.edge import loops
from slam_it.edge import map as maps
from slam_it.edge.database import Indexed
from slam_it.edge.loops import Merge, MergeDetection, Merging, Outcome, Pending
from slam_it.edge.map import Map
from slam_it.foundations.errors import InvalidInputError
from tests.edge.conftest import MERGE_SCALE, SPLIT, TwoMaps
from tests.edge.support import CAMERA, OPTIONS

FOUND_AT = -2
"""The active map's keyframe where the merge is found at once: the second last, matched near the
first map's start (its links confirm it)."""

NOWHERE = 4
"""An active keyframe that sees nothing the first map saw."""

NARROW = (3, 2, 0)
"""Welding windows narrower than the maps (:data:`slam_it.edge.loops.WELD_NEIGHBOURS`,
``WELD_WIDEN``, ``WELD_ROUNDS``), so the matched map has keyframes outside its window to hold
and the essential graph has keyframes to move."""


@pytest.fixture(scope="module", params=["rigid", "scaled"])
def two(request: pytest.FixtureRequest) -> TwoMaps:
    """Return the two maps, in RGB-D and in monocular."""
    chosen: TwoMaps = request.getfixturevalue(f"two_{request.param}")
    return chosen


def _mode(two: TwoMaps) -> str:
    """Return the mode the two maps stand for."""
    return "monocular" if two.scaled else "rgbd"


def _others(two: TwoMaps) -> list[Indexed]:
    """Return the matched map, as merge detection takes the other maps."""
    return [Indexed(0, two.matched, two.matched_database)]


def _detect(
    two: TwoMaps,
    row: int,
    pending: Merging | None = None,
    *,
    confirmations: int | None = None,
    min_keyframes: int | None = None,
) -> MergeDetection:
    """Detect at the active keyframe of a row (negative from the end), a setting or two changed."""
    changed = {
        f"loops.{name}": value
        for name, value in (("confirmations", confirmations), ("min_keyframes", min_keyframes))
        if value is not None
    }
    return loops.detect_merge(
        two.active, two.active_database, CAMERA, int(two.active.kf_id[row]),
        others=_others(two), pending=pending, mode=_mode(two),
        options=OPTIONS.with_settings(changed),
    )  # fmt: skip


def _found(two: TwoMaps) -> Merge:
    """Return the merge found on :data:`FOUND_AT`."""
    found = _detect(two, FOUND_AT)
    assert found.merge is not None
    return found.merge


def _true_pose(two: TwoMaps, merged: loops.Merged, k: int) -> np.ndarray:
    """Return the true ``T_cam_world`` of a merged map's keyframe row."""
    keyframe = int(merged.map.kf_id[k])
    if keyframe < merged.keyframe_offset:
        return two.poses[keyframe]
    return two.poses[SPLIT + keyframe - merged.keyframe_offset]


def _errors(two: TwoMaps, merged: loops.Merged) -> np.ndarray:
    """Return each merged keyframe's distance from its true place, in metres."""
    m = merged.map
    return np.array(
        [np.linalg.norm(maps.camera_centre(m.kf_pose[k])
                        - maps.camera_centre(_true_pose(two, merged, k)))
         for k in range(m.keyframes)]
    )  # fmt: skip


# --- detection ----------------------------------------------------------------------------


def test_a_merge_is_found_where_the_path_comes_back(two: TwoMaps) -> None:
    """Found at once on the second last active keyframe, matched to the first map's start.

    Its similarity places the keyframe within 3 cm of its true place in the first map's world,
    and carries the active map's scale to within 5% in monocular; it is rigid in RGB-D.
    """
    found = _detect(two, FOUND_AT)
    assert found.summary.outcome is Outcome.CLOSED
    assert found.pending is None
    assert found.merge is not None
    region = found.merge.loop
    assert found.merge.map == 0
    assert region.current == int(two.active.kf_id[FOUND_AT])
    assert region.matched <= 4
    assert found.summary.confirmations == OPTIONS.loops.confirmations
    assert found.maps == (0,) * len(found.summary.candidates)
    true = two.poses[SPLIT + two.active.keyframes + FOUND_AT]
    placed = loops.rigid_of(region.similarity)
    assert np.linalg.norm(maps.camera_centre(placed) - maps.camera_centre(true)) < 0.03
    scale = loops.scale_of(region.similarity)
    if two.scaled:
        assert scale == pytest.approx(MERGE_SCALE, rel=0.05)
    else:
        assert scale == pytest.approx(1.0, abs=1e-12)
    json.dumps(found.summary.as_dict(), allow_nan=False)


def test_nothing_is_merged_where_the_other_map_never_was(two: TwoMaps) -> None:
    """An active keyframe that sees nothing the first map saw: no merge, none carried."""
    found = _detect(two, NOWHERE)
    assert found.summary.outcome is Outcome.NONE
    assert (found.pending, found.merge) == (None, None)


def test_a_small_map_looks_for_no_other(two: TwoMaps) -> None:
    """Below ``loops.min_keyframes`` keyframes, detection skips, carrying what it was given."""
    state = Pending(
        matched=0, last=1, similarity=np.eye(4), confirmations=1, misses=0,
        points=np.zeros(0, dtype=np.int64), matched_points=np.zeros(0, dtype=np.int64),
    )  # fmt: skip
    pending = Merging(0, state)
    found = _detect(two, -1, pending, min_keyframes=two.active.keyframes + 1)
    assert found.summary.outcome is Outcome.SKIPPED
    assert found.pending is pending
    assert found.merge is None
    assert found.summary.confirmations == 1


def test_a_merge_short_of_confirmations_is_carried_until_found(two: TwoMaps) -> None:
    """With one confirmation more needed than its links give, the merge is carried, then found.

    Started on the second last active keyframe, it is confirmed on the last, through the
    first map, and found there.
    """
    needed = OPTIONS.loops.confirmations + 1
    started = _detect(two, FOUND_AT, confirmations=needed)
    assert started.summary.outcome is Outcome.STARTED
    assert started.merge is None
    pending = started.pending
    assert pending is not None
    assert pending.map == 0
    assert pending.pending.last == int(two.active.kf_id[FOUND_AT])
    found = _detect(two, -1, pending, confirmations=needed)
    assert found.summary.outcome is Outcome.CLOSED
    assert found.summary.confirmation is not None
    assert found.summary.confirmation.confirmed
    assert found.summary.candidates == ()  # confirmed: no search
    assert found.merge is not None
    assert found.merge.map == 0
    assert found.merge.loop.current == int(two.active.kf_id[-1])
    assert found.merge.loop.matched == pending.pending.matched


def test_a_merge_unconfirmed_twice_in_a_row_is_dropped(two: TwoMaps) -> None:
    """Carried to an active keyframe that does not see it, a merge misses; two misses drop it."""
    started = _detect(two, FOUND_AT, confirmations=4)
    assert started.pending is not None
    stray = Merging(0, _carried_from(started.pending.pending, two.active, NOWHERE - 1))
    missed = _detect(two, NOWHERE, stray, confirmations=4)
    assert missed.summary.outcome is Outcome.NONE
    assert missed.pending is not None
    assert missed.pending.pending.misses == 1
    assert missed.summary.confirmation is not None
    assert not missed.summary.confirmation.confirmed
    dropped = _detect(two, NOWHERE, missed.pending, confirmations=4)
    assert dropped.summary.outcome is Outcome.DROPPED
    assert (dropped.pending, dropped.merge) == (None, None)


def _carried_from(pending: Pending, m: Map, row: int) -> Pending:
    """Return a merge being confirmed as if its last keyframe were another."""
    return Pending(
        matched=pending.matched,
        last=int(m.kf_id[row]),
        similarity=pending.similarity,
        confirmations=pending.confirmations,
        misses=pending.misses,
        points=pending.points,
        matched_points=pending.matched_points,
    )


def test_merge_detection_refuses_what_it_cannot_search(two: TwoMaps) -> None:
    """A map given twice, or a merge being confirmed in a map not given: InvalidInputError."""
    keyframe = int(two.active.kf_id[-1])
    with pytest.raises(InvalidInputError, match="each map once"):
        loops.detect_merge(
            two.active, two.active_database, CAMERA, keyframe, others=_others(two) * 2,
            pending=None, mode=_mode(two), options=OPTIONS,
        )  # fmt: skip
    started = _detect(two, FOUND_AT, confirmations=4)
    assert started.pending is not None
    with pytest.raises(InvalidInputError, match="not among the others"):
        loops.detect_merge(
            two.active, two.active_database, CAMERA, keyframe, others=_others(two),
            pending=Merging(7, started.pending.pending), mode=_mode(two), options=OPTIONS,
        )  # fmt: skip


def test_a_merge_takes_priority_over_a_loop_on_its_keyframe(two: TwoMaps) -> None:
    """A loop closed on the keyframe a merge was found on is forgotten; one being confirmed is not.

    The loop's summary still says what loop detection found.
    """
    merged = _detect(two, FOUND_AT)
    nothing = _detect(two, NOWHERE)
    region = _found(two).loop
    state = Pending(
        matched=0, last=region.current, similarity=region.similarity, confirmations=1, misses=0,
        points=region.points, matched_points=region.matched_points,
    )  # fmt: skip
    closed = loops.Detection(None, region, loops.DetectionSummary(Outcome.CLOSED))
    carried = loops.Detection(state, None, loops.DetectionSummary(Outcome.STARTED))
    first = loops.merge_first(closed, merged)
    assert (first.loop, first.pending) == (None, None)
    assert first.summary.outcome is Outcome.CLOSED
    assert loops.merge_first(closed, nothing) is closed
    assert loops.merge_first(carried, merged) is carried


# --- the merge ----------------------------------------------------------------------------


def test_the_merge_puts_the_active_map_where_it_belongs(two: TwoMaps) -> None:
    """Merged without global BA, every keyframe is within 3 cm of its place, in the first world.

    Here each welding window holds all of its map, so the matched map has no keyframe outside
    to hold, and its window is held: the matched map stays where it was. The active map's
    keyframes and points follow the matched map's, every id moved on by the matched map's next;
    its tree hangs under the matched keyframe, the matched map's root the merged map's.
    """
    found = _found(two)
    merged = loops.merge(
        two.active, two.matched, CAMERA, found, mode=_mode(two),
        options=OPTIONS.with_settings({"loops.global_ba": False}),
    )  # fmt: skip
    m = merged.map
    assert (merged.keyframe_offset, merged.point_offset) == (
        two.matched.next_kf_id, two.matched.next_pt_id,
    )  # fmt: skip
    np.testing.assert_array_equal(
        m.kf_id, np.concatenate([two.matched.kf_id, two.active.kf_id + merged.keyframe_offset])
    )
    np.testing.assert_array_equal(m.kf_time[two.matched.keyframes :], two.active.kf_time)
    assert _errors(two, merged).max() < 0.03
    np.testing.assert_allclose(
        m.kf_pose[: two.matched.keyframes], two.matched.kf_pose, rtol=0, atol=1e-12
    )
    np.testing.assert_array_equal(m.kf_id[m.kf_parent < 0], [0])
    current = found.loop.current + merged.keyframe_offset
    assert m.kf_parent[maps.keyframe_row(m, current)] == found.loop.matched
    ids = two.active.pt_id + merged.point_offset
    assert np.all(np.isin(ids, m.pt_id) | np.isin(ids, m.gone_pt_id))
    s = merged.summary
    assert (s.map, s.current, s.matched) == (0, current, found.loop.matched)
    assert s.fused > 0
    assert s.weld.held == s.matched_window
    assert s.weld.second is not None
    assert s.global_ba is None
    json.dumps(s.as_dict(), allow_nan=False)


def test_the_paper_s_weld_holds_the_matched_map_outside_its_window(
    two: TwoMaps, monkeypatch: pytest.MonkeyPatch
) -> None:
    """With narrow windows, the matched map's keyframes outside its window hold the weld.

    Both windows' keyframes are optimised; the essential graph carries the weld to the rest of
    both maps; every keyframe ends within 3 cm of its place.
    """
    for name, value in zip(("WELD_NEIGHBOURS", "WELD_WIDEN", "WELD_ROUNDS"), NARROW, strict=True):
        monkeypatch.setattr(loops, name, value)
    merged = loops.merge(
        two.active, two.matched, CAMERA, _found(two), mode=_mode(two),
        options=OPTIONS.with_settings({"loops.global_ba": False}),
    )  # fmt: skip
    s = merged.summary
    assert s.weld.keyframes == s.window + s.matched_window
    assert s.weld.held > 0
    assert s.essential.final_error < s.essential.initial_error
    assert _errors(two, merged).max() < 0.03


def test_global_ba_follows_the_merge(two: TwoMaps) -> None:
    """With global BA, every keyframe within 3.5 cm of its place, the first map's first held.

    The points fused at the weld were made twice, from keypoints a few pixels apart, so their
    observations disagree by as much (as at a loop): global BA's fit moves the map a little.
    """
    found = _found(two)
    adjusted = loops.merge(two.active, two.matched, CAMERA, found, mode=_mode(two), options=OPTIONS)
    assert adjusted.summary.global_ba is not None
    assert _errors(two, adjusted).max() < 0.035
    graph_only = loops.merge(
        two.active, two.matched, CAMERA, found, mode=_mode(two),
        options=OPTIONS.with_settings({"loops.global_ba": False}),
    )  # fmt: skip
    np.testing.assert_allclose(
        adjusted.map.kf_pose[0], graph_only.map.kf_pose[0], rtol=0, atol=1e-12
    )


def test_the_merge_refuses_what_it_cannot_join(two_rigid: TwoMaps) -> None:
    """A keyframe not in its map, or maps of different pyramids: InvalidInputError."""
    found = _found(two_rigid)
    stray = Merge(0, loops.Loop(
        current=two_rigid.active.next_kf_id, matched=found.loop.matched,
        similarity=found.loop.similarity, points=found.loop.points,
        matched_points=found.loop.matched_points,
    ))  # fmt: skip
    with pytest.raises(InvalidInputError, match="not in the map"):
        loops.merge(
            two_rigid.active, two_rigid.matched, CAMERA, stray, mode="rgbd", options=OPTIONS
        )
    other = replace(two_rigid.matched, scale_factor=two_rigid.matched.scale_factor * 1.1)
    with pytest.raises(InvalidInputError, match="pyramids differ"):
        loops.merge(two_rigid.active, other, CAMERA, found, mode="rgbd", options=OPTIONS)
