"""Merging maps' tests: a merge's candidates, its detection, the merge, relocalisation anywhere.

The two maps (:class:`tests.edge.conftest.TwoMaps`) hold the loop room's keyframes, the first
half the older, matched map in the truth's world, the rest the active map in a world of its own
(scaled too, in monocular), whose last keyframes come back over where the first map began.
Each stage's C++ and Rust match Python's reference, the GTSAM stages to GTSAM's convergence.
"""

from __future__ import annotations

import json
from dataclasses import replace
from typing import Any

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import covisibility, loops, pipeline
from slam_it.edge import map as maps
from slam_it.edge.atlas import Atlas
from slam_it.edge.database import Indexed
from slam_it.edge.loops import Merge, MergeDetection, Merging, Outcome, Pending
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.foundations import sim3, vocabulary
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import MERGE_SCALE, SPLIT, TwoMaps, keyframe_database
from tests.edge.support import (
    CAMERA,
    GRAPH_TOLERANCE,
    OPTIONS,
    assert_close,
    assert_maps_match,
    assert_records_match,
)

FOUND_AT = -2
"""The active map's keyframe where the merge is found at once: the second last, matched near the
first map's start (its links confirm it)."""

NOWHERE = 4
"""An active keyframe that sees nothing the first map saw."""

NARROW = OPTIONS.with_settings(
    {"loops.weld_neighbours": 3, "loops.weld_widen": 2, "loops.weld_rounds": 0}
)
"""Welding windows narrower than the maps, so the matched map has keyframes outside its window
to hold and the essential graph has keyframes to move."""


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


def _being_confirmed(region: loops.Loop) -> Pending:
    """Return a loop's region as a loop being confirmed, once, on its current keyframe."""
    return Pending(
        matched=0, last=region.current, similarity=region.similarity, confirmations=1, misses=0,
        points=region.points, matched_points=region.matched_points,
    )  # fmt: skip


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

    The forgotten loop's outcome says why: ``merged``.
    """
    merged = _detect(two, FOUND_AT)
    nothing = _detect(two, NOWHERE)
    region = _found(two).loop
    state = _being_confirmed(region)
    closed = loops.Detection(None, region, loops.DetectionSummary(Outcome.CLOSED))
    carried = loops.Detection(state, None, loops.DetectionSummary(Outcome.STARTED))
    first = loops.merge_first(closed, merged)
    assert (first.loop, first.pending) == (None, None)
    assert first.summary.outcome is Outcome.MERGED
    assert first.summary.candidates == closed.summary.candidates
    assert loops.merge_first(closed, nothing) is closed
    assert loops.merge_first(carried, merged) is carried


# --- the merge ----------------------------------------------------------------------------


def test_the_merge_puts_the_active_map_where_it_belongs(two: TwoMaps) -> None:
    """Merged without global BA, every keyframe is within 3 cm of its place, in the first world.

    Here each welding window holds all of its map, so the matched map has no keyframe outside
    to hold, and its window is held: the matched map stays where it was. The active map's
    keyframes and points follow the matched map's, every id moved on by the matched map's next;
    its tree hangs under the matched keyframe, the matched map's root the merged map's; the
    merge edge joins the two keyframes, and culling spares them.
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
    np.testing.assert_array_equal(m.merge_edges, [[found.loop.matched, current]])
    assert maps.has_merge(m, maps.keyframe_row(m, current))
    k = maps.keyframe_row(m, current)
    assert covisibility.cull(m, k, OPTIONS) == m
    s = merged.summary
    assert (s.map, s.current, s.matched) == (0, current, found.loop.matched)
    assert s.fused > 0
    assert s.weld.held == s.matched_window
    assert s.weld.second is not None
    assert s.global_ba is None
    json.dumps(s.as_dict(), allow_nan=False)


def test_the_paper_s_weld_holds_the_matched_map_outside_its_window(two: TwoMaps) -> None:
    """With narrow windows, the matched map's keyframes outside its window hold the weld.

    Both windows' keyframes are optimised; the essential graph carries the weld to the rest of
    both maps; every keyframe ends within 3 cm of its place.
    """
    merged = loops.merge(
        two.active, two.matched, CAMERA, _found(two), mode=_mode(two),
        options=NARROW.with_settings({"loops.global_ba": False}),
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


LOOP_MATCHED = 9
"""A loop being confirmed in the active map matched this active keyframe (row), one whose points
the active map's last keyframe observes (:data:`tests.edge.conftest.LOOP_WINDOW`)."""


def test_a_loop_being_confirmed_is_carried_through_a_monocular_merge(
    two_scaled: TwoMaps, words: vocabulary.Vocabulary
) -> None:
    """A loop carried into the merged map keeps its own scale; the next keyframe confirms it.

    The loop places its last keyframe, the one the merge is found on, where the active map has
    it. Carried into the merged map (:func:`slam_it.edge.pipeline._carried`), it places it where
    the merge carried it, rigidly (``rigid_of(T G⁻¹)``), its scale still 1; on the active map's
    last keyframe, the next, it is confirmed, its projection finding at least as many points as
    the same loop found in the active map (316; carried in the active map's units, 198).
    """
    a = two_scaled.active
    options = OPTIONS.with_settings({"loops.global_ba": False})
    last, following = a.keyframes + FOUND_AT, a.keyframes - 1
    pending = Pending(
        matched=int(a.kf_id[LOOP_MATCHED]), last=int(a.kf_id[last]),
        similarity=a.kf_pose[last].copy(), confirmations=1, misses=0,
        points=np.zeros(0, dtype=np.int64),
        matched_points=np.full(int(a.kf_start[last + 1] - a.kf_start[last]), -1, dtype=np.int64),
    )  # fmt: skip
    before = loops.detect(
        a, two_scaled.active_database, CAMERA, int(a.kf_id[following]), pending,
        mode="monocular", options=options,
    ).summary.confirmation  # fmt: skip
    assert before is not None
    assert before.confirmed
    found = _found(two_scaled)
    to_world = loops.merge_transform(a, found)
    merged = loops.merge(a, two_scaled.matched, CAMERA, found, mode="monocular", options=options)
    carried = pipeline._carried(pending, merged, to_world)
    assert (carried.matched, carried.last) == (
        pending.matched + merged.keyframe_offset, pending.last + merged.keyframe_offset,
    )  # fmt: skip
    rigid = loops.rigid_of(pending.similarity @ sim3.inverse(to_world))
    np.testing.assert_allclose(carried.similarity, rigid, rtol=0, atol=1e-12)
    m = merged.map
    after = loops.detect(
        m, keyframe_database(m, words), CAMERA, int(a.kf_id[following]) + merged.keyframe_offset,
        carried, mode="monocular", options=options,
    )  # fmt: skip
    assert after.summary.outcome is Outcome.CONFIRMED
    assert after.summary.confirmation is not None
    assert after.summary.confirmation.projected >= before.projected


# --- each backend as Python ---------------------------------------------------------------


def _assert_same_merge_detection(got: MergeDetection, expected: MergeDetection) -> None:
    """Check two merge detections: the decisions exactly, the similarities to GTSAM's."""
    assert_records_match(got.summary.as_dict(), expected.summary.as_dict(), GRAPH_TOLERANCE)
    assert got.maps == expected.maps
    assert (got.pending is None, got.merge is None) == (
        expected.pending is None, expected.merge is None,
    )  # fmt: skip
    pairs: list[tuple[int, int, Pending | loops.Loop, Pending | loops.Loop]] = []
    if got.pending is not None and expected.pending is not None:
        pairs.append((got.pending.map, expected.pending.map, got.pending.pending,
                      expected.pending.pending))  # fmt: skip
    if got.merge is not None and expected.merge is not None:
        pairs.append((got.merge.map, expected.merge.map, got.merge.loop, expected.merge.loop))
    for number, theirs_number, mine, theirs in pairs:
        assert number == theirs_number
        assert_close(mine.similarity, theirs.similarity, GRAPH_TOLERANCE)
        np.testing.assert_array_equal(mine.points, theirs.points)
        np.testing.assert_array_equal(mine.matched_points, theirs.matched_points)
        same = replace(
            mine, similarity=theirs.similarity, points=theirs.points,
            matched_points=theirs.matched_points,
        )  # fmt: skip
        assert same == theirs


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_detects_merges_as_python_does(backend: str, two: TwoMaps) -> None:
    """Nowhere, found at once, started and then carried, skipped: as Python decides."""
    require_function(backend, loops.__name__, "detect_merge")
    needed = OPTIONS.with_settings({"loops.confirmations": OPTIONS.loops.confirmations + 1})
    started = _detect(two, FOUND_AT, confirmations=OPTIONS.loops.confirmations + 1).pending
    assert started is not None
    small = OPTIONS.with_settings({"loops.min_keyframes": two.active.keyframes + 1})
    cases: list[tuple[int, Merging | None, Options]] = [
        (NOWHERE, None, OPTIONS),
        (FOUND_AT, None, OPTIONS),
        (FOUND_AT, None, needed),
        (-1, started, needed),
        (-1, started, small),
    ]
    for row, pending, options in cases:
        expected, got = (
            loops.detect_merge(
                two.active, two.active_database, CAMERA, int(two.active.kf_id[row]),
                others=_others(two), pending=pending, mode=_mode(two), options=options,
                backend=chosen,
            )
            for chosen in ("python", backend)
        )  # fmt: skip
        _assert_same_merge_detection(got, expected)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_gives_a_merge_priority_as_python_does(
    backend: str, two_rigid: TwoMaps
) -> None:
    """A loop closed with a merge found, with none, and one being confirmed: Python's results."""
    require_function(backend, loops.__name__, "merge_first")
    merged, nothing = _detect(two_rigid, FOUND_AT), _detect(two_rigid, NOWHERE)
    region = _found(two_rigid).loop
    state = _being_confirmed(region)
    closed = loops.Detection(None, region, merged.summary)
    carried = loops.Detection(state, None, nothing.summary)
    for loop, merging in ((closed, merged), (closed, nothing), (carried, merged)):
        expected, got = (
            loops.merge_first(loop, merging, backend=chosen) for chosen in ("python", backend)
        )
        assert got.summary.as_dict() == expected.summary.as_dict()
        for mine, theirs in ((got.pending, expected.pending), (got.loop, expected.loop)):
            assert (mine is None) == (theirs is None)
            if mine is not None and theirs is not None:
                assert json.dumps(_fields(mine)) == json.dumps(_fields(theirs))


def _fields(region: Pending | loops.Loop) -> dict[str, Any]:
    """Return a loop's or a pending loop's fields as JSON values, exactly."""
    return {
        name: value.tolist() if isinstance(value, np.ndarray) else value
        for name, value in vars(region).items()
    }


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(
    ("narrow", "global_ba"), [(False, False), (True, False), (False, True)],
    ids=["held window", "the paper's weld", "global BA"],
)  # fmt: skip
def test_every_backend_merges_as_python_does(
    backend: str, narrow: bool, global_ba: bool, two: TwoMaps
) -> None:
    """The same merged map, to GTSAM's convergence; the same offsets and counts exactly."""
    require_function(backend, loops.__name__, "merge")
    options = (NARROW if narrow else OPTIONS).with_settings({"loops.global_ba": global_ba})
    found = _found(two)
    expected, got = (
        loops.merge(
            two.active, two.matched, CAMERA, found, mode=_mode(two), options=options,
            backend=chosen,
        )
        for chosen in ("python", backend)
    )  # fmt: skip
    assert (got.keyframe_offset, got.point_offset) == (
        expected.keyframe_offset, expected.point_offset,
    )  # fmt: skip
    assert_maps_match(got.map, expected.map, GRAPH_TOLERANCE)
    assert_records_match(got.summary.as_dict(), expected.summary.as_dict(), GRAPH_TOLERANCE)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_core_refuses_a_merge_it_cannot_make(backend: str, two_rigid: TwoMaps) -> None:
    """Python refuses a keyframe outside its map, or an unknown mode, first; each core does too."""
    require_function(backend, loops.__name__, "merge")
    compiled = backends.implementation(backend, loops.__name__)
    found = _found(two_rigid)
    loop = found.loop

    def crossing(current: int) -> dict[str, Any]:
        return {
            "map": 0,
            "loop": {
                "current": current, "matched": loop.matched, "similarity": loop.similarity,
                "points": loop.points, "matched_points": loop.matched_points,
            },
        }  # fmt: skip

    arguments = {
        "active": two_rigid.active.as_dict(),
        "matched": two_rigid.matched.as_dict(),
        "camera": CAMERA.as_dict(),
        "options": OPTIONS.as_dict(),
    }
    with pytest.raises(ValueError, match="not in the map"):
        compiled.merge(**arguments, found=crossing(two_rigid.active.next_kf_id), mode="rgbd")
    with pytest.raises(ValueError, match="mode"):
        compiled.merge(**arguments, found=crossing(loop.current), mode="lidar")


def test_a_merge_with_a_map_not_kept_is_refused_by_its_number(two_rigid: TwoMaps) -> None:
    """The merge's matched map is not kept: InvalidInputError naming it, as the atlas's own.

    No run reaches it, a merge waiting being dropped when the track leaves its map (B4's review
    panel, run 2, H15: it raised a bare StopIteration).
    """
    found = _found(two_rigid)
    atlas = Atlas(OPTIONS)
    atlas.start()
    with pytest.raises(InvalidInputError, match=f"merge: map {found.map} is not kept"):
        pipeline._merged(
            pipeline.LoopState(), pipeline.MapOwner(), atlas, found, two_rigid.active,
            camera=CAMERA, mode=_mode(two_rigid), options=OPTIONS,
        )  # fmt: skip
