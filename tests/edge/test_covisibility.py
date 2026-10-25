"""The covisibility graph and the spanning tree, held to the brute-force fixture.

``tests/fixtures/edge/covisibility.json`` (``generate_covisibility.py``) gives, for small maps
of hand-picked and seeded points, each keyframe's links and connected keyframes, tracking's
local map for some frames, the parent :func:`~slam_it.edge.covisibility.adopt` gives, the
parents :func:`~slam_it.edge.covisibility.reparent` leaves (and so what
:func:`~slam_it.edge.covisibility.cull` leaves), the strong links, and the keyframes fusion and
local BA take (:func:`~slam_it.edge.mapping.fuse_targets`,
:func:`~slam_it.edge.mapping.local_ba_keyframes`), computed from point sets by the rules
alone. Every backend is held to it; the C++ and Rust native tests hold their cores to the same
file. The view a native writer publishes (:func:`~slam_it.edge.covisibility.view_keyframes`,
:func:`~slam_it.edge.map.sub_map`) is held to the same point sets, and tracking's local map
read from it to the whole map's.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import covisibility, mapping
from slam_it.edge import map as maps
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.orb import DESCRIPTOR_BYTES
from tests.conftest import load_fixture, require_function
from tests.edge.conftest import Run

CASES: list[dict[str, Any]] = load_fixture("edge/covisibility.json")["cases"]
NAMES = [case["name"] for case in CASES]


def fixture_map(case: dict[str, Any]) -> Map:
    """Build a case's map: one keypoint per entry of each keyframe's points, the rest blank.

    The cameras sit at the origin and the points a metre in front, so a map culled here keeps
    finite normals.
    """
    keyframes = case["keyframes"]
    observed = np.array([p for k in keyframes for p in k["points"]], dtype=np.int64)
    points = np.unique(observed[observed >= 0])
    t, p = len(observed), len(points)
    sizes = [len(k["points"]) for k in keyframes]
    return replace(
        maps.empty(8, 1.2),
        kf_id=np.array([k["id"] for k in keyframes], dtype=np.int64),
        kf_time=np.zeros(len(keyframes)),
        kf_pose=np.tile(np.eye(4), (len(keyframes), 1, 1)),
        kf_parent=np.array([k["parent"] for k in keyframes], dtype=np.int64),
        kf_start=np.concatenate([[0], np.cumsum(sizes)]).astype(np.int64),
        kp_xy=np.zeros((t, 2)),
        kp_level=np.zeros(t, dtype=np.int64),
        kp_angle=np.zeros(t, dtype=np.int64),
        kp_descriptor=np.zeros((t, DESCRIPTOR_BYTES), dtype=np.uint8),
        kp_bearing=np.tile([0.0, 0.0, 1.0], (t, 1)),
        kp_depth=np.full(t, np.nan),
        kp_right=np.full(t, np.nan),
        kp_point=observed,
        pt_id=points,
        pt_xyz=np.tile([0.0, 0.0, 1.0], (p, 1)),
        pt_descriptor=np.zeros((p, DESCRIPTOR_BYTES), dtype=np.uint8),
        pt_normal=np.tile([0.0, 0.0, 1.0], (p, 1)),
        pt_min_distance=np.ones(p),
        pt_max_distance=np.ones(p),
        pt_visible=np.ones(p, dtype=np.int64),
        pt_found=np.ones(p, dtype=np.int64),
        pt_first_kf=np.zeros(p, dtype=np.int64),
        pt_reference_kf=np.zeros(p, dtype=np.int64),
        next_kf_id=int(max(k["id"] for k in keyframes)) + 1,
        next_pt_id=int(points.max()) + 1 if p else 0,
    )


def fixture_options(case: dict[str, Any]) -> Options:
    """Return the settings a case is read with."""
    return Options().with_settings(
        {f"covisibility.{name}": value for name, value in case["settings"].items()}
    )


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_links_rank_the_shared_points(case: dict[str, Any], backend: str) -> None:
    """Each keyframe's links, and its connected keyframes, most shared first, ties by id.

    Its links: those sharing at least min_points, or its best; connected: every one sharing a
    point, however few.
    """
    chosen = require_function(backend, covisibility.__name__, "links")
    joined = require_function(backend, covisibility.__name__, "connected")
    m, options = fixture_map(case), fixture_options(case)
    for k in range(m.keyframes):
        for got, expected in (
            (covisibility.links(m, k, options, backend=chosen), case["links"][k]),
            (covisibility.connected(m, k, backend=joined), case["connected"][k]),
        ):
            assert [got[0].tolist(), got[1].tolist()] == expected, f"keyframe row {k}"


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_the_local_map_is_k1_then_k2(case: dict[str, Any], backend: str) -> None:
    """K1 most shared first; K2 a neighbour, a child and the parent of each, to the cap."""
    chosen = require_function(backend, covisibility.__name__, "local_map")
    m, options = fixture_map(case), fixture_options(case)
    for expected in case["local_maps"]:
        got = covisibility.local_map(m, np.array(expected["points"]), options, backend=chosen)
        assert got.keyframes.tolist() == expected["keyframes"]
        assert got.reference == expected["reference"]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_a_keyframe_adopts_the_one_it_shares_most_with(case: dict[str, Any], backend: str) -> None:
    """With its parent unset, a keyframe takes the one it shares most with; the first, none."""
    chosen = require_function(backend, covisibility.__name__, "adopt")
    m = fixture_map(case)
    for k, parent in enumerate(case["adopted"]):
        orphan = m.kf_parent.copy()
        orphan[k] = -1
        orphan[orphan == m.kf_id[k]] = -1  # its children too, so the map stays a forest
        got = covisibility.adopt(replace(m, kf_parent=orphan), k, backend=chosen)
        assert int(got.kf_parent[k]) == parent, f"keyframe row {k}"
        np.testing.assert_array_equal(np.delete(got.kf_parent, k), np.delete(orphan, k))


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_a_culled_keyframes_children_get_new_parents(case: dict[str, Any], backend: str) -> None:
    """SetBadFlag's rounds: the strongest link to a candidate first; the rest, its parent."""
    chosen = require_function(backend, covisibility.__name__, "reparent")
    m, options = fixture_map(case), fixture_options(case)
    for k, parents in enumerate(case["reparented"]):
        got = covisibility.reparent(m, k, options, backend=chosen)
        assert got.kf_parent.tolist() == parents, f"keyframe row {k}"


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_a_culled_keyframe_leaves_after_its_children(case: dict[str, Any], backend: str) -> None:
    """``cull``: the children given SetBadFlag's parents, then the keyframe erased; not id 0."""
    chosen = require_function(backend, covisibility.__name__, "cull")
    m, options = fixture_map(case), fixture_options(case)
    for k, parents in enumerate(case["reparented"]):
        if m.kf_id[k] == 0:
            with pytest.raises(InvalidInputError, match="is not culled"):
                covisibility.cull(m, k, options, backend=chosen)
            continue
        got = covisibility.cull(m, k, options, backend=chosen)
        assert got.kf_id.tolist() == np.delete(m.kf_id, k).tolist(), f"keyframe row {k}"
        assert got.kf_parent.tolist() == np.delete(parents, k).tolist(), f"keyframe row {k}"
        assert np.all(np.isfinite(got.pt_normal)), f"keyframe row {k}"


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_a_keyframe_with_a_loop_or_merge_edge_or_held_is_spared(
    case: dict[str, Any], backend: str
) -> None:
    """``cull`` leaves the map as it was for a keyframe with a loop or merge edge, or one held."""
    chosen = require_function(backend, covisibility.__name__, "cull")
    m, options = fixture_map(case), fixture_options(case)
    for k in range(m.keyframes):
        kf_id = int(m.kf_id[k])
        if kf_id == 0:
            continue
        held = covisibility.cull(m, k, options, held=[kf_id], backend=chosen)
        assert held.kf_id.tolist() == m.kf_id.tolist(), f"keyframe row {k} held"
        assert held.kf_parent.tolist() == m.kf_parent.tolist(), f"keyframe row {k} held"
        other = int(m.kf_id[0] if k else m.kf_id[1])
        looped = maps.add_loop_edge(m, kf_id, other)
        spared = covisibility.cull(looped, k, options, backend=chosen)
        assert spared.kf_id.tolist() == m.kf_id.tolist(), f"keyframe row {k} looped"
        assert spared.loop_edges.tolist() == looped.loop_edges.tolist()
        merged = maps.add_merge_edge(m, kf_id, other)
        spared = covisibility.cull(merged, k, options, backend=chosen)
        assert spared.kf_id.tolist() == m.kf_id.tolist(), f"keyframe row {k} merged"
        assert spared.merge_edges.tolist() == merged.merge_edges.tolist()


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_the_stages_take_the_graphs_keyframes(case: dict[str, Any], backend: str) -> None:
    """Fusion's targets, monocular and not, and local BA's keyframes, for the newest keyframe."""
    chosen = require_function(backend, mapping.__name__, "fuse_targets")
    chosen = require_function(chosen, mapping.__name__, "local_ba_keyframes")
    m = fixture_map(case)
    fusion = {f"mapping.{name}": value for name, value in case["fusion"].items()}
    options = fixture_options(case).with_settings(fusion)
    for mode, key in (("monocular", "monocular"), ("rgbd", "other"), ("stereo", "other")):
        got = mapping.fuse_targets(m, mode, options, backend=chosen)
        assert got.tolist() == case["fuse_targets"][key], mode
    assert mapping.local_ba_keyframes(m, options, backend=chosen).tolist() == case["local_ba"]


def _view_rows(case: dict[str, Any], local: list[int], reference: int) -> list[int]:
    """Return the rows a view holds, from the point sets alone.

    The local map's points' observers, and for each of those every keyframe sharing a point with
    it, its children and its parent; and the reference keyframe (an id, or -1).
    """
    keyframes = case["keyframes"]
    seen = [{p for p in k["points"] if p >= 0} for k in keyframes]
    ids = [k["id"] for k in keyframes]
    near = set().union(*(seen[k] for k in local))
    first = [k for k, own in enumerate(seen) if own & near]
    inside = set(first)
    for k in first:
        inside |= {other for other, own in enumerate(seen) if own & seen[k]}
        inside |= {child for child, kf in enumerate(keyframes) if kf["parent"] == ids[k]}
        inside |= {ids.index(keyframes[k]["parent"])} if keyframes[k]["parent"] in ids else set()
    inside |= {ids.index(reference)} if reference in ids else set()
    return sorted(inside)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_a_view_holds_what_the_next_frame_may_read(case: dict[str, Any], backend: str) -> None:
    """A view holds the local map's points' observers, their neighbours, and the reference.

    For each observer, every keyframe sharing a point with it, its children and its parent;
    nothing else.
    """
    chosen = require_function(backend, covisibility.__name__, "view_keyframes")
    m, options = fixture_map(case), fixture_options(case)
    for expected in case["local_maps"]:
        last = np.array(expected["points"])
        for reference in (-1, int(m.kf_id[-1])):
            got = covisibility.view_keyframes(m, last, reference, options, backend=chosen)
            assert got.tolist() == _view_rows(case, expected["keyframes"], reference)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_a_view_is_those_keyframes_and_their_points(case: dict[str, Any], backend: str) -> None:
    """A view's sub-map holds its keyframes, their keypoints and exactly their points.

    Ids kept, a parent outside it -1, the archive empty and the loops and next ids the map's;
    tracking's local map read from it is the whole map's.
    """
    chosen = require_function(backend, maps.__name__, "sub_map")
    m, options = fixture_map(case), fixture_options(case)
    for expected in case["local_maps"]:
        last = np.array(expected["points"])
        rows = covisibility.view_keyframes(m, last, -1, options)
        view = maps.sub_map(m, rows, backend=chosen)
        assert view.kf_id.tolist() == m.kf_id[rows].tolist()
        parents = m.kf_parent[rows]
        inside = np.where(np.isin(parents, view.kf_id), parents, -1)
        assert view.kf_parent.tolist() == inside.tolist()
        kept = m.kp_point[np.isin(m.keypoint_keyframe(), rows)]
        assert view.kp_point.tolist() == kept.tolist()
        assert view.pt_id.tolist() == np.unique(kept[kept >= 0]).tolist()
        assert np.array_equal(view.pt_xyz, m.pt_xyz[m.point_rows(view.pt_id)])
        assert (view.next_kf_id, view.next_pt_id) == (m.next_kf_id, m.next_pt_id)
        assert view.gone_pt_id.size == 0
        assert view.gone_kf_id.size == 0
        assert np.array_equal(view.loop_edges, m.loop_edges)
        on_view = covisibility.local_map(view, last, options)
        assert view.kf_id[on_view.keyframes].tolist() == m.kf_id[expected["keyframes"]].tolist()
        if expected["reference"] < 0:  # matched to no point: no view, no reference
            assert on_view.reference == -1
        else:
            assert view.kf_id[on_view.reference] == m.kf_id[expected["reference"]]


def test_a_view_refuses_rows_out_of_order() -> None:
    """A sub-map's rows are the map's keyframe rows, ascending, each once."""
    m = fixture_map(CASES[0])
    for rows in ([1, 0], [0, 0], [m.keyframes], [-1]):
        with pytest.raises(InvalidInputError, match="rows: "):
            maps.sub_map(m, np.array(rows))


@pytest.mark.parametrize("mode", ["monocular", "stereo", "rgbd"])
def test_tracking_reads_on_a_view_what_it_reads_on_the_map(mode: str, runs: dict[str, Run]) -> None:
    """On the room's last map, the next frame's local map is the same read from a view.

    From where its last keyframe left tracking, and from any of the points the next frame may
    match; the view is a part of the map.
    """
    m = runs[mode].pipeline.map
    assert m is not None
    options = Options()
    last = m.kp_point[m.keypoint_rows(m.keyframes - 1)]
    rows = covisibility.view_keyframes(m, last, int(m.kf_id[-1]), options)
    view = maps.sub_map(m, rows)
    assert view.keyframes <= m.keyframes
    assert view.points <= m.points
    near = covisibility.local_map(m, last, options).keyframes
    reachable = m.pt_id[m.observed_by(near)]
    for points in (last, reachable, reachable[::2]):
        whole = covisibility.local_map(m, points, options)
        part = covisibility.local_map(view, points, options)
        assert view.kf_id[part.keyframes].tolist() == m.kf_id[whole.keyframes].tolist()


def test_a_parent_is_not_erased() -> None:
    """``map.erase_keyframe`` refuses a keyframe that is still a parent: ``cull`` it instead.

    Python refuses before any backend runs; the C++ and Rust tests hold their cores' refusal.
    """
    case = next(c for c in CASES if c["name"] == "repair")
    m = fixture_map(case)
    with pytest.raises(InvalidInputError, match="still keyframe 2's parent"):
        maps.erase_keyframe(m, 1)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", CASES, ids=NAMES)
def test_strong_links_share_strong_points(case: dict[str, Any], backend: str) -> None:
    """Every pair sharing at least strong_points, by id."""
    chosen = require_function(backend, covisibility.__name__, "strong_links")
    got = covisibility.strong_links(fixture_map(case), fixture_options(case), backend=chosen)
    assert got.reshape(-1, 3).tolist() == case["strong_links"]


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_bad_row_or_point_is_refused(backend: str) -> None:
    """A keyframe row out of range, and a point id below -1."""
    chosen = require_function(backend, covisibility.__name__, "links")
    m, options = fixture_map(CASES[0]), fixture_options(CASES[0])
    with pytest.raises(InvalidInputError, match="k: "):
        covisibility.links(m, m.keyframes, options, backend=chosen)
    with pytest.raises(InvalidInputError, match="k: "):
        covisibility.connected(m, -1, backend=chosen)
    with pytest.raises(InvalidInputError, match="points: "):
        covisibility.local_map(m, np.array([3, -2]), options, backend=chosen)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_the_golden_record_keeps_one_tree_a_map(backend: str) -> None:
    """Every map of every run a language's golden record holds ends as one tree, at its first.

    Its keyframes not culled (a reset map's among them) each have a parent of the same map,
    alive too, but its first, which has none, and each chain of parents reaches the first.
    """
    for name, case in load_fixture(f"edge/room_runs_{backend}.json")["cases"].items():
        parents = np.array(case["keyframe_parents"], dtype=np.int64)
        why = np.array(case["keyframe_lives"][2], dtype=np.int64)  # by column: why each ended
        in_map = np.array(case["keyframe_maps"], dtype=np.int64)
        for number in np.unique(in_map):
            ids = np.flatnonzero(in_map == number)
            alive = {int(k) for k in ids if why[k] != maps.Removal.CULLED}
            root = int(ids.min())
            assert root in alive, name
            for k in alive:
                parent = int(parents[k])
                assert (parent == -1) == (k == root), f"{name}: keyframe {k}"
                assert parent == -1 or parent in alive, f"{name}: keyframe {k}"
                row = k
                for _ in range(len(alive)):
                    if row == root:
                        break
                    row = int(parents[row])
                assert row == root, f"{name}: keyframe {k}'s chain of parents"


@pytest.mark.parametrize("mode", ["monocular", "stereo", "rgbd"])
def test_the_room_keeps_a_spanning_tree(mode: str, runs: dict[str, Run]) -> None:
    """The room's last map, run through the pipeline, is one tree rooted at its first keyframe.

    Each keyframe but the first has a parent, one of the map's, which is among its links, and
    its chain of parents reaches the first keyframe: no cycle.
    """
    m = runs[mode].pipeline.map
    assert m is not None
    assert m.kf_id[0] == 0
    assert m.kf_parent[0] == -1
    rows = m.keyframe_rows(m.kf_parent[1:])
    assert np.all(rows >= 0)
    every = Options().with_settings({"covisibility.min_points": 1})
    for k, parent in enumerate(rows, start=1):
        assert parent in covisibility.links(m, k, every)[0].tolist(), f"keyframe row {k}"
    parents = np.concatenate([[-1], rows])
    for k in range(m.keyframes):
        row, steps = k, 0
        while row > 0 and steps < m.keyframes:
            row, steps = int(parents[row]), steps + 1
        assert row == 0, f"keyframe row {k}'s chain of parents"
