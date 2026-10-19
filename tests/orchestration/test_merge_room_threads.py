"""The merge room threaded: tracking pauses between two frames for the merge loop closing found.

The online-SLAM design's section 14, item 4, as built threaded (option B): loop closing finds
the merge and tracking makes it at its next frame, mapping's queue mapped first, a global BA
running superseded, and tracking's last frame carried into the merged map, its ids moved on by
the merge's offsets (:meth:`slam_it.edge.pipeline._Threads.merge_waiting`). Python's run is in
process, so its state can be read across the merge; each frame waits for its threads, which
Python's interpreter lock runs slower than the frames come (the loop room's threaded Python run
is measured, not tested), so that the merge comes where it does in order. The native programs'
runs, fed at the room's rate, say how their first frame after the merge was placed, and end
with their threads joined, held to the merge room's bar in order (``test_merge_room.py``): one
map at the end, merged at no wrong place, its final trajectory near the truth.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it.edge import frames, pipeline
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.eval import runs
from slam_it.orchestration import vo
from tests.conftest import require_backend
from tests.edge import room
from tests.edge.support import CAMERA, words_for
from tests.orchestration.conftest import require_program
from tests.orchestration.datasets import LOOP_CASES, MERGE_OPTIONS, Room, merge_frames
from tests.orchestration.test_merge_room import assert_merged

THREADED = [
    pytest.param("cpp", False, id="cpp"),
    pytest.param("cpp", True, id="cpp-locks"),
    pytest.param("rust", False, id="rust"),
]
"""The native programs' threaded schemes: C++'s design and ORB-SLAM's locking, and Rust's."""


@dataclass(frozen=True)
class _Seen:
    """Tracking's state and the map, read just before or after a merge."""

    points: NDArray[np.int64]
    anchor: int
    map: Map
    corrections: int


def _seen(run: pipeline.Pipeline) -> _Seen:
    """Read tracking's last frame's points and anchor, the map, and the view's corrections."""
    last, anchor, view = run.tracking.last_tracked, run.tracking.anchor, run.owner.view
    assert last is not None
    assert anchor is not None
    assert view is not None
    return _Seen(last.points.copy(), anchor, run.owner.read(), view.corrections)


def _row(ids: NDArray[np.int64], wanted: int) -> int:
    """Return the row of id `wanted` in a map's ids, -1 if it holds none."""
    rows = np.flatnonzero(ids == wanted)
    return int(rows[0]) if rows.size else -1


def _run_merging(
    options: Options, rendered_loop: list[room.Frame]
) -> tuple[
    pipeline.Pipeline, list[pipeline.FrameResult], list[tuple[_Seen, _Seen, tuple[int, int]]]
]:
    """Run the merge room threaded, in RGB-D, in process, reading each merge as it is made.

    Each frame waits for mapping and loop closing to handle its keyframe (the module
    docstring). Returns the finished pipeline, the frames' results, and for each merge
    tracking's state before and after it, with the merge's keyframe and point offsets (the
    matched map's next ids).
    """
    require_backend("cpp")
    run = pipeline.Pipeline("rgbd", CAMERA, options, words_for(options), threaded=True)
    threads = run._threads  # the merge read as tracking makes it
    assert threads is not None
    made: list[tuple[_Seen, _Seen, tuple[int, int]]] = []
    merge_waiting = threads.merge_waiting

    def read_merge(tracking: pipeline.TrackingState) -> float:
        waiting = run.closing.waiting
        if waiting is None:
            return merge_waiting(tracking)
        before = _seen(run)
        matched = next(k.map for k in run.atlas.kept() if k.number == waiting.merge.map)
        waited = merge_waiting(tracking)
        made.append((before, _seen(run), (matched.next_kf_id, matched.next_pt_id)))
        return waited

    threads.merge_waiting = read_merge  # type: ignore[method-assign]
    results = []
    for f in merge_frames(rendered_loop):
        built = frames.rgbd(
            f.gray, f.depth, f.timestamp, CAMERA, run.extraction(), backend="cpp"
        )  # the C++ core extracts, for speed
        results.append(run.track(built))
        threads.drain()
    run.finish()
    return run, results, made


def test_python_threaded_carries_tracking_into_the_merged_map(
    rendered_loop: list[room.Frame],
) -> None:
    """Python threaded merges the merge room, tracking's last frame carried into the merged map.

    Across the merge, the last frame's point ids and its anchor keyframe move on by the merge's
    offsets, each still naming what it named (the same keyframe, by its time; each point the
    merged map kept, by the keyframe that first saw it), and the view's count moves on; the
    next frame is placed by the motion model. At the end the run is one map, and every thread
    it started has ended.
    """
    before = set(threading.enumerate())
    run, results, made = _run_merging(MERGE_OPTIONS, rendered_loop)
    assert not [t for t in threading.enumerate() if t not in before], "a thread was left"
    assert len(made) == 1, "one merge, made by tracking"
    old, new, (kf_offset, pt_offset) = made[0]
    assert run.closing.waiting is None
    assert new.corrections == old.corrections + 1
    assert new.anchor == old.anchor + kf_offset
    was, now = _row(old.map.kf_id, old.anchor), _row(new.map.kf_id, new.anchor)
    assert was >= 0
    assert now >= 0
    assert new.map.kf_time[now] == old.map.kf_time[was]
    np.testing.assert_array_equal(new.points, np.where(old.points >= 0, old.points + pt_offset, -1))
    kept = 0
    for point in new.points[new.points >= 0]:
        row = _row(new.map.pt_id, int(point))
        if row < 0:  # fused into the matched map's point, or culled
            continue
        first = old.map.pt_first_kf[_row(old.map.pt_id, int(point) - pt_offset)]
        assert new.map.pt_first_kf[row] == first + kf_offset
        kept += 1
    assert kept > 0
    after = next(
        k for k, r in enumerate(results) if r.map == 0 and k > 0 and results[k - 1].map == 1
    )
    assert results[after].summary.placed_by is pipeline.Placement.MOTION
    assert run.atlas.map_id == 0
    assert np.unique(run.lifetimes().keyframe_maps).tolist() == [0]


def _after_merge(run: runs.Run) -> int:
    """Return the first frame tracked in the merged map, map 0, after the second map's last."""
    second = [k for k, d in enumerate(run.diagnostics) if d["map"] == 1]
    return second[-1] + 1


@pytest.mark.parametrize(("backend", "locks"), THREADED)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_a_native_threaded_run_merges_the_merge_room(  # noqa: PLR0917 -- the case, the fixtures
    dataset: str, mode: str, backend: str, locks: bool, merge_folders: Room, tmp_path: Path
) -> None:
    """Threaded, the merge room ends in one map, merged at the right place, near the truth.

    The first frame after the merge is placed by the motion model: tracking's last frame was
    carried into the merged map. The program ends, its threads joined.
    """
    folder = merge_folders.tum if dataset == "tum" else merge_folders.euroc
    run = vo.run(
        dataset, folder, mode, tmp_path / "run", backend=require_program(backend),
        options=MERGE_OPTIONS, threaded=True, locks=locks,
    )  # fmt: skip
    assert (run.threaded, run.locks) == (True, locks)
    after = _after_merge(run)
    assert run.diagnostics[after]["map"] == 0
    assert run.diagnostics[after]["placed_by"] == pipeline.Placement.MOTION.value
    assert_merged(run, merge_folders, mode)
