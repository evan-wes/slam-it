"""The loop room's bar: every language closes its loop, at the right place, and maps well.

The online-SLAM design's section 10 holds each language to a bar of results on the same data,
not to the others' decisions. On the loop room (:func:`tests.edge.room.loop_path`, written by
:func:`tests.orchestration.datasets.loop_frames`), each backend's run must: track nearly every
frame, in one map; close a loop, and none at a wrong place (judged as the test-set measure
judges real data, by :func:`slam_it.eval.loops.judge_loops`: by the truth, its two keyframes
more than :data:`slam_it.eval.loops.WRONG_DISTANCE` apart or looking more than
:data:`slam_it.eval.loops.WRONG_ANGLE` apart, so they could not have seen the same place);
keep its final trajectory within :data:`LOOP_ATE` of the truth, and closer than as tracked; and
leave its points near the room's walls (:data:`SURFACE`). The bounds are about half again B3's
first runs' (2026-10-17, the same in every backend in RGB-D and stereo): final ATE RMSE 4.67 cm
(RGB-D), 3.64 cm (stereo) and 1.14 to 1.34 cm (monocular); the points' median distance to the
walls 7.3, 7.1 and 3.9 to 4.3 cm; 276 of the 280 frames tracked in monocular, every one
otherwise.

The compiled backends hold the same bar threaded (:data:`THREADED`), their frames fed at the
room's rate (the online-SLAM design's section 12), their programs on the performance cores
(:func:`tests.orchestration.conftest.fast_cores`); Python's threaded run is measured, not tested.
Their writers publish views of a part of the map, each recorded with its size.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from slam_it import backends
from slam_it.eval import loops, runs
from tests.edge.support import OPTIONS
from tests.orchestration.conftest import RoomRun
from tests.orchestration.datasets import LOOP_CASES, Room, wall_distance

LOOP_ATE = {"rgbd": 0.07, "stereo": 0.055, "monocular": 0.02}
"""The final trajectory's ATE (RMSE, metres) each mode stays under."""

SURFACE = {"rgbd": 0.11, "stereo": 0.11, "monocular": 0.065}
"""The median distance, in metres, of the final points to the room's walls and boxes, the map
placed on the truth by its ATE alignment."""

TRACKED = 0.95
"""The share of the frames tracked at least: the design's bar."""

THREADED = ("cpp", "rust")
"""The backends whose threaded runs are held to the bar."""

RUNS = [
    *(pytest.param(backend, False, id=backend) for backend in backends.BACKENDS),
    *(pytest.param(backend, True, id=f"{backend}-threaded") for backend in THREADED),
]
"""Each backend's run in order, and the threaded runs."""


def _run(
    request: pytest.FixtureRequest, dataset: str, mode: str, backend: str, threaded: bool
) -> runs.Run:
    """Return a case's run, in order or threaded, run the first time it is asked for."""
    fixture = "loop_threaded_runs" if threaded else "loop_runs"
    return request.getfixturevalue(fixture)(dataset, mode, backend)  # type: ignore[no-any-return]


@pytest.mark.parametrize(("backend", "threaded"), RUNS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_every_backend_closes_the_loop_room(  # noqa: PLR0917 -- the case, the fixtures
    request: pytest.FixtureRequest,
    dataset: str,
    mode: str,
    backend: str,
    threaded: bool,
    loop_folders: Room,
) -> None:
    """Nearly every frame tracked in one map, a loop closed, the final trajectory near the truth.

    The final trajectory is closer to the truth than the poses as tracked: the loop corrected
    the drift. The run says whether it was threaded, and every keyframe mapped how its steps
    were scheduled.
    """
    run = _run(request, dataset, mode, backend, threaded)
    assert run.threaded is threaded
    start = next(i for i, d in enumerate(run.diagnostics) if d["keyframe"])
    for i, d in enumerate(run.diagnostics):  # each keyframe mapped says how
        assert (d["schedule"] is not None) == (d["keyframe"] and i > start)
    assert len(run.timestamps) >= TRACKED * len(run.diagnostics)
    assert len(run.maps()) == 1
    assert len(run.keyframe_loops) >= 1
    final = runs.evaluate_absolute(run, loop_folders.timestamps, loop_folders.poses)
    tracked = runs.evaluate_absolute(run, loop_folders.timestamps, loop_folders.poses, tracked=True)
    assert final.ate_rmse < LOOP_ATE[mode]
    assert final.ate_rmse < tracked.ate_rmse


@pytest.mark.parametrize(("backend", "threaded"), RUNS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_each_correction_names_its_loop_by_the_runs_ids(
    request: pytest.FixtureRequest, dataset: str, mode: str, backend: str, threaded: bool
) -> None:
    """Each loop's correction names its two keyframes by the run's ids, as the loop closed.

    In the order the loops closed, a correction's ``matched_run`` and ``current_run`` are the
    ``keyframe_loops.npy`` row the run viewer finds the loop by, beside its ``matched`` and
    ``current`` by the map's ids, which a merge renumbers.
    """
    run = _run(request, dataset, mode, backend, threaded)
    corrections = [
        d["loop"]["correction"] for d in run.diagnostics
        if d.get("loop") is not None and d["loop"]["correction"] is not None
    ]  # fmt: skip
    named = [sorted((c["matched_run"], c["current_run"])) for c in corrections]
    assert named == run.keyframe_loops.tolist()


@pytest.mark.parametrize("backend", THREADED)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_threaded_the_views_published_are_parts_of_the_map(
    request: pytest.FixtureRequest, dataset: str, mode: str, backend: str
) -> None:
    """Each view a writer published is a part of the map, its size recorded on its frame.

    One at the start and more as each keyframe is mapped; each holds a keyframe and a point at
    least, and no more than the run made; most hold fewer keyframes than the final map.
    """
    run = _run(request, dataset, mode, backend, threaded=True)
    views = [view for d in run.diagnostics for view in d["views"]]
    mapped = sum(d["schedule"] is not None for d in run.diagnostics)
    assert len(views) > mapped
    for view in views:
        assert 1 <= view["keyframes"] <= len(run.keyframe_timestamps)
        assert 1 <= view["points"] <= len(run.points)
        assert view["bytes"] > 0
    final = len(run.final_keyframes()[0])
    assert np.median([view["keyframes"] for view in views]) < final


@pytest.mark.parametrize(("backend", "threaded"), RUNS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_no_loop_closes_at_a_wrong_place(  # noqa: PLR0917 -- the case, the fixtures
    request: pytest.FixtureRequest,
    dataset: str,
    mode: str,
    backend: str,
    threaded: bool,
    loop_folders: Room,
) -> None:
    """By the truth, every loop's two keyframes were near each other and looked alike.

    Judged by the test-set measure's judge (B3's review panel, A7), which finds every keyframe
    in the truth: the room's frames are the truth's.
    """
    run = _run(request, dataset, mode, backend, threaded)
    truth = (loop_folders.timestamps, loop_folders.poses)
    judged = loops.judge_loops(run, truth, runs.evaluate_absolute(run, *truth))
    assert [loop["wrong_place"] for loop in judged] == [False] * len(run.keyframe_loops)


@pytest.mark.parametrize(("backend", "threaded"), RUNS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_the_points_lie_on_the_walls(  # noqa: PLR0917 -- the case, the fixtures
    request: pytest.FixtureRequest,
    dataset: str,
    mode: str,
    backend: str,
    threaded: bool,
    loop_folders: Room,
) -> None:
    """Placed on the truth by the trajectory's alignment, the points lie near the room's walls."""
    run = _run(request, dataset, mode, backend, threaded)
    assert wall_distance(run, loop_folders) < SURFACE[mode]


def test_a_loop_closes_with_relocalisation_off(loop_folders: Room, tmp_path: Path) -> None:
    """Loop closing alone keeps the keyframe database, and closes the room's loop.

    Before B3's review panel (H7) the database went with relocalisation, and loop closing
    without it closed nothing (run 2, H23). Python, monocular: the quickest case to run.
    """
    options = OPTIONS.with_settings({"relocalization.enabled": False})
    run = RoomRun(loop_folders, options, tmp_path)("tum", "monocular", "python")
    assert run.vocabulary is not None
    assert len(run.keyframe_loops) >= 1
