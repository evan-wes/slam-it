"""Whole sequences through every backend.

The room from its TUM and EuRoC folders, the native programs' command line, and their failures.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge.options import Options
from slam_it.eval import budgets, runs, vocabularies
from slam_it.foundations.errors import DegenerateError, InvalidInputError, NotConvergedError
from slam_it.orchestration import vo
from tests.conftest import FIXTURES, assert_matches_fixture, load_fixture
from tests.edge.support import (
    ATE_BOUND,
    FRAMES,
    MONOCULAR_START,
    OPTIONS,
    SOLVER_TOLERANCE,
    assert_close,
    assert_records_match,
)
from tests.fixtures.edge.generate_room_runs import record, summaries
from tests.orchestration.conftest import RoomRun, require_program
from tests.orchestration.datasets import (
    BLANK_AT,
    CASES,
    FOUND_AT,
    FOUND_BLANKS,
    LOST_AT,
    WINDOW_AT,
    WINDOW_BLANKS,
    WINDOW_OPTIONS,
    Room,
    room_inputs,
)

SEQUENCES = FIXTURES / "eval" / "sequences"
"""The readers' fixture folders: their files listed, their images absent."""


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_backend_tracks_the_room(
    dataset: str, mode: str, backend: str, room_folders: Room, room_runs: RoomRun
) -> None:
    """Every frame after the start is tracked, close to the truth, and the run says what ran."""
    run = room_runs(dataset, mode, backend)
    assert (run.dataset, run.mode, run.backend) == (dataset, mode, backend)
    assert run.sequence == f"room_{dataset}"
    assert run.settings == OPTIONS.as_dict()
    assert len(run.diagnostics) == FRAMES
    assert all(d["state"] == "tracking" for d in run.diagnostics[MONOCULAR_START:])
    started = FRAMES - MONOCULAR_START if mode == "monocular" else FRAMES
    assert len(run.timestamps) >= started
    assert len(run.final_keyframes()[0]) >= 2
    assert len(run.final_points()) > 0
    error = runs.evaluate(run, room_folders.timestamps, room_folders.poses)
    assert error.matched == len(run.timestamps)
    assert error.ate_rmse < ATE_BOUND[mode]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_frame_times_its_steps(
    dataset: str, mode: str, backend: str, room_runs: RoomRun
) -> None:
    """Every frame times reading, building and tracking; only keyframes after the start map.

    The frame that starts the map is initialisation, which tracking's time holds, as ORB-SLAM's
    tracking thread initialises.
    """
    run = room_runs(dataset, mode, backend)
    for d in run.diagnostics:
        assert all(d[step] > 0.0 for step in runs.STEPS[:3])
    start = next(i for i, d in enumerate(run.diagnostics) if d["keyframe"])
    mapped = [d["map_seconds"] > 0.0 for d in run.diagnostics]
    assert mapped == [d["keyframe"] and i > start for i, d in enumerate(run.diagnostics)]
    assert all(d["map_seconds"] == 0.0 for d in run.diagnostics if not d["keyframe"])


def _assert_same_run(native: runs.Run, reference: runs.Run) -> None:
    """Check that a native run made the reference's decisions and placed what it placed.

    The same frames tracked with the same matches, the same keyframes, poses and points within
    the solver tolerance, the same lives for every point and keyframe, and the same stage
    summaries (their bundle adjustments' costs within it too).
    """
    np.testing.assert_array_equal(native.timestamps, reference.timestamps)
    np.testing.assert_array_equal(native.keyframe_timestamps, reference.keyframe_timestamps)
    np.testing.assert_array_equal(native.point_lives, reference.point_lives)
    np.testing.assert_array_equal(native.keyframe_lives, reference.keyframe_lives)
    np.testing.assert_array_equal(native.keyframe_parents, reference.keyframe_parents)
    np.testing.assert_array_equal(native.keyframe_links, reference.keyframe_links)
    assert native.points.shape == reference.points.shape
    assert_close(native.poses, reference.poses, SOLVER_TOLERANCE)
    assert_close(native.keyframe_poses, reference.keyframe_poses, SOLVER_TOLERANCE)
    assert_close(native.points, reference.points, SOLVER_TOLERANCE)
    states = [(d["state"], d["tracked"], d["keyframe"]) for d in native.diagnostics]
    assert states == [(d["state"], d["tracked"], d["keyframe"]) for d in reference.diagnostics]
    assert_records_match(summaries(native), summaries(reference))


def _assert_same_features(runs_of: RoomRun, dataset: str, mode: str, backend: str) -> None:
    """Check that a native run recorded Python's feature records, byte for byte."""
    native, reference = (
        runs_of.features(dataset, mode, backend),
        runs_of.features(dataset, mode, "python"),
    )
    np.testing.assert_array_equal(native.records, reference.records)
    np.testing.assert_array_equal(native.pixels, reference.pixels)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_backends_agree(dataset: str, mode: str, backend: str, room_runs: RoomRun) -> None:
    """A native program makes Python's decisions and places every pose and point as Python does.

    It records every keypoint's pixel, level, fate and point exactly as Python does.
    """
    _assert_same_run(room_runs(dataset, mode, backend), room_runs(dataset, mode, "python"))
    _assert_same_features(room_runs, dataset, mode, backend)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_keypoints_fate_agrees_with_its_frame(
    dataset: str, mode: str, backend: str, room_runs: RoomRun
) -> None:
    """Each frame's fates count what its diagnostics say, and name points of the right lives.

    A frame that started the map made a point of each keypoint it observes; a mapped keyframe
    made as many as its mapping made from depth and triangulated. A tracked frame's inliers are
    its tracked keypoints, but on a keyframe also those tracked to a point its mapping culled
    and then made into a new one. A point made is one the frame made; one tracked was made
    before the frame and still in the map as it began.
    """
    run, features = room_runs(dataset, mode, backend), room_runs.features(dataset, mode, backend)
    frame, fate, point = features.records[:, 0], features.records[:, 2], features.records[:, 3]
    unmatched, _, tracked, made = range(len(runs.FATES))
    assert np.array_equal(np.unique(frame), np.arange(len(run.diagnostics)))
    assert np.all((point == -1) == (fate == unmatched))
    for k, d in enumerate(run.diagnostics):
        counts = np.bincount(fate[frame == k], minlength=len(runs.FATES))
        if d["placed_by"] == "initialization":
            assert (counts[made], counts[tracked]) == (d["tracked"], 0)
        else:
            mapped = d["mapping"]
            assert counts[made] == (
                0 if mapped is None else mapped["from_depth"] + mapped["triangulated"]
            )
            assert counts[tracked] <= d["tracked"] <= counts[tracked] + counts[made]
            assert mapped is not None or counts[tracked] == d["tracked"]
    lives = run.point_lives
    assert np.all(lives[point[fate == made], 0] == frame[fate == made])
    held = point[fate == tracked]
    assert np.all(lives[held, 0] < frame[fate == tracked])
    assert np.all((lives[held, 1] == -1) | (lives[held, 1] >= frame[fate == tracked]))


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_gap_room_fails_once_and_recovers_twice(
    dataset: str, mode: str, backend: str, gap_runs: RoomRun
) -> None:
    """The blank frame fails; across the gap PnP recovers twice; every other frame is tracked.

    The blank frame fails, the fallback too; across the gap the motion model fails and PnP
    recovers, twice (the recovery's velocity spans the gap); every other frame from the start
    is tracked (review panel findings A3, A14). The frames' stage summaries say which ran the
    fallback and what placed them.
    """
    run = gap_runs(dataset, mode, backend)
    fallbacks = [d for d in run.diagnostics if d["pnp"] is not None]
    assert [d["placed_by"] for d in fallbacks] == [None, "local_pnp", "local_pnp"]
    assert run.diagnostics.index(fallbacks[0]) == BLANK_AT
    started = next(i for i, d in enumerate(run.diagnostics) if d["tracked"] > 0)
    untracked = [i for i, d in enumerate(run.diagnostics) if i >= started and d["tracked"] == 0]
    assert untracked == [BLANK_AT]
    assert len(run.timestamps) == len(run.diagnostics) - started - 1
    assert all(d["state"] == "tracking" for d in run.diagnostics[started:])


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_backends_agree_across_the_gap(
    dataset: str, mode: str, backend: str, gap_runs: RoomRun
) -> None:
    """Through a recovery, a native program makes Python's decisions as on the whole room."""
    _assert_same_run(gap_runs(dataset, mode, backend), gap_runs(dataset, mode, "python"))
    _assert_same_features(gap_runs, dataset, mode, backend)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_backends_agree_off_the_defaults(
    dataset: str, mode: str, backend: str, moved_runs: RoomRun
) -> None:
    """With the newer settings off their defaults, a native program makes Python's decisions.

    Review panel finding H21: at the defaults, a native that hard-coded a setting would agree.
    """
    _assert_same_run(moved_runs(dataset, mode, backend), moved_runs(dataset, mode, "python"))
    _assert_same_features(moved_runs, dataset, mode, backend)


@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_moved_settings_change_the_decisions(
    dataset: str, mode: str, moved_runs: RoomRun, gap_runs: RoomRun
) -> None:
    """Off their defaults, the settings change what Python decides on the gap room.

    So the agreement off the defaults is evidence that the natives read them.
    """
    moved = record(
        moved_runs(dataset, mode, "python"), moved_runs.features(dataset, mode, "python")
    )
    default = record(gap_runs(dataset, mode, "python"), gap_runs.features(dataset, mode, "python"))
    assert moved != default


ROOMS = ("room", "gap", "lost", "kept", "found", "window")
"""The rooms the golden record holds, each run by its fixture ``<room>_runs``."""

GOLDEN = load_fixture("edge/room_runs.json")
"""What the pipeline decided on the room and the gap room (``generate_room_runs.py``)."""


def test_the_golden_record_is_of_these_rooms() -> None:
    """The record is of the rooms and settings the tests run, and holds exactly their cases.

    Checked before any decision, so a change to the rooms, rather than to the pipeline, is
    named as one (review panel finding A20).
    """
    assert GOLDEN["inputs"] == room_inputs()
    names = {f"{name}/{dataset}/{mode}" for name in ROOMS for dataset, mode in CASES}
    assert set(GOLDEN["cases"]) == names


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("name", ROOMS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_backend_matches_the_golden_record(
    dataset: str, mode: str, name: str, backend: str, request: pytest.FixtureRequest
) -> None:
    """Every backend's run is the recorded one.

    A decision changed in all three languages fails here until the record is regenerated
    (review panel finding A1).
    """
    runs_of: RoomRun = request.getfixturevalue(f"{name}_runs")
    run = runs_of(dataset, mode, backend)
    expected = GOLDEN["cases"][f"{name}/{dataset}/{mode}"]
    floating = {"poses": None, "summaries": None}
    assert record(run, runs_of.features(dataset, mode, backend)) | floating == expected | floating
    golden = np.reshape(expected["poses"], (-1, 16))  # a run whose every map was reset has none
    assert_matches_fixture(run.poses.reshape(-1, 16), golden, GOLDEN["tolerance"])
    assert_records_match(summaries(run), expected["summaries"], SOLVER_TOLERANCE)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_a_lost_track_starts_a_new_map_and_the_run_goes_on(
    dataset: str, mode: str, backend: str, lost_runs: RoomRun, kept_runs: RoomRun
) -> None:
    """Each blank stretch loses the track; the frames after start the next map.

    At the room's settings each lost map, of a few keyframes, is reset: its keyframes and points
    end on its lost frame, and its frames' poses are dropped. With every lost map kept, its
    poses and keyframes stay, and the run tracks in a segment per map. A stereo or RGB-D run
    starts map 2 after the second loss and tracks it to the room's end; a monocular one has
    too few frames left to start it.
    """
    losses = [at + OPTIONS.loss.lost_frames - 1 for at in LOST_AT]
    reset_code = runs.REMOVALS.index("reset")
    last = [0, 1] if mode == "monocular" else [0, 1, 2]
    for runs_of, reset in ((lost_runs, True), (kept_runs, False)):
        run = runs_of(dataset, mode, backend)
        assert len(run.diagnostics) == FRAMES  # the run goes on to the room's end
        states = [d["state"] for d in run.diagnostics]
        assert [k for k, state in enumerate(states) if state == "lost"] == losses
        for map_id, at in enumerate(losses):
            lost = run.diagnostics[at]
            assert (lost["map"], lost["lost"]["reset"]) == (map_id, reset)
            removed = run.keyframe_lives[run.keyframe_maps == map_id]
            if reset:
                assert np.all(removed[:, 1:] == [at, reset_code])
            else:
                assert np.all(removed[:, 1] == -1)
        after = {d["map"] for d in run.diagnostics[losses[-1] + 1 :]}
        assert after == ({None} if mode == "monocular" else {2})
        assert run.maps().tolist() == last
        kept = [m for m in last if not (reset and m < len(losses))]
        assert sorted(set(run.pose_maps.tolist())) == kept
        span = runs.tracking(run, vo.open_source(dataset, runs_of.sequence(dataset), mode).times)
        assert [s.map for s in span.segments] == kept
        assert (span.losses, span.resets, span.relocalizations) == (2, 2 * int(reset), 0)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_a_track_found_again_is_counted_as_relocalised(
    dataset: str, mode: str, backend: str, found_runs: RoomRun
) -> None:
    """The frame after the found room's blank ones is relocalised in map 0, and counted."""
    run = found_runs(dataset, mode, backend)
    back = FOUND_AT + FOUND_BLANKS
    placed = [k for k, d in enumerate(run.diagnostics) if d["placed_by"] == runs.RELOCALIZED]
    assert placed == [back]
    assert run.diagnostics[back]["map"] == 0
    span = runs.tracking(run, vo.open_source(dataset, found_runs.sequence(dataset), mode).times)
    assert (span.losses, span.resets, span.relocalizations) == (0, 0, 1)
    assert [s.map for s in span.segments] == [0]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_window_room_loses_its_kept_maps_by_the_time(
    dataset: str, mode: str, backend: str, window_runs: RoomRun
) -> None:
    """Each kept map is lost 0.05 s after its first failed frame, before ``lost_frames`` failures.

    Review panel findings L1 and L2. In stereo and RGB-D, the frame after the first blank
    stretch is relocalised in map 0, and the second stretch's last blank frame loses it; an
    RGB-D run first culls keyframes, which leave the database before the relocalisation. A
    monocular run loses its small maps by the time before the stretches.
    """
    run = window_runs(dataset, mode, backend)
    lost = [k for k, d in enumerate(run.diagnostics) if d["state"] == "lost"]
    assert lost
    for k in lost:
        assert run.diagnostics[k]["lost"]["reset"] is False
        failed = 0  # the failed frames of the lost map, up to its lost frame
        while failed <= k and run.diagnostics[k - failed]["tracked"] == 0:
            failed += 1
        assert 1 < failed < WINDOW_OPTIONS.loss.lost_frames
    if mode == "monocular":
        return
    back = FOUND_AT + WINDOW_BLANKS
    placed = [k for k, d in enumerate(run.diagnostics) if d["placed_by"] == runs.RELOCALIZED]
    assert placed == [back]
    assert run.diagnostics[back]["map"] == 0
    assert lost == [WINDOW_AT + FOUND_BLANKS - 1]
    if mode == "rgbd":
        culled = run.keyframe_lives[run.keyframe_lives[:, 2] == runs.REMOVALS.index("culled")]
        assert len(culled) > 0
        assert np.all(culled[:, 1] < back)


def test_a_request_crosses_as_a_command_line() -> None:
    """Only settings off their defaults are passed, integers as integers, reals round-tripping."""
    options = Options().with_settings({"mapping.window": 7, "stereo.close_depth": 0.1 + 0.2})
    request = vo.Request(
        "tum", Path("seq"), "rgbd", Path("out"), 2, 5, options=options, record_features=True
    )
    assert request.arguments() == [
        *("--dataset", "tum", "--sequence", "seq", "--mode", "rgbd", "--out", "out"),
        *("--first", "2", "--count", "5", "--record-features"),
        *("--option", "stereo.close_depth=0.30000000000000004", "--option", "mapping.window=7"),
    ]
    assert "--record-features" not in replace(request, record_features=False).arguments()


def test_parse_reads_a_command_line() -> None:
    """Every flag is read, in any order; the settings named are set, the rest default."""
    request = vo.parse(
        [*("--mode", "rgbd", "--dataset", "tum", "--sequence", "seq", "--out", "out"),
         *("--first", "3", "--record-features", "--count", "10", "--option", "mapping.window=7"),
         *("--option", "stereo.close_depth=35.5")]
    )  # fmt: skip
    expected = Options().with_settings({"mapping.window": 7, "stereo.close_depth": 35.5})
    assert request == vo.Request("tum", Path("seq"), "rgbd", Path("out"), 3, 10, expected, True)
    plain = vo.parse(["--dataset", "tum", "--sequence", "s", "--mode", "rgbd", "--out", "o"])
    assert (plain.first, plain.count, plain.options) == (0, None, Options())
    assert not plain.record_features


@pytest.mark.parametrize(
    ("more", "message"),
    [
        (["--mode", "lidar"], "--mode: given twice"),
        (["--first", "x"], "--first: expected an integer, got 'x'"),
        (["--first", "1_000"], "--first: expected an integer"),
        (["--count"], "--count: expected a value"),
        (["--option", "windows=7"], "--option: no setting 'windows'"),
        (["--option", "mapping.window=7.5"], "mapping.window: expected an integer, got '7.5'"),
        (
            ["--option", "stereo.close_depth=inf"],
            "stereo.close_depth: expected a finite number, got 'inf'",
        ),
        (["--option", "stereo.close_depth=3_5"], "stereo.close_depth: expected a finite number"),
        (["--option", "stereo.close_depth=+-5"], "stereo.close_depth: expected a finite number"),
        (["--option", "window"], "--option: expected KEY=VALUE, got 'window'"),
        (["--dataset", "euroc"], "--dataset: given twice"),
        (["--verbose", "1"], "unknown argument '--verbose'"),
        (["--record-features", "--record-features"], "--record-features: given twice"),
        (["--vocabulary", "a", "--vocabulary", "b"], "--vocabulary: given twice"),
        (
            ["--option", "relocalization.enabled=maybe"],
            "relocalization.enabled: expected true or false, got 'maybe'",
        ),
        (
            ["--option", "relocalization.enabled=1"],
            "relocalization.enabled: expected true or false, got '1'",
        ),
    ],
)
def test_parse_refuses_what_the_native_programs_refuse(more: list[str], message: str) -> None:
    """A bad flag, value or setting, a repeated flag, or a missing value: a usage error.

    The cases and messages of the native programs' own tests.
    """
    base = ["--dataset", "tum", "--sequence", "seq", "--mode", "rgbd", "--out", "out"]
    with pytest.raises(vo.UsageError, match=re.escape(message)):
        vo.parse([*base, *more])


@pytest.mark.parametrize(
    ("text", "value"),
    [
        ("stereo.close_depth=35.5", 35.5),
        ("stereo.close_depth=+0.5", 0.5),
        ("stereo.close_depth=.5", 0.5),
        ("stereo.close_depth=5.", 5.0),
        ("stereo.close_depth=1e5", 1e5),
        ("stereo.close_depth= 2.0 ", 2.0),
        ("stereo.close_depth=-1", -1.0),  # a number; its range is the settings' to check
        ("mapping.window=10", 10),
    ],
)
def test_parse_setting_reads_what_the_programs_read(text: str, value: float) -> None:
    """Decimals with a sign, a lone point or an exponent; integers whole."""
    key, read = vo.parse_setting(text)
    assert (key, read, type(read)) == (text.partition("=")[0], value, type(value))


@pytest.mark.parametrize(
    "text",
    [
        "stereo.close_depth=inf",
        "stereo.close_depth=nan",
        "stereo.close_depth=1e999",
        "stereo.close_depth=3_5",
        "stereo.close_depth=1e",
        "stereo.close_depth=+-5",
        "stereo.close_depth=",
        "stereo.close_depth=0x10",
        "mapping.window=1.0",
        "mapping.window=1_000",
        "no_such_setting=1",
        "stereo.close_depth",
    ],
)
def test_parse_setting_refuses_what_the_programs_refuse(text: str) -> None:
    """Not finite, past float64, underscores, two signs, hex, a fraction for an integer."""
    with pytest.raises(vo.UsageError):
        vo.parse_setting(text)


def test_parse_refuses_an_unknown_mode_or_a_missing_flag() -> None:
    """A mode by another name, or a required flag left out: a usage error."""
    base = ["--dataset", "tum", "--sequence", "seq", "--out", "out"]
    with pytest.raises(vo.UsageError, match="--mode: expected monocular, stereo or rgbd"):
        vo.parse([*base, "--mode", "lidar"])
    with pytest.raises(vo.UsageError, match="--mode: missing"):
        vo.parse(base)


def test_a_setting_out_of_range_is_a_failed_run_not_a_usage_error() -> None:
    """As the natives' check refuses it after reading the command line, with exit code 3."""
    base = ["--dataset", "tum", "--sequence", "seq", "--mode", "rgbd", "--out", "out"]
    with pytest.raises(InvalidInputError, match=r"mapping\.window") as raised:
        vo.parse([*base, "--option", "mapping.window=0"])
    assert not isinstance(raised.value, vo.UsageError)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_every_program_records_its_process(backend: str, tmp_path: Path) -> None:
    """Each backend's program writes its run with its own peak memory and CPUs, and says so."""
    request = vo.Request(
        "tum", SEQUENCES / "tum_camera", "rgbd", tmp_path, count=0, vocabulary=vocabularies.DEFAULT
    )
    done = subprocess.run(
        [*vo.command(require_program(backend)), *request.arguments()],
        capture_output=True, text=True, check=False,
    )  # fmt: skip
    assert done.returncode == 0, done.stderr
    assert done.stdout.startswith("slam_it_vo: 0 of 0 frames tracked")
    run = runs.read_run(tmp_path)
    assert run.peak_memory is not None
    assert run.peak_memory > 2**20
    assert run.cpus == sorted(os.sched_getaffinity(0))


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_run_records_its_settings(backend: str, tmp_path: Path) -> None:
    """A run of no frames records the settings asked for, every one, exactly.

    Its peak memory is its program's; Python's in-process run, which shares its process with
    whatever called it, records none.
    """
    options = Options().with_settings(
        {
            "features.n_features": 700,
            "stereo.close_depth": 0.1 + 0.2,
            "adjustment.chi2_mono": 5.0,
            "seed": 3,
        }
    )
    run = vo.run(
        "tum",
        SEQUENCES / "tum_camera",
        "rgbd",
        tmp_path,
        backend=require_program(backend),
        options=options,
        count=0,
    )
    assert run.settings == options.as_dict()
    assert len(run.timestamps) == len(run.diagnostics) == 0
    assert (run.peak_memory is None) == (backend == "python")


@pytest.mark.parametrize(
    ("dataset", "mode", "message"),
    [
        ("kitti", "rgbd", "dataset"),
        ("tum", "lidar", "mode"),
        ("tum", "stereo", "no stereo pairs"),
        ("euroc", "rgbd", "no depth maps"),
    ],
)
def test_refuses_what_a_dataset_does_not_record(dataset: str, mode: str, message: str) -> None:
    """An unknown dataset or mode, or a mode the dataset has no sensor for, is refused."""
    with pytest.raises(InvalidInputError, match=message):
        vo.run(dataset, SEQUENCES / "tum_camera", mode, "unused")


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rgbd_needs_the_depth_sensors_bf(backend: str, tmp_path: Path) -> None:
    """An RGB-D run on a folder whose camera gives no bf is refused, by every backend."""
    with pytest.raises(InvalidInputError, match="bf"):
        vo.run(
            "tum",
            SEQUENCES / "tum_no_depth",
            "rgbd",
            tmp_path,
            backend=require_program(backend),
            count=1,
        )


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_stereo_run_matches_with_its_settings(
    backend: str, room_folders: Room, tmp_path: Path
) -> None:
    """The stereo matcher's settings reach every backend's frames (review panel finding A15).

    Below a Hamming distance of 1 hardly a pair matches, so the first frame starts its map with
    a handful of points, where the default's makes hundreds; the run records the setting.
    """
    made = {}
    for name, options in (
        ("default", OPTIONS),
        ("strict", OPTIONS.with_settings({"stereo.max_distance": 1})),
    ):
        run = vo.run(
            "euroc",
            room_folders.euroc,
            "stereo",
            tmp_path / name,
            backend=require_program(backend),
            options=options,
            count=1,
        )
        made[name] = len(run.points)
        assert run.settings["stereo"]["max_distance"] == options.stereo.max_distance
    assert 0 < 10 * made["strict"] < made["default"]


def test_refuses_negative_frames() -> None:
    """A negative first frame or count is refused."""
    with pytest.raises(InvalidInputError, match="at least 0"):
        vo.run("tum", SEQUENCES / "tum_camera", "rgbd", "unused", count=-1)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_program_fails_a_file_it_cannot_read_or_write_alike(backend: str, tmp_path: Path) -> None:
    """A file a program cannot read or write fails the run alike in every program.

    A missing image, or a run folder it cannot write, is a failed run of kind InvalidInput, its
    message prefixed and naming the file.
    """
    command = vo.command(require_program(backend))
    missing = vo.Request(
        "euroc", SEQUENCES / "euroc", "stereo", tmp_path, count=1, vocabulary=vocabularies.DEFAULT
    )
    done = subprocess.run(
        [*command, *missing.arguments()], capture_output=True, text=True, check=False
    )
    assert done.returncode == vo.EXIT_FAILED
    assert re.match(r"slam_it_vo: .*mav0/cam0/data/\d+\.png", done.stderr)
    blocked = tmp_path / "a_file"
    blocked.write_text("")
    unwritable = vo.Request(
        "tum",
        SEQUENCES / "tum_camera",
        "rgbd",
        blocked / "run",
        count=0,
        vocabulary=vocabularies.DEFAULT,
    )
    done = subprocess.run(
        [*command, *unwritable.arguments()], capture_output=True, text=True, check=False
    )
    assert done.returncode == vo.EXIT_FAILED, done.stderr
    assert done.stderr.startswith("slam_it_vo: ")
    assert "a_file" in done.stderr


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_a_failed_program_raises_its_error(backend: str, tmp_path: Path) -> None:
    """A native run that fails raises the error of its kind, with the program's message."""
    with pytest.raises(InvalidInputError, match="no such file"):
        vo.run(
            "euroc",
            SEQUENCES / "euroc",
            "stereo",
            tmp_path,
            backend=require_program(backend),
            count=1,
        )


def test_a_killed_program_says_so(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A program killed by a signal (a budget's memory cap kills with 9) raises, naming it."""
    program = tmp_path / "slam_it_vo"
    program.write_text("#!/bin/sh\nkill -9 $$\n")
    program.chmod(0o755)
    monkeypatch.setattr(backends, "program", lambda backend, name: program)
    with pytest.raises(RuntimeError, match="killed by signal 9"):
        vo.run("tum", SEQUENCES / "tum_camera", "rgbd", tmp_path / "run", backend="cpp", count=0)


@pytest.mark.budget
@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_backend_runs_the_room_within_the_laptop_budget(  # noqa: PLR0917 -- case, fixtures
    dataset: str, mode: str, backend: str, room_folders: Room, room_runs: RoomRun, tmp_path: Path
) -> None:
    """Pinned and capped, each program takes the decisions it takes unbudgeted, within memory."""
    budget = budgets.BUDGETS["laptop-cpu"]
    folder = room_folders.tum if dataset == "tum" else room_folders.euroc
    run = vo.run(
        dataset, folder, mode, tmp_path, backend=require_program(backend), options=OPTIONS,
        budget=budget,
    )  # fmt: skip
    _assert_same_run(run, room_runs(dataset, mode, "python"))
    assert run.peak_memory is not None
    assert 0 < run.peak_memory < budget.memory
    assert run.cpus == budgets.cpus(budget)
    assert run.memory_cap == budget.memory
    assert budgets.judge(run).count == FRAMES


@pytest.mark.budget
def test_a_program_past_its_budgets_memory_is_killed(room_folders: Room, tmp_path: Path) -> None:
    """Python's program within 16 MiB (its interpreter alone holds more) is killed, not swapped."""
    tiny = budgets.Budget("tiny", cores=1, memory=16 * 2**20, stands_for="nothing real")
    with pytest.raises(RuntimeError, match="killed by signal 9, within tiny's 16777216 bytes"):
        vo.run("tum", room_folders.tum, "rgbd", tmp_path, backend="python", budget=tiny)


@pytest.mark.parametrize(
    ("code", "error"),
    [(vo.EXIT_FAILED, InvalidInputError), (vo.EXIT_FAILED + 1, DegenerateError),
     (vo.EXIT_FAILED + 2, NotConvergedError)],
)  # fmt: skip
def test_a_programs_exit_code_names_its_error(
    code: int, error: type[Exception], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A failed program's exit code raises the error of its kind, with the program's message."""
    program = tmp_path / "slam_it_vo"
    program.write_text(f"#!/bin/sh\necho 'slam_it_vo: it failed' >&2\nexit {code}\n")
    program.chmod(0o755)
    monkeypatch.setattr(backends, "program", lambda backend, name: program)
    with pytest.raises(error, match="it failed"):
        vo.run("tum", SEQUENCES / "tum_camera", "rgbd", tmp_path / "run", backend="cpp", count=0)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_program_refuses_a_bad_command_line(backend: str) -> None:
    """A command line the program cannot read exits with the usage, and code EXIT_USAGE."""
    program = vo.command(require_program(backend))
    for arguments in (["--dataset", "tum"], ["--option", "windows=7"], ["--mode"]):
        done = subprocess.run([*program, *arguments], capture_output=True, text=True, check=False)
        assert done.returncode == vo.EXIT_USAGE
        assert done.stderr.startswith("slam_it_vo: ")
        assert "\nusage: " in done.stderr


def test_the_vocabulary_and_a_flag_setting_cross_the_command_line() -> None:
    """``--vocabulary`` is read and passed on; a flag setting crosses as true or false."""
    base = ["--dataset", "tum", "--sequence", "seq", "--mode", "rgbd", "--out", "out"]
    request = vo.parse([*base, "--vocabulary", "words", "--option", "relocalization.enabled=false"])
    assert (request.vocabulary, request.options.relocalization.enabled) == (Path("words"), False)
    arguments = request.arguments()
    assert arguments[arguments.index("--vocabulary") + 1] == "words"
    assert arguments[arguments.index("--option") + 1] == "relocalization.enabled=false"
    assert vo.parse(base).vocabulary is None
    assert vo.parse_setting("relocalization.enabled= true ") == ("relocalization.enabled", True)


@pytest.mark.parametrize(
    ("relocalize", "vocabulary", "message"),
    [
        (True, None, "--vocabulary: relocalize needs one"),
        (False, Path("words"), "--vocabulary: given, but relocalize is off"),
    ],
)
def test_check_refuses_a_vocabulary_missing_or_unasked_for(
    relocalize: bool, vocabulary: Path | None, message: str
) -> None:
    """Relocalising needs a vocabulary; one given with relocalisation off is refused."""
    request = vo.Request(
        "tum", SEQUENCES / "tum_camera", "rgbd", Path("out"),
        options=Options().with_settings({"relocalization.enabled": relocalize}),
        vocabulary=vocabulary,
    )  # fmt: skip
    with pytest.raises(InvalidInputError, match=re.escape(message)):
        vo.check(request)


def test_run_relocalises_with_the_committed_vocabulary_unless_told_otherwise(
    tmp_path: Path,
) -> None:
    """Unnamed, a relocalising run's vocabulary is the committed one, which it records.

    Off, none is recorded, and one given is refused before anything runs.
    """
    on = vo.run("tum", SEQUENCES / "tum_camera", "rgbd", tmp_path / "on", count=0)
    assert on.vocabulary == vocabularies.record(vocabularies.DEFAULT)
    off_options = Options().with_settings({"relocalization.enabled": False})
    off = vo.run(
        "tum", SEQUENCES / "tum_camera", "rgbd", tmp_path / "off", count=0, options=off_options
    )
    assert off.vocabulary is None
    with pytest.raises(InvalidInputError, match="given, but relocalize is off"):
        vo.run(
            "tum", SEQUENCES / "tum_camera", "rgbd", tmp_path / "refused", count=0,
            options=off_options, vocabulary=vocabularies.DEFAULT,
        )  # fmt: skip
    assert not (tmp_path / "refused").exists()


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("more", "message"),
    [
        (["--vocabulary", "a", "--vocabulary", "b"], "--vocabulary: given twice"),
        (
            ["--option", "relocalization.enabled=maybe"],
            "relocalization.enabled: expected true or false, got 'maybe'",
        ),
    ],
)
def test_every_program_refuses_a_vocabulary_twice_and_a_flag_not_true_or_false(
    backend: str, more: list[str], message: str
) -> None:
    """Every program's usage error, with Python's parse's message (review panel finding H10)."""
    base = ["--dataset", "tum", "--sequence", "seq", "--mode", "rgbd", "--out", "out"]
    program = vo.command(require_program(backend))
    done = subprocess.run([*program, *base, *more], capture_output=True, text=True, check=False)
    assert done.returncode == vo.EXIT_USAGE
    assert done.stderr.startswith(f"slam_it_vo: {message}")


def _foreign_vocabulary(folder: Path) -> Path:
    """Write a copy of the reader's fixture whose header names another descriptor."""
    cases = json.loads((FIXTURES / "eval" / "vocabulary_cases.json").read_text())["cases"]
    (case,) = [c for c in cases if c["name"] == "another descriptor's words"]
    shutil.copytree(FIXTURES / "eval" / "vocabulary", folder)
    for name, data in case["files"].items():
        (folder / name).write_bytes(bytes.fromhex(data))
    return folder


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("refused", ["none", "unasked", "foreign"])
def test_every_program_refuses_a_vocabulary_missing_unasked_or_foreign(
    backend: str, refused: str, tmp_path: Path
) -> None:
    """A vocabulary missing, unasked for or foreign: every program fails the run and writes none.

    Relocalising without a vocabulary, given one with relocalisation off, or given one of another
    descriptor, every program exits 3 and says why (review panel findings A1 and run 2's H17:
    before B1's fixes, the native programs read a foreign vocabulary and exited 0).
    """
    out = tmp_path / "run"
    request = vo.Request("tum", SEQUENCES / "tum_camera", "rgbd", out, count=0)
    message = "--vocabulary: relocalize needs one"
    if refused == "unasked":
        request = replace(
            request,
            options=Options().with_settings({"relocalization.enabled": False}),
            vocabulary=vocabularies.DEFAULT,
        )
        message = "--vocabulary: given, but relocalize is off"
    elif refused == "foreign":
        folder = _foreign_vocabulary(tmp_path / "foreign")
        request = replace(request, vocabulary=folder)
        message = f"{folder}: trained on descriptor version 99"
    program = vo.command(require_program(backend))
    done = subprocess.run(
        [*program, *request.arguments()], capture_output=True, text=True, check=False
    )
    assert done.returncode == vo.EXIT_FAILED
    assert done.stderr.startswith(f"slam_it_vo: {message}")
    assert not (out / "run.json").exists()


def test_a_run_set_up_for_its_frames_size_gets_their_features() -> None:
    """A 16:9 clip imported at 854x480 runs with 1,334 features; its other settings, ORB-SLAM's."""
    assert vo.options_for(854, 480) == Options().with_settings({"features.n_features": 1334})
    assert vo.options_for(320, 240) == Options()
