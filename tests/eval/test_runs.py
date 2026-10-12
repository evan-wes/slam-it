"""The run format: round trips, TUM's trajectory format, scoring, feature records, failures."""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it.edge.map import Removal
from slam_it.edge.pipeline import Fate
from slam_it.eval import runs, tum
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError


def _poses(n: int, seed: int = 0) -> NDArray[np.float64]:
    """Draw ``n`` random rigid transforms."""
    rng = np.random.default_rng(seed)
    out = np.tile(np.eye(4), (n, 1, 1))
    out[:, :3, :3] = so3.exp(rng.uniform(-2, 2, (n, 3)))
    out[:, :3, 3] = rng.uniform(-5, 5, (n, 3))
    return out


def _lives(count: int, frames: int, removed: dict[int, tuple[int, Removal]]) -> NDArray[np.int64]:
    """Return lives made one per frame in turn; those in `removed` end some frames later."""
    out = np.full((count, 3), -1, dtype=np.int64)
    out[:, 0] = np.arange(count) % frames
    for row, (later, how) in removed.items():
        out[row, 1:] = [min(out[row, 0] + later, frames - 1), int(how)]
    return out


def _run(n: int = 12) -> runs.Run:
    """Build a small run with keyframes, points and their lives, settings and diagnostics."""
    times = (1403715274.312143 + 0.05 * np.arange(n)).astype(np.float64)
    gone = {1: (2, Removal.CULLED), 4: (0, Removal.FUSED), 9: (5, Removal.WEAK)}
    return runs.Run(
        dataset="euroc",
        sequence="V1_01_easy",
        mode="stereo",
        backend="cpp",
        timestamps=times,
        poses=_poses(n),
        keyframe_timestamps=times[::4],
        keyframe_poses=_poses(n)[::4],
        points=np.random.default_rng(1).normal(size=(30, 3)),
        settings={"orb": {"n_features": 1000}, "window": 7},
        diagnostics=[{"timestamp": float(t), "tracked": 120 + k} for k, t in enumerate(times)],
        peak_memory=123_456_789,
        cpus=[4, 5, 6, 7],
        memory_cap=8 * 2**30,
        point_lives=_lives(30, n, gone),
        keyframe_lives=np.array([[0, -1, -1], [4, 9, 0], [8, -1, -1]], dtype=np.int64),
    )


def test_round_trip(tmp_path: Path) -> None:
    """Every field survives writing and reading; times and positions exactly."""
    run = _run()
    runs.write_run(tmp_path / "run", run)
    back = runs.read_run(tmp_path / "run")
    assert (back.dataset, back.sequence, back.mode, back.backend) == (
        "euroc",
        "V1_01_easy",
        "stereo",
        "cpp",
    )
    np.testing.assert_array_equal(back.timestamps, run.timestamps)
    np.testing.assert_array_equal(back.poses[:, :3, 3], run.poses[:, :3, 3])
    np.testing.assert_allclose(back.poses, run.poses, rtol=0, atol=1e-15)
    np.testing.assert_array_equal(back.keyframe_poses, run.keyframe_poses)
    np.testing.assert_array_equal(back.points, run.points)
    np.testing.assert_array_equal(back.point_lives, run.point_lives)
    np.testing.assert_array_equal(back.keyframe_lives, run.keyframe_lives)
    assert back.settings == run.settings
    assert back.diagnostics == run.diagnostics
    assert (back.peak_memory, back.cpus, back.memory_cap) == (
        run.peak_memory,
        run.cpus,
        run.memory_cap,
    )


def test_the_final_map_is_the_rows_still_alive() -> None:
    """Points and keyframes whose lives did not end, in id order."""
    run = _run()
    np.testing.assert_array_equal(run.final_points(), np.delete(run.points, [1, 4, 9], axis=0))
    times, poses = run.final_keyframes()
    np.testing.assert_array_equal(times, run.keyframe_timestamps[[0, 2]])
    np.testing.assert_array_equal(poses, run.keyframe_poses[[0, 2]])


def test_the_removal_codes_are_the_maps() -> None:
    """A life's removal code is the edge map's :class:`Removal`, by the same names."""
    assert [r.name.lower() for r in Removal] == list(runs.REMOVALS)
    assert [int(r) for r in Removal] == list(range(len(runs.REMOVALS)))


def test_the_fate_codes_are_the_pipelines() -> None:
    """A feature record's fate code is the pipeline's :class:`Fate`, by the same names."""
    assert [f.name.lower() for f in Fate] == list(runs.FATES)
    assert [int(f) for f in Fate] == list(range(len(runs.FATES)))


def _record_features(folder: Path) -> None:
    """Record three frames of keypoints: every fate, and a frame with none."""
    none = np.zeros(0, dtype=np.int64)
    with runs.FeatureWriter(folder) as writer:
        writer.write([[1.5, 2.25], [3.0, 4.1]], [0, 2], [Fate.MADE, Fate.UNMATCHED], [4, -1])
        writer.write(np.zeros((0, 2)), none, none, none)
        writer.write([[5.0, 6.0], [7.0, 8.0]], [1, 1], [Fate.TRACKED, Fate.REJECTED], [4, 29])


def test_feature_records_round_trip(tmp_path: Path) -> None:
    """Each frame's keypoints come back in frame order: pixels as float32, records as int32."""
    run = replace(_run(), record_features=True)
    runs.write_run(tmp_path, run)
    _record_features(tmp_path)
    back = runs.read_run(tmp_path)
    assert back.record_features
    features = runs.read_features(tmp_path, back)
    assert features.pixels.dtype == np.dtype(np.float32)
    assert features.records.dtype == np.dtype(np.int32)
    expected = [[1.5, 2.25], [3.0, 4.1], [5.0, 6.0], [7.0, 8.0]]
    np.testing.assert_array_equal(features.pixels, np.array(expected, dtype=np.float32))
    assert features.records.tolist() == [[0, 0, 3, 4], [0, 2, 0, -1], [2, 1, 2, 4], [2, 1, 1, 29]]


def test_feature_files_count_their_rows_after_each_frame(tmp_path: Path) -> None:
    """Mid-run, each file reads whole up to the last frame written."""
    with runs.FeatureWriter(tmp_path) as writer:
        assert np.load(tmp_path / runs.RECORDS).shape == (0, 4)
        writer.write([[1.0, 2.0]], [0], [Fate.UNMATCHED], [-1])
        assert np.load(tmp_path / runs.PIXELS).shape == (1, 2)
        assert np.load(tmp_path / runs.RECORDS).tolist() == [[0, 0, 0, -1]]


@pytest.mark.parametrize(
    ("columns", "message"),
    [
        (([[1.0, 2.0, 3.0]], [0], [0], [-1]), "expected xy"),
        (([[1.0, 2.0]], [0, 1], [0], [-1]), "expected xy"),
        (([[1.0, 2.0]], [0.0], [0], [-1]), "must be integers"),
        (([[np.nan, 2.0]], [0], [0], [-1]), "pixels must be finite"),
        (([[1.0, 2.0]], [-1], [0], [-1]), "a level must be at least 0"),
        (([[1.0, 2.0]], [0], [4], [3]), "a fate must be a code of FATES"),
        (([[1.0, 2.0]], [0], [Fate.TRACKED], [-1]), "-1 exactly when unmatched"),
        (([[1.0, 2.0]], [0], [Fate.UNMATCHED], [3]), "-1 exactly when unmatched"),
        (([[1.0, 2.0]], [0], [Fate.MADE], [-2]), "-1 exactly when unmatched"),
    ],
)
def test_the_feature_writer_refuses_malformed_records(
    tmp_path: Path, columns: tuple[Any, ...], message: str
) -> None:
    """A malformed frame raises InvalidInputError and writes nothing."""
    with runs.FeatureWriter(tmp_path) as writer:
        writer.write([[1.0, 2.0]], [0], [Fate.UNMATCHED], [-1])
        before = (tmp_path / runs.RECORDS).read_bytes()
        with pytest.raises(InvalidInputError, match=f"features of frame 1: .*{message}"):
            writer.write(*columns)
        assert (tmp_path / runs.RECORDS).read_bytes() == before


def _rewrite(folder: Path, records: list[list[int]]) -> None:
    """Replace the feature records with these rows (their pixels to match)."""
    np.save(folder / runs.RECORDS, np.array(records, dtype=np.int32).reshape(-1, 4))
    np.save(folder / runs.PIXELS, np.zeros((len(records), 2), dtype=np.float32))


@pytest.mark.parametrize(
    ("damage", "error", "message"),
    [
        (lambda f: (f / runs.PIXELS).unlink(), FileNotFoundError, "feature_pixels.npy"),
        (
            lambda f: np.save(f / runs.PIXELS, np.zeros((3, 2), dtype=np.float32)),
            InvalidInputError,
            "4 records for 3 pixels",
        ),
        (
            lambda f: np.save(f / runs.RECORDS, np.zeros((4, 4), dtype=np.int64)),
            InvalidInputError,
            r"expected int32 \(N, 4\)",
        ),
        (lambda f: _rewrite(f, [[2, 0, 0, -1], [1, 0, 0, -1]]), InvalidInputError, "ascend"),
        (lambda f: _rewrite(f, [[12, 0, 0, -1]]), InvalidInputError, "ascend"),
        (lambda f: _rewrite(f, [[0, 0, 2, 30]]), InvalidInputError, "one the run made"),
        (lambda f: _rewrite(f, [[0, 0, 2, -1]]), InvalidInputError, "-1 exactly when unmatched"),
    ],
)
def test_read_features_refuses_malformed_records(
    tmp_path: Path, damage: Callable[[Path], object], error: type[Exception], message: str
) -> None:
    """Missing or disagreeing files, frames out of order or not the run's, unknown points."""
    run = replace(_run(), record_features=True)
    runs.write_run(tmp_path, run)
    _record_features(tmp_path)
    damage(tmp_path)
    with pytest.raises(error, match=message):
        runs.read_features(tmp_path, runs.read_run(tmp_path))


def test_a_run_without_features_has_none_to_read(tmp_path: Path) -> None:
    """Feature files left from another run are not this run's."""
    _record_features(tmp_path)
    runs.write_run(tmp_path, _run())
    with pytest.raises(FileNotFoundError, match="the run recorded no features"):
        runs.read_features(tmp_path, runs.read_run(tmp_path))


def test_an_unknown_process_round_trips_as_none(tmp_path: Path) -> None:
    """A run that was not a program's own (an in-process run's) writes nulls and reads None."""
    runs.write_run(tmp_path, replace(_run(), peak_memory=None, cpus=None, memory_cap=None))
    info = json.loads((tmp_path / "run.json").read_text())
    assert (info["peak_memory"], info["cpus"], info["memory_cap"]) == (None, None, None)
    back = runs.read_run(tmp_path)
    assert (back.peak_memory, back.cpus, back.memory_cap) == (None, None, None)


def test_reads_a_tum_trajectory(tmp_path: Path) -> None:
    """A trajectory as TUM's tools write it (comments, four decimals) reads as poses."""
    path = tmp_path / "groundtruth.txt"
    path.write_text(
        "# ground truth trajectory\n# timestamp tx ty tz qx qy qz qw\n"
        "1305031098.6659 1.3563 0.6305 1.6380 0.6132 0.5962 -0.3311 -0.3986\n"
        "1305031098.6758 1.3543 0.6326 1.6371 0.6129 0.5966 -0.3316 -0.3980\n"
    )
    times, poses = runs.read_trajectory(path)
    np.testing.assert_array_equal(times, [1305031098.6659, 1305031098.6758])
    np.testing.assert_array_equal(poses[0, :3, 3], [1.3563, 0.6305, 1.6380])
    np.testing.assert_allclose(
        poses[:, :3, :3] @ poses[:, :3, :3].transpose(0, 2, 1),
        np.tile(np.eye(3), (2, 1, 1)),
        atol=1e-12,
    )


def test_evaluate_scores_a_noisy_run() -> None:
    """A run with centimetre noise scores a centimetre-level ATE on its matched frames."""
    truth_times = np.arange(0.0, 10.0, 0.005)
    angles = np.column_stack([0.1 * truth_times, 0.3 * truth_times, np.zeros_like(truth_times)])
    truth = np.tile(np.eye(4), (len(truth_times), 1, 1))
    truth[:, :3, :3] = so3.exp(angles)
    truth[:, :3, 3] = np.column_stack([np.cos(truth_times), np.sin(truth_times), 0.1 * truth_times])
    frames = np.arange(0, len(truth_times), 10)
    estimate = truth[frames].copy()
    estimate[:, :3, 3] += 0.01 * np.random.default_rng(2).normal(size=(len(frames), 3))
    run = runs.Run("synthetic", "circle", "rgbd", "python", truth_times[frames] + 0.001, estimate)
    scored = runs.evaluate(run, truth_times, truth)
    assert scored.matched == len(frames)
    assert 0.01 < scored.absolute.rmse < 0.025
    assert scored.absolute.scale == 1.0
    assert scored.relative.delta == pytest.approx(0.05)  # the run's frame period
    assert len(scored.relative.pairs) == len(frames) - 1


def test_evaluate_rejects_a_run_whose_times_do_not_ascend() -> None:
    """The run's times are checked before they are paired with the ground truth."""
    times = np.arange(0.0, 1.0, 0.05)
    poses = np.tile(np.eye(4), (len(times), 1, 1))
    run = runs.Run("synthetic", "still", "rgbd", "python", times[::-1], poses)
    with pytest.raises(InvalidInputError, match="run timestamps: must ascend"):
        runs.evaluate(run, times, poses)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"timestamps": np.arange(12.0)[::-1]}, "trajectory timestamps: must ascend"),
        ({"settings": {"seed": np.int64(3)}}, "settings: not JSON"),
        ({"diagnostics": [{"tracked": 3}]}, r"diagnostics\[0\]: expected an object"),
        ({"diagnostics": [{"timestamp": float("nan")}]}, r"diagnostics\[0\]: not JSON"),
        ({"dataset": 3}, "dataset: expected a string"),
        ({"record_features": 1}, "record_features: expected a bool"),
        ({"peak_memory": -1}, "peak_memory: expected a whole number of bytes"),
        ({"peak_memory": 1.5e9}, "peak_memory: expected a whole number of bytes"),
        ({"peak_memory": True}, "peak_memory: expected a whole number of bytes"),
        ({"cpus": []}, "cpus: expected CPU numbers, ascending"),
        ({"cpus": [5, 4]}, "cpus: expected CPU numbers, ascending"),
        ({"cpus": [4, 4]}, "cpus: expected CPU numbers, ascending"),
        ({"cpus": [-1, 4]}, "cpus: expected CPU numbers, ascending"),
        ({"cpus": (4, 5)}, "cpus: expected CPU numbers, ascending"),
        ({"memory_cap": 0}, "memory_cap: expected a positive number of bytes"),
        ({"memory_cap": 2.0**30}, "memory_cap: expected a positive number of bytes"),
        (
            {"point_lives": np.zeros((29, 3), dtype=np.int64)},
            r"point_lives: expected int64 \(30, 3\)",
        ),
        ({"point_lives": np.zeros((30, 3))}, r"point_lives: expected int64 \(30, 3\)"),
        (
            {"keyframe_lives": np.array([[0, -1, -1], [4, 3, 0], [8, -1, -1]], dtype=np.int64)},
            "keyframe_lives: a life must end in its start's frame or later",
        ),
        (
            {"keyframe_lives": np.array([[0, -1, -1], [4, -1, 0], [8, -1, -1]], dtype=np.int64)},
            "keyframe_lives: why a life ended must be a removal code",
        ),
        (
            {"keyframe_lives": np.array([[0, -1, -1], [4, 9, 3], [8, -1, -1]], dtype=np.int64)},
            "keyframe_lives: why a life ended must be a removal code",
        ),
        (
            {"keyframe_lives": np.array([[0, -1, -1], [4, -1, -1], [12, -1, -1]], dtype=np.int64)},
            "keyframe_lives: a life must start in one of the run's 12 frames",
        ),
    ],
)
def test_write_run_checks_everything_before_writing(
    tmp_path: Path, change: dict[str, Any], message: str
) -> None:
    """A bad run raises InvalidInputError and leaves no folder behind."""
    with pytest.raises(InvalidInputError, match=message):
        runs.write_run(tmp_path / "run", replace(_run(), **change))
    assert not (tmp_path / "run").exists()


def test_write_run_replaces_files_whole(tmp_path: Path) -> None:
    """Writing again replaces every file and leaves no temporary file."""
    runs.write_run(tmp_path, _run())
    runs.write_run(tmp_path, replace(_run(), sequence="V1_02_medium"))
    assert runs.read_run(tmp_path).sequence == "V1_02_medium"
    assert sorted(p.name for p in tmp_path.iterdir()) == [
        "diagnostics.jsonl",
        "keyframe_lives.npy",
        "keyframe_poses.npy",
        "keyframes.npy",
        "point_lives.npy",
        "points.npy",
        "run.json",
        "trajectory.txt",
    ]


@pytest.mark.parametrize(
    ("edit", "message"),
    [
        (lambda info: info.update(version=2), "version 3"),
        (lambda info: info.update(format="other"), "format"),
        (lambda info: info.update(mode="lidar"), "mode"),
        (lambda info: info.update(backend="java"), "backend"),
        (lambda info: info.pop("mode"), "missing mode"),
        (lambda info: info.update(sequence=7), "sequence must be a string"),
        (lambda info: info.update(settings=[1]), "settings must be an object"),
        (lambda info: info.pop("record_features"), "missing record_features"),
        (lambda info: info.update(record_features=1), "record_features must be true or false"),
        (lambda info: info.pop("peak_memory"), "missing peak_memory"),
        (lambda info: info.update(peak_memory="1 GB"), "peak_memory: expected a whole number"),
        (lambda info: info.pop("cpus"), "missing cpus"),
        (lambda info: info.update(cpus=[7, 4]), "cpus: expected CPU numbers"),
        (lambda info: info.pop("memory_cap"), "missing memory_cap"),
        (lambda info: info.update(memory_cap=-1), "memory_cap: expected a positive number"),
    ],
)
def test_read_run_rejects_unknown_info(tmp_path: Path, edit: object, message: str) -> None:
    """A run.json of another format, version, mode or backend, or missing a key, is refused."""
    runs.write_run(tmp_path, _run())
    info = json.loads((tmp_path / "run.json").read_text())
    assert callable(edit)
    edit(info)
    (tmp_path / "run.json").write_text(json.dumps(info))
    with pytest.raises(InvalidInputError, match=message):
        runs.read_run(tmp_path)


@pytest.mark.parametrize(
    ("name", "text", "message"),
    [
        ("run.json", "{not json", "run.json: not JSON"),
        ("run.json", "[]", "expected a JSON object"),
        ("diagnostics.jsonl", '{"timestamp": 1}\n{oops\n', "diagnostics.jsonl:2: not JSON"),
        ("diagnostics.jsonl", "[1]\n", "expected an object with a timestamp"),
        ("points.npy", "not an array", "points.npy: not a readable .npy array"),
        ("points.npy", "", "points.npy: not a readable .npy array"),
        ("run.json", "\udcff", "run.json: not UTF-8 text"),
        ("trajectory.txt", "\udcff", "trajectory.txt: not UTF-8 text"),
        ("diagnostics.jsonl", "\udcff", "diagnostics.jsonl: not UTF-8 text"),
    ],
)
def test_read_run_rejects_malformed_files(
    tmp_path: Path, name: str, text: str, message: str
) -> None:
    """Files that do not parse are InvalidInputError, named with their line where they have one."""
    runs.write_run(tmp_path, _run())
    (tmp_path / name).write_bytes(text.encode("utf-8", "surrogateescape"))  # \udcff: byte 0xff
    with pytest.raises(InvalidInputError, match=message):
        runs.read_run(tmp_path)


def test_read_run_names_the_expected_shape(tmp_path: Path) -> None:
    """A mis-shaped array's message reads (N, 4, 4), not a nested tuple."""
    runs.write_run(tmp_path, _run())
    np.save(tmp_path / "keyframe_poses.npy", np.zeros((3, 4)))
    with pytest.raises(InvalidInputError, match=r"expected float64 \(N, 4, 4\)"):
        runs.read_run(tmp_path)


def test_read_run_checks_the_lives(tmp_path: Path) -> None:
    """Lives of another dtype, or one beyond the run's frames, are refused."""
    runs.write_run(tmp_path, _run())
    np.save(tmp_path / "point_lives.npy", np.zeros((30, 3)))
    with pytest.raises(InvalidInputError, match=r"expected int64 \(N, 3\)"):
        runs.read_run(tmp_path)
    lives = _run().point_lives.copy()
    lives[0, 0] = 99
    np.save(tmp_path / "point_lives.npy", lives)
    with pytest.raises(InvalidInputError, match="a life must start in one of the run's 12 frames"):
        runs.read_run(tmp_path)


def test_read_run_reports_a_missing_file(tmp_path: Path) -> None:
    """A missing array file is FileNotFoundError, as a missing trajectory is."""
    runs.write_run(tmp_path, _run())
    (tmp_path / "points.npy").unlink()
    with pytest.raises(FileNotFoundError, match=r"points\.npy"):
        runs.read_run(tmp_path)


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("1.0 0 0 0 0 0 0 1\n0.5 0 0 0 0 0 0 1\n", "ascend"),
        ("1.0 0 0 0 0 0 0\n", "expected 8 finite numbers"),
        ("1.0 0 0 x 0 0 0 1\n", "expected numbers"),
        ("1.0 0 0 0 0 0 0 0\n", "quaternion is zero"),
    ],
)
def test_read_trajectory_rejects(tmp_path: Path, text: str, message: str) -> None:
    """Unordered times, short lines, non-numbers and zero quaternions are refused."""
    (tmp_path / "t.txt").write_text(text)
    with pytest.raises(InvalidInputError, match=message):
        runs.read_trajectory(tmp_path / "t.txt")


def test_ground_truth_is_the_readers_and_none_without_one(tmp_path: Path) -> None:
    """A sequence's ground truth is what its dataset's reader gives; one without has none."""
    folder, bare = tmp_path / "rgbd_dataset_freiburg1_xyz", tmp_path / "rgbd_dataset_freiburg1_360"
    for sequence in (folder, bare):
        sequence.mkdir()
        (sequence / "rgb.txt").write_text("100.000 rgb/100.000.png\n100.500 rgb/100.500.png\n")
    (folder / "groundtruth.txt").write_text("100.000 1 2 3 0 0 0 1\n100.500 1.5 2 3 0 0 0 1\n")
    truth = runs.ground_truth("tum", folder)
    assert truth is not None
    read = tum.read_sequence(folder)
    np.testing.assert_array_equal(truth[0], read.timestamps)
    np.testing.assert_array_equal(truth[1], read.T_world_cam)
    assert runs.ground_truth("tum", bare) is None
    with pytest.raises(InvalidInputError, match="expected one of"):
        runs.ground_truth("kitti", folder)
