"""The run viewer's bundles: the golden layout, alignment, frames, images, and refusals."""

from __future__ import annotations

import json
import shutil
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from slam_it import backends
from slam_it.eval import images, runs
from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import bundles, vo
from tests.conftest import FIXTURES
from tests.edge.support import FRAMES, OPTIONS
from tests.fixtures.orchestration.generate_bundle import found_run, short_runs
from tests.orchestration.conftest import RoomRun, require_program
from tests.orchestration.datasets import Room

GOLDEN = FIXTURES / "orchestration" / "bundle"
"""The golden bundle (``generate_bundle.py``), whose values the page's tests read."""

FOUND = FIXTURES / "orchestration" / "found_bundle"
"""The found bundle (``generate_bundle.py``): a relocalised frame, as the page's tests read it."""


def _keys(value: Any) -> Any:  # noqa: ANN401 -- a JSON value
    """Return a JSON value's keys, nested: an object's as a dict, anything else's type's name."""
    if isinstance(value, dict):
        return {key: _keys(item) for key, item in value.items()}
    return type(value).__name__


def _layout(bundle: Path) -> dict[str, Any]:
    """Return a bundle's layout: every file, an array's dtype and trailing shape, JSON's keys."""
    out: dict[str, Any] = {}
    for path in sorted(p for p in bundle.rglob("*") if p.is_file()):
        name = path.relative_to(bundle).as_posix()
        if path.suffix == ".npy":
            array = np.load(path, allow_pickle=False)
            out[name] = (array.dtype.str, array.shape[1:])
        elif path.suffix == ".json":
            out[name] = _keys(json.loads(path.read_text()))
        else:
            out[name] = "bytes"
    return out


def _bake(runs_of: RoomRun, dataset: str, mode: str, out: Path) -> dict[str, Any]:
    """Bake a room case's runs, every backend's; return ``bundle.json``."""
    folders = [runs_of.folder(dataset, mode, backend) for backend in backends.BACKENDS]
    bundles.bake(folders, runs_of.sequence(dataset), out)
    info: dict[str, Any] = json.loads((out / "bundle.json").read_text())
    return info


def _bearings(poses: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Return each point's unit direction in each camera, ``(K, N, 3)``, from ``T_world_cam``.

    A similarity moving cameras and points together leaves them as they were.
    """
    rotations, centres = poses[:, :3, :3], poses[:, :3, 3]
    seen = np.einsum("kji,knj->kni", rotations, points[None] - centres[:, None])
    out: np.ndarray = seen / np.linalg.norm(seen, axis=2, keepdims=True)
    return out


def _frame_jpeg(bundle: Path, k: int) -> bytes:
    """Return frame ``k``'s JPEG, cut from its chunk."""
    start, end = np.load(bundle / "images" / "spans.npy")[k]
    chunk = (bundle / "images" / f"chunk_{k // bundles.CHUNK:04d}.bin").read_bytes()
    return chunk[start:end]


@pytest.fixture(scope="session")
def golden_runs(room_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> list[Path]:
    """Run the room as the golden bundle was run: briefly, with every backend."""
    for backend in backends.BACKENDS:
        require_program(backend)
    return short_runs(room_folders.tum, tmp_path_factory.mktemp("short_runs"))


def test_the_bake_keeps_the_golden_bundles_layout(
    golden_runs: list[Path], room_folders: Room, tmp_path: Path
) -> None:
    """The golden bundle's files, each array of its dtype and trailing shape, each JSON's keys.

    So the page's tests, which read the golden bundle's values, read what the bake writes.
    """
    bundles.bake(golden_runs, room_folders.tum, tmp_path / "bundle")
    assert _layout(tmp_path / "bundle") == _layout(GOLDEN)
    # The tables the page checks a bundle against: the golden's are the run format's.
    golden = json.loads((GOLDEN / "bundle.json").read_text())
    assert (golden["fates"], golden["steps"]) == (list(runs.FATES), list(runs.STEPS))


def test_the_bake_keeps_the_found_bundles_layout(found_folders: Room, tmp_path: Path) -> None:
    """The found bundle's files, as the bake writes them; its frame 34 relocalised in map 0.

    So the page's tests of a relocalised frame read what the bake writes (review panel finding
    A6).
    """
    bundles.bake([found_run(found_folders.tum, tmp_path / "runs")], found_folders.tum,
                 tmp_path / "bundle")  # fmt: skip
    assert _layout(tmp_path / "bundle") == _layout(FOUND)
    records = json.loads((FOUND / "python" / "records.json").read_text())
    placed = [
        (frame, records["map"][k])
        for k, frame in enumerate(records["frame"])
        if records["placed_by"][k] == runs.RELOCALIZED
    ]
    assert placed == [(34, 0)]


@pytest.mark.parametrize(("mode", "alignment"), [("rgbd", "se3"), ("monocular", "sim3")])
def test_each_backend_is_moved_onto_the_truth(
    mode: str, alignment: str, room_runs: RoomRun, tmp_path: Path
) -> None:
    """With ground truth, each run's centres sit on it by its own ATE; its poses stay rigid.

    Its keyframes and points move with it: each point is seen from each keyframe as in the run.
    """
    info = _bake(room_runs, "tum", mode, tmp_path)
    assert info["truth"] is True
    truth = np.load(tmp_path / "truth.npy")
    assert truth.shape == (FRAMES, 3)
    assert not np.any(np.isnan(truth))
    for backend, entry in info["backends"].items():
        poses = np.load(tmp_path / backend / "poses.npy")
        at = np.load(tmp_path / backend / "pose_frames.npy")
        gap = np.linalg.norm(poses[:, :, 3] - truth[at], axis=1)
        assert np.sqrt(np.mean(gap**2)) == pytest.approx(entry["score"]["ate_rmse"], rel=1e-4)
        assert entry["alignment"] == alignment
        keyframes = np.load(tmp_path / backend / "keyframe_poses.npy").astype(np.float64)
        rotations = keyframes[:, :, :3]
        identities = np.broadcast_to(np.eye(3), rotations.shape)
        np.testing.assert_allclose(rotations @ rotations.transpose(0, 2, 1), identities, atol=1e-5)
        run = room_runs("tum", mode, backend)
        points = np.load(tmp_path / backend / "points.npy").astype(np.float64)
        np.testing.assert_allclose(
            _bearings(keyframes, points), _bearings(run.keyframe_poses, run.points), atol=1e-5
        )
        assert entry["frames"] == [0, FRAMES]


def _split(run: runs.Run, at: int, moved: np.ndarray) -> runs.Run:
    """Return the run as two maps, the second from frame `at` on, moved by a rigid `moved`.

    Its poses, keyframes and points made from frame `at` are map 1's, in a frame of its own, as
    the next map after a lost track is.
    """
    pose_frames = np.searchsorted([d["timestamp"] for d in run.diagnostics], run.timestamps)
    pose_maps = (pose_frames >= at).astype(np.int64)
    keyframe_maps = (run.keyframe_lives[:, 0] >= at).astype(np.int64)
    point_maps = (run.point_lives[:, 0] >= at).astype(np.int64)
    poses, keyframes, points = run.poses.copy(), run.keyframe_poses.copy(), run.points.copy()
    poses[pose_maps == 1] = moved @ poses[pose_maps == 1]
    keyframes[keyframe_maps == 1] = moved @ keyframes[keyframe_maps == 1]
    points[point_maps == 1] = points[point_maps == 1] @ moved[:3, :3].T + moved[:3, 3]
    diagnostics = [{**d, "map": int(k >= at)} for k, d in enumerate(run.diagnostics)]
    return replace(
        run, poses=poses, keyframe_poses=keyframes, points=points, diagnostics=diagnostics,
        pose_maps=pose_maps, keyframe_maps=keyframe_maps, point_maps=point_maps,
        record_features=False,
    )  # fmt: skip


def test_each_map_is_moved_onto_the_truth_by_its_own_alignment(
    room_runs: RoomRun, tmp_path: Path
) -> None:
    """A run of two maps, the second in a frame of its own: each map sits on the truth.

    Each is moved by its own alignment, its keyframes and points with it (so each point is seen
    from each keyframe of its map as in the run; across maps, two alignments of the centres
    alone need not agree on a short map's turn about its path); the header lists both maps,
    with their frames, tracked frames and scores, and pools the ATE over them.
    """
    run = room_runs("tum", "rgbd", "python")
    moved = np.eye(4)
    moved[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    moved[:3, 3] = [5.0, -3.0, 1.0]
    folder = tmp_path / "two"
    runs.write_run(folder, _split(run, FRAMES // 2, moved))
    bundles.bake([folder], room_runs.sequence("tum"), tmp_path / "bundle")
    info = json.loads((tmp_path / "bundle" / "bundle.json").read_text())["backends"]["python"]
    truth = np.load(tmp_path / "bundle" / "truth.npy")
    poses = np.load(tmp_path / "bundle" / "python" / "poses.npy")
    at = np.load(tmp_path / "bundle" / "python" / "pose_frames.npy")
    maps = np.load(tmp_path / "bundle" / "python" / "pose_maps.npy")
    assert maps.dtype == np.int32
    gap = np.linalg.norm(poses[:, :, 3] - truth[at], axis=1)
    assert gap.max() < 0.05  # both maps on the truth: one alignment for both would be metres off
    assert [m["id"] for m in info["maps"]] == [0, 1]
    first, second = info["maps"]
    assert (first["frames"], second["frames"]) == ([0, FRAMES // 2], [FRAMES // 2, FRAMES])
    assert first["tracked"] + second["tracked"] == len(run.timestamps) == info["tracked"]
    assert not first["reset"]
    assert not second["reset"]
    for entry, part in ((first, maps == 0), (second, maps == 1)):
        assert entry["score"]["ate_rmse"] == pytest.approx(
            np.sqrt(np.mean(gap[part] ** 2)), rel=1e-4
        )
    assert info["score"]["ate_rmse"] == pytest.approx(np.sqrt(np.mean(gap**2)), rel=1e-4)
    keyframes = np.load(tmp_path / "bundle" / "python" / "keyframe_poses.npy").astype(np.float64)
    points = np.load(tmp_path / "bundle" / "python" / "points.npy").astype(np.float64)
    split = _split(run, FRAMES // 2, moved)
    for map_id in (0, 1):
        k, p = split.keyframe_maps == map_id, split.point_maps == map_id
        np.testing.assert_allclose(
            _bearings(keyframes[k], points[p]),
            _bearings(run.keyframe_poses[k], run.points[p]),
            atol=1e-5,
        )


def test_a_later_run_is_renumbered_to_the_bundles_frames(
    room_runs: RoomRun, tmp_path: Path
) -> None:
    """A run from frame 5: its records, poses, lives and features name the bundle's frames."""
    later = tmp_path / "later"
    run = vo.run(
        "tum", room_runs.sequence("tum"), "rgbd", later, options=OPTIONS, first=5, count=10,
        record_features=True,
    )  # fmt: skip
    whole = room_runs.folder("tum", "rgbd", "cpp")
    bundles.bake([whole, later], room_runs.sequence("tum"), tmp_path / "bundle")
    info = json.loads((tmp_path / "bundle" / "bundle.json").read_text())
    assert (info["frames"], info["backends"]["python"]["frames"]) == (FRAMES, [5, 15])
    folder = tmp_path / "bundle" / "python"
    records = json.loads((folder / "records.json").read_text())
    assert records["frame"] == list(range(5, 15))
    assert records["state"] == [d["state"] for d in run.diagnostics]
    lives = np.load(folder / "point_lives.npy")
    np.testing.assert_array_equal(lives[:, 0], run.point_lives[:, 0] + 5)
    ended = run.point_lives[:, 1] >= 0
    np.testing.assert_array_equal(lives[ended, 1], run.point_lives[ended, 1] + 5)
    np.testing.assert_array_equal(lives[~ended, 1:], -1)
    features = runs.read_features(later, run)
    baked = np.load(folder / runs.RECORDS)
    np.testing.assert_array_equal(baked[:, 0], features.records[:, 0] + 5)
    np.testing.assert_array_equal(baked[:, 1:], features.records[:, 1:])
    np.testing.assert_array_equal(np.load(folder / runs.PIXELS), features.pixels)
    tracked = [k + 5 for k, d in enumerate(run.diagnostics) if d["tracked"] > 0]
    np.testing.assert_array_equal(np.load(folder / "pose_frames.npy"), tracked)


def test_the_extent_is_the_whole_sequences(room_runs: RoomRun, tmp_path: Path) -> None:
    """A run of a part of the sequence: the extent is the whole's, timed from the bundle's first.

    So the page can show what no run reached, before the run's first frame and after its last.
    """
    sequence = room_runs.sequence("tum")
    part = tmp_path / "part"
    vo.run("tum", sequence, "rgbd", part, options=OPTIONS, first=5, count=10)
    bundles.bake([part], sequence, tmp_path / "bundle")
    info = json.loads((tmp_path / "bundle" / "bundle.json").read_text())
    times = vo.open_source("tum", sequence, "rgbd").times
    assert (info["frames"], info["first_time"]) == (10, times[5])
    assert info["extent"] == {
        "frames": FRAMES,
        "start": times[0] - times[5],
        "end": times[FRAMES - 1] - times[5],
    }


@pytest.mark.parametrize(
    ("dataset", "mode", "colour"), [("tum", "rgbd", True), ("euroc", "stereo", False)]
)
def test_the_images_are_what_the_pipeline_saw(
    dataset: str, mode: str, colour: bool, room_runs: RoomRun, tmp_path: Path
) -> None:
    """Each frame's JPEG is the image the pipeline saw (a stereo pair's left one rectified).

    Its bytes are that image's encoding, cut whole from its chunk.
    """
    info = _bake(room_runs, dataset, mode, tmp_path)
    assert info["images"] == {"chunk": bundles.CHUNK, "quality": bundles.QUALITY, "colour": colour}
    source = vo.open_source(dataset, room_runs.sequence(dataset), mode)
    for k in (0, FRAMES // 2, FRAMES - 1):
        assert _frame_jpeg(tmp_path, k) == images.encode_jpeg(source.view(k), bundles.QUALITY)


def test_without_truth_each_run_stays_in_its_own_frame(room_runs: RoomRun, tmp_path: Path) -> None:
    """No ground truth: no alignment, no score, no truth file; the poses are the run's own."""
    info = _bake(room_runs, "euroc", "stereo", tmp_path)
    assert info["truth"] is False
    assert not (tmp_path / "truth.npy").exists()
    for backend, entry in info["backends"].items():
        assert (entry["alignment"], entry["score"]) == (None, None)
        assert [m["score"] for m in entry["maps"]] == [None]
        run = room_runs("euroc", "stereo", backend)
        poses = np.load(tmp_path / backend / "poses.npy")
        np.testing.assert_array_equal(poses, run.poses[:, :3].astype(np.float32))


def test_a_run_its_alignment_cannot_place_stays_in_its_own_frame(
    room_runs: RoomRun, tmp_path: Path
) -> None:
    """A monocular run that never initialised has no frame to align by.

    It is baked unscored in its own frame, beside a run that was aligned, rather than failing
    the bundle.
    """
    early = tmp_path / "early"
    run = vo.run(
        "tum", room_runs.sequence("tum"), "monocular", early, options=OPTIONS, count=5,
        record_features=True,
    )  # fmt: skip
    assert len(run.timestamps) == 0
    whole = room_runs.folder("tum", "monocular", require_program("cpp"))
    bundles.bake([whole, early], room_runs.sequence("tum"), tmp_path / "bundle")
    info = json.loads((tmp_path / "bundle" / "bundle.json").read_text())
    assert info["truth"] is True
    unplaced, placed = info["backends"]["python"], info["backends"]["cpp"]
    assert (unplaced["alignment"], unplaced["score"]) == (None, None)
    assert placed["alignment"] == "sim3"
    assert placed["score"] is not None
    points = np.load(tmp_path / "bundle" / "python" / "points.npy")
    np.testing.assert_array_equal(points, run.points.astype(np.float32))


def test_a_reference_too_sparse_for_the_rpe_still_places_the_run(
    room_runs: RoomRun, tmp_path: Path
) -> None:
    """With a pose for every third frame, no two matched frames are a period apart.

    The run is placed by its ATE alone, its RPE null, rather than left in its own frame (review
    panel finding A10, round two).
    """
    sequence = tmp_path / "sparse" / room_runs.sequence("tum").name
    shutil.copytree(room_runs.sequence("tum"), sequence)
    lines = (sequence / "groundtruth.txt").read_text().splitlines()
    kept = [line for line in lines if line.startswith("#")]
    kept += [line for line in lines if not line.startswith("#")][::3]
    (sequence / "groundtruth.txt").write_text("\n".join(kept) + "\n")
    folder = room_runs.folder("tum", "rgbd", "python")
    bundles.bake([folder], sequence, tmp_path / "bundle")
    entry = json.loads((tmp_path / "bundle" / "bundle.json").read_text())["backends"]["python"]
    assert entry["alignment"] == "se3"
    assert entry["score"]["matched"] == (FRAMES + 2) // 3
    assert (entry["score"]["rpe_translation_rmse"], entry["score"]["rpe_rotation_rmse"]) == (
        None,
        None,
    )
    assert entry["maps"][0]["score"]["ate_rmse"] == entry["score"]["ate_rmse"]


def test_a_run_on_a_line_stays_in_its_own_frame(room_runs: RoomRun, tmp_path: Path) -> None:
    """Centres on a line leave the alignment's rotation free: that run, too, is unscored."""
    run = room_runs("tum", "rgbd", "python")
    poses = run.poses.copy()
    poses[:, :3, 3] = np.arange(len(poses))[:, None] * [0.1, 0.0, 0.0]
    folder = tmp_path / "line"
    runs.write_run(folder, replace(run, poses=poses, record_features=False))
    bundles.bake([folder], room_runs.sequence("tum"), tmp_path / "bundle")
    entry = json.loads((tmp_path / "bundle" / "bundle.json").read_text())["backends"]["python"]
    assert (entry["alignment"], entry["score"]) == (None, None)


def test_a_bake_that_fails_leaves_nothing_behind(
    room_runs: RoomRun, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A bake that fails once writing has begun leaves no folder; baking again succeeds."""
    folder, sequence = room_runs.folder("tum", "rgbd", "python"), room_runs.sequence("tum")
    out = tmp_path / "bundles" / "room"

    def full(*_: object) -> None:
        raise OSError("no space left")

    monkeypatch.setattr(bundles, "_write_backend", full)
    with pytest.raises(OSError, match="no space left"):
        bundles.bake([folder], sequence, out)
    assert list((tmp_path / "bundles").iterdir()) == []
    monkeypatch.undo()
    out.mkdir()  # a new, empty folder is taken as the bundle's
    bundles.bake([folder], sequence, out)
    assert (out / "bundle.json").exists()
    assert [p.name for p in (tmp_path / "bundles").iterdir()] == ["room"]


def test_the_records_and_header_are_the_runs(room_runs: RoomRun, tmp_path: Path) -> None:
    """Each backend's columns are its diagnostics; its header counts what the run holds."""
    info = _bake(room_runs, "tum", "rgbd", tmp_path)
    for backend, entry in info["backends"].items():
        run = room_runs("tum", "rgbd", backend)
        records = json.loads((tmp_path / backend / "records.json").read_text())
        for key in run.diagnostics[0]:
            assert records[key] == [d[key] for d in run.diagnostics]
        assert (entry["tracked"], entry["keyframes"], entry["points"]) == (
            len(run.timestamps),
            len(run.final_keyframes()[0]),
            len(run.final_points()),
        )
        assert entry["made"] == {
            "keyframes": len(run.keyframe_timestamps),
            "points": len(run.points),
        }
        assert entry["features"] is True
        assert entry["latency"]["late"] >= 0


def test_the_bake_refuses_what_is_not_one_sequences_runs(
    room_runs: RoomRun, tmp_path: Path
) -> None:
    """Runs of two modes, two of one backend, another sequence's folder, a used folder, none."""
    rgbd = room_runs.folder("tum", "rgbd", "python")
    monocular = room_runs.folder("tum", "monocular", "python")
    tum, euroc = room_runs.sequence("tum"), room_runs.sequence("euroc")
    out = tmp_path / "bundle"
    cases: list[tuple[list[Path], Path, str]] = [
        ([rgbd, room_runs.folder("tum", "monocular", require_program("cpp"))], tum, "not of"),
        ([rgbd, rgbd], tum, "one run per backend"),
        ([monocular], euroc, "the runs are of room_tum"),
        ([], tum, "at least one run folder"),
    ]
    for folders, sequence, message in cases:
        with pytest.raises(InvalidInputError, match=message):
            bundles.bake(folders, sequence, out)
    assert not out.exists()
    out.mkdir()
    (out / "kept.txt").write_text("not ours")
    with pytest.raises(InvalidInputError, match="expected a new or empty folder"):
        bundles.bake([rgbd], tum, out)
    assert [p.name for p in out.iterdir()] == ["kept.txt"]


def test_main_bakes_and_says_what(
    room_runs: RoomRun, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The program bakes the runs given and prints the bundle's sequence, frames and backends."""
    folder = room_runs.folder("tum", "rgbd", "python")
    sequence = room_runs.sequence("tum")
    out = tmp_path / "bundle"
    assert bundles.main([str(folder), "--sequence", str(sequence), "--out", str(out)]) == 0
    assert capsys.readouterr().out == f"{out}: room_tum (rgbd), {FRAMES} frames, backends python\n"
