"""7-Scenes import: a synthetic sequence in 7-Scenes' layout; failures."""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
import pytest

from slam_it.eval import images, runs, sevenscenes, tum
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import vo

NUMBERS = (0, 1, 3)
"""The frames written: frame 2 missing from the numbering, as a gap would be."""


def _pose(k: int) -> np.ndarray:
    """Return frame `k`'s true pose: a turn about the vertical, a step along x."""
    pose = np.eye(4)
    pose[:3, :3] = so3.exp(np.array([0.0, 0.05 * k, 0.0]))
    pose[:3, 3] = [0.1 * k, -0.2, 1.5]
    return pose


def _printed(pose: np.ndarray) -> str:
    """Return a pose as 7-Scenes prints it: four rows, tab-separated, ``9.9798417e-001``."""

    def number(value: float) -> str:
        return re.sub(r"e([+-])(\d\d)$", r"e\g<1>0\2", f"{value:.7e}")

    return "".join("\t".join(f"  {number(v)}" for v in row) + "\t\n" for row in pose)


def _write(sequence: Path, size: tuple[int, int] = sevenscenes.SIZE) -> None:
    """Write a sequence: each frame's colour image, a depth file (never read) and its pose."""
    sequence.mkdir(parents=True)
    width, height = size
    for k in NUMBERS:
        stem = sequence / f"frame-{k:06d}"
        colour = np.zeros((height, width, 3), dtype=np.uint8)
        colour[..., 0] = 60 * k
        images.write_png(stem.with_name(stem.name + ".color.png"), colour)
        stem.with_name(stem.name + ".depth.png").write_bytes(b"not read")
        stem.with_name(stem.name + ".pose.txt").write_text(_printed(_pose(k)))


def test_a_sequence_imports_to_a_tum_folder_the_pipeline_reads(tmp_path: Path) -> None:
    """Each colour image linked, timed at 30 Hz by its number; the camera; the true poses.

    The printed poses' rotations come back projected onto SO(3); a monocular run reads the
    folder; the depth is left behind, and the import says what it read.
    """
    _write(tmp_path / "chess" / "seq-01")
    out = tmp_path / "imported" / "7scenes-chess-seq-01"
    sevenscenes.import_sequence(tmp_path / "chess" / "seq-01", out)
    frames = tum.read_frames(out)
    times = [k / sevenscenes.FPS for k in NUMBERS]
    assert [f"{t:.6f}" for t in frames.timestamps] == [f"{t:.6f}" for t in times]
    assert (out / "rgb" / f"{times[2]:.6f}.png").is_symlink()
    np.testing.assert_array_equal(images.read_gray(frames.images[0]), images.read_gray(
        tmp_path / "chess" / "seq-01" / "frame-000000.color.png"
    ))  # fmt: skip
    camera = tum.read_camera(out)
    assert (camera.model, camera.width, camera.height, camera.bf) == ("pinhole", 640, 480, None)
    np.testing.assert_array_equal(camera.intrinsics, sevenscenes.INTRINSICS)
    stamps, poses = runs.read_trajectory(out / "groundtruth.txt")
    np.testing.assert_array_equal(stamps, frames.timestamps)  # the frames' times exactly
    for pose, k in zip(poses, NUMBERS, strict=True):
        np.testing.assert_allclose(pose, _pose(k), atol=1e-6)
        np.testing.assert_allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3), atol=1e-12)
    assert not (out / "depth.txt").exists()
    source = json.loads((out / "source.json").read_text())
    assert source == {"kind": "7scenes", "sequence": "chess/seq-01", "depth": None}
    assert len(vo.open_source("tum", out, "monocular").times) == len(NUMBERS)


def test_the_program_imports_a_scenes_sequences(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``python -m slam_it.eval.sevenscenes SCENE OUT``: each sequence, in order, named for it."""
    for name in ("seq-02", "seq-01"):
        _write(tmp_path / "heads" / name)
    (tmp_path / "heads" / "TrainSplit.txt").write_text("sequence1\n")
    assert sevenscenes.main([str(tmp_path / "heads"), str(tmp_path / "out")]) == 0
    folders = [tmp_path / "out" / f"7scenes-heads-seq-0{k}" for k in (1, 2)]
    assert capsys.readouterr().out == "".join(f"{f}: 3 frames\n" for f in folders)
    assert json.loads((folders[1] / "source.json").read_text())["sequence"] == "heads/seq-02"
    with pytest.raises(InvalidInputError, match="no seq-XX folder"):
        sevenscenes.import_scene(tmp_path / "out", tmp_path / "again")


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (lambda s: (s / "frame-000001.pose.txt").write_text("1 0 0\n"), "16 finite numbers"),
        (lambda s: (s / "frame-000001.pose.txt").write_text("a " * 16), "a non-number"),
        (
            lambda s: (s / "frame-000001.pose.txt").write_text(_printed(np.diag([2.0, 1, 1, 1]))),
            "not a rigid transform",
        ),
        (
            lambda s: (s / "frame-000001.pose.txt").write_text(_printed(np.diag([1.0, 1, -1, 1]))),
            "not a rigid transform",
        ),
        (
            lambda s: (s / "frame-000001.pose.txt").write_text(
                _printed(np.vstack([np.eye(4)[:3], [0.0, 0.0, 1.0, 1.0]]))
            ),
            "not a rigid transform",
        ),
    ],
)
def test_a_malformed_pose_is_refused(tmp_path: Path, change: object, message: str) -> None:
    """A pose that is not 16 numbers, not numbers, scaled, mirrored or not affine: refused.

    Nothing is left where the import would have written.
    """
    _write(tmp_path / "seq-01")
    change(tmp_path / "seq-01")  # type: ignore[operator]
    with pytest.raises(InvalidInputError, match=message):
        sevenscenes.import_sequence(tmp_path / "seq-01", tmp_path / "outs" / "tum")
    assert not (tmp_path / "outs").exists() or list((tmp_path / "outs").iterdir()) == []


def test_a_missing_pose_is_refused(tmp_path: Path) -> None:
    """Every frame's pose is read: one missing fails the import."""
    _write(tmp_path / "seq-01")
    (tmp_path / "seq-01" / "frame-000003.pose.txt").unlink()
    with pytest.raises(FileNotFoundError, match=r"frame-000003\.pose\.txt"):
        sevenscenes.import_sequence(tmp_path / "seq-01", tmp_path / "tum")


def test_another_size_or_no_frames_is_refused(tmp_path: Path) -> None:
    """The intrinsics are for 640x480; a folder without colour images is no sequence."""
    _write(tmp_path / "small", size=(320, 240))
    with pytest.raises(InvalidInputError, match="320x240, where 7-Scenes' camera is 640x480"):
        sevenscenes.import_sequence(tmp_path / "small", tmp_path / "tum")
    (tmp_path / "empty").mkdir()
    with pytest.raises(InvalidInputError, match=r"no frame-XXXXXX\.color\.png"):
        sevenscenes.import_sequence(tmp_path / "empty", tmp_path / "tum")
