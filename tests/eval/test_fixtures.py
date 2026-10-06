"""The shared reader and run fixtures, held to the Python readers and writer.

The C++ and Rust tests hold their readers and run writers to ``sequences.json`` and the golden
run folder; these hold the Python side to them, so a fixture never drifts from what Python
reads and writes.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from slamlab.eval import euroc, runs, tum

FIXTURES = Path(__file__).parent.parent / "fixtures" / "eval"


def _load(name: str) -> Any:  # noqa: ANN401 -- the fixture's JSON
    """Load a fixture's JSON."""
    return json.loads((FIXTURES / name).read_text())


def _relative(path: Path | None, root: Path) -> str | None:
    """Return a path relative to its folder, or None."""
    return None if path is None else path.relative_to(root).as_posix()


def test_tum_folders_read_as_expected() -> None:
    """Every TUM folder's frames and camera are the expected ones."""
    for name, expected in _load("sequences.json")["cases"]["tum"].items():
        frames = tum.read_frames(FIXTURES / "sequences" / name)
        assert frames.timestamps.tolist() == expected["timestamps"]
        assert [_relative(p, frames.root) for p in frames.images] == expected["images"]
        assert [_relative(p, frames.root) for p in frames.depths] == expected["depths"]
        camera = expected["camera"]
        assert frames.camera.model == camera["model"]
        assert frames.camera.intrinsics.tolist() == camera["intrinsics"]
        assert (frames.camera.width, frames.camera.height) == (camera["width"], camera["height"])
        assert frames.camera.depth_scale == camera["depth_scale"]


def test_the_euroc_folder_reads_as_expected() -> None:
    """The EuRoC folder's frames and cameras are the expected ones."""
    expected = _load("sequences.json")["cases"]["euroc"]["euroc"]
    frames = euroc.read_frames(FIXTURES / "sequences" / "euroc")
    assert frames.timestamps.tolist() == expected["timestamps"]
    assert [_relative(p, frames.root) for p in frames.left] == expected["left"]
    assert [_relative(p, frames.root) for p in frames.right] == expected["right"]
    for camera, wanted in ((frames.left_camera, expected["left_camera"]),
                           (frames.right_camera, expected["right_camera"])):  # fmt: skip
        assert camera.intrinsics.tolist() == wanted["intrinsics"]
        assert camera.T_body_cam.tolist() == wanted["T_body_cam"]
        assert camera.shape == (wanted["rows"], wanted["cols"])


def test_the_golden_run_reads_back() -> None:
    """The golden folder holds run_values.json's values, as Python wrote them."""
    values = _load("run_values.json")
    run = runs.read_run(FIXTURES / "run")
    assert (run.dataset, run.sequence, run.mode, run.backend) == (
        values["dataset"],
        values["sequence"],
        values["mode"],
        "python",
    )
    assert run.timestamps.tolist() == values["timestamps"]
    np.testing.assert_allclose(run.poses, values["poses"], rtol=0, atol=1e-15)
    assert run.keyframe_timestamps.tolist() == values["keyframe_timestamps"]
    assert run.keyframe_poses.tolist() == values["keyframe_poses"]
    assert run.points.tolist() == values["points"]
    assert run.settings == values["settings"]
    assert run.diagnostics == values["diagnostics"]
