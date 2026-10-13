"""ZJU-SenseTime import: a synthetic sequence with the benchmark's sensor files; failures."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from slam_it.eval import images, runs, tum, zju
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError

# B0's camera and ground-truth sensor.yaml, as published (an iPhone X).
CAMERA = """%YAML 1.0
# sense-slam sensor.yaml
type: camera
description: iPhone X
frequency: 30  # [hz]
camera_model: pinhole
distortion_model: radtan

# intrinsic parameters
intrinsic:
  camera: [525.4229, 526.9468000000001, 326.4371, 239.5824]  # fx fy cx cy
  distortion: [0, 0, 0, 0]  # k1 k2 r1 r2

# sensor to body transform
extrinsic:
  q: [0.707107, -0.707107, 0, 0]  # x y z w
  p: [0.026, 0.017, 0]  # [m]
  t: 0  # [s]
"""

GROUND_TRUTH = """%YAML 1.0
# sense-slam sensor.yaml
type: groundtruth
description: ""
frequency: 400  # [hz]

# noise parameters
intrinsic:
  sigma_q: 0.015  # [rad]
  sigma_p: 0.001  # [m]

# sensor to body transform
extrinsic:
  q: [0, 0, 0, 1]  # x y z w
  p: [0, 0, 0]  # [m]
  t: 0  # [s]
"""

TIMES = (129876.844074916, 129876.877400916, 129876.910726916)
"""The frames' times, B0's first three."""


def _write(sequence: Path, camera: str = CAMERA, times: tuple[float, ...] = TIMES) -> None:
    """Write a three-frame sequence: images, the frames' and the ground truth's data.csv."""
    folder = sequence / "camera" / "images"
    folder.mkdir(parents=True)
    (sequence / "camera" / "sensor.yaml").write_text(camera)
    rows = ["#t[s:double],filename[string]"]
    for k, time in enumerate(times):
        name = f"{round(time * 1e9)}.png"
        images.write_png(folder / name, np.full((48, 64), 40 * k, dtype=np.uint8))
        rows.append(f"{time!r},{name}")
    (sequence / "camera" / "data.csv").write_text("\n".join(rows) + "\n")
    (sequence / "groundtruth").mkdir()
    (sequence / "groundtruth" / "sensor.yaml").write_text(GROUND_TRUTH)
    truth = ["#t[s:double],q.x,q.y,q.z,q.w,p.x,p.y,p.z"]
    for k, time in enumerate(times):
        z, w = float(np.sin(0.05 * k)), float(np.cos(0.05 * k))  # a turn about z
        truth.append(f"{time - 0.0004!r},0,0,{z!r},{w!r},{k * 0.1},0.2,1.3")
    (sequence / "groundtruth" / "data.csv").write_text("\n".join(truth) + "\n")


def test_a_sequence_imports_to_a_tum_folder_the_pipeline_reads(tmp_path: Path) -> None:
    """Each frame linked, its time to the microsecond; the camera; the camera's true pose.

    The ground truth is the body's pose; the camera's is it times the camera's extrinsic.
    """
    _write(tmp_path / "B0")
    zju.import_sequence(tmp_path / "B0", tmp_path / "tum")
    frames = tum.read_frames(tmp_path / "tum")
    assert [f"{t:.6f}" for t in frames.timestamps] == [f"{t:.6f}" for t in TIMES]
    assert (tmp_path / "tum" / "rgb" / f"{TIMES[1]:.6f}.png").is_symlink()
    np.testing.assert_array_equal(images.read_gray(frames.images[1]), np.full((48, 64), 40))
    camera = tum.read_camera(tmp_path / "tum")
    assert (camera.model, camera.width, camera.height) == ("pinhole", 64, 48)
    np.testing.assert_allclose(camera.intrinsics, [525.4229, 526.9468, 326.4371, 239.5824])
    stamps, poses = runs.read_trajectory(tmp_path / "tum" / "groundtruth.txt")
    body = np.eye(4)
    body[:3, :3] = so3.from_quaternion(np.array([0.0, 0.0, np.sin(0.1), np.cos(0.1)]))
    body[:3, 3] = [0.2, 0.2, 1.3]
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = so3.from_quaternion(
        np.array([0.707107, -0.707107, 0.0, 0.0]) / np.sqrt(2 * 0.707107**2)
    )
    extrinsic[:3, 3] = [0.026, 0.017, 0.0]
    np.testing.assert_allclose(stamps[2], TIMES[2] - 0.0004)
    np.testing.assert_allclose(poses[2], body @ extrinsic, atol=1e-12)


def test_a_distorted_camera_imports_as_radtan(tmp_path: Path) -> None:
    """Distortion that is not zero is kept: the camera is written radial-tangential."""
    _write(
        tmp_path / "A0",
        CAMERA.replace("distortion: [0, 0, 0, 0]", "distortion: [0.1, -0.2, 0.001, 0.002]"),
    )
    zju.import_sequence(tmp_path / "A0", tmp_path / "tum")
    camera = tum.read_camera(tmp_path / "tum")
    assert camera.model == "radtan"
    np.testing.assert_allclose(camera.intrinsics[4:], [0.1, -0.2, 0.001, 0.002, 0.0])


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (
            lambda s: (s / "camera" / "images" / f"{round(TIMES[1] * 1e9)}.png").unlink(),
            "image is missing",
        ),
        (
            lambda s: (s / "camera" / "sensor.yaml").write_text(
                CAMERA.replace("radtan", "equidistant")
            ),
            "radtan",
        ),
        (
            lambda s: (s / "camera" / "sensor.yaml").write_text(
                CAMERA.replace("q: [0.707107, -0.707107, 0, 0]", "q: [0, 0, 0, 0]")
            ),
            "non-zero q",
        ),
        (
            lambda s: (s / "groundtruth" / "data.csv").write_text("#t\n1.0,0,0,0,0,0,0,0\n"),
            "zero quaternion",
        ),
    ],
)
def test_a_malformed_sequence_is_refused(tmp_path: Path, change: object, message: str) -> None:
    """A missing image, another distortion model, a zero quaternion: refused by name."""
    _write(tmp_path / "B0")
    change(tmp_path / "B0")  # type: ignore[operator]
    with pytest.raises(InvalidInputError, match=message):
        zju.import_sequence(tmp_path / "B0", tmp_path / "tum")


@pytest.mark.parametrize("name", ["../../secret.png", "/etc/hostname", "sub/0.png", ".."])
def test_a_frame_named_outside_the_images_folder_is_refused(tmp_path: Path, name: str) -> None:
    """``data.csv`` names a file in the images folder, never a path out of it or into another."""
    _write(tmp_path / "B0")
    (tmp_path / "B0" / "camera" / "data.csv").write_text(f"#t,name\n{TIMES[0]},{name}\n")
    with pytest.raises(InvalidInputError, match="is not a file in images/"):
        zju.import_sequence(tmp_path / "B0", tmp_path / "tum")


def test_frames_out_of_order_are_refused(tmp_path: Path) -> None:
    """Times that do not increase, in the frames or the ground truth."""
    _write(tmp_path / "B0", times=(TIMES[0], TIMES[2], TIMES[1]))
    with pytest.raises(InvalidInputError, match="the times do not increase"):
        zju.import_sequence(tmp_path / "B0", tmp_path / "tum")


def test_a_folder_already_holding_files_is_refused(tmp_path: Path) -> None:
    """The import writes a new or empty folder, never over one."""
    _write(tmp_path / "B0")
    (tmp_path / "tum").mkdir()
    (tmp_path / "tum" / "rgb.txt").write_text("# someone's\n")
    with pytest.raises(InvalidInputError, match="new or empty"):
        zju.import_sequence(tmp_path / "B0", tmp_path / "tum")
