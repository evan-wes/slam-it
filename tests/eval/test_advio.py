"""ADVIO import: a synthetic sequence in ADVIO's layout, its calibration in Kalibr's; failures."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import numpy as np
import pytest

from slam_it.eval import advio, images, lenses, runs, tum
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError
from tests.edge import room
from tests.eval.recordings import ROOM_CAMERA, encode, rgb

FRAMES = 6
"""The video's frames, at ADVIO's 60 fps."""

DISTORTION = (0.0478, 0.0339, -0.00033, -0.00091)
"""ADVIO's ``iphone-02.yaml`` k1, k2, p1, p2."""

CAM_IMU = np.eye(4)
CAM_IMU[:3, :3] = np.diag([1.0, -1.0, -1.0]) @ so3.exp(np.array([0.003, -0.005, 0.004]))
CAM_IMU[:3, 3] = [-0.009, 0.0756, -0.0055]
"""A ``T_cam_imu`` as ADVIO's is: the IMU's axes are the camera's with y and z reversed."""


def _calibration(camera: lenses.Pinhole, cam_imu: np.ndarray) -> str:
    """Return a calibration file in Kalibr's format, as ADVIO publishes them."""
    rows = "\n".join(f"        - [{', '.join(repr(float(v)) for v in r)}]" for r in cam_imu)
    k1, k2, p1, p2 = DISTORTION
    return f"""label: "Iphone - original_calibration"
cameras:
- camera:
    label: cam0
    image_height: {camera.height}
    image_width: {camera.width}
    type: pinhole
    intrinsics:
      cols: 1
      rows: 4
      data: [{camera.fx}, {camera.fy},{camera.cx}, {camera.cy}]
    distortion:
      type: radial-tangential
      parameters:
        cols: 1
        rows: 4
        data: [{k1}, {k2}, {p1},
    {p2}]
    T_cam_imu:
        cols: 4
        rows: 4
        data:
{rows}
"""


def _write(root: Path, rendered: list[room.Frame], rotation: int) -> Path:
    """Write a sequence, ``advio-01``, and the calibrations' folder; return the sequence.

    The room's frames are stored on their side, the video declaring `rotation` to turn them
    upright; the calibration describes them upright. The ground truth is the IMU's pose at the
    frames' times, as ADVIO's is.
    """
    sequence = root / "advio-01"
    (sequence / "iphone").mkdir(parents=True)
    (sequence / "ground-truth").mkdir()
    turns = round(rotation / 90) % 4
    stored = [np.rot90(rgb(f.gray), -turns) for f in rendered[:FRAMES]]
    encode(sequence / "iphone" / "frames.mov", stored, rate=60, rotation=rotation)
    clock = [0.3 + i / 60 for i in range(FRAMES)]
    (sequence / "iphone" / "frames.csv").write_text(
        "".join(f"{t!r},{i + 1}\n" for i, t in enumerate(clock))
    )
    lines = []
    for t, f in zip(clock, rendered, strict=True):
        world_imu = f.T_world_cam @ CAM_IMU
        x, y, z, w = (float(v) for v in so3.to_quaternion(world_imu[:3, :3]))
        position = ",".join(repr(float(v)) for v in world_imu[:3, 3])
        lines.append(f"{t!r},{position},{w!r},{x!r},{y!r},{z!r}\n")
    (sequence / "ground-truth" / "pose.csv").write_text("".join(lines))
    calibrations = root / "calibration"
    calibrations.mkdir()
    (calibrations / "iphone-02.yaml").write_text(_calibration(ROOM_CAMERA, CAM_IMU))
    return sequence


@pytest.fixture(scope="module")
def rendered() -> list[room.Frame]:
    """Render the room."""
    return room.sequence(FRAMES)


@pytest.mark.parametrize("rotation", [-90, 90])
def test_a_sequence_imports_upright_timed_by_its_clock_with_the_cameras_truth(
    rotation: int, rendered: list[room.Frame], tmp_path: Path
) -> None:
    """Frames upright, kept distorted and timed by ``frames.csv``; the truth moved to the camera.

    ADVIO's iPhone video declares -90; at 60 fps every other frame is kept. The ground truth is
    the IMU's poses, ``T_world_cam = T_world_imu T_cam_imu^-1``.
    """
    sequence = _write(tmp_path, rendered, rotation)
    advio.import_sequence(sequence, tmp_path / "calibration", tmp_path / "tum")
    frames = tum.read_frames(tmp_path / "tum")
    np.testing.assert_allclose(
        frames.timestamps, [0.3 + i / 60 for i in range(0, FRAMES, 2)], atol=1e-6
    )  # rgb.txt's microseconds
    for file, frame in zip(frames.images, rendered[::2], strict=True):
        assert np.abs(images.read_gray(file).astype(int) - frame.gray).max() <= 2
    camera = frames.camera
    assert (camera.model, camera.width, camera.height) == ("radtan", 320, 240)
    np.testing.assert_allclose(
        camera.intrinsics, lenses.RadTan(ROOM_CAMERA, DISTORTION).intrinsics(), atol=1e-9
    )
    times, poses = runs.read_trajectory(tmp_path / "tum" / "groundtruth.txt")
    np.testing.assert_allclose(times, [0.3 + i / 60 for i in range(FRAMES)], atol=1e-6)
    np.testing.assert_allclose(poses, [f.T_world_cam for f in rendered], atol=1e-9)


def test_a_smaller_short_side_scales_the_lens(rendered: list[room.Frame], tmp_path: Path) -> None:
    """At a short side of 120 the frames are 160x120, the lens scaled, its distortion kept."""
    sequence = _write(tmp_path, rendered, -90)
    advio.import_sequence(sequence, tmp_path / "calibration", tmp_path / "tum", short_side=120)
    camera = tum.read_camera(tmp_path / "tum")
    expected = lenses.RadTan(ROOM_CAMERA, DISTORTION).scaled(160, 120)
    np.testing.assert_allclose(camera.intrinsics, expected.intrinsics(), atol=1e-9)


@pytest.mark.parametrize(
    ("number", "name"),
    [(1, "iphone-02.yaml"), (12, "iphone-02.yaml"), (13, "iphone-03.yaml"),
     (17, "iphone-03.yaml"), (18, "iphone-01.yaml"), (19, "iphone-01.yaml"),
     (20, "iphone-04.yaml"), (23, "iphone-04.yaml")],
)  # fmt: skip
def test_each_sequence_takes_its_batchs_calibration(number: int, name: str) -> None:
    """ADVIO's calibration README: 1-12, 13-17, 18-19 and 20-23, each batch its own file."""
    assert advio.calibration_name(Path(f"/data/advio-{number:02d}")) == name


@pytest.mark.parametrize("name", ["advio-00", "advio-24", "advio-1", "sequence-01"])
def test_a_folder_not_named_for_a_sequence_is_refused(name: str) -> None:
    """Only advio-01 to advio-23."""
    with pytest.raises(InvalidInputError, match="advio-01 to advio-23"):
        advio.calibration_name(Path(name))


def _replace(path: Path, old: str, new: str) -> None:
    """Replace a file's text."""
    path.write_text(path.read_text().replace(old, new))


def _skew(root: Path) -> None:
    """Make the calibration's ``T_cam_imu`` not a rotation."""
    skewed = CAM_IMU.copy()
    skewed[0, 0] = 2.0
    (root / "calibration" / "iphone-02.yaml").write_text(_calibration(ROOM_CAMERA, skewed))


def _backwards() -> str:
    """Return a ``frames.csv`` numbered right whose times go backwards."""
    return "".join(f"{0.5 - i / 60!r},{i + 1}\n" for i in range(FRAMES))


CALIBRATION = Path("calibration") / "iphone-02.yaml"
"""The test calibration's place under the folder written."""


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (lambda r: _replace(r / CALIBRATION, "radial-tangential", "equidistant"), "radial-tan"),
        (lambda r: _replace(r / CALIBRATION, "T_cam_imu", "T_imu"), "no `T_cam_imu`"),
        (_skew, "not a rigid transform"),
        (lambda r: _replace(r / "advio-01/iphone/frames.csv", ",3\n", ",4\n"), "numbered"),
        (lambda r: (r / "advio-01/iphone/frames.csv").write_text(_backwards()), "not increase"),
        (lambda r: (r / "advio-01/ground-truth/pose.csv").write_text("1,2,3\n"), "8 finite"),
        (
            lambda r: (r / "advio-01/ground-truth/pose.csv").write_text("0,0,0,0,0,0,0,0\n"),
            "a zero quaternion",
        ),
        (lambda r: (r / "advio-01/iphone/frames.csv").write_text("0.3,1\n"), "than the 1 timed"),
    ],
)
def test_a_malformed_sequence_or_calibration_is_refused(
    change: Callable[[Path], None], message: str, rendered: list[room.Frame], tmp_path: Path
) -> None:
    """Each refusal of a sequence or its calibration.

    A distortion of another model, no or a skewed extrinsic, frames misnumbered or out of order,
    a short or degenerate ground truth, a clock that does not time every frame.
    """
    _write(tmp_path, rendered, -90)
    change(tmp_path)
    with pytest.raises(InvalidInputError, match=message):
        advio.import_sequence(tmp_path / "advio-01", tmp_path / "calibration", tmp_path / "tum")


def test_main_imports_and_says_what(
    rendered: list[room.Frame], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The program imports a sequence and gives its frames, size and model."""
    sequence = _write(tmp_path, rendered, -90)
    out = tmp_path / "tum"
    assert advio.main([str(sequence), str(tmp_path / "calibration"), str(out)]) == 0
    assert capsys.readouterr().out == f"{out}: 3 frames, 320x240 (radtan), of advio-01\n"
