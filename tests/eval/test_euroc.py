"""EuRoC reader: a synthetic ASL sequence with EuRoC's own calibration files, and failures."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab.eval import euroc
from slamlab.foundations import se3, so3
from slamlab.foundations.errors import InvalidInputError

# EuRoC's cam0 and cam1 sensor.yaml, as published.
CAM0 = """%YAML:1.0
# General sensor definitions.
sensor_type: camera
comment: VI-Sensor cam0 (MT9M034)

# Sensor extrinsics wrt. the body-frame.
T_BS:
  cols: 4
  rows: 4
  data: [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0]

# Camera specific definitions.
rate_hz: 20
resolution: [752, 480]
camera_model: pinhole
intrinsics: [458.654, 457.296, 367.215, 248.375] #fu, fv, cu, cv
distortion_model: radial-tangential
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
"""

CAM1 = """%YAML:1.0
# General sensor definitions.
sensor_type: camera
comment: VI-Sensor cam1 (MT9M034)

# Sensor extrinsics wrt. the body-frame.
T_BS:
  cols: 4
  rows: 4
  data: [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
         0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
        -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
         0.0, 0.0, 0.0, 1.0]

# Camera specific definitions.
rate_hz: 20
resolution: [752, 480]
camera_model: pinhole
intrinsics: [457.587, 456.134, 379.999, 255.238] #fu, fv, cu, cv
distortion_model: radial-tangential
distortion_coefficients: [-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05]
"""

START = 1403636579763555584  # MH_01_easy's first frame, in ns


def _body_pose(k: int) -> NDArray[np.float64]:
    """Return a smooth synthetic body pose for ground-truth sample `k`."""
    pose = np.eye(4)
    pose[:3, :3] = so3.exp(np.array([0.01 * k, -0.02 * k, 0.005 * k]))
    pose[:3, 3] = [0.1 * k, 2.0 - 0.05 * k, 1.0 + 0.01 * k]
    return pose


def _write(root: Path, *, cam0: str = CAM0, frames: int = 5, gaps: tuple[int, ...] = ()) -> None:
    """Write an ASL sequence: two cameras at 20 Hz (cam1 missing `gaps`), truth at 200 Hz."""
    for name, yaml, skip in (("cam0", cam0, ()), ("cam1", CAM1, gaps)):
        folder = root / "mav0" / name
        (folder / "data").mkdir(parents=True)
        (folder / "sensor.yaml").write_text(yaml)
        rows = ["#timestamp [ns],filename"]
        for f in range(frames):
            if f not in skip:
                stamp = START + f * 50_000_000
                rows.append(f"{stamp},{stamp}.png")
        (folder / "data.csv").write_text("\n".join(rows) + "\n")
    truth = root / "mav0" / "state_groundtruth_estimate0"
    truth.mkdir(parents=True)
    rows = ["#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], ..."]
    for k in range(10 * frames):
        pose = _body_pose(k)
        x, y, z, w = so3.to_quaternion(pose[:3, :3])
        t = pose[:3, 3]
        # 1 ms late, like a real 200 Hz stream sampled off the camera's clock.
        stamp = START + k * 5_000_000 + 1_000_000
        rows.append(f"{stamp},{t[0]},{t[1]},{t[2]},{w},{x},{y},{z},0,0,0,0,0,0,0,0,0")
    (truth / "data.csv").write_text("\n".join(rows) + "\n")


def test_reads_frames_poses_and_cameras(tmp_path: Path) -> None:
    """Frames with both images are kept, with T_world_body T_body_cam0 and both cameras."""
    _write(tmp_path, gaps=(2,))
    sequence = euroc.read_sequence(tmp_path)
    kept = [0, 1, 3, 4]
    np.testing.assert_allclose(sequence.timestamps, (START + np.array(kept) * 50_000_000) / 1e9)
    assert [p.name for p in sequence.left] == [f"{START + f * 50_000_000}.png" for f in kept]
    assert all(p.parent.parent.name == "cam1" for p in sequence.right)
    camera = sequence.left_camera
    np.testing.assert_array_equal(
        camera.intrinsics,
        [
            458.654,
            457.296,
            367.215,
            248.375,
            -0.28340811,
            0.07395907,
            0.00019359,
            1.76187114e-05,
            0.0,
        ],
    )
    assert camera.shape == (480, 752)
    for row, f in enumerate(kept):
        expected = _body_pose(10 * f) @ camera.T_body_cam
        np.testing.assert_allclose(sequence.T_world_cam[row], expected, atol=1e-12)
    right_left = sequence.T_right_left
    se3.check_transform(right_left.reshape(1, 4, 4))
    assert 0.10 < np.linalg.norm(right_left[:3, 3]) < 0.12  # EuRoC's 11 cm baseline


def test_frames_without_a_close_pose_are_dropped(tmp_path: Path) -> None:
    """With a tolerance under the 1 ms offset, no frame has a pose."""
    _write(tmp_path)
    with pytest.raises(InvalidInputError, match="no frame has a ground-truth pose"):
        euroc.read_sequence(tmp_path, max_difference=0.0005)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (("camera_model: pinhole", "camera_model: omni"), "expected camera_model: pinhole"),
        (("radial-tangential", "equidistant"), "expected distortion_model"),
        (
            ("intrinsics: [458.654, 457.296, 367.215, 248.375]", "intrinsics: [458.654]"),
            "expected 4 intrinsics",
        ),
        (("resolution: [752, 480]", "resolution: [752, x]"), "`resolution` holds a non-number"),
        (("resolution: [752, 480]", "resolution: [752.5, 480]"), "a resolution of two integers"),
        (("resolution: [752, 480]", "resolution: [752, 0]"), "a resolution of two integers"),
        (("resolution: [752, 480]", "resolution: [752, inf]"), "expected finite numbers"),
        (("248.375]", "nan]"), "expected finite numbers"),
        (("0.999660727178", "1.999660727178"), "rotation"),
    ],
)
def test_rejects_malformed_calibration(
    tmp_path: Path, change: tuple[str, str], message: str
) -> None:
    """Unsupported models, short lists, non-numbers and non-rigid poses are refused."""
    _write(tmp_path, cam0=CAM0.replace(*change))
    with pytest.raises(InvalidInputError, match=message):
        euroc.read_sequence(tmp_path)


@pytest.mark.parametrize(
    ("file", "edit", "message"),
    [
        ("cam0/data.csv", lambda text: text + "12345\n", "expected `timestamp,filename`"),
        ("cam0/data.csv", lambda text: text + "x1,a.png\n", "expected `timestamp,filename`"),
        (
            "cam0/data.csv",
            lambda text: text + text.splitlines()[1] + "\n",
            r"timestamp \d+ repeats",
        ),
        (
            "state_groundtruth_estimate0/data.csv",
            lambda text: text + "1,0,0\n",
            "expected 7 finite values",
        ),
        (
            "state_groundtruth_estimate0/data.csv",
            lambda text: text + "1,nan,0,0,1,0,0,0\n",
            "expected 7 finite values",
        ),
        (
            "state_groundtruth_estimate0/data.csv",
            lambda text: text + "1,x,0,0,1,0,0,0\n",
            "expected numbers",
        ),
        (
            "state_groundtruth_estimate0/data.csv",
            lambda text: text + "1,0,0,0,0,0,0,0\n",
            "a non-zero quaternion",
        ),
        (
            "state_groundtruth_estimate0/data.csv",
            lambda text: text + text.splitlines()[1] + "\n",
            "a timestamp repeats",
        ),
        ("state_groundtruth_estimate0/data.csv", lambda text: "#header\n", "no ground-truth"),
    ],
)
def test_rejects_malformed_rows(tmp_path: Path, file: str, edit: object, message: str) -> None:
    """Short or non-numeric rows, non-finite poses and repeated timestamps are refused."""
    _write(tmp_path)
    path = tmp_path / "mav0" / file
    assert callable(edit)
    path.write_text(edit(path.read_text()))
    with pytest.raises(InvalidInputError, match=message):
        euroc.read_sequence(tmp_path)


def test_rejects_a_negative_tolerance(tmp_path: Path) -> None:
    """The association tolerance must be non-negative."""
    _write(tmp_path)
    with pytest.raises(InvalidInputError, match="max_difference"):
        euroc.read_sequence(tmp_path, max_difference=-1.0)
