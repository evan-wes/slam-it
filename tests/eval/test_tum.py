"""TUM RGB-D reader tests, on a small synthetic sequence folder."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from slam_it.eval import images, tum
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError


def _write(folder: Path) -> None:
    """Write a three-frame sequence in which one frame has no pose within 0.02 s."""
    folder.mkdir()
    (folder / "rgb.txt").write_text(
        "# color images\n# timestamp filename\n"
        "100.000 rgb/100.000.png\n100.500 rgb/100.500.png\n103.000 rgb/103.000.png\n"
    )
    (folder / "groundtruth.txt").write_text(
        "# ground truth trajectory\n# timestamp tx ty tz qx qy qz qw\n"
        "99.995 1.0 2.0 3.0 0.0 0.0 0.0 1.0\n"
        "100.510 1.5 2.0 3.0 0.0 0.0 0.7071 0.7071\n"
        "102.900 9.0 9.0 9.0 0.0 0.0 0.0 1.0\n"
    )


def test_reads_and_associates_frames(tmp_path: Path) -> None:
    """Frames take the nearest pose within 0.02 s, others are dropped, quaternions normalised."""
    folder = tmp_path / "rgbd_dataset_freiburg1_xyz"
    _write(folder)
    sequence = tum.read_sequence(folder)
    assert sequence.camera.model == "radtan"
    np.testing.assert_array_equal(sequence.timestamps, [100.0, 100.5])
    assert sequence.images == [folder / "rgb/100.000.png", folder / "rgb/100.500.png"]
    first = np.eye(4)
    first[:3, 3] = [1.0, 2.0, 3.0]
    np.testing.assert_allclose(sequence.T_world_cam[0], first, rtol=0, atol=0)
    quarter = so3.exp([0.0, 0.0, np.pi / 2])
    np.testing.assert_allclose(sequence.T_world_cam[1][:3, :3], quarter, rtol=0, atol=1e-12)
    np.testing.assert_array_equal(sequence.intrinsics, tum.INTRINSICS["freiburg1"])
    assert sequence.depths == [None, None]


def test_associates_depth_maps(tmp_path: Path) -> None:
    """Each kept frame takes the nearest depth map within 0.02 s, or None."""
    folder = tmp_path / "rgbd_dataset_freiburg1_xyz"
    _write(folder)
    (folder / "depth.txt").write_text(
        "# depth maps\n# timestamp filename\n"
        "99.900 depth/99.900.png\n100.010 depth/100.010.png\n100.600 depth/100.600.png\n"
    )
    sequence = tum.read_sequence(folder)
    assert sequence.depths == [folder / "depth/100.010.png", None]


@pytest.mark.parametrize(
    ("lines", "message"),
    [
        ("100.0 rgb/a.png\nnan rgb/b.png\n", "expected finite values"),
        ("100.5 rgb/a.png\n100.0 rgb/b.png\n", "must ascend"),
    ],
)
def test_rejects_unordered_or_non_finite_stamps(tmp_path: Path, lines: str, message: str) -> None:
    """A NaN or out-of-order stamp in rgb.txt is refused, not silently dropped or kept."""
    folder = tmp_path / "rgbd_dataset_freiburg1_xyz"
    _write(folder)
    (folder / "rgb.txt").write_text("# colour images\n" + lines)
    with pytest.raises(ValueError, match=message):
        tum.read_sequence(folder)


def test_rejects_unknown_sensors_and_empty_associations(tmp_path: Path) -> None:
    """A folder without its sensor in the name, or with no frame near a pose, is an error."""
    folder = tmp_path / "my_sequence"
    _write(folder)
    with pytest.raises(ValueError, match="freiburg"):
        tum.read_sequence(folder)
    desk = tmp_path / "rgbd_dataset_freiburg2_desk"
    _write(desk)
    with pytest.raises(ValueError, match="no frame"):
        tum.read_sequence(desk, max_difference=1e-6)


@pytest.mark.parametrize(
    ("groundtruth", "message"),
    [
        ("100.0 1.0 2.0 3.0 0.0 0.0 0.0\n", r"groundtruth.txt:1: expected 8 fields"),
        ("100.0 1.0 2.0 3.0 0.0 0.0 0.0 x\n", r"groundtruth.txt:1: expected numbers"),
        ("100.0 1.0 2.0 3.0 0.0 0.0 0.0 0.0\n", r"groundtruth.txt:1: .*non-zero quaternion"),
        ("# only a comment\n", "no ground-truth poses"),
    ],
)
def test_malformed_ground_truth_is_named_with_its_line(
    tmp_path: Path, groundtruth: str, message: str
) -> None:
    """A short, non-numeric or zero-quaternion row, or no rows, is an error naming the file."""
    folder = tmp_path / "rgbd_dataset_freiburg1_xyz"
    _write(folder)
    (folder / "groundtruth.txt").write_text(groundtruth)
    with pytest.raises(ValueError, match=message):
        tum.read_sequence(folder)


def test_rejects_a_negative_max_difference(tmp_path: Path) -> None:
    """The association tolerance must be non-negative."""
    folder = tmp_path / "rgbd_dataset_freiburg1_xyz"
    _write(folder)
    with pytest.raises(ValueError, match="max_difference"):
        tum.read_sequence(folder, max_difference=-1.0)


def test_a_malformed_depth_file_is_named_with_its_line(tmp_path: Path) -> None:
    """A depth map listed with a timestamp that is not a number is an error naming the line."""
    folder = tmp_path / "rgbd_dataset_freiburg1_xyz"
    _write(folder)
    (folder / "depth.txt").write_text("# depth maps\n100.010 depth/a.png\nnoon depth/b.png\n")
    with pytest.raises(ValueError, match=r"depth.txt:3: expected a numeric timestamp"):
        tum.read_sequence(folder)


def _camera_file(folder: Path, text: str) -> None:
    """Write a camera.txt."""
    (folder / "camera.txt").write_text(text)


CAMERA_TXT = """# the recording's camera
model pinhole
intrinsics 525.0 526.0 319.5 239.5  # fx fy cx cy
size 640 480
depth_scale 1000
bf 41.5
"""


def test_frames_need_no_ground_truth(tmp_path: Path) -> None:
    """A run's frames: every rgb.txt row and its depth map, no groundtruth.txt needed."""
    folder = tmp_path / "my_recording"
    folder.mkdir()
    (folder / "rgb.txt").write_text(
        "1.0 rgb/1.png\n1.5 rgb/1b.png\n1.5 rgb/1c.png\n2.0 rgb/2.png\n"
    )
    (folder / "depth.txt").write_text("1.01 depth/1.png\n2.5 depth/2.png\n")
    _camera_file(folder, CAMERA_TXT)
    frames = tum.read_frames(folder)
    np.testing.assert_array_equal(frames.timestamps, [1.0, 1.5, 1.5, 2.0])
    assert frames.images[-1] == folder / "rgb/2.png"
    assert frames.depths == [folder / "depth/1.png", None, None, None]
    assert frames.camera.model == "pinhole"
    np.testing.assert_array_equal(frames.camera.intrinsics, [525.0, 526.0, 319.5, 239.5])
    assert (frames.camera.width, frames.camera.height, frames.camera.depth_scale) == (
        640,
        480,
        1000.0,
    )


def test_the_camera_file_may_give_bf(tmp_path: Path) -> None:
    """``bf`` is read when camera.txt gives it, and optional: monocular runs need none."""
    folder = tmp_path / "my_recording"
    folder.mkdir()
    _camera_file(folder, CAMERA_TXT)
    assert tum.read_camera(folder).bf == 41.5
    _camera_file(folder, CAMERA_TXT.replace("bf 41.5\n", ""))
    assert tum.read_camera(folder).bf is None


def test_camera_file_takes_precedence_over_the_name(tmp_path: Path) -> None:
    """A benchmark-named folder with a camera.txt reads its camera from the file."""
    folder = tmp_path / "rgbd_dataset_freiburg1_xyz"
    _write(folder)
    _camera_file(folder, CAMERA_TXT)
    assert tum.read_sequence(folder).camera.model == "pinhole"
    without = tmp_path / "rgbd_dataset_freiburg3_x"
    without.mkdir()
    camera = tum.read_camera(without)
    assert (camera.model, camera.width, camera.depth_scale) == ("radtan", 640, tum.DEPTH_SCALE)
    assert camera.bf == tum.SENSOR_BF


@pytest.mark.parametrize(
    ("text", "message"),
    [
        (CAMERA_TXT + "baseline 0.1\n", "unknown or repeated key 'baseline'"),
        (CAMERA_TXT + "size 1 1\n", "unknown or repeated key 'size'"),
        (CAMERA_TXT.replace("depth_scale 1000\n", ""), "missing depth_scale"),
        (CAMERA_TXT.replace("size 640 480", "size 640"), "two sizes"),
        (CAMERA_TXT.replace("525.0 526.0", "525.0 x"), "expected numbers"),
        (CAMERA_TXT.replace("model pinhole", "model radtan"), "intrinsics"),
        (CAMERA_TXT.replace("model pinhole", "model fisheye"), "unknown camera model"),
        (CAMERA_TXT.replace("size 640 480", "size 0 480"), "size"),
        (CAMERA_TXT.replace("depth_scale 1000", "depth_scale -1"), "depth_scale"),
        (CAMERA_TXT.replace("bf 41.5", "bf 0"), "bf must be"),
        (CAMERA_TXT.replace("bf 41.5", "bf 40 41"), "one bf"),
        (CAMERA_TXT + "bf 40\n", "unknown or repeated key 'bf'"),
    ],
)
def test_rejects_a_malformed_camera_file(tmp_path: Path, text: str, message: str) -> None:
    """Every key once, each with well-formed values."""
    _camera_file(tmp_path, text)
    with pytest.raises(InvalidInputError, match=message):
        tum.read_camera(tmp_path)


def test_depth_scale_divides_the_stored_values(tmp_path: Path) -> None:
    """A depth map's values over the camera's depth scale, zeros as NaN."""
    path = tmp_path / "d.png"
    Image.fromarray(np.array([[0, 1000], [2500, 65535]], dtype=np.uint16)).save(path)
    depth = tum.read_depth(path, 1000.0)
    assert np.isnan(depth[0, 0])
    np.testing.assert_allclose(depth[0, 1:], [1.0])
    np.testing.assert_allclose(depth[1], [2.5, 65.535])
    with pytest.raises(InvalidInputError, match="depth_scale"):
        tum.read_depth(path, 0.0)


def test_what_the_writers_write_the_readers_read(tmp_path: Path) -> None:
    """write_camera and write_list round-trip through read_camera and read_frames."""
    camera = tum.Camera("pinhole", np.array([500.5, 501.25, 319.5, 239.75]), 640, 480, 5000.0, 40.0)
    tum.write_camera(tmp_path, camera, note="written by the test")
    assert (tmp_path / tum.CAMERA_FILE).read_text().startswith("# written by the test\n")
    read = tum.read_camera(tmp_path)
    np.testing.assert_array_equal(read.intrinsics, camera.intrinsics)
    assert (read.model, read.width, read.height, read.depth_scale, read.bf) == (
        "pinhole", 640, 480, 5000.0, 40.0,
    )  # fmt: skip
    gray = np.zeros((480, 640), np.uint8)
    entries = [(f"{t:.6f}", f"rgb/{t:.6f}.png") for t in (0.0, 0.5)]
    for _, name in entries:
        images.write_png(tmp_path / name, np.repeat(gray[:, :, None], 3, axis=2))
    tum.write_list(tmp_path / "rgb.txt", entries)
    frames = tum.read_frames(tmp_path)
    np.testing.assert_array_equal(frames.timestamps, [0.0, 0.5])
    with pytest.raises(InvalidInputError):
        tum.write_camera(tmp_path, tum.Camera("pinhole", np.array([1.0]), 640, 480, 1.0, None))
