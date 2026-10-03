"""TUM RGB-D reader tests, on a small synthetic sequence folder."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from slamlab.eval import tum
from slamlab.foundations import so3


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
    assert sequence.sensor == "freiburg1"
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
        "100.010 depth/100.010.png\n99.900 depth/99.900.png\n100.600 depth/100.600.png\n"
    )
    sequence = tum.read_sequence(folder)
    assert sequence.depths == [folder / "depth/100.010.png", None]


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
