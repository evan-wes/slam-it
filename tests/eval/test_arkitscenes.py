"""ARKitScenes import and LiDAR depth: a synthetic capture in the raw split's layout; failures."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from slam_it.eval import arkitscenes, images, lidar, runs, tum
from slam_it.foundations import se3, so3
from slam_it.foundations.errors import InvalidInputError

VIDEO = "41069021"
"""The capture's video id, in its files' names."""

FRAMES = (3112.887, 3112.920, 3112.953)
"""The colour frames' times, 30 Hz."""

DEPTHS = (3112.886, 3112.903, 3112.919, 3112.936, 3112.952)
"""The depth maps' times, 60 Hz."""


def test_depth_is_kept_by_confidence_and_brought_to_the_frame_by_nearest_neighbour() -> None:
    """Each colour pixel takes the depth pixel its centre falls in; low confidence is zero."""
    depth = np.array([[1000, 2000], [3000, 4000]], dtype=np.uint16)
    confidence = np.array([[2, 1], [2, 0]], dtype=np.uint8)
    high = lidar.registered_depth(depth, confidence, (4, 4))
    np.testing.assert_array_equal(
        high, [[1000, 1000, 0, 0], [1000, 1000, 0, 0], [3000, 3000, 0, 0], [3000, 3000, 0, 0]]
    )
    medium = lidar.registered_depth(depth, confidence, (4, 4), lidar.MEDIUM)
    assert medium[0, 3] == 2000
    assert medium[3, 3] == 0
    # 256x192 to 640x480: 2.5 colour pixels a depth pixel, each rounded to the nearest.
    ramp = np.tile(np.arange(256, dtype=np.uint16), (192, 1))
    wide = lidar.registered_depth(ramp, np.full((192, 256), 2, np.uint8), (640, 480))
    assert wide.shape == (480, 640)
    np.testing.assert_array_equal(wide[0, :6], [0, 0, 1, 1, 1, 2])
    assert lidar.bf(500.0) == pytest.approx(40.0)


@pytest.mark.parametrize(
    ("shapes", "size", "threshold", "message"),
    [
        (((2, 2), (2, 3)), (4, 4), 2, "one 2-D shape"),
        (((2, 2), (2, 2)), (4, 3), 2, "aspect"),
        (((2, 2), (2, 2)), (1, 1), 2, "at least its size"),
        (((2, 2), (2, 2)), (4, 4), 3, "0, 1 or 2"),
    ],
)
def test_depth_that_does_not_fit_the_frame_is_refused(
    shapes: tuple[tuple[int, int], tuple[int, int]],
    size: tuple[int, int],
    threshold: int,
    message: str,
) -> None:
    """Maps of two shapes, another aspect, a smaller frame, or a threshold outside 0 to 2."""
    depth = np.zeros(shapes[0], np.uint16)
    confidence = np.zeros(shapes[1], np.uint8)
    with pytest.raises(InvalidInputError, match=message):
        lidar.registered_depth(depth, confidence, size, threshold)


def _write(capture: Path) -> None:
    """Write a capture: three 64x48 colour frames, their intrinsics, 32x24 depth, ARKit's poses."""
    for k, time in enumerate(FRAMES):
        name = f"{VIDEO}_{time:.3f}"
        images.write_png(
            capture / "vga_wide" / f"{name}.png", np.full((48, 64, 3), 30 * k, np.uint8)
        )
        intrinsics = capture / "vga_wide_intrinsics" / f"{name}.pincam"
        intrinsics.parent.mkdir(parents=True, exist_ok=True)
        intrinsics.write_text(f"64 48 {50.0 + k} {50.5 + k} 32.0 24.0\n")
    for k, time in enumerate(DEPTHS):
        name = f"{VIDEO}_{time:.3f}.png"
        images.write_depth(capture / "lowres_depth" / name, np.full((24, 32), 1000 + k, np.uint16))
        confidence = np.full((24, 32), 2, np.uint8)
        confidence[0, 0] = 1
        images.write_png(capture / "confidence" / name, confidence)
    lines = [
        f"{t} 0.0 0.1 0.0 {0.01 * k} 0.0 -1.5"
        for k, t in enumerate((3112.88, 3112.90, 3112.92, 3112.94, 3112.96))
    ]
    (capture / arkitscenes.TRAJECTORY).write_text("\n".join(lines) + "\n")


def test_a_capture_imports_to_an_rgbd_folder_the_pipeline_reads(tmp_path: Path) -> None:
    """Frames linked; each with its nearest depth, kept and upsampled; ARKit's poses inverted.

    The camera is the frames' median intrinsics, with LiDAR's depth scale and ``bf``.
    """
    _write(tmp_path / VIDEO)
    arkitscenes.import_capture(tmp_path / VIDEO, tmp_path / "tum")
    frames = tum.read_frames(tmp_path / "tum")
    assert [f"{t:.3f}" for t in frames.timestamps] == [f"{t:.3f}" for t in FRAMES]
    assert all(depth is not None for depth in frames.depths)
    path = frames.depths[1]  # 3112.920: the map of 3112.919
    assert path is not None
    second = images.read_depth(path)
    assert second.shape == (48, 64)
    assert second[0, 0] == second[1, 1] == 0  # medium confidence, its 2x2 block
    assert second[2, 2] == 1002
    camera = tum.read_camera(tmp_path / "tum")
    assert (camera.model, camera.width, camera.height, camera.depth_scale) == (
        "pinhole",
        64,
        48,
        1000.0,
    )
    np.testing.assert_allclose(camera.intrinsics, [51.0, 51.5, 32.0, 24.0])
    assert camera.bf == pytest.approx(lidar.BASELINE * 51.0)
    stamps, poses = runs.read_trajectory(tmp_path / "tum" / "groundtruth.txt")
    cam_world = np.eye(4)
    cam_world[:3, :3] = so3.exp(np.array([0.0, 0.1, 0.0]))
    cam_world[:3, 3] = [0.02, 0.0, -1.5]
    np.testing.assert_allclose(stamps[2], 3112.92)
    np.testing.assert_allclose(poses[2], se3.inverse(cam_world), atol=1e-12)


def test_a_frame_without_depth_near_it_has_none(tmp_path: Path) -> None:
    """A depth map more than 20 ms from every frame is not used: that frame runs without depth."""
    _write(tmp_path / VIDEO)
    for time in DEPTHS[2:]:
        for folder in ("lowres_depth", "confidence"):
            (tmp_path / VIDEO / folder / f"{VIDEO}_{time:.3f}.png").unlink()
    arkitscenes.import_capture(tmp_path / VIDEO, tmp_path / "tum")
    frames = tum.read_frames(tmp_path / "tum")
    assert [depth is not None for depth in frames.depths] == [True, True, False]


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (
            lambda c: (c / "vga_wide_intrinsics" / f"{VIDEO}_{FRAMES[1]:.3f}.pincam").write_text(
                "64 48 x\n"
            ),
            "six numbers",
        ),
        (
            lambda c: (c / "vga_wide_intrinsics" / f"{VIDEO}_{FRAMES[1]:.3f}.pincam").write_text(
                "32 24 1 1 1 1\n"
            ),
            "differ in size",
        ),
        (lambda c: (c / arkitscenes.TRAJECTORY).write_text("1 0 0 0 0 0\n"), "seven finite"),
        (
            lambda c: (c / arkitscenes.TRAJECTORY).write_text("2 0 0 0 0 0 0\n1 0 0 0 0 0 0\n"),
            "do not increase",
        ),
        (
            lambda c: (c / "vga_wide" / f"{VIDEO}_{FRAMES[0]:.3f}.png").rename(
                c / "vga_wide" / "bad_name.png"
            ),
            "<video_id>_<seconds>",
        ),
    ],
)
def test_a_malformed_capture_is_refused(tmp_path: Path, change: object, message: str) -> None:
    """Bad intrinsics, frames of two sizes, a short or unordered trajectory, a misnamed file."""
    _write(tmp_path / VIDEO)
    change(tmp_path / VIDEO)  # type: ignore[operator]
    with pytest.raises(InvalidInputError, match=message):
        arkitscenes.import_capture(tmp_path / VIDEO, tmp_path / "tum")


@pytest.mark.parametrize("threshold", [lidar.HIGH, lidar.MEDIUM])
def test_the_import_records_its_capture_and_threshold(threshold: int, tmp_path: Path) -> None:
    """``source.json`` says what made the folder: ARKitScenes, the capture, the confidence."""
    _write(tmp_path / VIDEO)
    arkitscenes.import_capture(tmp_path / VIDEO, tmp_path / "tum", threshold)
    record = json.loads((tmp_path / "tum" / "source.json").read_text())
    assert record == {
        "kind": "arkitscenes",
        "capture": VIDEO,
        "depth": {"min_confidence": threshold},
    }


def test_an_import_that_fails_halfway_leaves_nothing(tmp_path: Path) -> None:
    """The last frame's depth map is of another aspect: refused by name, nothing written."""
    _write(tmp_path / VIDEO)
    name = f"{VIDEO}_{DEPTHS[-1]:.3f}.png"
    images.write_depth(tmp_path / VIDEO / "lowres_depth" / name, np.zeros((20, 32), np.uint16))
    images.write_png(tmp_path / VIDEO / "confidence" / name, np.zeros((20, 32), np.uint8))
    with pytest.raises(InvalidInputError, match=f"{name}: .*aspect"):
        arkitscenes.import_capture(tmp_path / VIDEO, tmp_path / "outs" / "tum")
    assert list((tmp_path / "outs").iterdir()) == []
