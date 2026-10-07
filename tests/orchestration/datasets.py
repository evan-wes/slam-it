"""The synthetic room as dataset folders: TUM-format and EuRoC, whole or with a gap.

The room (:mod:`tests.edge.room`), rendered with its right views, is written as a TUM-format
folder (gray PNGs, 16-bit depth PNGs, ``rgb.txt``, ``depth.txt`` and ``camera.txt``, with
ORB-SLAM's TUM ``bf``) and as a EuRoC sequence (``mav0/cam0`` and ``cam1``: PNGs,
``data.csv`` and ``sensor.yaml``, the right camera :data:`tests.edge.room.BASELINE` to the
left one's right, no distortion). The gap room drops the frames of :data:`GAP`. The tests'
fixtures (``tests/orchestration/conftest.py``) and the golden record's generator
(``tests/fixtures/edge/generate_room_runs.py``) both write them from here.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from PIL import Image

from slamlab.eval import tum
from tests.edge import room
from tests.edge.conftest import OPTIONS

SENSOR = """sensor_type: camera
comment: the room's {name}
T_BS:
  cols: 4
  rows: 4
  data: [{pose}]
rate_hz: 30
resolution: [{cols}, {rows}]
camera_model: pinhole
intrinsics: [{fx}, {fy}, {cx}, {cy}]
distortion_model: radial-tangential
distortion_coefficients: [0.0, 0.0, 0.0, 0.0]
"""
"""A camera's ``sensor.yaml``, in EuRoC's layout."""


@dataclass(frozen=True)
class Room:
    """The room's folders and its truth."""

    tum: Path
    euroc: Path
    timestamps: NDArray[np.float64]
    poses: NDArray[np.float64]


def png(path: Path, values: NDArray[np.uint8] | NDArray[np.uint16]) -> None:
    """Write a gray PNG, 8- or 16-bit as the array is."""
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(values).save(path)


def write_tum(folder: Path, rendered: list[room.Frame]) -> None:
    """Write the room as a TUM-format folder that describes its own camera."""
    camera = room.SMALL
    rgb, depth = ["# timestamp filename"], ["# timestamp filename"]
    for f in rendered:
        stamp = f"{f.timestamp:.6f}"
        png(folder / "rgb" / f"{stamp}.png", f.gray)
        scaled = np.clip(np.round(f.depth * tum.DEPTH_SCALE), 0, np.iinfo(np.uint16).max)
        png(folder / "depth" / f"{stamp}.png", scaled.astype(np.uint16))
        rgb.append(f"{stamp} rgb/{stamp}.png")
        depth.append(f"{stamp} depth/{stamp}.png")
    (folder / "rgb.txt").write_text("\n".join(rgb) + "\n")
    (folder / "depth.txt").write_text("\n".join(depth) + "\n")
    (folder / "camera.txt").write_text(
        f"model pinhole\nintrinsics {camera.fx} {camera.fy} {camera.cx} {camera.cy}\n"
        f"size {camera.width} {camera.height}\ndepth_scale {tum.DEPTH_SCALE}\n"
        f"bf {tum.SENSOR_BF}\n"
    )


def write_euroc(folder: Path, rendered: list[room.Frame]) -> None:
    """Write the room as a EuRoC sequence: the left camera is the body."""
    camera = room.SMALL
    for name, offset in (("cam0", 0.0), ("cam1", room.BASELINE)):
        pose = np.eye(4)
        pose[0, 3] = offset
        root = folder / "mav0" / name
        rows = ["#timestamp [ns],filename"]
        for f in rendered:
            stamp = round(f.timestamp * 1e9)
            view = f.gray if name == "cam0" else f.right
            assert view is not None
            png(root / "data" / f"{stamp}.png", view)
            rows.append(f"{stamp},{stamp}.png")
        (root / "data.csv").write_text("\n".join(rows) + "\n")
        (root / "sensor.yaml").write_text(
            SENSOR.format(
                name=name,
                pose=", ".join(repr(float(v)) for v in pose.ravel()),
                cols=camera.width,
                rows=camera.height,
                fx=camera.fx,
                fy=camera.fy,
                cx=camera.cx,
                cy=camera.cy,
            )
        )


GAP = (20, 30)
"""The frames the gap room drops: a jump the motion model, narrowed, cannot bridge."""

GAP_OPTIONS = replace(OPTIONS, motion_window=2.0, stereo_motion_window=2.0)
"""The room's settings with the motion model's window narrowed: across the gap, tracking the
last frame fails and the pipeline recovers by PnP against the local map."""


def write_room(base: Path, rendered: list[room.Frame]) -> Room:
    """Write rendered frames as both datasets' folders under a base."""
    tum_folder, euroc_folder = base / "room_tum", base / "room_euroc"
    write_tum(tum_folder, rendered)
    write_euroc(euroc_folder, rendered)
    return Room(
        tum_folder,
        euroc_folder,
        np.array([f.timestamp for f in rendered]),
        np.array([f.T_world_cam for f in rendered]),
    )


CASES = [("tum", "rgbd"), ("tum", "monocular"), ("euroc", "stereo"), ("euroc", "monocular")]
"""Each dataset in each mode it records."""
