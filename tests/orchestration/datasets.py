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
from typing import Any

import numpy as np
from numpy.typing import NDArray
from PIL import Image

from slam_it_all.edge.options import Options
from slam_it_all.eval import runs, tum
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS

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
    """Write the room as a TUM-format folder that describes its own camera, with its truth."""
    camera = room.SMALL
    rgb: list[tuple[str, str]] = []
    depth: list[tuple[str, str]] = []
    for f in rendered:
        stamp = f"{f.timestamp:.6f}"
        png(folder / "rgb" / f"{stamp}.png", f.gray)
        scaled = np.clip(np.round(f.depth * tum.DEPTH_SCALE), 0, np.iinfo(np.uint16).max)
        png(folder / "depth" / f"{stamp}.png", scaled.astype(np.uint16))
        rgb.append((stamp, f"rgb/{stamp}.png"))
        depth.append((stamp, f"depth/{stamp}.png"))
    tum.write_list(folder / "rgb.txt", rgb)
    tum.write_list(folder / "depth.txt", depth)
    runs.write_trajectory(
        folder / "groundtruth.txt",
        [f.timestamp for f in rendered],
        np.array([f.T_world_cam for f in rendered]),
    )
    intrinsics = np.array([camera.fx, camera.fy, camera.cx, camera.cy])
    tum.write_camera(
        folder,
        tum.Camera(
            "pinhole", intrinsics, camera.width, camera.height, tum.DEPTH_SCALE, tum.SENSOR_BF
        ),
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
"""The frames the gap room drops: a jump the motion model, narrowed, cannot bridge. The
recovery frame's velocity then spans the gap, so the frame after it falls back too: the gap
room recovers twice."""

BLANK_AT = 10
"""Where the gap room has a blank frame (no texture, no depth), before the gap: a frame that
fails, after which the motion model predicts no motion (review panel finding A14)."""

GAP_OPTIONS = replace(OPTIONS, motion_window=2.0, stereo_motion_window=2.0, max_frames=2)
"""The room's settings with the motion model's window narrowed: across the gap, tracking the
last frame fails and the pipeline recovers by PnP against the local map. ``max_frames`` is
lowered below the keyframes the map holds, so the hold on keyframes just after a recovery
decides (monocular)."""


MOVED_OPTIONS = replace(
    GAP_OPTIONS,
    th_low=45,
    th_high=90,
    stereo_max_distance=70,
    stereo_band=2.5,
    stereo_window=4,
    stereo_search=6,
    stereo_outlier_factor=2.5,
    min_motion_inliers=12,
    aligned_view_cos=0.995,
    aligned_view_radius=2.0,
    oblique_view_radius=5.0,
    local_map_ratio=0.75,
    min_distance_margin=0.7,
    max_distance_margin=1.3,
    first_ref_ratio=0.5,
    ref_min_observations=2,
    ref_min_observations_early=1,
    triangulation_neighbours=5,
    mono_triangulation_neighbours=8,
    epipolar_chi2=5.0,
    min_epipole_distance2=80.0,
    min_baseline_depth=0.02,
    recent_point_keyframes=4,
    point_cull_age=3,
    init_ba_iterations=15,
    init_parallax_cos=0.99995,
    relocalize_ratio=0.8,
    recovery_window_frames=3,
    recovery_tracked=40,
    recovery_local_window=4.0,
    ba_min_keyframes=4,
    min_init_depth_points=150,
)
"""The gap room's settings with every one the review panel's fixes added moved off its default
(finding H21): a native that read one at its default, or hard-coded it, would decide unlike
Python here, where at the defaults it could not."""


def gap_frames(rendered: list[room.Frame]) -> list[room.Frame]:
    """Return the gap room: the room without :data:`GAP`'s frames, and a blank frame.

    Parameters
    ----------
    rendered : list of tests.edge.room.Frame
        The room's frames.

    Returns
    -------
    list of tests.edge.room.Frame
        The gap room's frames, the blank one halfway in time between its neighbours.
    """
    kept = rendered[: GAP[0]] + rendered[GAP[1] :]
    before, after = kept[BLANK_AT - 1], kept[BLANK_AT]
    between = replace(blank(after), timestamp=(before.timestamp + after.timestamp) / 2)
    return [*kept[:BLANK_AT], between, *kept[BLANK_AT:]]


def blank(frame: room.Frame) -> room.Frame:
    """Return a frame blanked: its images and depth zero, nothing in them to track."""
    return replace(
        frame,
        gray=np.zeros_like(frame.gray),
        right=None if frame.right is None else np.zeros_like(frame.right),
        depth=np.zeros_like(frame.depth),
    )


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


def room_inputs() -> dict[str, Any]:
    """Return what the golden record's runs depend on besides the pipeline.

    Returns
    -------
    dict
        The frames rendered, the gap, the blank frame, and each room's settings off their
        defaults: recorded with the runs and checked first, so a change to the rooms reads as
        one, not as a change to what the pipeline decides.
    """
    defaults = Options().as_dict()

    def moved(options: Options) -> dict[str, Any]:
        return {k: v for k, v in options.as_dict().items() if v != defaults[k]}

    return {
        "frames": FRAMES,
        "gap": list(GAP),
        "blank_at": BLANK_AT,
        "room": moved(OPTIONS),
        "gap_room": moved(GAP_OPTIONS),
    }
