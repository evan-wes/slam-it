"""The synthetic room as dataset folders: TUM-format and EuRoC, whole or with a gap.

The room (:mod:`tests.edge.room`), rendered with its right views, is written as a TUM-format
folder (gray PNGs, 16-bit depth PNGs, ``rgb.txt``, ``depth.txt`` and ``camera.txt``, with
ORB-SLAM's TUM ``bf``) and as a EuRoC sequence (``mav0/cam0`` and ``cam1``: PNGs,
``data.csv`` and ``sensor.yaml``, the right camera :data:`tests.edge.room.BASELINE` to the
left one's right, no distortion). The gap room drops the frames of :data:`GAP`. The loop room
follows :func:`tests.edge.room.loop_path` instead, back over where it began
(:func:`loop_frames`); the merge room is the loop room blinded long enough to start a new map
(:func:`merge_frames`). The tests' fixtures (``tests/orchestration/conftest.py``) and the golden
records' generator (``tests/fixtures/edge/generate_room_runs.py``) both write them from here.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from PIL import Image

from slam_it.edge.options import Options
from slam_it.eval import runs, tum
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

GAP_OPTIONS = OPTIONS.with_settings(
    {"tracking.motion_window": 2.0, "tracking.stereo_motion_window": 2.0, "tracking.max_frames": 2}
)
"""The room's settings with the motion model's window narrowed: across the gap, tracking the
last frame fails and the pipeline recovers by PnP against the local map. ``tracking.max_frames`` is
lowered below the keyframes the map holds, so the hold on keyframes just after a recovery
decides (monocular)."""


MOVED_OPTIONS = GAP_OPTIONS.with_settings(
    {
        "matching.th_low": 45,
        "matching.th_high": 90,
        "stereo.max_distance": 70,
        "stereo.band": 2.5,
        "stereo.window": 4,
        "stereo.search": 6,
        "stereo.outlier_factor": 2.5,
        "tracking.min_motion_inliers": 12,
        "tracking.aligned_view_cos": 0.995,
        "tracking.aligned_view_radius": 2.0,
        "tracking.oblique_view_radius": 5.0,
        "tracking.local_map_ratio": 0.75,
        "tracking.min_distance_margin": 0.7,
        "tracking.max_distance_margin": 1.3,
        "keyframes.first_ref_ratio": 0.5,
        "keyframes.ref_min_observations": 2,
        "keyframes.ref_min_observations_early": 1,
        "mapping.triangulation_neighbours": 5,
        "mapping.mono_triangulation_neighbours": 8,
        "mapping.epipolar_chi2": 5.0,
        "mapping.min_epipole_distance2": 80.0,
        "mapping.min_baseline_depth": 0.02,
        "mapping.recent_point_keyframes": 4,
        "mapping.point_cull_age": 3,
        "initialization.ba_iterations": 15,
        "initialization.ransac_iterations": 150,
        "initialization.parallax_cos": 0.99995,
        "relocalization.ratio": 0.8,
        "tracking.recovery_window_frames": 3,
        "tracking.recovery_tracked": 40,
        "tracking.recovery_local_window": 4.0,
        "adjustment.min_keyframes": 4,
        "loss.reset_keyframes": 9,
        "mapping.fuse_neighbours": 2,
        "mapping.mono_fuse_neighbours": 2,
        "mapping.fuse_second_neighbours": 1,
        "covisibility.min_points": 100,
        "covisibility.strong_points": 150,
        "covisibility.local_keyframes": 1,
        "covisibility.local_neighbours": 1,
    }
)
"""The gap room's settings with every one the review panel's fixes added moved off its default
(finding H21): a native that read one at its default, or hard-coded it, would decide unlike
Python here, where at the defaults it could not. B2's settings (its panel's H3): each of fusion's
three counts, ``covisibility.min_points`` and ``covisibility.strong_points``, put back at its
default with the rest moved, changes the room's decisions; the local map's two caps cannot, as
the keyframes that see a frame's points are every keyframe the room's local map could take, so
the covisibility fixture's capped local maps hold them in the three languages."""


LOST_AT = (12, 31)
"""Where the lost room's two blank stretches begin: ``loss.lost_frames`` of the room's frames from
each are blanked, so the track is lost on the last of each and the frames after start a new
map. A stereo or RGB-D run reaches a third map, whose ids follow the first two maps' (review
panel finding L4); a monocular one starts no map in the frames left after the second. The
second stretch cuts a monocular run's map 1, started on frame 28, to 3 tracked frames, where
it tracked 12 to the room's end with one stretch (finding L10): the room's 40 frames leave no
later place for it."""

KEPT_OPTIONS = OPTIONS.with_settings(
    {"loss.reset_keyframes": 0, "relocalization.enabled": False, "loops.enabled": False}
)
"""The room's settings with every lost map kept: the lost room's lost maps, of a few keyframes
each, are otherwise reset (ORB-SLAM3's reset of a map lost with 10 keyframes or fewer). Without
relocalisation, so that a kept map is lost after ``loss.lost_frames`` failures, as before
milestone 2B: relocalising, a map above ``loss.reset_keyframes`` is lost only after
``loss.relocalize_seconds``, longer than the room. Without loop closing too, so that the run
keeps no keyframe database, as it kept none before B3's review panel (H7) kept one for loop
closing alone."""


FOUND_AT = 31
"""Where the found room's blank frames begin: the camera, blinded for :data:`FOUND_BLANKS`
frames, comes back where it began, at frame :data:`FOUND_BACK`, and goes on from there."""

FOUND_BLANKS = 3
"""The found room's blank frames: fewer than ``loss.lost_frames``, so the track is not lost."""

FOUND_BACK = 2
"""The rendered frame the found room's camera comes back to after its blank frames."""

FOUND_OPTIONS = OPTIONS.with_settings({"tracking.pnp_min_inliers": 1 << 20})
"""The found room's settings: the local fallback out of reach, so that only relocalisation,
through the keyframe database, can find the frame the camera comes back on (the room's map is
small enough for the fallback's local map to hold all of it)."""


WINDOW_BLANKS = 2
"""The window room's first blank frames, from :data:`FOUND_AT`: one fewer than the found room's.
The second, 0.033 s after the first, is within the room's 0.05 s ``loss.relocalize_seconds``, so the
track is not lost on it; the frame after, back at :data:`FOUND_BACK` and 0.067 s after the first,
is relocalised, because the time is judged only on a frame that every attempt fails, as
ORB-SLAM3 judges it after ``Relocalization``. A native that judged it first would lose the map
there."""

WINDOW_AT = 36
"""Where the window room's second blank stretch begins: :data:`FOUND_BLANKS` frames, longer than
its ``loss.relocalize_seconds`` and shorter than ``loss.lost_frames``, so that its map, kept, is
lost by the time on the last of them, and the frame after starts a new one."""

WINDOW_OPTIONS = FOUND_OPTIONS.with_settings(
    {
        "loss.reset_keyframes": 0,
        "loss.relocalize_seconds": 0.05,
        "mapping.redundant_fraction": 0.6,
        "mapping.redundant_observers": 1,
        "tracking.recovery_tracked": 60,
        "relocalization.inlier_ratio": 0.4,
        "relocalization.min_inliers": 12,
        "relocalization.confidence": 0.95,
        "relocalization.ransac_iterations": 200,
        "relocalization.window": 2.0,
        "relocalization.narrow_inliers": 25,
        "relocalization.narrow_window": 1.0,
        "relocalization.narrow_distance": 30,
    }
)
"""The window room's settings (review panel findings L1 and L2): the found room's, every map
kept; lost 0.05 s after its first failed frame, on the third frame at 30 Hz, where
``loss.lost_frames`` would lose it on the fifth; a keyframe culled when 60% of its points are seen
by one other keyframe, so that an RGB-D run culls several, which leave its database, before the
frame is relocalised; ``tracking.recovery_tracked`` above the relocalised frame's first inliers, so
that the search by projection runs; and the relocalisation settings off their defaults. Here only
the search's window and RANSAC's confidence decide (review panel finding L9): the stage parity's
cases (``tests/edge/test_relocalization.py``'s ``DECISIVE``) move each setting where it alone
decides."""


def window_frames(rendered: list[room.Frame]) -> list[room.Frame]:
    """Return the window room: the found room with two blank stretches.

    Parameters
    ----------
    rendered : list of tests.edge.room.Frame
        The room's frames.

    Returns
    -------
    list of tests.edge.room.Frame
        As many frames, at the room's times: frames 0 to ``FOUND_AT - 1``, :data:`WINDOW_BLANKS`
        blank ones, then the frames from :data:`FOUND_BACK` on, of which the
        :data:`FOUND_BLANKS` from :data:`WINDOW_AT` are blanked.
    """
    after = len(rendered) - FOUND_AT - WINDOW_BLANKS
    order: list[int | None] = [
        *range(FOUND_AT), *([None] * WINDOW_BLANKS), *range(FOUND_BACK, FOUND_BACK + after)
    ]  # fmt: skip
    for k in range(WINDOW_AT, WINDOW_AT + FOUND_BLANKS):
        order[k] = None
    return [
        replace(
            blank(rendered[FOUND_AT - 1]) if source is None else rendered[source],
            timestamp=at.timestamp,
        )
        for source, at in zip(order, rendered, strict=True)
    ]


def found_frames(rendered: list[room.Frame]) -> list[room.Frame]:
    """Return the found room: blank frames from :data:`FOUND_AT`, then the start seen again.

    Parameters
    ----------
    rendered : list of tests.edge.room.Frame
        The room's frames.

    Returns
    -------
    list of tests.edge.room.Frame
        As many frames, at the room's times: frames 0 to ``FOUND_AT - 1``, :data:`FOUND_BLANKS`
        blank ones, then the frames from :data:`FOUND_BACK` on, each at the time of its place.
    """
    after = len(rendered) - FOUND_AT - FOUND_BLANKS
    order = [*range(FOUND_AT), *([None] * FOUND_BLANKS), *range(FOUND_BACK, FOUND_BACK + after)]
    return [
        replace(
            blank(rendered[FOUND_AT - 1]) if source is None else rendered[source],
            timestamp=at.timestamp,
        )
        for source, at in zip(order, rendered, strict=True)
    ]


def lost_frames(rendered: list[room.Frame]) -> list[room.Frame]:
    """Return the lost room: ``loss.lost_frames`` frames blanked from each :data:`LOST_AT`.

    Parameters
    ----------
    rendered : list of tests.edge.room.Frame
        The room's frames.

    Returns
    -------
    list of tests.edge.room.Frame
        The lost room's frames, as many as the room's, at the same times.
    """
    blanked = {at + k for at in LOST_AT for k in range(OPTIONS.loss.lost_frames)}
    return [blank(f) if k in blanked else f for k, f in enumerate(rendered)]


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

LOOP_CASES = [("tum", "rgbd"), ("tum", "monocular"), ("euroc", "stereo")]
"""The loop room's cases: each mode once (its monocular case alone in TUM's folder, Python's
monocular run being the longest the tests make)."""


def loop_frames() -> list[room.Frame]:
    """Render the loop room with its right views: :data:`tests.edge.room.LOOP_FRAMES` frames.

    Returns
    -------
    list of tests.edge.room.Frame
        The frames.
    """
    return room.sequence(room.LOOP_FRAMES, stereo=True, loop=True)


MERGE_AT = 60
"""Where the merge room's black frames begin: the loop room's camera about a quarter of the way
round (90 degrees of its turn)."""

MERGE_BLANKS = 96
"""The merge room's black frames: 3.2 s at the room's 30 Hz, longer than the 3 s
``loss.relocalize_seconds`` relocalises a lost map for, meant to lose the track and start a new
map on the frames after, which begin about 235 degrees round; the camera comes back over the
first map's start at 360 degrees. Its bar, with merging in the pipeline, is milestone 2B's B4's
second phase, which shows whether a run does so."""


def merge_frames(rendered: list[room.Frame]) -> list[room.Frame]:
    """Return the merge room: the loop room, its frames from :data:`MERGE_AT` blank.

    :data:`MERGE_BLANKS` of them.

    Parameters
    ----------
    rendered : list of tests.edge.room.Frame
        The loop room's frames (:func:`loop_frames`).

    Returns
    -------
    list of tests.edge.room.Frame
        As many frames, at the same times, the black ones blank.
    """
    black = range(MERGE_AT, MERGE_AT + MERGE_BLANKS)
    return [blank(f) if k in black else f for k, f in enumerate(rendered)]


def room_inputs() -> dict[str, Any]:
    """Return what the golden record's runs depend on besides the pipeline.

    Returns
    -------
    dict
        The frames rendered, the gap, the blank frame, where the lost room's blank frames begin
        and how many, the loop room's frames, radius, turn and pitch, and each room's settings
        off their defaults: recorded with the runs and
        checked first, so a change to the rooms reads as one, not as a change to what the
        pipeline decides.
    """
    defaults = Options().settings()

    def moved(options: Options) -> dict[str, Any]:
        return {k: v for k, v in options.settings().items() if v != defaults[k]}

    return {
        "frames": FRAMES,
        "gap": list(GAP),
        "blank_at": BLANK_AT,
        "lost_at": list(LOST_AT),
        "lost_frames": OPTIONS.loss.lost_frames,
        "room": moved(OPTIONS),
        "gap_room": moved(GAP_OPTIONS),
        "kept_room": moved(KEPT_OPTIONS),
        "found": [FOUND_AT, FOUND_BLANKS, FOUND_BACK],
        "found_room": moved(FOUND_OPTIONS),
        "window": [WINDOW_BLANKS, WINDOW_AT],
        "window_room": moved(WINDOW_OPTIONS),
        "loop": [room.LOOP_FRAMES, room.LOOP_RADIUS, room.LOOP_TURN, room.LOOP_PITCH],
    }
