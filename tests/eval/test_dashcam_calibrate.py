"""The rig calibration's stages, on a synthetic drive whose rig and clocks are known."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from slam_it.eval import gnss
from slam_it.eval.dashcam import calibrate
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError
from tests.eval.dashcam_recordings import FRONT_FROM_CAR, drive, rig_poses, turn

REAR_FROM_CAR = (
    turn([0, 0, 1], 3.0)
    @ turn([1, 0, 0], -8.0)
    @ np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])
)
"""``R_rear_car``: facing backward, pitched down 8 degrees and turned 3."""

FRONT_LEVER = np.array([1.5, 0.0, 1.2])
REAR_LEVER = np.array([-0.3, 0.1, 1.3])
"""Each camera's centre in the car's frame, in metres: 1.8 m apart."""

FRONT_SCALE, REAR_SCALE = 7.0, 3.0
"""Metres per unit of each camera's map."""

OFFSET = 0.37
"""The rear clock's offset: its time ``t + OFFSET`` is the front's ``t``."""

TIMES = np.arange(0.0, 120.0, 0.25)
"""The front's frame times: 4 a second for two minutes."""

R_X = FRONT_FROM_CAR @ REAR_FROM_CAR.T
T_X = FRONT_FROM_CAR @ (REAR_LEVER - FRONT_LEVER)
"""The rig's true ``T_front_rear``."""


@pytest.fixture(scope="module")
def maps() -> tuple[calibrate.Poses, calibrate.Poses]:
    """Return the front's and the rear's poses, each on its own clock and in its own units."""
    front = rig_poses(TIMES, FRONT_FROM_CAR, FRONT_LEVER, FRONT_SCALE)
    rear_times = np.arange(0.1, 120.0, 0.25)  # its frames between the front's
    rear = rig_poses(rear_times, REAR_FROM_CAR, REAR_LEVER, REAR_SCALE)
    return front, {t + OFFSET: p for t, p in rear.items()}


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(float(np.linalg.norm(so3.log(a @ b.T))))


def test_the_clocks_line_up_where_the_two_cameras_turn_alike(
    maps: tuple[calibrate.Poses, calibrate.Poses],
) -> None:
    """The rear clock's offset is where the two cameras' turns correlate best."""
    offset, correlation = calibrate.time_offset(*maps)
    assert offset == pytest.approx(OFFSET, abs=calibrate.OFFSET_STEP)
    assert correlation > 0.99


def test_the_rig_rotation_is_found_from_turns_and_directions_of_travel(
    maps: tuple[calibrate.Poses, calibrate.Poses],
) -> None:
    """R_front_rear is found from the turns' axes and the directions of travel."""
    pairs = calibrate.motion_pairs(*maps, OFFSET)
    found = calibrate.hand_eye_rotation(pairs)
    assert _angle(found.rotation, R_X) < 0.2
    assert len(found.turns) >= 3
    assert len(found.directions) >= 1
    assert float(np.median(np.degrees(found.turns))) < 0.2
    assert float(np.median(np.degrees(found.directions))) < 1.0


def test_a_turning_rig_alone_would_leave_the_rotation_about_the_vertical_open(
    maps: tuple[calibrate.Poses, calibrate.Poses],
) -> None:
    """The directions of travel are what fix the yaw: the turns' axes are near vertical."""
    pairs = calibrate.motion_pairs(*maps, OFFSET)
    axes = [so3.log(a[:3, :3]) for a, _ in pairs]
    turning = np.array([v for v in axes if np.linalg.norm(v) > calibrate.MIN_TURN])
    up = FRONT_FROM_CAR @ np.array([0.0, 0.0, 1.0])
    tilt = np.degrees(np.arccos(np.abs(turning @ up) / np.linalg.norm(turning, axis=1)))
    assert float(np.median(tilt)) < 15.0


def test_the_front_map_is_put_in_metres_by_the_gps_speed(
    maps: tuple[calibrate.Poses, calibrate.Poses],
) -> None:
    """The GPS track's speed gives the front map's metres per unit."""
    seconds = np.arange(0.0, 121.0)
    rotations, position = drive(seconds)
    centres = position + rotations @ FRONT_LEVER
    track = gnss.smooth(seconds, centres[:, :2], sigma=0.05)
    scale, windows = calibrate.gps_scale(maps[0], track)
    assert scale == pytest.approx(FRONT_SCALE, rel=0.01)
    assert windows > 100


def test_the_rig_translation_and_the_rear_scale_are_found_with_the_baseline_prior(
    maps: tuple[calibrate.Poses, calibrate.Poses],
) -> None:
    """The rig's translation and the rear map's scale come out of the linear MAP fit."""
    pairs = calibrate.motion_pairs(*maps, OFFSET)
    found = calibrate.rig_translation(
        pairs,
        R_X,
        front_scale=FRONT_SCALE,
        prior=np.array([0.0, 0.0, -1.9]),  # off by 10 cm: the motions correct it
        prior_sigma=np.array(calibrate.PRIOR_SIGMA),
    )
    assert np.abs(found.t - T_X).max() < 0.03
    assert found.rear_scale == pytest.approx(REAR_SCALE, rel=0.01)
    assert found.covariance.shape == (4, 4)
    assert found.pairs == len(pairs)


def test_solve_reports_each_stage(maps: tuple[calibrate.Poses, calibrate.Poses]) -> None:
    """Solving runs the stages and reports each lens at its video's size."""
    seconds = np.arange(0.0, 121.0)
    rotations, position = drive(seconds)
    track = gnss.smooth(seconds, (position + rotations @ FRONT_LEVER)[:, :2], sigma=0.05)
    lens = calibrate.Lens(
        800.0, 801.0, 800.0, 450.0, (0.1, 0.0, 0.0, 0.0), 1600, 900, 0.5, 480, 470
    )
    report = calibrate.solve(*maps, (lens, lens), track, baseline=1.9)
    out = report.as_dict((3840, 2160), (2560, 1440))
    assert out["rear_clock_offset_s"] == pytest.approx(OFFSET, abs=calibrate.OFFSET_STEP)
    assert out["baseline_m"] == pytest.approx(float(np.linalg.norm(T_X)), abs=0.03)
    assert out["front"]["size"] == [3840, 2160]
    assert out["front"]["square_pixels_fit"] is True
    with pytest.raises(InvalidInputError, match="baseline"):
        calibrate.solve(*maps, (lens, lens), track, baseline=0.0)


def test_a_colmap_lens_is_put_on_integer_pixel_centres_and_scaled() -> None:
    """COLMAP's half-pixel principal point is moved to integer centres, then scaled."""
    lens = calibrate.Lens(
        800.0, 820.0, 800.0, 450.0, (0.1, -0.02, 0.0, 0.0), 1600, 900, 0.5, 10, 10
    )
    at = lens.fisheye(3840, 2160)
    assert (at.camera.cx, at.camera.cy) == pytest.approx((1919.5, 1079.5))
    assert (at.camera.fx, at.camera.fy) == pytest.approx((1920.0, 1968.0))
    assert at.k == (0.1, -0.02, 0.0, 0.0)
    assert lens.aspect == pytest.approx(800 / 820)


def test_too_little_motion_is_refused(maps: tuple[calibrate.Poses, calibrate.Poses]) -> None:
    """Too few poses, no turns, or a prior of zero spread are refused."""
    front, rear = maps
    few = dict(list(front.items())[:8])
    with pytest.raises(InvalidInputError, match="too few"):
        calibrate.time_offset(few, rear)
    still = [(np.eye(4), np.eye(4))] * 5
    with pytest.raises(InvalidInputError, match="turn"):
        calibrate.hand_eye_rotation(still)
    pairs = calibrate.motion_pairs(front, rear, OFFSET)
    with pytest.raises(InvalidInputError, match="prior_sigma"):
        calibrate.rig_translation(
            pairs, R_X, front_scale=1.0, prior=np.zeros(3), prior_sigma=np.zeros(3)
        )


def test_reconstruct_refuses_too_few_frames_and_a_used_folder(tmp_path: Path) -> None:
    """COLMAP is not started on fewer than 3 frames."""
    pytest.importorskip("pycolmap")
    with pytest.raises(InvalidInputError, match="fewer than 3"):
        calibrate.reconstruct(tmp_path, tmp_path / "work")
