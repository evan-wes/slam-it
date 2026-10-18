"""The GNSS track: the local frame, the gated filter and smoother, and planar scoring."""

from __future__ import annotations

import math

import numpy as np
import pytest

from slam_it.eval import gnss, runs
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError


def test_degrees_become_metres_east_and_north() -> None:
    """Latitude and longitude become metres east and north of the origin."""
    out = gnss.local_frame([0.0, 0.001, 0.0], [0.0, 0.0, 0.001], origin=(0.0, 0.0))
    metres = gnss.EARTH_RADIUS * math.radians(0.001)
    assert out == pytest.approx(np.array([[0.0, 0.0], [0.0, metres], [metres, 0.0]]))
    # away from the equator a degree of longitude is shorter by the latitude's cosine
    north = gnss.local_frame([60.0, 60.0], [10.0, 10.001])
    assert north[1, 0] == pytest.approx(metres * 0.5, rel=1e-6)
    with pytest.raises(InvalidInputError, match="range"):
        gnss.local_frame([91.0], [0.0])


def _straight(n: int, noise: float, seed: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times = np.arange(n, dtype=np.float64)
    truth = np.stack([10.0 * times, 4.0 * times], axis=1)
    rng = np.random.default_rng(seed)
    return times, truth, truth + rng.normal(scale=noise, size=truth.shape)


def test_the_smoother_beats_the_fixes_and_refuses_a_misread() -> None:
    """The smoothed track is closer than the fixes, and a misread fix is refused."""
    times, truth, fixes = _straight(60, 3.0)
    fixes[30] += [150.0, -80.0]  # a misread digit
    track = gnss.smooth(times, fixes, sigma=3.0)
    assert track.refused[30]
    assert int(track.refused.sum()) == 1
    error = np.linalg.norm(track.position - truth, axis=1)
    raw = np.linalg.norm(np.delete(fixes, 30, axis=0) - np.delete(truth, 30, axis=0), axis=1)
    assert float(np.sqrt(np.mean(error**2))) < 0.6 * float(np.sqrt(np.mean(raw**2)))
    assert track.velocity[30] == pytest.approx([10.0, 4.0], abs=2.0)  # 3 m fixes, 1 a second


def test_a_misread_first_fix_does_not_start_the_filter() -> None:
    """A misread first fix is refused: the start is fitted to the first few."""
    times, truth, fixes = _straight(20, 0.5)
    fixes[0] += [500.0, 500.0]
    track = gnss.smooth(times, fixes, sigma=0.5)
    assert track.refused[0]
    assert np.linalg.norm(track.position[5:] - truth[5:], axis=1).max() < 2.0


def test_fixes_refused_again_and_again_mean_the_filter_lost_the_car() -> None:
    """Fixes refused RESTART times in a row are taken: the filter, not they, was wrong."""
    times, truth, fixes = _straight(40, 0.5)
    fixes[20:] += [300.0, 0.0]  # the fixes jump and stay: it is the filter that is wrong
    track = gnss.smooth(times, fixes, sigma=0.5)
    assert track.refused[20 : 20 + gnss.RESTART - 1].all()
    assert not track.refused[20 + gnss.RESTART - 1 :].any()
    after = 20 + gnss.RESTART + 2
    assert np.linalg.norm(track.position[after:] - fixes[after:], axis=1).max() < 2.0
    assert np.linalg.norm(track.position[:18] - truth[:18], axis=1).max() < 2.0


def test_the_track_is_interpolated_within_its_span_only() -> None:
    """The track interpolates within its span and refuses times outside it."""
    times, truth, _ = _straight(10, 0.0)
    track = gnss.smooth(times, truth, sigma=0.1)
    assert track.at([2.5]) == pytest.approx(np.array([[25.0, 10.0]]), abs=0.05)
    with pytest.raises(InvalidInputError, match="outside"):
        track.at([20.0])


@pytest.mark.parametrize(
    "bad",
    [
        {"times": [0.0, 0.0], "position": [[0, 0], [1, 1]]},
        {"times": [0.0], "position": [[0, 0]]},
        {"times": [0.0, 1.0], "position": [[0, 0], [1, 1]], "sigma": 0.0},
    ],
)
def test_bad_fixes_are_refused(bad: dict[str, object]) -> None:
    """Times that do not increase, a single fix, or a zero sigma are refused."""
    with pytest.raises(InvalidInputError):
        gnss.smooth(**bad)  # type: ignore[arg-type]


def test_a_planar_similarity_is_recovered() -> None:
    """Umeyama's 2D similarity is recovered exactly; points that do not spread are refused."""
    rng = np.random.default_rng(3)
    source = rng.normal(size=(20, 2))
    angle = 0.7
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    target = 2.5 * source @ rotation.T + [3.0, -1.0]
    scale, found, shift = gnss.align_planar(source, target)
    assert scale == pytest.approx(2.5)
    assert found == pytest.approx(rotation)
    assert shift == pytest.approx([3.0, -1.0])
    with pytest.raises(InvalidInputError, match="spread"):
        gnss.align_planar(np.zeros((3, 2)), target[:3])


def test_a_run_is_scored_in_its_plane_of_motion_in_metres() -> None:
    """A map tilted out of level, scaled and mirrored scores near zero against its own track."""
    times = np.arange(0.0, 60.0, 0.5)
    east = 40.0 * np.cos(times / 10.0)
    north = 25.0 * np.sin(times / 7.0)
    track = gnss.smooth(times, np.stack([east, north], axis=1), sigma=0.01, accel=5.0)
    flat = np.stack([east, -north, np.zeros_like(times)], axis=1) / 5.0  # mirrored, 5 m a unit
    tilt = so3.exp(np.array([0.3, -0.2, 1.0]))
    poses = np.repeat(np.eye(4)[None], len(times), axis=0)
    poses[:, :3, 3] = flat @ tilt.T + [1.0, 2.0, 3.0]
    run = runs.Run(
        "dashcam", "drive", "monocular", "python", times, poses,
        pose_maps=np.zeros(len(times), dtype=np.int64),
    )  # fmt: skip
    (scored,) = gnss.score(run, track, times)
    assert scored.frames == len(times)
    assert scored.scale == pytest.approx(5.0, rel=1e-3)
    assert scored.rmse < 0.1
    assert scored.flatness < 1e-6
    with pytest.raises(InvalidInputError, match="clock"):
        gnss.score(run, track, times[:-1])
