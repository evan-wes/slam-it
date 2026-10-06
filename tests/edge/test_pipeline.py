"""The pipeline's tests: the room in every mode, start to end, and the pipeline's rules."""

from __future__ import annotations

import numpy as np
import pytest

from slamlab.edge import frames, pipeline
from slamlab.edge.options import Options
from slamlab.eval.metrics import absolute_error
from slamlab.foundations import orb
from slamlab.foundations.errors import InvalidInputError
from tests.edge import room
from tests.edge.conftest import CAMERA, OPTIONS, Run

ATE_BOUND = {"rgbd": 0.02, "stereo": 0.02, "monocular": 0.03}
"""The room's ATE (RMSE, metres) each mode stays under: about half again its measured value
(1.2, 1.4 and 2.0 cm; monocular aligned with a similarity)."""

MONOCULAR_START = 10
"""The monocular map starts within this many frames."""


@pytest.mark.parametrize("mode", ["monocular", "stereo", "rgbd"])
def test_the_room_is_tracked_to_the_end(mode: str, runs: dict[str, Run]) -> None:
    """Every frame after the start is tracked, and the trajectory is close to the truth."""
    run = runs[mode]
    first = next(i for i, r in enumerate(run.results) if r.pose is not None)
    assert first == 0 if mode != "monocular" else first < MONOCULAR_START
    assert all(r.pose is not None for r in run.results[first:])
    assert run.pipeline.state is pipeline.State.TRACKING
    assert run.results[first].keyframe
    estimate = np.array([np.linalg.inv(r.pose) for r in run.results[first:] if r.pose is not None])
    error = absolute_error(estimate, np.array(run.truth[first:]), with_scale=mode == "monocular")
    assert error.rmse < ATE_BOUND[mode]
    assert run.pipeline.map is not None
    assert run.pipeline.map.keyframes >= 2


def test_monocular_frames_start_with_more_features() -> None:
    """Before the map, a monocular frame gets init_feature_factor times the features."""
    camera = frames.Camera("pinhole", room.SMALL.intrinsics, 320, 240)
    mono = pipeline.Pipeline("monocular", camera, OPTIONS)
    assert mono.extraction().n_features == OPTIONS.init_feature_factor * OPTIONS.n_features
    depth = pipeline.Pipeline("rgbd", CAMERA, OPTIONS)
    assert depth.extraction().n_features == OPTIONS.n_features


def _empty() -> frames.Frame:
    """Return a frame with no keypoints."""
    features = orb.Features(
        orb.Keypoints(
            xy=np.zeros((0, 2)),
            level=np.zeros(0, dtype=np.int64),
            level_xy=np.zeros((0, 2), dtype=np.int64),
            response=np.zeros(0, dtype=np.int64),
            angle=np.zeros(0, dtype=np.int64),
        ),
        np.zeros((0, orb.DESCRIPTOR_BYTES), dtype=np.uint8),
        scale_factor=OPTIONS.scale_factor,
        levels=OPTIONS.levels,
    )
    return frames.Frame(0.0, features, np.zeros((0, 3)), np.zeros(0), np.zeros(0))


def test_empty_frames_lose_the_track(rendered: list[room.Frame]) -> None:
    """lost_frames frames in a row untracked lose the track; the run then ends."""
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS)
    first = rendered[0]
    started = run.track(frames.rgbd(first.gray, first.depth, 0.0, CAMERA, run.extraction()))
    assert started.state is pipeline.State.TRACKING
    for _ in range(OPTIONS.lost_frames):
        assert run.track(_empty()).pose is None
    assert run.state is pipeline.State.LOST
    again = run.track(frames.rgbd(first.gray, first.depth, 1.0, CAMERA, run.extraction()))
    assert again.pose is None
    assert again.state is pipeline.State.LOST


def test_a_monocular_reference_needs_enough_features(rendered: list[room.Frame]) -> None:
    """A frame with too few features drops the monocular reference rather than pairing."""
    camera = frames.Camera("pinhole", room.SMALL.intrinsics, 320, 240)
    run = pipeline.Pipeline("monocular", camera, OPTIONS)
    first = rendered[0]
    run.track(frames.monocular(first.gray, 0.0, camera, run.extraction()))
    run.track(_empty())
    assert run._reference is None


@pytest.mark.parametrize(
    ("mode", "baseline", "message"),
    [("lidar", 0.1, "mode"), ("rgbd", 0.0, "baseline"), ("stereo", 0.0, "baseline")],
)
def test_rejects_a_bad_mode_or_baseline(mode: str, baseline: float, message: str) -> None:
    """An unknown mode, or a depth mode without a baseline, is refused."""
    camera = frames.Camera("pinhole", room.SMALL.intrinsics, 320, 240, baseline)
    with pytest.raises(InvalidInputError, match=message):
        pipeline.Pipeline(mode, camera, Options())
