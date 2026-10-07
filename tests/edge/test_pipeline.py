"""The pipeline's tests: the room in every mode, start to end, and the pipeline's rules."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab.edge import frames, pipeline, tracking
from slamlab.edge.options import Options
from slamlab.eval.metrics import absolute_error
from slamlab.foundations import orb
from slamlab.foundations.errors import InvalidInputError
from tests.edge import room
from tests.edge.conftest import ATE_BOUND, CAMERA, MONOCULAR_START, OPTIONS, Run


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


def test_a_failed_frame_ends_the_velocity(
    rendered: list[room.Frame], monkeypatch: pytest.MonkeyPatch
) -> None:
    """The frame after one that was not tracked is predicted with no motion."""
    velocities: list[NDArray[np.float64] | None] = []
    predict = tracking.predict

    def recording(
        last_pose: NDArray[np.float64], velocity: NDArray[np.float64] | None
    ) -> NDArray[np.float64]:
        velocities.append(velocity)
        return predict(last_pose, velocity)

    monkeypatch.setattr(tracking, "predict", recording)
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS)
    for f in rendered[:4]:
        run.track(frames.rgbd(f.gray, f.depth, f.timestamp, CAMERA, run.extraction()))
        if f is rendered[2]:
            assert run.track(_empty()).pose is None
            assert velocities[-1] is not None  # the failed frame was predicted with motion
    assert velocities[-1] is None


def test_a_frame_with_little_depth_waits_for_the_next(rendered: list[room.Frame]) -> None:
    """A first frame with almost no depth starts no map: the next one does (finding L13).

    ORB-SLAM starts from any 500 keypoints and resets a map lost within five keyframes; the
    lite pipeline has no reset, so a start from a handful of depths would end the run.
    """
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS)
    first, second = rendered[0], rendered[1]
    sparse = np.zeros_like(first.depth)
    sparse[::40, ::40] = first.depth[::40, ::40]  # a few dozen pixels of depth
    waited = run.track(frames.rgbd(first.gray, sparse, 0.0, CAMERA, run.extraction()))
    assert waited.pose is None
    assert run.state is pipeline.State.NOT_INITIALIZED
    started = run.track(frames.rgbd(second.gray, second.depth, 1.0, CAMERA, run.extraction()))
    assert started.state is pipeline.State.TRACKING
    assert run.map is not None
    assert run.map.points >= OPTIONS.min_init_depth_points


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
