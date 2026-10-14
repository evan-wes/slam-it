"""The pipeline's tests: the room in every mode, start to end, and the pipeline's rules."""

from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it.edge import frames, pipeline, tracking
from slam_it.edge.initialization import Outcome
from slam_it.edge.map import Removal
from slam_it.edge.options import Options
from slam_it.edge.pipeline import Placement
from slam_it.eval.metrics import absolute_error
from slam_it.foundations import orb
from slam_it.foundations.errors import InvalidInputError
from tests.edge import room
from tests.edge.conftest import Run
from tests.edge.support import ATE_BOUND, CAMERA, MONOCULAR_START, OPTIONS


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


@pytest.mark.parametrize("mode", ["monocular", "stereo", "rgbd"])
def test_every_frame_says_how_its_stages_went(mode: str, runs: dict[str, Run]) -> None:
    """A frame's summary holds the stages it ran, and only those.

    Initialisation before the map; then tracking the last frame, the fallback only when that
    failed, the local map on a placed frame, and mapping on each keyframe after the first.
    """
    results = runs[mode].results
    start = next(i for i, r in enumerate(results) if r.keyframe)
    for i, r in enumerate(results):
        s = r.summary
        json.dumps(s.as_dict(), allow_nan=False)  # what a run writes
        if i < start:
            assert s.init is not None
            assert s.init.outcome is not Outcome.INITIALIZED
            assert (s.placed_by, s.motion, s.local_map, s.mapping) == (None, None, None, None)
        elif i == start:
            assert s.init is not None
            assert s.init.outcome is Outcome.INITIALIZED
            assert s.placed_by is Placement.INITIALIZATION
            assert (s.motion, s.mapping) == (None, None)
        else:
            assert s.init is None
            assert s.motion is not None
            assert (s.pnp is None) == (s.placed_by is Placement.MOTION)
            if r.pose is not None:
                assert s.local_map is not None
                assert s.local_map.inliers == r.tracked
            assert (s.mapping is not None) == r.keyframe
    first = results[0].summary.init
    assert first is not None
    if mode == "monocular":
        assert first.outcome is Outcome.REFERENCE_SET
        assert first.matches is None


@pytest.mark.parametrize("mode", ["monocular", "stereo", "rgbd"])
def test_every_point_and_keyframe_has_a_life(mode: str, runs: dict[str, Run]) -> None:
    """Every id the map gave has a life; the ones alive are the map's, where the map has them.

    Lives start in the frame that made them, in id order, and end in that frame or later, with
    why; a removed point's position is where it last was.
    """
    run = runs[mode]
    m = run.pipeline.map
    assert m is not None
    lives = run.pipeline.lifetimes()
    for life, made_ids, live_ids in (
        (lives.point_lives, m.next_pt_id, m.pt_id),
        (lives.keyframe_lives, m.next_kf_id, m.kf_id),
    ):
        assert life.shape == (made_ids, 3)
        made, removed, how = life.T
        assert np.all(np.diff(made) >= 0)  # ids are given in order
        assert made.min() >= 0
        assert made.max() < len(run.results)
        alive = removed < 0
        np.testing.assert_array_equal(np.flatnonzero(alive), live_ids)
        assert np.all(how[alive] == -1)
        assert np.all((removed[~alive] >= made[~alive]) & (removed[~alive] < len(run.results)))
        assert np.all(np.isin(how[~alive], [int(r) for r in Removal]))
    np.testing.assert_array_equal(lives.points[m.pt_id], m.pt_xyz)
    assert np.all(lives.point_maps == 0)  # one map
    assert np.all(lives.keyframe_maps == 0)
    np.testing.assert_array_equal(lives.keyframe_times[m.kf_id], m.kf_time)
    np.testing.assert_array_equal(lives.keyframe_poses[m.kf_id], m.kf_pose)
    assert np.all(np.diff(lives.keyframe_times) > 0)
    assert np.all(lives.keyframe_lives[~(lives.keyframe_lives[:, 1] < 0), 2] == Removal.CULLED)
    assert len(m.gone_pt_id) == len(m.gone_kf_id) == 0  # dated and cleared every frame
    # Only mapping removes, on keyframes after the start: the lives agree with its summaries.
    start = next(i for i, r in enumerate(run.results) if r.keyframe)
    mapped = [r.summary.mapping for r in run.results if r.summary.mapping is not None]
    removed_points = sum(s.culled_points + s.fused + s.weak_points for s in mapped)
    assert int(np.count_nonzero(lives.point_lives[:, 1] > start)) == removed_points
    culled_keyframes = sum(s.culled_keyframes for s in mapped)
    assert int(np.count_nonzero(lives.keyframe_lives[:, 1] >= 0)) == culled_keyframes


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


def _lose_and_restart(
    rendered: list[room.Frame], options: Options
) -> tuple[pipeline.Pipeline, list[pipeline.FrameResult]]:
    """Start a map on the first frame, lose it on lost_frames empty ones, start the next."""
    run = pipeline.Pipeline("rgbd", CAMERA, options)
    first, second = rendered[0], rendered[1]
    results = [run.track(frames.rgbd(first.gray, first.depth, 0.0, CAMERA, run.extraction()))]
    results += [run.track(_empty()) for _ in range(options.lost_frames)]
    results.append(run.track(frames.rgbd(second.gray, second.depth, 1.0, CAMERA, run.extraction())))
    return run, results


def test_a_lost_track_resets_a_small_map_and_starts_a_new_one(
    rendered: list[room.Frame],
) -> None:
    """lost_frames frames in a row untracked lose the track; the next frame starts map 1.

    The lost map has one keyframe, under reset_keyframes, so it is reset: its keyframe and
    points end on the frame the track was lost on (ORB-SLAM3's ResetActiveMap).
    """
    run, results = _lose_and_restart(rendered, OPTIONS)
    started, *failed, lost, restarted = results
    assert (started.state, started.map) == (pipeline.State.TRACKING, 0)
    assert all(r.pose is None and r.state is pipeline.State.TRACKING for r in failed)
    assert all(r.map == 0 and r.summary.lost is None for r in failed)
    assert (lost.state, lost.map, lost.pose) == (pipeline.State.LOST, 0, None)
    closed = pipeline.LostSummary(keyframes=1, points=started.tracked, reset=True)
    assert lost.summary.lost == closed
    assert restarted.state is pipeline.State.TRACKING
    assert (restarted.map, restarted.keyframe) == (1, True)
    assert restarted.summary.placed_by is Placement.INITIALIZATION
    lives = run.lifetimes()
    first_points = started.tracked
    np.testing.assert_array_equal(lives.keyframe_maps, [0, 1])
    made_in = np.repeat([0, 1], [first_points, restarted.tracked])
    np.testing.assert_array_equal(lives.point_maps, made_in)
    lost_at = len(results) - 2
    np.testing.assert_array_equal(lives.keyframe_lives[0], [0, lost_at, int(Removal.RESET)])
    assert np.all(lives.point_lives[:first_points, 1:] == [lost_at, int(Removal.RESET)])
    assert np.all(lives.point_lives[first_points:] == [len(results) - 1, -1, -1])
    # The new map numbers its own points from 0; the run's ids follow the first map's.
    assert run.map is not None
    np.testing.assert_array_equal(lives.points[first_points + run.map.pt_id], run.map.pt_xyz)
    made = restarted.fates.point[restarted.fates.fate == pipeline.Fate.MADE]
    assert sorted(made) == list(range(first_points, first_points + restarted.tracked))


def test_a_lost_map_above_reset_keyframes_is_kept(rendered: list[room.Frame]) -> None:
    """With reset_keyframes 0, the lost map is kept.

    Its keyframe and points stay alive, where they were when it closed, and the tracking state
    starts again with the next map.
    """
    run, results = _lose_and_restart(rendered, replace(OPTIONS, reset_keyframes=0))
    started, lost, restarted = results[0], results[-2], results[-1]
    closed = pipeline.LostSummary(keyframes=1, points=started.tracked, reset=False)
    assert lost.summary.lost == closed
    lives = run.lifetimes()
    assert np.all(lives.keyframe_lives[:, 1:] == -1)
    assert np.all(lives.point_lives[:, 1:] == -1)
    first = lives.points[: started.tracked]
    assert np.all(np.isfinite(first))
    assert np.any(first != 0.0)
    np.testing.assert_array_equal(lives.keyframe_poses[0], np.eye(4))  # the first map's origin
    assert (restarted.map, restarted.state) == (1, pipeline.State.TRACKING)
    np.testing.assert_array_equal(lives.keyframe_poses[1], np.eye(4))  # the second's own
    assert run._reference is None
    assert run._failures == 0


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


def test_a_frame_with_little_depth_starts_the_map(rendered: list[room.Frame]) -> None:
    """A frame with enough keypoints starts a map however few have depth (finding L13).

    ORB-SLAM's rule: a map too thin to track is lost and reset, and the next frame starts again.
    """
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS)
    first = rendered[0]
    sparse = np.zeros_like(first.depth)
    sparse[:, 150:160] = first.depth[:, 150:160]  # depth only in a narrow strip
    built = frames.rgbd(first.gray, sparse, 0.0, CAMERA, run.extraction())
    started = run.track(built)
    assert started.state is pipeline.State.TRACKING
    assert run.map is not None
    assert 0 < run.map.points == np.count_nonzero(~np.isnan(built.depth)) < 100


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
