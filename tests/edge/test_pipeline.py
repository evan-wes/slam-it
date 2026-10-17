"""The pipeline's tests: the room in every mode, start to end, and the pipeline's rules."""

from __future__ import annotations

import json

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
from tests.edge.support import ATE_BOUND, CAMERA, MONOCULAR_START, OPTIONS, words_for


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
            assert (s.relocalization is None) == (
                s.placed_by in (Placement.MOTION, Placement.LOCAL_PNP)
            )
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
    """Before the map, a monocular frame gets features.init_factor times the features."""
    camera = frames.Camera("pinhole", room.SMALL.intrinsics, 320, 240)
    mono = pipeline.Pipeline("monocular", camera, OPTIONS, words_for(OPTIONS))
    assert (
        mono.extraction().n_features == OPTIONS.features.init_factor * OPTIONS.features.n_features
    )
    depth = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    assert depth.extraction().n_features == OPTIONS.features.n_features


def _empty(timestamp: float = 0.0) -> frames.Frame:
    """Return a frame with no keypoints, at `timestamp`."""
    features = orb.Features(
        orb.Keypoints(
            xy=np.zeros((0, 2)),
            level=np.zeros(0, dtype=np.int64),
            level_xy=np.zeros((0, 2), dtype=np.int64),
            response=np.zeros(0, dtype=np.int64),
            angle=np.zeros(0, dtype=np.int64),
        ),
        np.zeros((0, orb.DESCRIPTOR_BYTES), dtype=np.uint8),
        scale_factor=OPTIONS.features.scale_factor,
        levels=OPTIONS.features.levels,
    )
    return frames.Frame(timestamp, features, np.zeros((0, 3)), np.zeros(0), np.zeros(0))


def _lose_and_restart(
    rendered: list[room.Frame], options: Options
) -> tuple[pipeline.Pipeline, list[pipeline.FrameResult]]:
    """Start a map on the first frame, lose it on empty ones a second apart, start the next.

    A map of one keyframe is lost on the fifth either way: after loss.lost_frames (5) failures if it
    is small, or once more than loss.relocalize_seconds (3) have passed since the first if not.
    """
    run = pipeline.Pipeline("rgbd", CAMERA, options, words_for(options))
    first, second = rendered[0], rendered[1]
    results = [run.track(frames.rgbd(first.gray, first.depth, 0.0, CAMERA, run.extraction()))]
    results += [run.track(_empty(float(t))) for t in range(1, 6)]
    results.append(run.track(frames.rgbd(second.gray, second.depth, 6.0, CAMERA, run.extraction())))
    return run, results


def test_a_lost_track_resets_a_small_map_and_starts_a_new_one(
    rendered: list[room.Frame],
) -> None:
    """loss.lost_frames frames in a row untracked lose the track; the next frame starts map 1.

    The lost map has one keyframe, and loss.reset_keyframes is 1, the rule's boundary: "1 or fewer"
    resets it (review panel finding L3), its keyframe and points ending on the frame the track
    was lost on (ORB-SLAM3's ResetActiveMap).
    """
    run, results = _lose_and_restart(rendered, OPTIONS.with_settings({"loss.reset_keyframes": 1}))
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
    """With loss.reset_keyframes 0, one under the lost map's one keyframe, the lost map is kept.

    Its keyframe and points stay alive, where they were when it closed, and the tracking state
    starts again with the next map.
    """
    run, results = _lose_and_restart(rendered, OPTIONS.with_settings({"loss.reset_keyframes": 0}))
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


@pytest.mark.parametrize(
    ("reset_keyframes", "step", "lost_on"),
    [
        (0, 0.25, 14),  # a kept map: lost past 3 s after the first failure, at 0.25 s: at 3.5 s
        (1, 0.25, 5),  # a small map: lost on the fifth failure, whatever the time
        (1, 0.001, 5),
    ],
)
def test_a_large_map_is_lost_in_time_and_a_small_one_in_frames(
    rendered: list[room.Frame], reset_keyframes: int, step: float, lost_on: int
) -> None:
    """Time loses a map above loss.reset_keyframes; loss.lost_frames failures, a smaller one."""
    options = OPTIONS.with_settings({"loss.reset_keyframes": reset_keyframes})
    run = pipeline.Pipeline("rgbd", CAMERA, options, words_for(options))
    first = rendered[0]
    assert run.track(frames.rgbd(first.gray, first.depth, 0.0, CAMERA, run.extraction())).keyframe
    failed = [run.track(_empty(step * (k + 1))) for k in range(lost_on)]
    assert [r.state for r in failed[:-1]] == [pipeline.State.TRACKING] * (lost_on - 1)
    assert failed[-1].state is pipeline.State.LOST
    assert all(r.summary.relocalization is not None for r in failed)


def test_off_a_kept_map_is_lost_after_lost_frames_and_nothing_relocalizes(
    rendered: list[room.Frame],
) -> None:
    """Relocalisation off, A2's rule: lost on the loss.lost_frames-th failure, whatever the time."""
    options = OPTIONS.with_settings({"relocalization.enabled": False, "loss.reset_keyframes": 0})
    run = pipeline.Pipeline("rgbd", CAMERA, options)
    first = rendered[0]
    assert run.track(frames.rgbd(first.gray, first.depth, 0.0, CAMERA, run.extraction())).keyframe
    failed = [run.track(_empty(0.001 * (k + 1))) for k in range(options.loss.lost_frames)]
    assert [r.state for r in failed] == [pipeline.State.TRACKING] * 4 + [pipeline.State.LOST]
    assert all(r.summary.relocalization is None for r in failed)


@pytest.mark.parametrize(("relocalize", "message"), [(True, "needs one"), (False, "is off")])
def test_the_vocabulary_is_needed_exactly_when_relocalizing(relocalize: bool, message: str) -> None:
    """Relocalising without a vocabulary, or given one while not: InvalidInputError."""
    options = OPTIONS.with_settings({"relocalization.enabled": relocalize})
    given = None if relocalize else words_for(OPTIONS)
    with pytest.raises(InvalidInputError, match=f"vocabulary: .*{message}"):
        pipeline.Pipeline("rgbd", CAMERA, options, given)


def test_when_the_fallback_fails_relocalization_places_the_frame_in_its_map(
    rendered: list[room.Frame],
) -> None:
    """With the local fallback out of reach, a frame back at the start is found by its words.

    It is placed in the same map, through the database, and the track goes on.
    """
    options = OPTIONS.with_settings(
        {"tracking.pnp_min_inliers": 1 << 20, "loss.reset_keyframes": 0}
    )
    run = pipeline.Pipeline("rgbd", CAMERA, options, words_for(options))
    for f in rendered[:31]:
        run.track(frames.rgbd(f.gray, f.depth, f.timestamp, CAMERA, run.extraction()))
    lost = [run.track(_empty(rendered[30].timestamp + 0.01 * (k + 1))) for k in range(3)]
    assert all(r.state is pipeline.State.TRACKING and r.pose is None for r in lost)
    back = rendered[2]
    found = run.track(frames.rgbd(back.gray, back.depth, 1.5, CAMERA, run.extraction()))
    assert (found.state, found.map) == (pipeline.State.TRACKING, 0)
    assert found.summary.placed_by is Placement.RELOCALIZATION
    assert found.summary.relocalization is not None
    assert found.summary.relocalization.placed
    assert found.tracked >= OPTIONS.tracking.recovery_tracked
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
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
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
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    first = rendered[0]
    sparse = np.zeros_like(first.depth)
    sparse[:, 150:160] = first.depth[:, 150:160]  # depth only in a narrow strip
    built = frames.rgbd(first.gray, sparse, 0.0, CAMERA, run.extraction())
    started = run.track(built)
    assert started.state is pipeline.State.TRACKING
    assert run.map is not None
    assert 0 < run.map.points == np.count_nonzero(~np.isnan(built.depth)) < 100


def test_a_map_without_points_is_lost_and_the_third_map_follows_the_first_two(
    rendered: list[room.Frame],
) -> None:
    """A frame without depth starts a map of no points; tracked frames lose it; maps go on.

    Map 0 has no points, so no frame after it is placed: loss.lost_frames frames lose it, and it is
    reset. Map 1 starts on the next frame and is lost on empty frames; map 2 starts after it,
    its keyframe and points numbered after both maps' (review panel finding L4).
    """
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    first = rendered[0]
    no_depth = frames.rgbd(first.gray, np.zeros_like(first.depth), 0.0, CAMERA, run.extraction())
    results = [run.track(no_depth)]
    assert (results[0].map, results[0].tracked) == (0, 0)
    for k, f in enumerate(rendered[1 : 1 + OPTIONS.loss.lost_frames], start=1):
        results.append(run.track(frames.rgbd(f.gray, f.depth, k, CAMERA, run.extraction())))
    assert all(r.pose is None for r in results[1:])
    assert results[-1].summary.lost == pipeline.LostSummary(keyframes=1, points=0, reset=True)
    second = rendered[1 + OPTIONS.loss.lost_frames]
    restarted = run.track(frames.rgbd(second.gray, second.depth, 10.0, CAMERA, run.extraction()))
    assert restarted.map == 1
    for _ in range(OPTIONS.loss.lost_frames):
        lost = run.track(_empty())
    assert (lost.map, lost.summary.lost) == (1, pipeline.LostSummary(1, restarted.tracked, True))
    third = rendered[2 + OPTIONS.loss.lost_frames]
    again = run.track(frames.rgbd(third.gray, third.depth, 20.0, CAMERA, run.extraction()))
    assert (again.map, again.keyframe) == (2, True)
    lives = run.lifetimes()
    np.testing.assert_array_equal(lives.keyframe_maps, [0, 1, 2])
    np.testing.assert_array_equal(lives.keyframe_poses[2], np.eye(4))  # map 2's own origin
    before = restarted.tracked  # map 0 made none
    made = again.fates.point[again.fates.fate == pipeline.Fate.MADE]
    assert sorted(made) == list(range(before, before + again.tracked))
    assert run.map is not None
    np.testing.assert_array_equal(lives.points[before + run.map.pt_id], run.map.pt_xyz)
    np.testing.assert_array_equal(lives.point_maps, np.repeat([1, 2], [before, again.tracked]))


def test_a_monocular_reference_needs_enough_features(rendered: list[room.Frame]) -> None:
    """A frame with too few features drops the monocular reference rather than pairing."""
    camera = frames.Camera("pinhole", room.SMALL.intrinsics, 320, 240)
    run = pipeline.Pipeline("monocular", camera, OPTIONS, words_for(OPTIONS))
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
        pipeline.Pipeline(mode, camera, Options(), words_for(Options()))


def test_the_database_holds_the_maps_keyframes_through_culling(
    rendered: list[room.Frame],
) -> None:
    """A culled keyframe leaves the database: after every frame it holds the map's keyframes.

    The room culls no keyframe at ORB-SLAM's redundancy (90% of a keyframe's points seen by 3
    others); at 60% seen by one other, it culls several (review panel finding L2).
    """
    options = OPTIONS.with_settings(
        {"mapping.redundant_fraction": 0.6, "mapping.redundant_observers": 1}
    )
    run = pipeline.Pipeline("rgbd", CAMERA, options, words_for(options))
    for f in rendered:
        run.track(frames.rgbd(f.gray, f.depth, f.timestamp, CAMERA, run.extraction()))
        assert run.map is not None
        assert run._database is not None
        assert run._database.keyframes.tolist() == sorted(run.map.kf_id.tolist())
    lives = run.lifetimes().keyframe_lives
    assert np.count_nonzero(lives[:, 2] == Removal.CULLED) > 0
