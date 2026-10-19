"""The pipeline in threads of its own (milestone 2B's B3b): the room tracked, the handshakes held.

Threaded runs do not repeat exactly, so these hold what must be true however the threads
interleave: every frame tracked, every keyframe given mapped once, the records whole, a stop
refusing keyframes, the busy rule, the view published, and a lost map settled before it closes.
The bar on the loop room (``tests/orchestration/test_loop_room.py``) is the natives' to run
threaded; Python's threaded run of it is measured, not tested (the design's section 13).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it.edge import frames, pipeline
from slam_it.edge import map as maps
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.edge.threads import Writer
from slam_it.eval.metrics import absolute_error
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_backend
from tests.edge import room
from tests.edge.support import ATE_BOUND, CAMERA, OPTIONS, words_for


def _frame(f: room.Frame, run: pipeline.Pipeline) -> frames.Frame:
    """Build an RGB-D frame of the room for `run` (the C++ core extracts, for speed)."""
    return frames.rgbd(f.gray, f.depth, f.timestamp, CAMERA, run.extraction(), backend="cpp")


def _threaded(options: Options = OPTIONS) -> pipeline.Pipeline:
    """Return a threaded RGB-D pipeline."""
    require_backend("cpp")
    return pipeline.Pipeline("rgbd", CAMERA, options, words_for(options), threaded=True)


def test_the_room_is_tracked_and_every_keyframe_mapped(rendered: list[room.Frame]) -> None:
    """Every frame tracked near the truth; each keyframe given is mapped once; the records hold.

    A keyframe's mapping comes back as a completion on its frame; the lives count the first
    keyframe and each mapped one; the trajectory places every frame tracked, in order.
    """
    run = _threaded()
    results = [run.track(_frame(f, run)) for f in rendered]
    run.finish()
    done = run.completed()
    assert all(r.pose is not None for r in results)
    estimate = np.array([np.linalg.inv(r.pose) for r in results if r.pose is not None])
    error = absolute_error(estimate, np.array([f.T_world_cam for f in rendered]))
    assert error.rmse < ATE_BOUND["rgbd"]
    given = [k for k, r in enumerate(results) if r.keyframe and k > 0]
    mapped = sorted(c.frame for c in done if "mapping" in c.entries)
    assert mapped == given
    assert all(c.entries["schedule"] is not None for c in done if "mapping" in c.entries)
    lives = run.lifetimes()
    assert len(lives.keyframe_times) == 1 + len(given)
    trajectory = run.trajectory()
    assert len(trajectory.timestamps) == len(results)
    assert np.all(np.diff(trajectory.timestamps) > 0)
    assert all(r.map_seconds == r.loop_seconds == 0.0 for r in results)


def test_a_finished_run_tracks_no_more(rendered: list[room.Frame]) -> None:
    """After the run's end, a frame is refused rather than given to threads that have ended."""
    run = _threaded()
    run.track(_frame(rendered[0], run))
    run.finish()
    with pytest.raises(InvalidInputError, match="finished"):
        run.track(_frame(rendered[1], run))


def test_inline_nothing_completes_later(rendered: list[room.Frame]) -> None:
    """Inline, each frame's result holds its mapping: nothing comes later."""
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    for f in rendered[:12]:
        run.track(_frame(f, run))
    run.finish()
    assert run.completed() == []


def test_a_stop_refuses_every_keyframe_asked(rendered: list[room.Frame]) -> None:
    """While mapping is stopped, tracking asks for no keyframe it gets: each is refused."""
    run = _threaded()
    for f in rendered[:8]:
        run.track(_frame(f, run))
    writer = run.owner.writer
    assert writer is not None
    run.completed()
    with writer.stopped():
        held = [run.track(_frame(f, run)) for f in rendered[8:]]
    run.finish()
    assert any(r.asked for r in held)
    assert all(r.refused and not r.keyframe for r in held if r.asked)


@dataclass
class _Busy:
    """Local mapping as a test sets it: busy, with keyframes waiting."""

    waiting_count: int
    interrupted: bool = False

    def idle(self) -> bool:
        """Return False: busy."""
        return False

    def stopping(self) -> bool:
        """Return False: not stopped."""
        return False

    def waiting(self) -> int:
        """Return the keyframes waiting."""
        return self.waiting_count

    def interrupt(self) -> None:
        """Note the interruption."""
        self.interrupted = True


@pytest.mark.parametrize(("waiting", "taken"), [(0, True), (2, True), (3, False)])
def test_busy_mapping_queues_up_to_three(
    rendered: list[room.Frame], waiting: int, taken: bool
) -> None:
    """Busy, a wanted RGB-D keyframe interrupts the local BA and is queued while fewer than 3 wait.

    Replayed inline to the room's first frame that asks for a keyframe, past
    ``tracking.max_frames`` since the last, so it wants one busy too.
    """
    require_backend("cpp")
    inline = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    results = [inline.track(_frame(f, inline)) for f in rendered]
    asks = next(k for k, r in enumerate(results) if k > 0 and r.asked)
    run = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    for f in rendered[:asks]:
        run.track(_frame(f, run))
    run.tracking.since_keyframe = OPTIONS.tracking.max_frames
    busy = _Busy(waiting)
    view = run.owner.view
    assert view is not None
    found = pipeline._track_frame(
        run.tracking, run.owner, view, CAMERA, _frame(rendered[asks], run), mode="rgbd",
        options=OPTIONS, keyframe_base=run.atlas.keyframe_base, mapper=busy,
    )  # fmt: skip
    assert found.asked
    assert busy.interrupted
    assert (found.keyframe, found.refused, found.queue) == (taken, not taken, waiting)


def _counted(m: Map) -> Map:
    """Return `m` with each point seen once more."""
    return maps.count(m, m.pt_id, visible=np.ones(m.points, dtype=np.int64))


def _viewed(owner: pipeline.MapOwner) -> Map:
    """Return the map of the view `owner` published last."""
    view = owner.view
    assert view is not None
    return view.map


def _correction() -> NDArray[np.float64]:
    """Return a rigid move of the world, as a loop's correction moves a map.

    20 degrees about the vertical and 0.5 m along x: far past the motion model's search, so a
    frame placed by it after the move was predicted from a last frame that moved too.
    """
    turn = np.radians(20.0)
    g = np.eye(4)
    g[:3, :3] = [
        [np.cos(turn), 0.0, np.sin(turn)],
        [0.0, 1.0, 0.0],
        [-np.sin(turn), 0.0, np.cos(turn)],
    ]
    g[0, 3] = 0.5
    return g


CORRECTION = _correction()
"""The correction the tests apply (:func:`_correction`)."""


def _moved(m: Map, g: NDArray[np.float64]) -> Map:
    """Return `m` with its world moved by `g`, as a loop's correction moves a map rigidly.

    A keyframe's ``T_cam_world`` becomes ``T_cam_world g⁻¹``; a point ``x`` becomes ``g x``, its
    viewing direction turned with it.
    """
    rotation, shift = g[:3, :3], g[:3, 3]
    return replace(
        m,
        kf_pose=m.kf_pose @ np.linalg.inv(g),
        pt_xyz=m.pt_xyz @ rotation.T + shift,
        pt_normal=m.pt_normal @ rotation.T,
    )


def _centre(pose: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return a camera's centre in the world, from its ``T_cam_world``."""
    return np.asarray(-pose[:3, :3].T @ pose[:3, 3])


def test_tracking_reads_the_view_and_its_counts_wait_for_mapping(
    rendered: list[room.Frame],
) -> None:
    """Threaded, the view changes only when published; counts reach the map through mapping."""
    inline = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    inline.track(_frame(rendered[0], inline))
    m = inline.owner.read()
    owner = pipeline.MapOwner(words_for(OPTIONS), Writer())
    owner.start(m)
    assert _viewed(owner) is owner.map is m
    owner.count(m.pt_id[:3], m.pt_id[:2])
    assert owner.map is m  # waiting for mapping
    owner.apply_counts()
    assert _viewed(owner) is m  # not published yet
    assert owner.map is not None
    np.testing.assert_array_equal(owner.map.pt_visible[:3], m.pt_visible[:3] + 1)
    np.testing.assert_array_equal(owner.map.pt_found[:2], m.pt_found[:2] + 1)
    later = _counted(m)
    owner.publish(later)
    assert _viewed(owner) is later
    assert owner.map is not later  # published, not applied
    owner.apply_mapping(later)
    assert _viewed(owner) is owner.map is later


def test_after_a_correction_the_next_frame_follows_its_keyframe(
    rendered: list[room.Frame],
) -> None:
    """A correction's view carries its count, and the next frame moves its last frame with it.

    The map corrected under a stop (moved rigidly, as a loop's correction moves it): the view
    published with it carries the new count, and a view read before still carries the old one.
    The next frame, tracked on the one view it reads, first moves the last frame with the
    keyframe it was tracked against, so the motion model places it, in the corrected world
    (B3b's review panel, A1, L2 and H12).
    """
    run = _threaded()
    for f in rendered[:6]:
        run.track(_frame(f, run))
    last = run.tracking.last_tracked
    assert last is not None
    before = run.owner.view
    assert before is not None
    with run.owner.stopped() as m:
        run.owner.apply_correction(_moved(m, CORRECTION))
    after = run.owner.view
    assert after is not None
    assert after.corrections == run.owner.corrections == before.corrections + 1
    np.testing.assert_allclose(
        after.map.kf_pose[0], before.map.kf_pose[0] @ np.linalg.inv(CORRECTION)
    )
    result = run.track(_frame(rendered[6], run))
    run.finish()
    assert result.summary.placed_by is pipeline.Placement.MOTION
    assert result.pose is not None
    moved = last.pose @ np.linalg.inv(CORRECTION)
    assert np.linalg.norm(_centre(result.pose) - _centre(moved)) < 0.05


def test_a_closed_maps_counts_never_reach_the_next(rendered: list[room.Frame]) -> None:
    """Counts waiting when the map is lost go with it: the next map's points keep theirs.

    The next map numbers its points from 0 again, so a count kept across the loss would land
    on the new map's points of the same ids (B3b's review panel, H1).
    """
    inline = pipeline.Pipeline("rgbd", CAMERA, OPTIONS, words_for(OPTIONS))
    inline.track(_frame(rendered[0], inline))
    m = inline.owner.read()
    owner = pipeline.MapOwner(words_for(OPTIONS), Writer())
    owner.start(m)
    owner.count(m.pt_id[:3], m.pt_id[:2])
    owner.close()
    owner.start(m)  # the next map, its ids from 0 as the closed one's
    owner.apply_counts()
    assert owner.map is not None
    np.testing.assert_array_equal(owner.map.pt_visible, m.pt_visible)
    np.testing.assert_array_equal(owner.map.pt_found, m.pt_found)


def test_a_lost_map_settles_the_threads_then_closes(rendered: list[room.Frame]) -> None:
    """Lost, the map waits for mapping, then closes as inline: kept or reset, the next started."""
    require_backend("cpp")
    options = OPTIONS.with_settings({"loss.reset_keyframes": 0})
    outcomes = []
    for threaded in (False, True):
        run = pipeline.Pipeline("rgbd", CAMERA, options, words_for(options), threaded=threaded)
        first, second = rendered[0], rendered[1]
        results = [run.track(frames.rgbd(first.gray, first.depth, 0.0, CAMERA, run.extraction()))]
        empty = replace(first, gray=np.zeros_like(first.gray))
        results += [
            run.track(frames.rgbd(empty.gray, empty.depth, float(t), CAMERA, run.extraction()))
            for t in range(1, 6)
        ]
        results.append(
            run.track(frames.rgbd(second.gray, second.depth, 6.0, CAMERA, run.extraction()))
        )
        run.finish()
        outcomes.append([(r.state, r.map, r.summary.lost) for r in results])
    assert outcomes[0] == outcomes[1]
    assert outcomes[1][-2][2] == pipeline.LostSummary(keyframes=1, points=results[0].tracked,
                                                      reset=False)  # fmt: skip
    assert outcomes[1][-1][:2] == (pipeline.State.TRACKING, 1)
