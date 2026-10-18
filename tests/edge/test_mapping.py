"""Local mapping's tests: every backend against the reference, and its rules.

On the room's truth map with a later frame tracked against it: each step on the map with
that frame inserted as its newest keyframe, and the whole of :func:`mapping.process`. What
fusion and local BA take from the graph (:func:`mapping.fuse_targets`,
:func:`mapping.local_ba_keyframes`) is held to the covisibility fixture's maps in
``tests/edge/test_covisibility.py``.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field, replace
from functools import partial

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import map as maps
from slam_it.edge import mapping, residuals, tracking
from slam_it.edge.frames import Frame, Tracked
from slam_it.edge.map import Map
from slam_it.foundations import matching
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import Truth
from tests.edge.support import (
    CAMERA,
    OPTIONS,
    SOLVER_TOLERANCE,
    assert_maps_match,
    assert_records_match,
)

LAST, CURRENT = 36, 39
"""The room's last keyframe, and the frame tracked from it and inserted."""

MODES = ("monocular", "stereo", "rgbd")
"""Every mode (the frames are RGB-D's; the mode decides the steps' rules)."""


@dataclass(frozen=True)
class Mapped:
    """The truth map, a frame tracked against it, the map with it inserted, its reference."""

    map: Map
    frame: Frame
    tracked: Tracked
    inserted: Map
    reference: int


@pytest.fixture(scope="module")
def mapped(truth: Truth) -> Mapped:
    """Track frame :data:`CURRENT` from keyframe :data:`LAST` and insert it as a keyframe."""
    m = truth.map
    last = Tracked(pose=truth.poses[LAST], points=m.kp_point[m.keypoint_rows(3)].copy())
    frame = truth.frames[CURRENT]
    found = tracking.track_last_frame(
        m, CAMERA, frame, truth.frames[LAST], last,
        predicted=truth.poses[CURRENT], mode="rgbd", options=OPTIONS,
    ).tracked  # fmt: skip
    assert found is not None
    local = tracking.track_local_map(m, CAMERA, frame, found, mode="rgbd", options=OPTIONS)
    inserted = maps.insert_keyframe(m, frame, local.tracked.pose, local.tracked.points)
    return Mapped(m, frame, local.tracked, inserted, local.reference)


def _same(backend: str, function: str, *args: object, **kwargs: object) -> None:
    """Check one step: every backend's map (or value) as the reference's."""
    chosen = require_function(backend, mapping.__name__, function)
    expected = getattr(mapping, function)(*args, **kwargs)
    got = getattr(mapping, function)(*args, **kwargs, backend=chosen)
    if isinstance(expected, Map):
        assert_maps_match(got, expected, SOLVER_TOLERANCE)
    elif isinstance(expected, mapping.Mapped):
        assert_maps_match(got.map, expected.map, SOLVER_TOLERANCE)
        assert_records_match(got.summary.as_dict(), expected.summary.as_dict())
    else:
        np.testing.assert_array_equal(got, expected)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("mode", MODES)
def test_steps_match_the_reference(backend: str, mode: str, mapped: Mapped) -> None:
    """Every backend runs each step as the reference does, in every mode."""
    m = mapped.inserted
    _same(backend, "need_keyframe", mapped.map, CAMERA, mapped.frame.depth, mapped.tracked,
          reference=mapped.reference, mode=mode, options=OPTIONS)  # fmt: skip
    _same(backend, "create_points_from_depth", m, CAMERA, OPTIONS)
    _same(backend, "cull_points", m, mode, OPTIONS)
    _same(backend, "triangulate_new_points", m, CAMERA, mode, OPTIONS)
    _same(backend, "fuse_points", m, CAMERA, mode, OPTIONS)
    _same(backend, "cull_keyframes", m, CAMERA, mode, OPTIONS)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_matching_and_counts_match_the_reference(backend: str, mapped: Mapped) -> None:
    """Triangulation matches agree exactly; culling agrees on randomised counts."""
    m = mapped.inserted
    newest = m.keyframes - 1
    for k2 in range(newest):
        _same(backend, "match_for_triangulation", m, CAMERA, newest, k2, OPTIONS)
    rng = np.random.default_rng(5)
    visible = rng.integers(1, 9, m.points)
    counted = replace(m, pt_visible=visible, pt_found=rng.integers(0, visible + 1))
    for mode in MODES:
        _same(backend, "cull_points", counted, mode, OPTIONS)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("mode", MODES)
def test_process_matches_the_reference(backend: str, mode: str, mapped: Mapped) -> None:
    """Every backend runs the whole of local mapping as the reference does."""
    _same(backend, "process", mapped.map, CAMERA, mapped.frame, mapped.tracked,
          mode=mode, options=OPTIONS)  # fmt: skip


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_local_ba_waits_for_enough_keyframes(backend: str, mapped: Mapped) -> None:
    """Before the map holds ``adjustment.min_keyframes``, no local BA: the poses stay as tracked."""
    waiting = OPTIONS.with_settings({"adjustment.min_keyframes": mapped.map.keyframes + 2})
    args = (mapped.map, CAMERA, mapped.frame, mapped.tracked)
    waited = mapping.process(*args, mode="rgbd", options=waiting)
    held = waited.map
    np.testing.assert_array_equal(held.kf_pose[-1], mapped.tracked.pose)
    rows = mapped.map.keyframe_rows(held.kf_id[:-1])
    np.testing.assert_array_equal(held.kf_pose[:-1], mapped.map.kf_pose[rows])
    assert (waited.summary.solves, waited.summary.outliers) == ((), 0)
    ran = mapping.process(*args, mode="rgbd", options=OPTIONS)
    assert not np.array_equal(ran.map.kf_pose[-1], mapped.tracked.pose)
    assert len(ran.summary.solves) == 2
    _same(backend, "process", *args, mode="rgbd", options=waiting)


def test_keyframe_needed_when_tracking_thins(mapped: Mapped) -> None:
    """Few tracked points against the reference: a keyframe; too few to be one: none."""
    m, frame, tracked = mapped.map, mapped.frame, mapped.tracked
    matched = np.flatnonzero(tracked.points >= 0)

    def keeping(count: int) -> Tracked:
        points = np.full_like(tracked.points, -1)
        points[matched[:count]] = tracked.points[matched[:count]]
        return Tracked(pose=tracked.pose, points=points)

    def need(t: Tracked) -> bool:
        return mapping.need_keyframe(
            m, CAMERA, frame.depth, t, reference=m.keyframes - 1, mode="monocular", options=OPTIONS
        )

    assert need(keeping(OPTIONS.keyframes.min_inliers + 1))
    assert not need(keeping(OPTIONS.keyframes.min_inliers))
    assert not need(tracked)


def _reference_points(m: Map, row: int) -> int:
    """Return the points a keyframe sees from enough keyframes (``TrackedMapPoints``), by hand."""
    keyframes = OPTIONS.keyframes
    least = (
        keyframes.ref_min_observations_early if m.keyframes <= 2 else keyframes.ref_min_observations
    )
    seen = m.kp_point[m.keypoint_rows(row)]
    rows = m.point_rows(seen[seen >= 0])
    return int(np.count_nonzero(maps.observation_weight(m, rows[rows >= 0]) >= least))


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_the_keyframe_decision_counts_against_its_reference(backend: str, mapped: Mapped) -> None:
    """The same frame, against two reference keyframes seeing different counts, decides both ways.

    It tracks the share of the fewer the one sees, so needs no keyframe against it, and less than
    the share of the more the other sees, so needs one against that.
    """
    m, frame, tracked = mapped.map, mapped.frame, mapped.tracked
    counts = [_reference_points(m, row) for row in range(m.keyframes)]
    few, many = int(np.argmin(counts)), int(np.argmax(counts))
    ratio = OPTIONS.keyframes.mono_ref_ratio
    kept = int(np.ceil(ratio * counts[few]))
    assert OPTIONS.keyframes.min_inliers < kept < ratio * counts[many], counts
    matched = np.flatnonzero(tracked.points >= 0)[:kept]
    points = np.full_like(tracked.points, -1)
    points[matched] = tracked.points[matched]
    t = Tracked(pose=tracked.pose, points=points)

    def need(reference: int) -> bool:
        return mapping.need_keyframe(
            m, CAMERA, frame.depth, t, reference=reference, mode="monocular", options=OPTIONS,
            backend=backend,
        )  # fmt: skip

    assert not need(few)
    assert need(many)


def test_points_from_depth_start_nearest(mapped: Mapped) -> None:
    """New points sit at the newest keyframe's free keypoints' depths, nearest first."""
    m = mapped.inserted
    created = mapping.create_points_from_depth(m, CAMERA, OPTIONS)
    rows = np.flatnonzero((created.kp_point >= 0) & (m.kp_point < 0))
    assert len(rows) > 0
    assert np.all(rows >= m.kf_start[-2])
    free = np.arange(m.kf_start[-2], m.kf_start[-1])
    free = free[(m.kp_point[free] < 0) & ~np.isnan(m.kp_depth[free])]
    assert m.kp_depth[rows].max() <= np.sort(m.kp_depth[free])[len(rows) - 1] + 1e-12
    p = created.point_rows(created.kp_point[rows])
    local = created.pt_xyz[p] @ m.kf_pose[-1][:3, :3].T + m.kf_pose[-1][:3, 3]
    np.testing.assert_allclose(local[:, 2], m.kp_depth[rows])


def test_rarely_found_new_points_are_culled(mapped: Mapped) -> None:
    """A recent point found in under a quarter of its views goes; an older one stays."""
    m = mapped.inserted
    found = m.pt_found.copy()
    visible = m.pt_visible.copy()
    recent = np.flatnonzero(m.kf_id[-1] - m.pt_first_kf <= OPTIONS.mapping.recent_point_keyframes)
    old = np.flatnonzero(m.kf_id[-1] - m.pt_first_kf > OPTIONS.mapping.recent_point_keyframes)
    assert len(recent)
    assert len(old)
    visible[[recent[0], old[0]]] = 10
    found[[recent[0], old[0]]] = 2
    culled = mapping.cull_points(replace(m, pt_found=found, pt_visible=visible), "rgbd", OPTIONS)
    assert m.pt_id[recent[0]] not in culled.pt_id
    assert m.pt_id[old[0]] in culled.pt_id


def test_triangulation_matches_free_keypoints_once(mapped: Mapped) -> None:
    """Each pair joins two free keypoints, close in descriptor; no keypoint twice."""
    m = mapped.inserted
    pairs = mapping.match_for_triangulation(m, CAMERA, m.keyframes - 1, m.keyframes - 2, OPTIONS)
    assert len(pairs) > 0
    assert np.all(m.kp_point[pairs] < 0)
    assert len(np.unique(pairs[:, 0])) == len(pairs) == len(np.unique(pairs[:, 1]))
    bits = matching.distances(m.kp_descriptor[pairs[:, 0]], m.kp_descriptor[pairs[:, 1]])
    assert np.all(np.diagonal(bits) <= OPTIONS.matching.th_low)


def test_triangulated_points_reproject_in_both_views(mapped: Mapped) -> None:
    """Every new point is in front of and within the chi-square test in its two keyframes."""
    m = mapped.inserted
    grown = mapping.triangulate_new_points(m, CAMERA, "rgbd", OPTIONS)
    new = np.flatnonzero(~np.isin(grown.pt_id, m.pt_id))
    assert len(new) > 0
    keyframe_of = grown.keypoint_keyframe()
    for p in new:
        rows = np.flatnonzero(grown.kp_point == grown.pt_id[p])
        assert len(rows) == 2
        assert keyframe_of[rows].max() == grown.keyframes - 1
        for row in rows:
            pose = grown.kf_pose[keyframe_of[row]]
            local = grown.pt_xyz[p] @ pose[:3, :3].T + pose[:3, 3]
            error = residuals.chi2(
                CAMERA, local[None], grown.kp_xy[[row]], grown.kp_right[[row]],
                residuals.sigma2(grown, grown.kp_level[[row]]),
            )  # fmt: skip
            assert error[0] < residuals.thresholds(grown.kp_right[[row]], OPTIONS)[0]


def test_fusion_adds_observations(mapped: Mapped) -> None:
    """Fusing finds the newest keyframe's points in its neighbours, and theirs in it."""
    m = mapped.inserted
    fused = mapping.fuse_points(m, CAMERA, "rgbd", OPTIONS)
    assert np.count_nonzero(fused.kp_point >= 0) > np.count_nonzero(m.kp_point >= 0)
    assert fused.points <= m.points


def test_a_redundant_keyframe_is_culled(mapped: Mapped) -> None:
    """A keyframe whose every point three others see at its level goes; the first stays."""
    m, frame, tracked = mapped.map, mapped.frame, mapped.tracked
    for _ in range(4):  # the same view, four times: each copy redundant to the others
        m = maps.insert_keyframe(m, frame, tracked.pose, tracked.points)
    culled = mapping.cull_keyframes(m, CAMERA, "monocular", OPTIONS)
    assert culled.keyframes < m.keyframes
    assert culled.kf_id[0] == 0
    assert culled.kf_id[-1] == m.kf_id[-1]


@pytest.mark.parametrize("mode", MODES)
def test_process_adds_a_keyframe(mode: str, mapped: Mapped) -> None:
    """Local mapping keeps the new keyframe newest, and adds points from it."""
    m = mapped.map
    result = mapping.process(m, CAMERA, mapped.frame, mapped.tracked, mode=mode, options=OPTIONS)
    out = result.map
    assert out.kf_id[-1] == m.next_kf_id
    newest = out.kp_point[out.keypoint_rows(out.keyframes - 1)]
    assert np.count_nonzero(newest >= 0) >= mapped.tracked.inliers // 2
    s = result.summary
    made = s.from_depth + s.triangulated
    assert out.points == m.points + made - s.culled_points - s.fused - s.weak_points
    assert out.keyframes == m.keyframes + 1 - s.culled_keyframes
    assert (s.from_depth == 0) == (mode == "monocular")


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rejects_malformed_arguments(backend: str, mapped: Mapped) -> None:
    """A bad mode, keyframe rows, matches, reference keyframe or an empty map raise first."""
    m = mapped.inserted
    empty = maps.empty(OPTIONS.features.levels, OPTIONS.features.scale_factor)
    short = Tracked(pose=mapped.tracked.pose, points=mapped.tracked.points[:-1])
    calls: list[Callable[[], object]] = [
        lambda: mapping.cull_points(m, "lidar", OPTIONS, backend=backend),
        lambda: mapping.cull_points(empty, "rgbd", OPTIONS, backend=backend),
        lambda: mapping.match_for_triangulation(m, CAMERA, 0, 0, OPTIONS, backend=backend),
        lambda: mapping.match_for_triangulation(
            m, CAMERA, 0, m.keyframes, OPTIONS, backend=backend
        ),
        lambda: mapping.create_points_from_depth(empty, CAMERA, OPTIONS, backend=backend),
        lambda: mapping.cull_keyframes(empty, CAMERA, "rgbd", OPTIONS, backend=backend),
        lambda: mapping.fuse_targets(empty, "rgbd", OPTIONS, backend=backend),
        lambda: mapping.fuse_targets(m, "lidar", OPTIONS, backend=backend),
        lambda: mapping.local_ba_keyframes(empty, OPTIONS, backend=backend),
        *(
            partial(
                mapping.need_keyframe,
                m,
                CAMERA,
                mapped.frame.depth,
                mapped.tracked,
                reference=reference,
                mode="rgbd",
                options=OPTIONS,
                backend=backend,
            )
            for reference in (-1, m.keyframes)
        ),
        lambda: mapping.process(
            m, CAMERA, mapped.frame, short, mode="rgbd", options=OPTIONS, backend=backend
        ),
    ]
    for call in calls:
        with pytest.raises(InvalidInputError):
            call()


@dataclass
class _Asked:
    """Local mapping's thread, as a test sets it: a keyframe waiting, a stop, an abort."""

    queued: bool = False
    stop: bool = False
    abort: bool = False
    published: list[Map] = field(default_factory=list)

    def waiting(self) -> bool:
        """Return whether a keyframe waits."""
        return self.queued

    def stopping(self) -> bool:
        """Return whether a stop is requested."""
        return self.stop

    def aborted(self) -> bool:
        """Return whether the local BA is to stop."""
        return self.abort

    def publish(self, m: Map) -> None:
        """Keep the map published."""
        self.published.append(m)


def _scheduled(mapped: Mapped, asked: _Asked | None) -> mapping.Mapped:
    """Map the frame with `asked` interrupting (None: inline)."""
    return mapping.process(
        mapped.map, CAMERA, mapped.frame, mapped.tracked, mode="rgbd", options=OPTIONS,
        interrupts=asked,
    )  # fmt: skip


def test_inline_every_step_runs(mapped: Mapped) -> None:
    """Uninterrupted, the schedule says every step ran, and the map is as with no thread."""
    inline = _scheduled(mapped, None)
    quiet = _scheduled(mapped, _Asked())
    ran = mapping.Scheduled.RAN
    assert inline.schedule == quiet.schedule == mapping.Schedule(ran, ran, ran)
    assert_maps_match(quiet.map, inline.map)
    assert quiet.summary == inline.summary


def test_a_waiting_keyframe_skips_fusion_ba_and_culling(mapped: Mapped) -> None:
    """With another keyframe queued, only the points are made, then published for tracking."""
    asked = _Asked(queued=True)
    out = _scheduled(mapped, asked)
    skipped = mapping.Scheduled.SKIPPED
    assert out.schedule == mapping.Schedule(skipped, skipped, skipped)
    assert (out.summary.fused, out.summary.solves, out.summary.culled_keyframes) == (0, (), 0)
    assert out.summary.triangulated > 0
    assert len(asked.published) == 1
    assert asked.published[0].points == out.map.points  # the points made, then nothing removed


def test_a_stop_skips_ba_and_culling_not_fusion(mapped: Mapped) -> None:
    """A stop requested passes over the local BA and culling; fusion still runs."""
    out = _scheduled(mapped, _Asked(stop=True))
    scheduled = mapping.Scheduled
    assert out.schedule == mapping.Schedule(scheduled.RAN, scheduled.SKIPPED, scheduled.SKIPPED)
    assert out.summary.solves == ()


def test_an_abort_stops_the_local_ba(mapped: Mapped) -> None:
    """Aborted from its start, the local BA solves nothing; culling still runs."""
    out = _scheduled(mapped, _Asked(abort=True))
    scheduled = mapping.Scheduled
    assert out.schedule == mapping.Schedule(scheduled.RAN, scheduled.ABORTED, scheduled.RAN)
    assert out.summary.solves == ()


def test_a_compiled_backend_refuses_interrupts(mapped: Mapped) -> None:
    """Only the Python reference is interrupted so far: a compiled one is refused, not ignored."""
    with pytest.raises(InvalidInputError, match="interrupts"):
        mapping.process(
            mapped.map, CAMERA, mapped.frame, mapped.tracked, mode="rgbd", options=OPTIONS,
            interrupts=_Asked(), backend="cpp",
        )  # fmt: skip


def test_mapping_busy_asks_for_a_keyframe_less_often(mapped: Mapped) -> None:
    """Busy, a wanted keyframe also needs the frames since the last, or with depth, poor tracking.

    The frame tracks 60 of its reference's points: wanted while idle; busy, monocular wants it
    only once ``tracking.max_frames`` frames have passed; with depth, it is under a quarter
    (ORB-SLAM's ``c1c``) only if the reference sees more than 240.
    """
    m, frame, tracked = mapped.map, mapped.frame, mapped.tracked
    matched = np.flatnonzero(tracked.points >= 0)
    thin = replace(tracked, points=np.where(np.isin(np.arange(len(tracked.points)), matched[:60]),
                                            tracked.points, -1))  # fmt: skip
    ask = partial(mapping.need_keyframe, m, CAMERA, frame.depth, thin,
                  reference=mapped.reference, options=OPTIONS)  # fmt: skip
    seen = m.kp_point[m.keypoint_rows(mapped.reference)]
    rows = m.point_rows(seen[seen >= 0])
    reference_points = int(np.count_nonzero(maps.observation_weight(m, rows[rows >= 0]) >= 3))
    assert ask(mode="monocular")  # idle: wanted
    assert not ask(mode="monocular", idle=False, since_keyframe=0)
    assert ask(mode="monocular", idle=False, since_keyframe=OPTIONS.tracking.max_frames)
    poor = OPTIONS.keyframes.busy_ref_ratio * reference_points > 60
    assert ask(mode="rgbd", idle=False, since_keyframe=0) == poor
    with pytest.raises(InvalidInputError, match="idle"):
        ask(mode="rgbd", idle=False, backend="cpp")
