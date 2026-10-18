"""Relocalisation's tests: frames of the room placed through its keyframes, and the rules.

On the room's truth map, its keyframes' words those of the committed vocabulary.
"""

from __future__ import annotations

import copy

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import frames, relocalization
from slam_it.edge import map as maps
from slam_it.edge.database import Database, Indexed
from slam_it.edge.frames import Frame, Tracked
from slam_it.edge.options import Options
from slam_it.foundations import se3
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge import room
from tests.edge.conftest import LOOP_EVERY, Truth, TwoMaps
from tests.edge.support import CAMERA, OPTIONS, SOLVER_TOLERANCE, assert_close, assert_records_match

NOISE = -1
"""A case's frame index standing for :func:`_noise_frame`."""


def _error(pose: np.ndarray, truth: np.ndarray) -> tuple[float, float]:
    """Return a pose's distance from the truth, in metres and degrees."""
    gap = pose @ se3.inverse(truth)
    angle = np.degrees(np.arccos(np.clip((np.trace(gap[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
    return float(np.linalg.norm(gap[:3, 3])), float(angle)


@pytest.mark.parametrize("index", [3, 18, 30, 39])
def test_a_frame_of_the_room_is_placed_near_its_true_pose(
    truth: Truth, room_database: Database, index: int
) -> None:
    """Placed with tracking.recovery_tracked inliers or more, within 2 cm and half a degree."""
    found = relocalization.relocalize(
        truth.map, room_database, CAMERA, truth.frames[index], OPTIONS
    )
    assert found.tracked is not None
    assert found.tracked.inliers >= OPTIONS.tracking.recovery_tracked
    metres, degrees = _error(found.tracked.pose, truth.poses[index])
    assert metres < 0.02
    assert degrees < 0.5
    s = found.summary
    assert s.placed
    assert s.words > 0
    last = s.candidates[-1]
    assert last.found == found.tracked.inliers
    assert last.matches >= (last.inliers or 0) >= OPTIONS.relocalization.min_inliers
    assert np.all(found.matched[found.tracked.points >= 0] >= 0)


def _noise_frame() -> Frame:
    """Return a frame of a noise image, at a constant depth: features of no place."""
    rng = np.random.default_rng(3)
    noise = rng.integers(0, 256, (CAMERA.height, CAMERA.width), dtype=np.uint8)
    return frames.rgbd(noise, np.full(noise.shape, 2.0), 0.0, CAMERA, OPTIONS.extraction())


def test_a_frame_of_elsewhere_is_not_placed(truth: Truth, room_database: Database) -> None:
    """A noise image's features place nothing: no candidate reaches recovery_tracked."""
    frame = _noise_frame()
    found = relocalization.relocalize(truth.map, room_database, CAMERA, frame, OPTIONS)
    assert found.tracked is None
    assert not found.summary.placed
    assert all((c.found or 0) < OPTIONS.tracking.recovery_tracked for c in found.summary.candidates)


def test_too_few_matches_by_word_skip_every_candidate(
    truth: Truth, room_database: Database
) -> None:
    """A candidate with fewer than relocalization.min_matches never reaches RANSAC."""
    strict = OPTIONS.with_settings({"relocalization.min_matches": 1 << 20})
    found = relocalization.relocalize(truth.map, room_database, CAMERA, truth.frames[30], strict)
    assert found.tracked is None
    assert found.summary.candidates
    assert all(c.iterations is None for c in found.summary.candidates)
    assert np.count_nonzero(found.matched >= 0) == found.summary.candidates[-1].matches


@pytest.mark.parametrize(
    ("matches", "needed", "samples"),
    [
        (100, 50, 35),  # half the matches, at 0.99: log(0.01) / log(1 - 0.5^3)
        (15, 10, 14),  # relocalization.min_inliers binds: e = 10 / 15
        (10, 10, 1),  # every match must be an inlier: one sample
        (8, 10, 1),  # fewer matches than inliers needed: never drawn
    ],
)
def test_the_ransac_budget_is_mlpnps(matches: int, needed: int, samples: int) -> None:
    """MLPnP's SetRansacParameters: the inliers needed, and the samples that find them."""
    assert relocalization.ransac_budget(matches, OPTIONS) == (needed, samples)
    capped = OPTIONS.with_settings({"relocalization.ransac_iterations": 20})
    assert relocalization.ransac_budget(matches, capped) == (needed, min(samples, 20))


def test_the_searches_by_projection_top_up_a_weak_start(
    truth: Truth, room_database: Database
) -> None:
    """Asked for more inliers than matches by word give, the searches add the keyframe's others."""
    demanding = OPTIONS.with_settings({"tracking.recovery_tracked": 150})
    found = relocalization.relocalize(truth.map, room_database, CAMERA, truth.frames[30], demanding)
    gains = [
        c.found - c.refined
        for c in found.summary.candidates
        if c.found is not None and c.refined is not None
    ]
    assert any(gain > 0 for gain in gains)


def test_between_the_bounds_after_the_first_search_a_narrower_one_runs(
    truth: Truth, room_database: Database, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Over relocalization.narrow_inliers, under tracking.recovery_tracked: the narrow search."""
    searched: list[tuple[float, int]] = []
    original = relocalization.search_keyframe

    def recorded(*args: object, **kwargs: object) -> np.ndarray:
        searched.append((kwargs["window"], kwargs["distance"]))  # type: ignore[arg-type]
        return original(*args, **kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(relocalization, "search_keyframe", recorded)
    demanding = OPTIONS.with_settings({"tracking.recovery_tracked": 175})
    found = relocalization.relocalize(truth.map, room_database, CAMERA, truth.frames[6], demanding)
    assert searched == [
        (OPTIONS.relocalization.window, OPTIONS.matching.th_high),
        (OPTIONS.relocalization.narrow_window, OPTIONS.relocalization.narrow_distance),
    ]
    last = found.summary.candidates[-1]
    assert last.found is not None
    assert OPTIONS.relocalization.narrow_inliers < last.found < demanding.tracking.recovery_tracked
    assert found.tracked is None


def test_a_keyframe_searched_at_the_true_pose_finds_its_points(truth: Truth) -> None:
    """Every match lies near its point's projection; the points skipped are not found."""
    m = truth.map
    frame, pose = truth.frames[37], truth.poses[37]
    start = Tracked(pose=pose, points=np.full(frame.size, -1, dtype=np.int64))
    found = relocalization.search_keyframe(
        m, CAMERA, frame, start, 3,
        skip=np.zeros(0, dtype=np.int64), window=OPTIONS.relocalization.window,
        distance=OPTIONS.matching.th_high, options=OPTIONS,
    )  # fmt: skip
    hits = np.flatnonzero(found >= 0)
    assert len(hits) > 50
    xyz = m.pt_xyz[m.point_rows(found[hits])]
    local = xyz @ pose[:3, :3].T + pose[:3, 3]
    gap = np.linalg.norm(CAMERA.project(local) - frame.features.keypoints.xy[hits], axis=1)
    assert np.median(gap) < 2.0
    skipped = relocalization.search_keyframe(
        m, CAMERA, frame, start, 3,
        skip=found[hits], window=OPTIONS.relocalization.window,
        distance=OPTIONS.matching.th_high, options=OPTIONS,
    )  # fmt: skip
    assert not set(skipped[skipped >= 0].tolist()) & set(found[hits].tolist())


DECISIVE = [
    ("relocalization.inlier_ratio", 1.0, 6, 175),
    ("relocalization.min_inliers", 85, 30, 150),
    ("relocalization.confidence", 0.95, 6, 175),
    ("relocalization.ransac_iterations", 1, 6, 175),
    ("relocalization.window", 2.0, 30, 150),
    ("relocalization.narrow_inliers", 168, 6, 170),
    ("relocalization.narrow_window", 2.0, 6, 170),
    ("relocalization.narrow_distance", 50, 6, 170),
]
"""Each of relocalisation's settings off its default (the setting, its value, the frame and
``tracking.recovery_tracked``) where it alone changes Python's decisions (review panel findings
L1 and L9, by finding H21's rule): a native that hard-coded it would decide unlike Python
there. At 170 inliers needed, frame 6's narrow search decides; at 175, its RANSAC; frame 30's
wide search at 150, and its 84 matches against a minimum of 85."""

COVERED_ELSEWHERE = {
    "relocalization.min_matches",
    "relocalization.common_words",
    "relocalization.neighbours",
    "relocalization.group_score",
}
"""Relocalisation's settings moved by other tests: the matches by word below, the candidates'
in the database's tests."""


def _moved(setting: str, value: float, recovery_tracked: int) -> Options:
    """Return the settings with one moved, and ``tracking.recovery_tracked`` set."""
    return OPTIONS.with_settings({"tracking.recovery_tracked": recovery_tracked, setting: value})


def _decided(found: relocalization.Relocalized) -> tuple[object, ...]:
    """Return what a relocalisation decided: its summary, the points it placed with, its matches."""
    points = None if found.tracked is None else found.tracked.points.tolist()
    return found.summary.as_dict(), points, found.matched.tolist()


def test_every_relocalisation_setting_is_moved_where_it_decides() -> None:
    """Each of relocalisation's own settings is in :data:`DECISIVE`, or moved by another test.

    Its switch and its ratio test, which the fallback shares, are not counted, as before.
    """
    settings = {name for name in OPTIONS.settings() if name.startswith("relocalization.")}
    settings -= {"relocalization.enabled", "relocalization.ratio"}
    assert settings == {d[0] for d in DECISIVE} | COVERED_ELSEWHERE


@pytest.mark.parametrize("case", DECISIVE, ids=[d[0] for d in DECISIVE])
def test_each_moved_setting_alone_changes_what_relocalisation_decides(
    case: tuple[str, float, int, int], truth: Truth, room_database: Database
) -> None:
    """Moved alone, the setting changes Python's result.

    The backends' agreement on the same case (below) then shows that the natives read it.
    """
    setting, value, index, recovery_tracked = case
    frame = truth.frames[index]
    moved = relocalization.relocalize(
        truth.map, room_database, CAMERA, frame, _moved(setting, value, recovery_tracked)
    )
    at_default = OPTIONS.with_settings({"tracking.recovery_tracked": recovery_tracked})
    default = relocalization.relocalize(truth.map, room_database, CAMERA, frame, at_default)
    assert _decided(moved) != _decided(default)


def test_the_narrow_search_can_place_the_frame(truth: Truth, room_database: Database) -> None:
    """Needing 169 inliers, frame 6 is placed by the narrow search's last optimisation.

    The second optimisation keeps 168; the narrow search adds the rest, and the frame's points
    are the last optimisation's (the second's outliers kept until it).
    """
    demanding = OPTIONS.with_settings({"tracking.recovery_tracked": 169})
    found = relocalization.relocalize(truth.map, room_database, CAMERA, truth.frames[6], demanding)
    assert found.tracked is not None
    last = found.summary.candidates[-1]
    assert last.found is not None
    assert last.found >= demanding.tracking.recovery_tracked > 168


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("index", "settings"),
    [
        (3, OPTIONS),
        (18, OPTIONS),
        (30, OPTIONS),
        (39, OPTIONS),
        (NOISE, OPTIONS),
        (30, OPTIONS.with_settings({"relocalization.min_matches": 1 << 20})),
        (30, OPTIONS.with_settings({"tracking.recovery_tracked": 150})),
        (6, OPTIONS.with_settings({"tracking.recovery_tracked": 175})),
        (6, OPTIONS.with_settings({"tracking.recovery_tracked": 169})),
        *[(index, _moved(setting, value, rt)) for setting, value, index, rt in DECISIVE],
    ],
    ids=[
        "3",
        "18",
        "30",
        "39",
        "noise",
        "too few matches",
        "wide search",
        "narrow search",
        "narrow search places",
        *[f"{d[0]} moved" for d in DECISIVE],
    ],
)
def test_every_backend_relocalizes_as_python_does(
    backend: str, index: int, settings: Options, truth: Truth, room_database: Database
) -> None:
    """The same decisions exactly; the poses to the solver tolerance (they optimise)."""
    chosen = require_function(backend, relocalization.__name__, "relocalize")
    frame = _noise_frame() if index == NOISE else truth.frames[index]
    expected = relocalization.relocalize(truth.map, room_database, CAMERA, frame, settings)
    got = relocalization.relocalize(
        truth.map, room_database, CAMERA, frame, settings, backend=chosen
    )
    assert (got.tracked is None) == (expected.tracked is None)
    if got.tracked is not None and expected.tracked is not None:
        assert_close(got.tracked.pose, expected.tracked.pose, SOLVER_TOLERANCE)
        np.testing.assert_array_equal(got.tracked.points, expected.tracked.points)
    np.testing.assert_array_equal(got.matched, expected.matched)
    assert_records_match(got.summary.as_dict(), expected.summary.as_dict(), SOLVER_TOLERANCE)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_candidate_the_map_lacks_is_refused(
    backend: str, truth: Truth, room_database: Database
) -> None:
    """A database out of step with its map (a keyframe it does not hold): InvalidInputError."""
    chosen = require_function(backend, relocalization.__name__, "relocalize")
    db = Database(room_database.vocabulary)
    db.add(99, truth.frames[30].features.descriptors)
    with pytest.raises(InvalidInputError, match="keyframe 99 is not in the map"):
        relocalization.relocalize(truth.map, db, CAMERA, truth.frames[30], OPTIONS, backend=chosen)


# --- relocalisation in every map (milestone 2B's B4) --------------------------------------


def _frame(index: int) -> tuple[Frame, np.ndarray]:
    """Render the loop room's frame at an index, in RGB-D, and return it with its true pose."""
    pose = room.loop_path(room.LOOP_FRAMES)[index]
    gray, depth, _ = room.Room().render(room.SMALL, pose)
    built = frames.rgbd(gray, depth, index / 30.0, CAMERA, OPTIONS.extraction())
    return built, np.linalg.inv(pose)


def _both(two: TwoMaps) -> list[Indexed]:
    """Return the two maps, the matched one numbered 0 and the active 1."""
    return [
        Indexed(0, two.matched, two.matched_database),
        Indexed(1, two.active, two.active_database),
    ]


@pytest.mark.parametrize(
    ("index", "number"),
    [(6 * LOOP_EVERY + 5, 0), (20 * LOOP_EVERY + 5, 1)],
    ids=["first", "active"],
)
def test_a_frame_is_placed_in_the_map_that_saw_its_place(
    two_rigid: TwoMaps, index: int, number: int
) -> None:
    """A frame only one map saw is placed in that map, in its world, within 2 cm of its place.

    Each map's candidates are tried, best first: the first map's place a frame 98 degrees round
    the loop room, the active map's one 309 degrees round (the active map starts at 210).
    """
    frame, true = _frame(index)
    across = relocalization.relocalize_across(_both(two_rigid), CAMERA, frame, OPTIONS)
    assert across.map == number
    assert across.relocalized.tracked is not None
    assert across.relocalized.summary.placed
    assert across.maps[-1] == number
    assert len(across.maps) == len(across.relocalized.summary.candidates)
    in_world = true if number == 0 else true @ np.linalg.inv(two_rigid.world)
    placed = across.relocalized.tracked.pose
    assert np.linalg.norm(maps.camera_centre(placed) - maps.camera_centre(in_world)) < 0.02


def test_relocalisation_in_every_map_refuses_what_it_cannot_search(two_rigid: TwoMaps) -> None:
    """No map, a map twice, or two vocabularies: InvalidInputError."""
    frame, _ = _frame(65)
    both = _both(two_rigid)
    with pytest.raises(InvalidInputError, match="at least one map"):
        relocalization.relocalize_across([], CAMERA, frame, OPTIONS)
    with pytest.raises(InvalidInputError, match="each map once"):
        relocalization.relocalize_across([both[0], both[0]], CAMERA, frame, OPTIONS)
    alien = Indexed(
        5, two_rigid.matched, Database(copy.copy(two_rigid.matched_database.vocabulary))
    )
    with pytest.raises(InvalidInputError, match="one vocabulary"):
        relocalization.relocalize_across([*both, alien], CAMERA, frame, OPTIONS)
