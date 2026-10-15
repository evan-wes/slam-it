"""Relocalisation's tests: frames of the room placed through its keyframes, and the rules.

On the room's truth map, its keyframes' words those of the committed vocabulary.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import frames, relocalization
from slam_it.edge.database import Database
from slam_it.edge.frames import Frame, Tracked
from slam_it.edge.options import Options
from slam_it.foundations import se3
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import Truth
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
    """Placed with recovery_tracked inliers or more, within 2 cm and half a degree."""
    found = relocalization.relocalize(
        truth.map, room_database, CAMERA, truth.frames[index], OPTIONS
    )
    assert found.tracked is not None
    assert found.tracked.inliers >= OPTIONS.recovery_tracked
    metres, degrees = _error(found.tracked.pose, truth.poses[index])
    assert metres < 0.02
    assert degrees < 0.5
    s = found.summary
    assert s.placed
    assert s.words > 0
    last = s.candidates[-1]
    assert last.found == found.tracked.inliers
    assert last.matches >= (last.inliers or 0) >= OPTIONS.reloc_min_inliers
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
    assert all((c.found or 0) < OPTIONS.recovery_tracked for c in found.summary.candidates)


def test_too_few_matches_by_word_skip_every_candidate(
    truth: Truth, room_database: Database
) -> None:
    """A candidate with fewer than reloc_min_matches never reaches RANSAC."""
    strict = replace(OPTIONS, reloc_min_matches=1 << 20)
    found = relocalization.relocalize(truth.map, room_database, CAMERA, truth.frames[30], strict)
    assert found.tracked is None
    assert found.summary.candidates
    assert all(c.iterations is None for c in found.summary.candidates)
    assert np.count_nonzero(found.matched >= 0) == found.summary.candidates[-1].matches


@pytest.mark.parametrize(
    ("matches", "needed", "samples"),
    [
        (100, 50, 35),  # half the matches, at 0.99: log(0.01) / log(1 - 0.5^3)
        (15, 10, 14),  # reloc_min_inliers binds: e = 10 / 15
        (10, 10, 1),  # every match must be an inlier: one sample
        (8, 10, 1),  # fewer matches than inliers needed: never drawn
    ],
)
def test_the_ransac_budget_is_mlpnps(matches: int, needed: int, samples: int) -> None:
    """MLPnP's SetRansacParameters: the inliers needed, and the samples that find them."""
    assert relocalization.ransac_budget(matches, OPTIONS) == (needed, samples)
    capped = replace(OPTIONS, reloc_ransac_iterations=20)
    assert relocalization.ransac_budget(matches, capped) == (needed, min(samples, 20))


def test_the_searches_by_projection_top_up_a_weak_start(
    truth: Truth, room_database: Database
) -> None:
    """Asked for more inliers than matches by word give, the searches add the keyframe's others."""
    demanding = replace(OPTIONS, recovery_tracked=150)
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
    """More than reloc_narrow_inliers but fewer than recovery_tracked: the narrow search runs."""
    searched: list[tuple[float, int]] = []
    original = relocalization.search_keyframe

    def recorded(*args: object, **kwargs: object) -> np.ndarray:
        searched.append((kwargs["window"], kwargs["distance"]))  # type: ignore[arg-type]
        return original(*args, **kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(relocalization, "search_keyframe", recorded)
    demanding = replace(OPTIONS, recovery_tracked=175)
    found = relocalization.relocalize(truth.map, room_database, CAMERA, truth.frames[6], demanding)
    assert searched == [
        (OPTIONS.reloc_window, OPTIONS.th_high),
        (OPTIONS.reloc_narrow_window, OPTIONS.reloc_narrow_distance),
    ]
    last = found.summary.candidates[-1]
    assert last.found is not None
    assert OPTIONS.reloc_narrow_inliers < last.found < demanding.recovery_tracked
    assert found.tracked is None


def test_a_keyframe_searched_at_the_true_pose_finds_its_points(truth: Truth) -> None:
    """Every match lies near its point's projection; the points skipped are not found."""
    m = truth.map
    frame, pose = truth.frames[37], truth.poses[37]
    start = Tracked(pose=pose, points=np.full(frame.size, -1, dtype=np.int64))
    found = relocalization.search_keyframe(
        m, CAMERA, frame, start, 3,
        skip=np.zeros(0, dtype=np.int64), window=OPTIONS.reloc_window,
        distance=OPTIONS.th_high, options=OPTIONS,
    )  # fmt: skip
    hits = np.flatnonzero(found >= 0)
    assert len(hits) > 50
    xyz = m.pt_xyz[m.point_rows(found[hits])]
    local = xyz @ pose[:3, :3].T + pose[:3, 3]
    gap = np.linalg.norm(CAMERA.project(local) - frame.features.keypoints.xy[hits], axis=1)
    assert np.median(gap) < 2.0
    skipped = relocalization.search_keyframe(
        m, CAMERA, frame, start, 3,
        skip=found[hits], window=OPTIONS.reloc_window, distance=OPTIONS.th_high, options=OPTIONS,
    )  # fmt: skip
    assert not set(skipped[skipped >= 0].tolist()) & set(found[hits].tolist())


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("index", "settings"),
    [
        (3, OPTIONS),
        (18, OPTIONS),
        (30, OPTIONS),
        (39, OPTIONS),
        (NOISE, OPTIONS),
        (30, replace(OPTIONS, reloc_min_matches=1 << 20)),
        (30, replace(OPTIONS, recovery_tracked=150)),
        (6, replace(OPTIONS, recovery_tracked=175)),
    ],
    ids=["3", "18", "30", "39", "noise", "too few matches", "wide search", "narrow search"],
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
