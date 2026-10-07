"""Initialisation's tests: every backend against the reference, and its rules.

On the room: what each start makes of its frames.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.edge import initialization
from slamlab.edge.frames import Frame, Tracked
from slamlab.edge.map import Map
from slamlab.edge.options import Options
from slamlab.foundations import matching, triangulation, twoview
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import CAMERA, OPTIONS, Truth
from tests.edge.support import SOLVER_TOLERANCE, TOLERANCE, assert_close, assert_maps_match

Pair = tuple[Frame, Frame]


def _assert_started_match(
    got: tuple[Map, Tracked] | None,
    expected: tuple[Map, Tracked] | None,
    tolerance: dict[str, float],
) -> None:
    """Check two starts: both None, or the same map and tracked frame."""
    assert (got is None) == (expected is None)
    if got is None or expected is None:
        return
    assert_maps_match(got[0], expected[0], tolerance)
    assert_close(got[1].pose, expected[1].pose, tolerance)
    np.testing.assert_array_equal(got[1].points, expected[1].points)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_from_depth_matches_the_reference(backend: str, truth: Truth) -> None:
    """Every backend starts the same map from an RGB-D frame."""
    chosen = require_function(backend, initialization.__name__, "initialize_from_depth")
    frame = truth.frames[0]
    expected = initialization.initialize_from_depth(frame, OPTIONS)
    assert expected is not None
    got = initialization.initialize_from_depth(frame, OPTIONS, backend=chosen)
    _assert_started_match(got, expected, TOLERANCE)
    exactly = replace(OPTIONS, min_init_features_depth=frame.size)  # N > 500: more than
    assert initialization.initialize_from_depth(frame, exactly, backend=chosen) is None


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_from_depth_counts_keypoints_then_depths(backend: str, truth: Truth) -> None:
    """The start counts every keypoint, as ORB-SLAM's, then needs a floor of them with depth.

    Points are made only where there is depth; the floor stands in for ORB-SLAM's reset of a
    map lost soon after it started (review panel finding L13).
    """
    chosen = require_function(backend, initialization.__name__, "initialize_from_depth")
    frame = truth.frames[0]
    sparse = replace(
        frame,
        depth=np.where(np.arange(frame.size) < 5, frame.depth, np.nan),
        right=np.where(np.arange(frame.size) < 5, frame.right, np.nan),
    )
    depths = int(np.count_nonzero(~np.isnan(sparse.depth)))
    assert depths > 0
    options = replace(OPTIONS, min_init_features_depth=frame.size - 1, min_init_depth_points=depths)
    started = initialization.initialize_from_depth(sparse, options, backend=chosen)
    assert started is not None
    assert started[0].points == depths
    deeper = replace(options, min_init_depth_points=depths + 1)
    assert initialization.initialize_from_depth(sparse, deeper, backend=chosen) is None


def test_from_depth_puts_a_point_at_every_keypoint_with_depth(truth: Truth) -> None:
    """One keyframe at the origin; each keypoint with depth sees a point at its depth."""
    frame = truth.frames[0]
    started = initialization.initialize_from_depth(frame, OPTIONS)
    assert started is not None
    m, tracked = started
    has_depth = ~np.isnan(frame.depth)
    assert m.keyframes == 1
    assert m.points == tracked.inliers == np.count_nonzero(has_depth)
    np.testing.assert_array_equal(tracked.pose, np.eye(4))
    np.testing.assert_array_equal(tracked.points >= 0, has_depth)
    np.testing.assert_allclose(m.pt_xyz[:, 2], frame.depth[has_depth])


def _own(frame: Frame) -> NDArray[np.float64]:
    """Return a reference's keypoints' own positions: where a search starts."""
    return frame.features.keypoints.xy.copy()


def _matched(pair: Pair, options: Options = OPTIONS) -> matching.Matches:
    """Match a pair from the reference's own positions."""
    return initialization.match_for_initialization(*pair, _own(pair[0]), options)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_matching_matches_the_reference(backend: str, monocular_pair: Pair) -> None:
    """Every backend finds the same initialisation matches, from any previous positions."""
    chosen = require_function(backend, initialization.__name__, "match_for_initialization")
    moved = _own(monocular_pair[0])
    moved[::2] += 3.0  # half the keypoints last matched elsewhere
    for previous in (_own(monocular_pair[0]), moved):
        expected = initialization.match_for_initialization(*monocular_pair, previous, OPTIONS)
        got = initialization.match_for_initialization(
            *monocular_pair, previous, OPTIONS, backend=chosen
        )
        np.testing.assert_array_equal(got.pairs, expected.pairs)
        np.testing.assert_array_equal(got.distances, expected.distances)


def test_matches_pair_level_zero_keypoints_within_the_window(monocular_pair: Pair) -> None:
    """Each match joins two level-0 keypoints within the window, close in descriptor."""
    reference, frame = monocular_pair
    found = _matched(monocular_pair)
    assert len(found.pairs) > OPTIONS.init_min_matches
    first, second = reference.features.keypoints, frame.features.keypoints
    assert np.all(first.level[found.pairs[:, 0]] == 0)
    assert np.all(second.level[found.pairs[:, 1]] == 0)
    gap = np.abs(first.xy[found.pairs[:, 0]] - second.xy[found.pairs[:, 1]])
    assert np.all(gap < OPTIONS.init_window)
    assert np.all(found.distances <= OPTIONS.th_low)
    assert len(np.unique(found.pairs[:, 1])) == len(found.pairs)


def test_the_search_follows_the_previous_matches(monocular_pair: Pair) -> None:
    """A keypoint is searched where it was last matched, not where it started."""
    reference, frame = monocular_pair
    away = _own(reference) + 10 * OPTIONS.init_window
    assert len(initialization.match_for_initialization(*monocular_pair, away, OPTIONS).pairs) == 0
    found = _matched(monocular_pair)
    followed = _own(reference)
    followed[found.pairs[:, 0]] = frame.features.keypoints.xy[found.pairs[:, 1]]
    again = initialization.match_for_initialization(*monocular_pair, followed, OPTIONS)
    assert len(again.pairs) >= len(found.pairs)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_monocular_matches_the_reference(backend: str, monocular_pair: Pair) -> None:
    """Every backend starts the same monocular map, and agrees when it does not start."""
    chosen = require_function(backend, initialization.__name__, "initialize_monocular")
    matches = _matched(monocular_pair)
    expected = initialization.initialize_monocular(CAMERA, *monocular_pair, matches, OPTIONS)
    assert expected is not None
    got = initialization.initialize_monocular(
        CAMERA, *monocular_pair, matches, OPTIONS, backend=chosen
    )
    _assert_started_match(got, expected, SOLVER_TOLERANCE)
    same = (monocular_pair[0], monocular_pair[0])  # no parallax at all
    assert (
        initialization.initialize_monocular(CAMERA, *same, _matched(same), OPTIONS, backend=chosen)
        is None
    )


def test_monocular_keeps_points_of_orb_slams_parallax(monocular_pair: Pair) -> None:
    """Every two-view point is kept, and the inliers between 0.36 and 1 degree too (CheckRT)."""
    reference, frame = monocular_pair
    pairs = _matched(monocular_pair).pairs
    rays = (reference.bearings[pairs[:, 0]], frame.bearings[pairs[:, 1]])
    result = twoview.initialize(*rays, twoview.Options(focal=CAMERA.focal, seed=OPTIONS.seed))
    assert result.status is twoview.Status.INITIALIZED
    kept, xyz = initialization._kept_points(result, *rays, OPTIONS)
    assert set(np.flatnonzero(result.triangulated)) <= set(kept)
    np.testing.assert_allclose(xyz[np.isin(kept, np.flatnonzero(result.triangulated))],
                               result.points[result.triangulated], atol=1e-12)  # fmt: skip
    seen = triangulation.two_view(result.relative_pose, rays[0][kept], rays[1][kept])
    assert np.all(seen.in_front)
    assert np.all(np.cos(seen.parallax) < OPTIONS.init_parallax_cos)


def test_monocular_map_has_unit_median_depth(monocular_pair: Pair) -> None:
    """Two keyframes, the first at the origin and fixed; the points' median depth is 1."""
    matches = _matched(monocular_pair)
    started = initialization.initialize_monocular(CAMERA, *monocular_pair, matches, OPTIONS)
    assert started is not None
    m, tracked = started
    assert m.keyframes == 2
    assert m.points >= OPTIONS.init_min_matches
    np.testing.assert_array_equal(m.kf_pose[0], np.eye(4))
    assert np.median(m.pt_xyz[:, 2]) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_array_equal(tracked.pose, m.kf_pose[1])
    assert tracked.inliers == np.count_nonzero(m.kp_point[m.keypoint_rows(1)] >= 0)


def test_monocular_needs_enough_matches(monocular_pair: Pair) -> None:
    """With fewer matches than ``init_min_matches``, no map."""
    strict = replace(OPTIONS, init_min_matches=10_000)
    matches = _matched(monocular_pair)
    assert initialization.initialize_monocular(CAMERA, *monocular_pair, matches, strict) is None


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rejects_frames_of_another_pyramid(
    backend: str, truth: Truth, monocular_pair: Pair
) -> None:
    """A frame extracted with another pyramid raises before any backend runs."""
    other = Options(levels=OPTIONS.levels - 1)
    with pytest.raises(InvalidInputError, match="pyramid"):
        initialization.initialize_from_depth(truth.frames[0], other, backend=backend)
    own, matches = _own(monocular_pair[0]), _matched(monocular_pair)
    with pytest.raises(InvalidInputError, match="pyramid"):
        initialization.match_for_initialization(*monocular_pair, own, other, backend=backend)
    with pytest.raises(InvalidInputError, match="pyramid"):
        initialization.initialize_monocular(
            CAMERA, *monocular_pair, matches, other, backend=backend
        )
    with pytest.raises(InvalidInputError, match="previous"):
        initialization.match_for_initialization(*monocular_pair, own[1:], OPTIONS, backend=backend)
    shifted = matches.pairs + monocular_pair[1].size
    beyond = matching.Matches(pairs=shifted, distances=matches.distances)
    with pytest.raises(InvalidInputError, match="pairs"):
        initialization.initialize_monocular(
            CAMERA, *monocular_pair, beyond, OPTIONS, backend=backend
        )
