"""Initialisation's tests: every backend against the reference, and its rules.

On the room: what each start makes of its frames.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from slamlab import backends
from slamlab.edge import initialization
from slamlab.edge.frames import Frame, Tracked
from slamlab.edge.map import Map
from slamlab.edge.options import TH_LOW, Options
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
    too_many = replace(OPTIONS, min_init_features_depth=frame.size + 1)
    assert initialization.initialize_from_depth(frame, too_many, backend=chosen) is None


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_from_depth_counts_every_keypoint(backend: str, truth: Truth) -> None:
    """The start needs keypoints, not depths: points are made only where there is depth."""
    chosen = require_function(backend, initialization.__name__, "initialize_from_depth")
    frame = truth.frames[0]
    sparse = replace(
        frame,
        depth=np.where(np.arange(frame.size) < 5, frame.depth, np.nan),
        right=np.where(np.arange(frame.size) < 5, frame.right, np.nan),
    )
    options = replace(OPTIONS, min_init_features_depth=frame.size)
    started = initialization.initialize_from_depth(sparse, options, backend=chosen)
    assert started is not None
    assert started[0].points == np.count_nonzero(~np.isnan(sparse.depth)) > 0


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


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_matching_matches_the_reference(backend: str, monocular_pair: Pair) -> None:
    """Every backend finds the same initialisation matches."""
    chosen = require_function(backend, initialization.__name__, "match_for_initialization")
    expected = initialization.match_for_initialization(*monocular_pair, OPTIONS)
    got = initialization.match_for_initialization(*monocular_pair, OPTIONS, backend=chosen)
    np.testing.assert_array_equal(got.pairs, expected.pairs)
    np.testing.assert_array_equal(got.distances, expected.distances)


def test_matches_pair_level_zero_keypoints_within_the_window(monocular_pair: Pair) -> None:
    """Each match joins two level-0 keypoints within the window, close in descriptor."""
    reference, frame = monocular_pair
    found = initialization.match_for_initialization(reference, frame, OPTIONS)
    assert len(found.pairs) > OPTIONS.init_min_matches
    first, second = reference.features.keypoints, frame.features.keypoints
    assert np.all(first.level[found.pairs[:, 0]] == 0)
    assert np.all(second.level[found.pairs[:, 1]] == 0)
    gap = np.abs(first.xy[found.pairs[:, 0]] - second.xy[found.pairs[:, 1]])
    assert np.all(gap < OPTIONS.init_window)
    assert np.all(found.distances <= TH_LOW)
    assert len(np.unique(found.pairs[:, 1])) == len(found.pairs)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_monocular_matches_the_reference(backend: str, monocular_pair: Pair) -> None:
    """Every backend starts the same monocular map, and agrees when it does not start."""
    chosen = require_function(backend, initialization.__name__, "initialize_monocular")
    expected = initialization.initialize_monocular(CAMERA, *monocular_pair, OPTIONS)
    assert expected is not None
    got = initialization.initialize_monocular(CAMERA, *monocular_pair, OPTIONS, backend=chosen)
    _assert_started_match(got, expected, SOLVER_TOLERANCE)
    same = (monocular_pair[0], monocular_pair[0])  # no parallax at all
    assert initialization.initialize_monocular(CAMERA, *same, OPTIONS, backend=chosen) is None


def test_monocular_map_has_unit_median_depth(monocular_pair: Pair) -> None:
    """Two keyframes, the first at the origin and fixed; the points' median depth is 1."""
    started = initialization.initialize_monocular(CAMERA, *monocular_pair, OPTIONS)
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
    assert initialization.initialize_monocular(CAMERA, *monocular_pair, strict) is None


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rejects_frames_of_another_pyramid(
    backend: str, truth: Truth, monocular_pair: Pair
) -> None:
    """A frame extracted with another pyramid raises before any backend runs."""
    other = Options(levels=OPTIONS.levels - 1)
    with pytest.raises(InvalidInputError, match="pyramid"):
        initialization.initialize_from_depth(truth.frames[0], other, backend=backend)
    with pytest.raises(InvalidInputError, match="pyramid"):
        initialization.match_for_initialization(*monocular_pair, other, backend=backend)
    with pytest.raises(InvalidInputError, match="pyramid"):
        initialization.initialize_monocular(CAMERA, *monocular_pair, other, backend=backend)
