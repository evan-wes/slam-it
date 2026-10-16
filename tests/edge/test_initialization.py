"""Initialisation's tests: every backend against the reference, and its rules.

On the room: what each start makes of its frames.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.edge import initialization
from slam_it.edge.frames import Frame
from slam_it.edge.initialization import Initialization, Outcome
from slam_it.edge.options import Options
from slam_it.foundations import matching, orb, triangulation, twoview
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import Truth
from tests.edge.support import (
    CAMERA,
    OPTIONS,
    SOLVER_TOLERANCE,
    TOLERANCE,
    assert_close,
    assert_maps_match,
    assert_records_match,
)

Pair = tuple[Frame, Frame]


def _assert_started_match(
    got: Initialization, expected: Initialization, tolerance: dict[str, float]
) -> None:
    """Check two attempts: the same summary, and no map or the same map and tracked frame."""
    assert_records_match(got.summary.as_dict(), expected.summary.as_dict(), tolerance)
    assert (got.started is None) == (expected.started is None)
    if got.started is None or expected.started is None:
        return
    assert_maps_match(got.started[0], expected.started[0], tolerance)
    assert_close(got.started[1].pose, expected.started[1].pose, tolerance)
    np.testing.assert_array_equal(got.started[1].points, expected.started[1].points)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_from_depth_matches_the_reference(backend: str, truth: Truth) -> None:
    """Every backend starts the same map from an RGB-D frame."""
    chosen = require_function(backend, initialization.__name__, "initialize_from_depth")
    frame = truth.frames[0]
    expected = initialization.initialize_from_depth(frame, OPTIONS)
    assert expected.started is not None
    assert expected.summary.outcome is Outcome.INITIALIZED
    got = initialization.initialize_from_depth(frame, OPTIONS, backend=chosen)
    _assert_started_match(got, expected, TOLERANCE)
    exactly = OPTIONS.with_settings(
        {"initialization.min_features_depth": frame.size}
    )  # N > 500: more than
    refused = initialization.initialize_from_depth(frame, exactly, backend=chosen)
    assert refused.started is None
    assert refused.summary.outcome is Outcome.TOO_FEW_FEATURES


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_from_depth_counts_keypoints_not_depths(backend: str, truth: Truth) -> None:
    """The start counts every keypoint, as ORB-SLAM's, not the keypoints with depth.

    Points are made only where there is depth, however few: a map too thin to track is lost and
    reset (review panel finding L13).
    """
    chosen = require_function(backend, initialization.__name__, "initialize_from_depth")
    frame = truth.frames[0]
    options = OPTIONS.with_settings({"initialization.min_features_depth": frame.size - 1})
    for kept in (5, 0):
        sparse = replace(
            frame,
            depth=np.where(np.arange(frame.size) < kept, frame.depth, np.nan),
            right=np.where(np.arange(frame.size) < kept, frame.right, np.nan),
        )
        depths = int(np.count_nonzero(~np.isnan(sparse.depth)))
        started = initialization.initialize_from_depth(sparse, options, backend=chosen).started
        assert started is not None
        assert started[0].points == depths == min(kept, depths)
        assert started[0].keyframes == 1


def test_from_depth_puts_a_point_at_every_keypoint_with_depth(truth: Truth) -> None:
    """One keyframe at the origin; each keypoint with depth sees a point at its depth."""
    frame = truth.frames[0]
    started = initialization.initialize_from_depth(frame, OPTIONS).started
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
    assert len(found.pairs) > OPTIONS.initialization.min_matches
    first, second = reference.features.keypoints, frame.features.keypoints
    assert np.all(first.level[found.pairs[:, 0]] == 0)
    assert np.all(second.level[found.pairs[:, 1]] == 0)
    gap = np.abs(first.xy[found.pairs[:, 0]] - second.xy[found.pairs[:, 1]])
    assert np.all(gap < OPTIONS.initialization.window)
    assert np.all(found.distances <= OPTIONS.matching.th_low)
    assert len(np.unique(found.pairs[:, 1])) == len(found.pairs)


def test_the_search_follows_the_previous_matches(monocular_pair: Pair) -> None:
    """A keypoint is searched where it was last matched, not where it started."""
    reference, frame = monocular_pair
    away = _own(reference) + 10 * OPTIONS.initialization.window
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
    assert expected.started is not None
    summary = expected.summary
    assert (summary.outcome, summary.matches) == (Outcome.INITIALIZED, len(matches.pairs))
    assert len(summary.solves) == 2  # the initial bundle adjustment's
    got = initialization.initialize_monocular(
        CAMERA, *monocular_pair, matches, OPTIONS, backend=chosen
    )
    _assert_started_match(got, expected, SOLVER_TOLERANCE)
    same = (monocular_pair[0], monocular_pair[0])  # no parallax at all
    flat = initialization.initialize_monocular(CAMERA, *same, _matched(same), OPTIONS)
    assert flat.started is None
    assert flat.summary.outcome not in {Outcome.INITIALIZED, Outcome.TOO_FEW_MATCHES}
    _assert_started_match(
        initialization.initialize_monocular(CAMERA, *same, _matched(same), OPTIONS, backend=chosen),
        flat,
        SOLVER_TOLERANCE,
    )


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_monocular_draws_at_most_init_ransac_iterations(backend: str, monocular_pair: Pair) -> None:
    """The two-view RANSAC draws at most ``init_ransac_iterations`` samples, in every backend.

    With a fifth of the matches made outliers, one sample a model starts the map from another
    model than the default's 200 do: a backend that ignored the setting would start the
    default's map.
    """
    chosen = require_function(backend, initialization.__name__, "initialize_monocular")
    matched = _matched(monocular_pair)
    pairs = matched.pairs.copy()
    rng = np.random.default_rng(0)
    wrong = rng.choice(len(pairs), len(pairs) // 5, replace=False)
    pairs[wrong, 1] = rng.permutation(pairs[wrong, 1])
    matches = replace(matched, pairs=pairs)
    one = OPTIONS.with_settings({"initialization.ransac_iterations": 1})
    expected = initialization.initialize_monocular(CAMERA, *monocular_pair, matches, one)
    default = initialization.initialize_monocular(CAMERA, *monocular_pair, matches, OPTIONS)
    assert expected.started is not None
    assert default.started is not None
    assert expected.started[0].points != default.started[0].points
    got = initialization.initialize_monocular(CAMERA, *monocular_pair, matches, one, backend=chosen)
    _assert_started_match(got, expected, SOLVER_TOLERANCE)


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
    assert np.all(np.cos(seen.parallax) < OPTIONS.initialization.parallax_cos)


def _imaged(template: Frame, xy: NDArray[np.float64]) -> Frame:
    """Return a frame of level-0 keypoints at these pixels, without depth, on the room's pyramid."""
    n = len(xy)
    keypoints = orb.Keypoints(
        xy=xy,
        level=np.zeros(n, dtype=np.int64),
        level_xy=np.rint(xy).astype(np.int64),
        response=np.ones(n, dtype=np.int64),
        angle=np.zeros(n, dtype=np.int64),
    )
    features = replace(
        template.features, keypoints=keypoints, descriptors=template.features.descriptors[:n]
    )
    return replace(
        template,
        features=features,
        bearings=CAMERA.bearings(xy),
        depth=np.full(n, np.nan),
        right=np.full(n, np.nan),
    )


def _near_and_far(template: Frame) -> tuple[Frame, Frame, matching.Matches]:
    """Return two views, 10 cm apart, of near points and far ones, matched.

    Seen across the baseline, the near points (1.5 to 3 m) have over 1 degree of parallax, the
    far ones (7 to 13 m) between 0.44 and 0.82 degrees.
    """
    rng = np.random.default_rng(5)
    xy = rng.uniform([40.0, 40.0], [280.0, 200.0], (300, 2))
    depth = np.concatenate([rng.uniform(1.5, 3.0, 200), rng.uniform(7.0, 13.0, 100)])
    rays = CAMERA.bearings(xy)
    points = rays * (depth / rays[:, 2])[:, None]
    seen = CAMERA.project(points - np.array([0.1, 0.0, 0.0]))
    pairs = np.column_stack([np.arange(300), np.arange(300)]).astype(np.int64)
    matches = matching.Matches(pairs=pairs, distances=np.zeros(300, dtype=np.int64))
    return _imaged(template, xy), _imaged(template, seen), matches


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_monocular_keeps_the_far_points_two_view_does_not(
    backend: str, monocular_pair: Pair
) -> None:
    """Points seen at 0.36 to 1 degree of parallax join two-view's (review panel finding H20).

    Two-view triangulates, for its own decision, only points seen at over 1 degree; ORB-SLAM's
    ``CheckRT`` keeps every inlier over 0.36. Of near and far points, only the near ones are
    two-view's: a map of more points has kept far ones, in every backend.
    """
    chosen = require_function(backend, initialization.__name__, "initialize_monocular")
    reference, frame, matches = _near_and_far(monocular_pair[0])
    rays = (reference.bearings, frame.bearings)
    result = twoview.initialize(*rays, twoview.Options(focal=CAMERA.focal, seed=OPTIONS.seed))
    assert result.status is twoview.Status.INITIALIZED
    started = initialization.initialize_monocular(
        CAMERA, reference, frame, matches, OPTIONS, backend=chosen
    ).started
    assert started is not None
    assert started[0].points > np.count_nonzero(result.triangulated)


def test_monocular_map_has_unit_median_depth(monocular_pair: Pair) -> None:
    """Two keyframes, the first at the origin and fixed; the points' median depth is 1."""
    matches = _matched(monocular_pair)
    started = initialization.initialize_monocular(CAMERA, *monocular_pair, matches, OPTIONS).started
    assert started is not None
    m, tracked = started
    assert m.keyframes == 2
    assert m.points >= OPTIONS.initialization.min_matches
    np.testing.assert_array_equal(m.kf_pose[0], np.eye(4))
    assert np.median(m.pt_xyz[:, 2]) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_array_equal(tracked.pose, m.kf_pose[1])
    assert tracked.inliers == np.count_nonzero(m.kp_point[m.keypoint_rows(1)] >= 0)


def test_monocular_needs_enough_matches(monocular_pair: Pair) -> None:
    """With fewer matches than ``init_min_matches``, no map."""
    strict = OPTIONS.with_settings({"initialization.min_matches": 10_000})
    matches = _matched(monocular_pair)
    refused = initialization.initialize_monocular(CAMERA, *monocular_pair, matches, strict)
    assert refused.started is None
    assert refused.summary.as_dict() == {
        "matches": len(matches.pairs),
        "outcome": "too_few_matches",
        "ba": [],
    }


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rejects_frames_of_another_pyramid(
    backend: str, truth: Truth, monocular_pair: Pair
) -> None:
    """A frame extracted with another pyramid raises before any backend runs."""
    other = Options().with_settings({"features.levels": OPTIONS.features.levels - 1})
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
