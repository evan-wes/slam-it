"""Descriptor-matching tests: exact parity, OpenCV's Hamming norm, the filters, failure modes."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

import cv2
import numpy as np
import pytest
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations import matching, orb
from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.orb import DESCRIPTOR_BYTES
from tests.conftest import (
    check_rejects,
    load_fixture,
    parity_params,
    protocol_members,
    require_function,
)

MATCHING = load_fixture("foundations/matching.json")


def _descriptors(value: object) -> NDArray[np.uint8]:
    """Return a fixture's descriptor set as an (N, 32) uint8 array (empty sets included)."""
    return np.asarray(value, dtype=np.uint8).reshape(-1, DESCRIPTOR_BYTES)


def _check_int64(got: object, expected: object) -> None:
    """Check an int64 array against expected values, exactly."""
    assert isinstance(got, np.ndarray)
    assert got.dtype == np.int64
    np.testing.assert_array_equal(got, np.asarray(expected, dtype=np.int64).reshape(got.shape))


def _column(values: list[float | None]) -> NDArray[np.float64]:
    """Return a fixture's right columns, ``null`` as NaN."""
    return np.array([np.nan if v is None else v for v in values], dtype=np.float64)


def _projections(raw: dict[str, Any]) -> matching.Projections:
    """Build a fixture case's projections."""
    return matching.Projections(
        xy=np.asarray(raw["xy"], dtype=np.float64).reshape(-1, 2),
        radius=np.asarray(raw["radius"], dtype=np.float64),
        min_level=np.asarray(raw["min_level"], dtype=np.int64),
        max_level=np.asarray(raw["max_level"], dtype=np.int64),
        descriptors=_descriptors(raw["descriptors"]),
        angle=np.asarray(raw["angle"], dtype=np.int64),
        right=_column(raw["right"]),
    )


def _features(
    xy: ArrayLike, level: ArrayLike, angle: ArrayLike, descriptors: ArrayLike, levels: int = 8
) -> orb.Features:
    """Build features from their matched fields (pixels and responses zero)."""
    positions = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    n = len(positions)
    keypoints = orb.Keypoints(
        xy=positions,
        level=np.asarray(level, dtype=np.int64).reshape(n),
        level_xy=np.zeros((n, 2), dtype=np.int64),
        response=np.zeros(n, dtype=np.int64),
        angle=np.asarray(angle, dtype=np.int64).reshape(n),
    )
    return orb.Features(keypoints, _descriptors(descriptors), 1.2, levels)


def _frame(raw: dict[str, Any]) -> tuple[orb.Features, NDArray[np.float64], NDArray[np.bool_]]:
    """Build a fixture case's frame: its features, right columns and taken flags."""
    features = _features(raw["xy"], raw["level"], raw["angle"], raw["descriptors"], raw["levels"])
    return features, _column(raw["right"]), np.asarray(raw["taken"], dtype=bool)


def _planted(
    n: int, extra: int, flips: int, seed: int
) -> tuple[NDArray[np.uint8], NDArray[np.uint8], NDArray[np.int64]]:
    """Draw two sets sharing `n` noisy copies (the second shuffled); say where each went."""
    rng = np.random.default_rng(seed)
    first = rng.integers(0, 256, (n + extra, DESCRIPTOR_BYTES), dtype=np.uint8)
    copies = first[:n].copy()
    for row in copies:
        for bit in rng.choice(8 * DESCRIPTOR_BYTES, flips, replace=False):
            row[bit // 8] ^= np.uint8(1 << (bit % 8))
    second = np.vstack([copies, rng.integers(0, 256, (extra, DESCRIPTOR_BYTES), dtype=np.uint8)])
    order = rng.permutation(len(second))
    where = np.argsort(order)[:n]  # where each copy landed in the shuffled second set
    return first, second[order], where


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    assert set(MATCHING["functions"]) == protocol_members(matching)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(MATCHING))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend reproduces the brute-force fixture exactly."""
    assert MATCHING["functions"][function]["tolerance"]["comparison"] == "exact"
    chosen = require_function(backend, matching.__name__, function)
    args = case["input"]
    if function == "distances":
        got = matching.distances(_descriptors(args[0]), _descriptors(args[1]), backend=chosen)
        _check_int64(got, case["expected"])
    elif function == "features_in_area":
        xy = np.asarray(args[0], dtype=np.float64).reshape(-1, 2)
        level = np.asarray(args[1], dtype=np.int64)
        got = matching.features_in_area(
            xy, level, args[2], args[3], min_level=args[4], max_level=args[5], backend=chosen
        )
        _check_int64(got, case["expected"])
    elif function == "search_by_projection":
        features, right, taken = _frame(args[1])
        found = matching.search_by_projection(
            _projections(args[0]),
            features,
            matching.SearchOptions(**args[2]),
            right=right,
            taken=taken,
            backend=chosen,
        )
        _check_int64(found.pairs, case["expected"]["pairs"])
        _check_int64(found.distances, case["expected"]["distances"])
    elif function == "search_by_bow":
        found = matching.search_by_bow(
            _descriptors(args[0]),
            np.asarray(args[1], dtype=np.int64),
            _descriptors(args[2]),
            np.asarray(args[3], dtype=np.int64),
            np.asarray(args[4], dtype=np.int64),
            np.asarray(args[5], dtype=np.int64),
            matching.Options(**args[6]),
            backend=chosen,
        )
        _check_int64(found.pairs, case["expected"]["pairs"])
        _check_int64(found.distances, case["expected"]["distances"])
    else:
        found = matching.match(
            _descriptors(args[0]),
            _descriptors(args[1]),
            np.asarray(args[2], dtype=np.int64),
            np.asarray(args[3], dtype=np.int64),
            matching.Options(**args[4]),
            backend=chosen,
        )
        _check_int64(found.pairs, case["expected"]["pairs"])
        _check_int64(found.distances, case["expected"]["distances"])


def test_distances_are_opencvs_hamming_norm() -> None:
    """Every distance equals OpenCV's NORM_HAMMING between the two rows."""
    rng = np.random.default_rng(1)
    a = rng.integers(0, 256, (20, DESCRIPTOR_BYTES), dtype=np.uint8)
    b = rng.integers(0, 256, (30, DESCRIPTOR_BYTES), dtype=np.uint8)
    got = matching.distances(a, b)
    theirs = [[cv2.norm(x, y, cv2.NORM_HAMMING) for y in b] for x in a]
    np.testing.assert_array_equal(got, theirs)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_recovers_planted_matches(backend: str) -> None:
    """Noisy copies among unrelated descriptors are found, and nothing else is."""
    chosen = require_function(backend, matching.__name__, "match")
    first, second, where = _planted(60, 200, 10, seed=2)
    found = matching.match(
        first, second, np.zeros(len(first), np.int64), np.zeros(len(second), np.int64),
        backend=chosen,
    )  # fmt: skip
    np.testing.assert_array_equal(found.pairs[:, 0], np.arange(60))
    np.testing.assert_array_equal(found.pairs[:, 1], where)
    assert np.all(found.distances == 10)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rotation_check_drops_inconsistent_matches(backend: str) -> None:
    """Matches whose orientation change disagrees with the bulk are dropped."""
    chosen = require_function(backend, matching.__name__, "match")
    first, second, where = _planted(50, 0, 5, seed=3)
    angles1 = np.random.default_rng(4).integers(0, 30, 50)
    angles2 = np.zeros(50, dtype=np.int64)
    angles2[where] = (angles1 + 7) % 30
    angles2[where[:4]] = (angles1[:4] + 19) % 30  # four strays, under a tenth of the bulk
    kept = matching.match(first, second, angles1, angles2, backend=chosen)
    np.testing.assert_array_equal(kept.pairs[:, 0], np.arange(4, 50))
    off = matching.Options(check_orientation=False)
    assert len(matching.match(first, second, angles1, angles2, off, backend=chosen).pairs) == 50


def test_each_second_descriptor_is_matched_once() -> None:
    """Of several first-set rows choosing one descriptor, the nearest keeps it."""
    rng = np.random.default_rng(5)
    target = rng.integers(0, 256, (1, DESCRIPTOR_BYTES), dtype=np.uint8)
    near, nearer = target.copy(), target.copy()
    near[0, 0] ^= 0b111
    nearer[0, 0] ^= 0b1
    first = np.vstack([near, nearer])
    found = matching.match(first, target, [0, 0], [0])
    np.testing.assert_array_equal(found.pairs, [[1, 0]])


def test_matching_by_word_pairs_only_under_a_shared_node() -> None:
    """Under one node it is match; a copy under another node is not found, its pair's lost."""
    first, second, where = _planted(30, 20, 10, seed=8)
    angles1, angles2 = np.zeros(len(first), np.int64), np.zeros(len(second), np.int64)
    one1, one2 = np.zeros(len(first), np.int64), np.zeros(len(second), np.int64)
    whole = matching.match(first, second, angles1, angles2)
    under_one = matching.search_by_bow(first, one1, second, one2, angles1, angles2)
    np.testing.assert_array_equal(under_one.pairs, whole.pairs)
    moved = one2.copy()
    moved[where[:10]] = 1  # the first ten copies under a node the first set lacks
    found = matching.search_by_bow(first, one1, second, moved, angles1, angles2)
    assert set(found.pairs[:, 0].tolist()) == set(whole.pairs[:, 0].tolist()) - set(range(10))
    assert np.all(moved[found.pairs[:, 1]] == one1[found.pairs[:, 0]])


GOOD = np.zeros((3, DESCRIPTOR_BYTES), dtype=np.uint8)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("function", "args", "message"),
    [
        ("distances", (GOOD.astype(np.int16), GOOD), r"descriptors1: expected an \(N, 32\) uint8"),
        ("distances", (GOOD, GOOD[:, :16]), r"descriptors2: expected an \(N, 32\) uint8"),
        ("match", (GOOD, GOOD, [0, 1], [0, 1, 2]), "angles1: expected 3 integers"),
        ("match", (GOOD, GOOD, [0, 1, 2], [0.0, 1.0, 2.0]), "angles2: expected 3 integers"),
        ("match", (GOOD, GOOD, [0, 1, 30], [0, 1, 2]), r"angles1: bins must lie in \[0, 30\)"),
        ("match", (GOOD, GOOD, [0, 1, 2], [-1, 1, 2]), r"angles2: bins must lie in \[0, 30\)"),
        ("match", (GOOD, GOOD, [True, False, True], [0, 1, 2]), "angles1: expected 3 integers"),
        (
            "search_by_bow",
            (GOOD, [0, 0], GOOD, [0, 0, 0], [0, 1, 2], [0, 1, 2]),
            "nodes1: expected 3 integers",
        ),
        (
            "search_by_bow",
            (GOOD, [0, 0, 0], GOOD, [0, -1, 0], [0, 1, 2], [0, 1, 2]),
            "nodes2: nodes must be at least 0",
        ),
        (
            "search_by_bow",
            (GOOD, [0, 0, 0], GOOD, [0, 0, 0], [0, 1, 2], [0, 1, 30]),
            r"angles2: bins must lie in \[0, 30\)",
        ),
    ],
)
def test_rejects_invalid_input(
    backend: str, function: str, args: tuple[object, ...], message: str
) -> None:
    """Malformed sets and angles raise InvalidInputError before any backend."""
    check_rejects(matching, function, args, message, backend)


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"max_distance": -1}, "max_distance: must be from 0 to 256"),
        ({"max_distance": 257}, "max_distance: must be from 0 to 256"),
        ({"ratio": 0.0}, r"ratio: must be in \(0, 1\]"),
        ({"ratio": 1.5}, r"ratio: must be in \(0, 1\]"),
        ({"ratio": float("nan")}, r"ratio: must be in \(0, 1\]"),
    ],
)
def test_options_reject_out_of_range(settings: dict[str, Any], message: str) -> None:
    """Every filter is range-checked when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        matching.Options(**settings)


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"max_distance": 50.0}, "max_distance: must be an integer, got float"),
        ({"max_distance": False}, "max_distance: must be an integer, got bool"),
        ({"ratio": "0.8"}, "ratio: must be a real number, got str"),
        ({"ratio": True}, "ratio: must be a real number, got bool"),
        ({"check_orientation": 1}, "check_orientation: must be a boolean, got int"),
    ],
)
def test_options_reject_wrong_types(settings: dict[str, Any], message: str) -> None:
    """A filter of the wrong type is refused, not converted differently by each backend."""
    with pytest.raises(InvalidInputError, match=message):
        matching.Options(**settings)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_numpy_settings_and_empty_lists_reach_every_backend(backend: str) -> None:
    """NumPy scalar filters become Python numbers; empty sets may come with empty lists."""
    chosen = require_function(backend, matching.__name__, "match")
    settings: dict[str, Any] = {
        "max_distance": np.int16(40),
        "ratio": np.float32(0.75),
        "check_orientation": np.False_,
    }
    options = matching.Options(**settings)
    assert [type(value) for value in asdict(options).values()] == [int, float, bool]
    first, second, _ = _planted(30, 50, 8, seed=6)
    zeros = np.zeros(len(first), np.int64), np.zeros(len(second), np.int64)
    plain = matching.Options(max_distance=40, ratio=0.75, check_orientation=False)
    got = matching.match(first, second, *zeros, options, backend=chosen)
    reference = matching.match(first, second, *zeros, plain)
    np.testing.assert_array_equal(got.pairs, reference.pairs)
    empty = matching.match(first[:0], second, [], zeros[1], backend=chosen)
    assert empty.pairs.shape == (0, 2)


def test_matching_takes_the_first_set_in_blocks() -> None:
    """A first set larger than one block matches as if taken whole."""
    first, second, where = _planted(700, 300, 6, seed=7)
    zeros = np.zeros(len(first), np.int64), np.zeros(len(second), np.int64)
    found = matching.match(first, second, *zeros)
    np.testing.assert_array_equal(found.pairs[:, 0], np.arange(700))
    np.testing.assert_array_equal(found.pairs[:, 1], where)


def _random_frame(
    rng: np.random.Generator, n: int, spread: tuple[float, float] = (640.0, 480.0)
) -> tuple[orb.Features, NDArray[np.float64], NDArray[np.bool_]]:
    """Draw a frame: positions over `spread`, some right columns, a few keypoints taken."""
    xy = rng.uniform(0.0, 1.0, (n, 2)) * np.asarray(spread)
    features = _features(
        xy,
        rng.integers(0, 8, n),
        rng.integers(0, 30, n),
        rng.integers(0, 256, (n, DESCRIPTOR_BYTES), dtype=np.uint8),
    )
    right = np.where(rng.random(n) < 0.5, xy[:, 0] - rng.uniform(0.0, 40.0, n), np.nan)
    return features, right, rng.random(n) < 0.1


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    "spread", [(640.0, 480.0), (0.0, 480.0), (1e-9, 1e-9), (5000.0, 3.0)], ids=str
)
def test_compiled_grids_find_what_the_brute_force_finds(
    backend: str, spread: tuple[float, float]
) -> None:
    """Every backend's search equals the reference's, whatever the keypoints' spread.

    The compiled cores search a grid over the keypoints' bounding box; a box of zero width,
    a tiny one, and a very flat one exercise its cell maps. Projections reuse noisy keypoint
    descriptors so that windows hold near and far candidates, ties and conflicts.
    """
    chosen = require_function(backend, matching.__name__, "search_by_projection")
    rng = np.random.default_rng(11)
    features, right, taken = _random_frame(rng, 1500, spread)
    m = 900
    source = rng.integers(0, 1500, m)
    descriptors = features.descriptors[source].copy()
    descriptors[:, 0] ^= rng.integers(0, 256, m, dtype=np.uint8)
    level = features.keypoints.level[source]
    projections = matching.Projections(
        xy=features.keypoints.xy[source] + rng.normal(0.0, 2.0, (m, 2)),
        radius=rng.uniform(0.5, 30.0, m),
        min_level=level - rng.integers(0, 2, m),
        max_level=level + rng.integers(0, 2, m),
        descriptors=descriptors,
        angle=np.where(  # one turn for most, so the rotation check keeps them; a few strays
            rng.random(m) < 0.9, (features.keypoints.angle[source] + 2) % 30, rng.integers(0, 30, m)
        ),
        right=np.where(rng.random(m) < 0.5, right[source] + rng.normal(0.0, 3.0, m), np.nan),
    )
    for options in [
        matching.SearchOptions(),
        matching.SearchOptions(max_distance=50, ratio=0.8, check_orientation=False),
    ]:
        reference = matching.search_by_projection(
            projections, features, options, right=right, taken=taken
        )
        got = matching.search_by_projection(
            projections, features, options, right=right, taken=taken, backend=chosen
        )
        assert len(reference.pairs) > 100
        np.testing.assert_array_equal(got.pairs, reference.pairs)
        np.testing.assert_array_equal(got.distances, reference.distances)
    for u, v, r in [(320.0, 240.0, 25.0), (0.0, 0.0, 1e-12), (-1e9, 0.0, 2e9)]:
        area = (features.keypoints.xy, features.keypoints.level, (u, v), r)
        expected = matching.features_in_area(*area, min_level=2, max_level=5)
        area_found = matching.features_in_area(*area, min_level=2, max_level=5, backend=chosen)
        np.testing.assert_array_equal(area_found, expected)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_defaults_track_the_last_frame(backend: str) -> None:
    """With no right columns or taken keypoints given, every keypoint is a candidate."""
    chosen = require_function(backend, matching.__name__, "search_by_projection")
    features, _, _ = _random_frame(np.random.default_rng(12), 50)
    kp = features.keypoints
    projections = matching.Projections(
        xy=kp.xy,
        radius=np.full(50, 0.5),
        min_level=kp.level,
        max_level=kp.level,
        descriptors=features.descriptors,
        angle=kp.angle,
        right=np.full(50, np.nan),
    )
    found = matching.search_by_projection(projections, features, backend=chosen)
    np.testing.assert_array_equal(found.pairs, np.column_stack([np.arange(50), np.arange(50)]))
    assert np.all(found.distances == 0)


def _good_projections(**changes: object) -> dict[str, Any]:
    """Two valid projections' fields, with `changes` applied."""
    fields: dict[str, Any] = {
        "xy": np.zeros((2, 2)),
        "radius": np.ones(2),
        "min_level": np.zeros(2, dtype=np.int64),
        "max_level": np.ones(2, dtype=np.int64),
        "descriptors": np.zeros((2, DESCRIPTOR_BYTES), dtype=np.uint8),
        "angle": np.zeros(2, dtype=np.int64),
        "right": np.full(2, np.nan),
    }
    fields.update(changes)
    return fields


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"xy": np.zeros((3, 2))}, r"projections.xy: expected shape \(2, 2\)"),
        ({"xy": np.array([[0.0, np.nan], [0.0, 0.0]])}, "projections.xy: expected finite"),
        ({"radius": np.array([1.0, 0.0])}, "projections.radius: must be positive"),
        ({"radius": np.array([1.0, np.inf])}, "projections.radius: expected finite"),
        ({"min_level": np.zeros(2)}, "projections.min_level: expected 2 integers"),
        ({"max_level": np.array([True, False])}, "projections.max_level: expected 2 integers"),
        ({"angle": np.array([0, 30])}, r"projections.angle: bins must lie in \[0, 30\)"),
        ({"right": np.array([np.inf, 1.0])}, "projections.right: expected finite"),
        ({"descriptors": np.zeros((2, 16), dtype=np.uint8)}, r"projections.descriptors: expected"),
    ],
)
def test_projections_reject_malformed_input(changes: dict[str, object], message: str) -> None:
    """Projections are checked when built, so every backend sees the same clean arrays."""
    with pytest.raises(InvalidInputError, match=message):
        matching.Projections(**_good_projections(**changes))


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("frame", "extras", "message"),
    [
        ({"xy": [[0.0, np.nan], [1.0, 1.0]]}, {}, "features.keypoints.xy: expected finite"),
        ({"level": [0, 8]}, {}, r"features.keypoints.level: levels must lie in \[0, 8\)"),
        ({"level": [-1, 0]}, {}, r"features.keypoints.level: levels must lie in \[0, 8\)"),
        ({"angle": [0, 30]}, {}, r"features.keypoints.angle: bins must lie in \[0, 30\)"),
        ({}, {"right": [1.0]}, r"right: expected shape \(2,\)"),
        ({}, {"right": [np.inf, 1.0]}, "right: expected finite"),
        ({}, {"taken": [1, 0]}, r"taken: expected booleans of shape \(2,\)"),
    ],
)
def test_search_rejects_a_malformed_frame(
    backend: str, frame: dict[str, Any], extras: dict[str, Any], message: str
) -> None:
    """A frame's features and extras are checked before any backend runs."""
    fields: dict[str, Any] = {
        "xy": [[0.0, 0.0], [1.0, 1.0]],
        "level": [0, 1],
        "angle": [0, 1],
        "descriptors": np.zeros((2, DESCRIPTOR_BYTES), dtype=np.uint8),
    }
    fields.update(frame)
    features = _features(**fields)
    projections = matching.Projections(**_good_projections())
    with pytest.raises(InvalidInputError, match=message):
        matching.search_by_projection(projections, features, backend=backend, **extras)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("args", "message"),
    [
        (([[0.0, np.inf]], [0], (0.0, 0.0), 1.0), "xy: expected finite"),
        (([0.0, 1.0], [0], (0.0, 0.0), 1.0), r"xy: expected shape \(N, 2\)"),
        (([[0.0, 0.0]], [-1], (0.0, 0.0), 1.0), "level: levels must be non-negative"),
        (([[0.0, 0.0]], [0.0], (0.0, 0.0), 1.0), "level: expected 1 integers"),
        (([[0.0, 0.0]], [0], (0.0, np.nan), 1.0), "center: expected finite"),
        (([[0.0, 0.0]], [0], (0.0, 0.0, 0.0), 1.0), r"center: expected shape \(2,\)"),
        (([[0.0, 0.0]], [0], (0.0, 0.0), 0.0), "radius: must be finite and positive"),
        (([[0.0, 0.0]], [0], (0.0, 0.0), np.inf), "radius: must be finite and positive"),
    ],
)
def test_features_in_area_rejects_invalid_input(
    backend: str, args: tuple[object, ...], message: str
) -> None:
    """Malformed keypoints and windows raise InvalidInputError before any backend."""
    check_rejects(matching, "features_in_area", args, message, backend)


def test_features_in_area_rejects_a_level_bound_of_the_wrong_type() -> None:
    """A level bound must be an integer."""
    bound: Any = 0.0  # the wrong type, on purpose
    with pytest.raises(InvalidInputError, match="min_level: must be an integer"):
        matching.features_in_area([[0.0, 0.0]], [0], (0.0, 0.0), 1.0, min_level=bound)


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"max_distance": 257}, "max_distance: must be from 0 to 256"),
        ({"ratio": 0.0}, r"ratio: must be in \(0, 1\]"),
        ({"check_orientation": 1}, "check_orientation: must be a boolean, got int"),
    ],
)
def test_search_options_are_checked(settings: dict[str, Any], message: str) -> None:
    """The search's filters are checked as :class:`Options`' are."""
    with pytest.raises(InvalidInputError, match=message):
        matching.SearchOptions(**settings)
