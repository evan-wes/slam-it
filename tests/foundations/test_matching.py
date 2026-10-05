"""Descriptor-matching tests: exact parity, OpenCV's Hamming norm, the filters, failure modes."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

import cv2
import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import matching
from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.orb import DESCRIPTOR_BYTES
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
