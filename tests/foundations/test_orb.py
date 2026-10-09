"""ORB tests: exact parity, OpenCV's FAST and descriptors, the quadtree, orientations."""

from __future__ import annotations

import math
from dataclasses import asdict
from fractions import Fraction
from typing import Any

import cv2
import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import image, orb
from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.image import full_image_coordinate, level_shapes
from tests.conftest import (
    check_rejects,
    load_fixture,
    parity_params,
    protocol_members,
    require_function,
)

ORB = load_fixture("foundations/orb.json")
FIELDS = ("xy", "level", "level_xy", "response", "angle")


def _scene(rows: int, cols: int, seed: int = 0) -> NDArray[np.uint8]:
    """Draw a textured image: random blocks with soft edges and a little noise."""
    rng = np.random.default_rng(seed)
    blocks = rng.integers(20, 236, (rows // 12 + 1, cols // 12 + 1))
    values = np.kron(blocks, np.ones((12, 12), dtype=np.int64))[:rows, :cols]
    values = cv2.GaussianBlur(values.astype(np.uint8), (5, 5), 1.5).astype(np.int64)
    values += rng.integers(-5, 6, (rows, cols))
    out: NDArray[np.uint8] = np.clip(values, 0, 255).astype(np.uint8)
    return out


def _check_int64(got: object, expected: object) -> None:
    """Check an int64 array against expected values, exactly."""
    assert isinstance(got, np.ndarray)
    assert got.dtype == np.int64
    np.testing.assert_array_equal(got, np.asarray(expected, dtype=np.int64))


def _check_keypoints(got: orb.Keypoints, expected: orb.Keypoints | dict[str, Any]) -> None:
    """Check every field of two keypoint sets, exactly."""
    for field in FIELDS:
        want = expected[field] if isinstance(expected, dict) else getattr(expected, field)
        value = getattr(got, field)
        assert value.dtype == (np.float64 if field == "xy" else np.int64), field
        np.testing.assert_array_equal(value, np.asarray(want).reshape(value.shape), err_msg=field)


def _check_descriptors(got: object, expected: object) -> None:
    """Check descriptors: an (N, 32) uint8 array equal to `expected`."""
    assert isinstance(got, np.ndarray)
    assert got.dtype == np.uint8
    np.testing.assert_array_equal(got, np.asarray(expected, dtype=np.uint8).reshape(got.shape))


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    assert set(ORB["functions"]) == protocol_members(orb)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(ORB))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend reproduces the fixture exactly."""
    assert ORB["functions"][function]["tolerance"]["comparison"] == "exact"
    chosen = require_function(backend, orb.__name__, function)
    args = case["input"]
    if function == "fast_scores":
        _check_int64(
            orb.fast_scores(np.asarray(args[0], np.uint8), backend=chosen), case["expected"]
        )
    elif function == "distribute":
        xy = np.asarray(args[0], dtype=np.int64).reshape(-1, 2)
        kept = orb.distribute(
            xy, np.asarray(args[1], np.int64), tuple(args[2]), args[3], backend=chosen
        )
        _check_int64(kept, case["expected"])
    elif function == "detect":
        found = orb.detect(np.asarray(args[0], np.uint8), orb.Options(**args[1]), backend=chosen)
        _check_keypoints(found, case["expected"])
    else:
        settings = orb.Options(**args[1])
        if function == "extract_levels":
            levels = [np.asarray(level, np.uint8) for level in args[0]]
            features = orb.extract_levels(levels, settings, backend=chosen)
        else:
            features = orb.extract(np.asarray(args[0], np.uint8), settings, backend=chosen)
        _check_keypoints(features.keypoints, case["expected"])
        _check_descriptors(features.descriptors, case["expected"]["descriptors"])
        assert (features.scale_factor, features.levels) == (
            case["expected"]["scale_factor"],
            case["expected"]["levels"],
        )


def test_xy_is_the_exact_full_image_coordinate() -> None:
    """Each keypoint's xy is its level pixel's centre carried through the resizes, exactly.

    The composed resize maps x to (x + 1/2) n_0 / n_l - 1/2, a rational: the float64 value is
    that rational correctly rounded.
    """
    frame = _scene(120, 160, seed=8)
    options = orb.Options(n_features=300, levels=6)
    keypoints = orb.detect(frame, options)
    shapes = level_shapes(120, 160, options.levels, options.scale_factor)
    assert len(set(keypoints.level.tolist())) == options.levels
    pixels = zip(keypoints.xy, keypoints.level, keypoints.level_xy, strict=True)
    for (x, y), level, (u, v) in pixels:
        rows, cols = shapes[level]
        assert x == float(Fraction(2 * int(u) + 1, 2) * Fraction(160, cols) - Fraction(1, 2))
        assert y == float(Fraction(2 * int(v) + 1, 2) * Fraction(120, rows) - Fraction(1, 2))


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_backends_agree_on_a_vga_frame(backend: str) -> None:
    """Every backend finds the reference's keypoints on a 640x480 frame, bit for bit."""
    chosen = require_function(backend, orb.__name__, "detect")
    frame = _scene(480, 640, seed=3)
    _check_keypoints(orb.detect(frame, backend=chosen), orb.detect(frame))
    _check_int64(orb.fast_scores(frame, backend=chosen), orb.fast_scores(frame))
    features, reference = orb.extract(frame, backend=chosen), orb.extract(frame)
    _check_keypoints(features.keypoints, reference.keypoints)
    _check_descriptors(features.descriptors, reference.descriptors)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_extract_levels_is_extract_on_the_images_pyramid(backend: str) -> None:
    """From the image's own pyramid, every backend extracts what extract does from the image."""
    chosen = require_function(backend, orb.__name__, "extract_levels")
    frame = _scene(480, 640, seed=3)
    settings = orb.Options()
    levels = image.pyramid(frame, settings.levels, settings.scale_factor)
    features, reference = orb.extract_levels(levels, backend=chosen), orb.extract(frame)
    _check_keypoints(features.keypoints, reference.keypoints)
    _check_descriptors(features.descriptors, reference.descriptors)


TWO = orb.Options(levels=2, scale_factor=1.25)
"""Two levels: a 10x10 image's pyramid is it and an 8x8 level."""


@pytest.mark.parametrize(
    ("levels", "message"),
    [
        ([np.zeros((10, 10), np.uint8)], "levels: expected 2 levels, got 1"),
        ([np.zeros((10, 10), np.uint8)] * 2, "levels: level 1 is 10x10, expected 8x8"),
        ([np.zeros((10, 10), np.uint8), np.zeros((8, 8), np.int64)], r"levels\[1\]"),
        (np.zeros((2, 10, 10), np.uint8), "levels: expected a sequence of images"),
    ],
)
@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_extract_levels_refuses_another_pyramid(backend: str, levels: object, message: str) -> None:
    """Levels of another count or shape than the settings' pyramid are refused, by all."""
    chosen = require_function(backend, orb.__name__, "extract_levels")
    given: Any = levels  # one case is not a sequence of levels, on purpose
    with pytest.raises(InvalidInputError, match=message):
        orb.extract_levels(given, TWO, backend=chosen)
    good = [np.zeros((10, 10), np.uint8), np.zeros((8, 8), np.uint8)]
    assert len(orb.extract_levels(good, TWO, backend=chosen).descriptors) == 0


@pytest.mark.parametrize("threshold", [7, 20])
def test_fast_is_opencvs(threshold: int) -> None:
    """Scores and non-maximum suppression reproduce OpenCV's FAST-9 exactly."""
    frame = _scene(240, 320, seed=4)
    scores = orb.fast_scores(frame)
    for suppress in (False, True):
        found = cv2.FastFeatureDetector.create(threshold, suppress).detect(frame)
        theirs = {(round(k.pt[0]), round(k.pt[1])): k.response for k in found}
        corners = scores >= threshold
        if suppress:
            padded = np.pad(scores, 1, constant_values=-1)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx or dy:
                        corners &= scores > padded[1 + dy : 241 + dy, 1 + dx : 321 + dx]
        ys, xs = np.nonzero(corners)
        ours = {(int(x), int(y)): float(scores[y, x]) for x, y in zip(xs, ys, strict=True)}
        if suppress:  # OpenCV scores corners only when it suppresses non-maxima
            assert ours == theirs
        else:
            assert set(ours) == set(theirs)


def test_descriptors_are_opencvs_up_to_rounding() -> None:
    """OpenCV's ORB, given our keypoints and angles, computes nearly the same descriptors.

    Its blur and its rotation of the pattern round in float, so a few bits differ; a wrong
    rotation, sign or bit order would differ in about half of them.
    """
    frame = _scene(240, 320, seed=8)
    features = orb.extract(frame, orb.Options(n_features=300, levels=1))
    given = [
        cv2.KeyPoint(float(x), float(y), 31, 12.0 * float(a), 0, 0)
        for (x, y), a in zip(features.keypoints.level_xy, features.keypoints.angle, strict=True)
    ]
    detector = cv2.ORB.create(nlevels=1, edgeThreshold=orb.EDGE_THRESHOLD, patchSize=31)
    described, theirs = detector.compute(frame, given)
    assert len(described) == len(given)
    differing = np.unpackbits(features.descriptors ^ np.asarray(theirs, np.uint8), axis=1).sum(
        axis=1
    )
    assert differing.mean() < 1.0
    assert differing.max() <= 8


def test_extract_describes_what_detect_finds() -> None:
    """Extraction's keypoints are detection's, and every descriptor is 32 bytes."""
    frame = _scene(200, 260, seed=9)
    features = orb.extract(frame, orb.Options(n_features=200, levels=3))
    _check_keypoints(features.keypoints, orb.detect(frame, orb.Options(n_features=200, levels=3)))
    assert features.descriptors.shape == (len(features.keypoints.xy), orb.DESCRIPTOR_BYTES)


def test_rotated_pattern_stays_inside_the_edge() -> None:
    """Every test pixel, rotated to every bin, lies within EDGE_THRESHOLD - 1 of the keypoint."""
    assert len(orb.PATTERN) == 8 * orb.DESCRIPTOR_BYTES
    points = np.asarray(orb.PATTERN, dtype=np.int64).reshape(-1, 2)  # (x, y), two per test
    half = 1 << (orb.ROTATION_BITS - 1)
    reach = 0
    for c, s in orb.ROTATIONS:  # (x c - y s, x s + y c) / 2^14, rounded half up
        x = (points[:, 0] * c - points[:, 1] * s + half) >> orb.ROTATION_BITS
        y = (points[:, 0] * s + points[:, 1] * c + half) >> orb.ROTATION_BITS
        reach = max(reach, int(np.abs(x).max()), int(np.abs(y).max()))
    assert reach == orb.EDGE_THRESHOLD - 1


def test_constants_follow_their_definitions() -> None:
    """UMAX is OpenCV's construction, and ROTATIONS rounds 2^14 (cos, sin) of 12k degrees."""
    half = orb.HALF_PATCH_SIZE
    umax = [0] * (half + 2)
    vmin = math.ceil(half * math.sqrt(2) / 2)
    for v in range(math.floor(half * math.sqrt(2) / 2 + 1) + 1):
        umax[v] = round(math.sqrt(half * half - v * v))
    v0 = 0
    for v in range(half, vmin - 1, -1):
        while umax[v0] == umax[v0 + 1]:
            v0 += 1
        umax[v] = v0
        v0 += 1
    assert tuple(umax[: half + 1]) == orb.UMAX
    scale = 2**orb.ROTATION_BITS
    for k, (c, s) in enumerate(orb.ROTATIONS):
        angle = math.radians(360 * k / orb.ANGLE_BINS)
        assert (c, s) == (round(scale * math.cos(angle)), round(scale * math.sin(angle)))


def test_orientation_is_the_nearest_bin() -> None:
    """Each keypoint's bin is the 12-degree step nearest its intensity centroid's angle."""
    frame = _scene(240, 320, seed=5)
    found = orb.detect(frame, orb.Options(n_features=300, levels=1))
    pixels = frame.astype(np.int64)
    for (x, y), angle in zip(found.level_xy, found.angle, strict=True):
        m10 = m01 = 0
        for v in range(-orb.HALF_PATCH_SIZE, orb.HALF_PATCH_SIZE + 1):
            for u in range(-orb.UMAX[abs(v)], orb.UMAX[abs(v)] + 1):
                m10 += u * pixels[y + v, x + u]
                m01 += v * pixels[y + v, x + u]
        degrees = math.degrees(math.atan2(m01, m10)) % 360
        assert round(degrees / 12) % orb.ANGLE_BINS == angle


def test_level_budgets_split_the_total() -> None:
    """The budgets fall geometrically and sum to the total (ORB-SLAM's 1000 over 8 levels)."""
    budgets = orb.level_budgets(1000, 8, 1.2)
    assert budgets == [217, 181, 151, 126, 105, 87, 73, 60]
    assert orb.level_budgets(5, 1, 1.2) == [5]
    assert sum(orb.level_budgets(777, 5, 1.6)) == 777


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_keypoints_stay_inside_and_distinct(backend: str) -> None:
    """Keypoints lie EDGE_THRESHOLD inside their level, once each, near the budget."""
    chosen = require_function(backend, orb.__name__, "detect")
    frame = _scene(300, 400, seed=6)
    options = orb.Options(n_features=500, levels=4)
    found = orb.detect(frame, options, backend=chosen)
    assert 450 <= len(found.xy) <= 510
    for level, (rows, cols) in enumerate(level_shapes(300, 400, 4, 1.2)):
        pixels = found.level_xy[found.level == level]
        assert np.all(pixels >= orb.EDGE_THRESHOLD)
        assert np.all(pixels[:, 0] < cols - orb.EDGE_THRESHOLD)
        assert np.all(pixels[:, 1] < rows - orb.EDGE_THRESHOLD)
        assert len(np.unique(pixels, axis=0)) == len(pixels)
    sizes = np.asarray(level_shapes(300, 400, 4, 1.2))[found.level]  # (rows, cols) each
    exact = np.column_stack(
        [
            full_image_coordinate(found.level_xy[:, 0], sizes[:, 1], 400),
            full_image_coordinate(found.level_xy[:, 1], sizes[:, 0], 300),
        ]
    )
    np.testing.assert_array_equal(found.xy, exact)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_quadtree_contract(backend: str) -> None:
    """Enough room keeps every distinct corner; a tight target keeps the strongest of each node."""
    chosen = require_function(backend, orb.__name__, "distribute")
    rng = np.random.default_rng(7)
    xy = np.unique(rng.integers(0, 100, (80, 2)), axis=0)
    response = rng.integers(0, 1000, len(xy))
    every = orb.distribute(xy, response, (0, 100, 0, 100), len(xy), backend=chosen)
    np.testing.assert_array_equal(every, np.arange(len(xy)))
    kept = orb.distribute(xy, response, (0, 100, 0, 100), 20, backend=chosen)
    assert 20 <= len(kept) <= 22
    assert np.all(np.diff(kept) > 0)
    assert len(orb.distribute(xy, response, (0, 100, 0, 100), 0, backend=chosen)) == 0
    same = np.array([[5, 5]] * 6)
    best = orb.distribute(same, np.array([1, 4, 9, 9, 2, 0]), (0, 10, 0, 10), 3, backend=chosen)
    _check_int64(best, [2])


GRAY = np.zeros((60, 60), dtype=np.uint8)
POINTS = np.array([[1, 2], [3, 4]])


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("function", "args", "message"),
    [
        ("fast_scores", (GRAY.astype(np.int16),), "image: expected a non-empty"),
        ("detect", (GRAY[:0],), "image: expected a non-empty"),
        (
            "detect",
            (np.zeros((10, 10), np.uint8), orb.Options(levels=8, scale_factor=2.0)),
            "would be empty",
        ),
        (
            "distribute",
            (POINTS.astype(float), [1, 2], (0, 5, 0, 5), 1),
            r"xy: expected an \(N, 2\) integer",
        ),
        (
            "distribute",
            (POINTS[:, :1], [1, 2], (0, 5, 0, 5), 1),
            r"xy: expected an \(N, 2\) integer",
        ),
        ("distribute", (POINTS, [1], (0, 5, 0, 5), 1), "response: expected 2 integers"),
        ("distribute", (POINTS, [1.0, 2.0], (0, 5, 0, 5), 1), "response: expected 2 integers"),
        (
            "distribute",
            (POINTS, np.array([1, 2], np.uint64), (0, 5, 0, 5), 1),
            "response: expected",
        ),
        ("distribute", (POINTS, [1, 2], (0, 5, 0), 1), "bounds: expected four integers"),
        ("distribute", (POINTS, [1, 2], (0, 5.0, 0, 5), 1), "bounds: expected four integers"),
        ("distribute", (POINTS, [1, 2], (5, 5, 0, 5), 1), "bounds: expected a non-empty box"),
        ("distribute", (POINTS, [1, 2], (-1, 5, 0, 5), 1), "bounds: expected a non-empty box"),
        ("distribute", (POINTS, [1, 2], (0, 3, 0, 5), 1), "every corner must lie inside"),
        ("distribute", (POINTS, [1, 2], (0, 5, 0, 5), -1), "target: must be from 0 to"),
        ("distribute", (POINTS, [1, 2], (0, 5, 0, 5), orb.MAX_FEATURES + 1), "target: must be"),
        ("distribute", (POINTS, [1, 2], (0, 5, 0, 5), 1.0), "target: must be an integer"),
        ("distribute", (POINTS, [1, 2], (0, 5, 0, 5), True), "target: must be an integer"),
        ("distribute", (POINTS.astype(bool), [1, 2], (0, 5, 0, 5), 1), "xy: expected"),
        ("distribute", (POINTS, [True, False], (0, 5, 0, 5), 1), "response: expected"),
    ],
)
def test_rejects_invalid_input(
    backend: str, function: str, args: tuple[object, ...], message: str
) -> None:
    """Malformed images, corners and boxes raise InvalidInputError before any backend."""
    check_rejects(orb, function, args, message, backend)


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"n_features": 0}, "n_features: must be from 1 to"),
        ({"levels": 0}, "levels: must be from 1 to 32"),
        ({"levels": 33}, "levels: must be from 1 to 32"),
        ({"scale_factor": 1.0}, "scale_factor: must be finite and above 1"),
        ({"scale_factor": math.inf}, "scale_factor: must be finite and above 1"),
        ({"n_features": orb.MAX_FEATURES + 1}, "n_features: must be from 1 to 1048576"),
        ({"fast_threshold": 0}, "fast_threshold: must be from 1 to 254"),
        ({"fast_threshold": 255}, "fast_threshold: must be from 1 to 254"),
        ({"min_fast_threshold": 21}, "min_fast_threshold: must be from 1 to 20, got 21"),
        ({"min_fast_threshold": 0}, "min_fast_threshold: must be from 1 to 20, got 0"),
    ],
)
def test_options_reject_out_of_range(settings: dict[str, Any], message: str) -> None:
    """Every setting is range-checked when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        orb.Options(**settings)


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"n_features": 1000.0}, "n_features: must be an integer, got float"),
        ({"n_features": True}, "n_features: must be an integer, got bool"),
        ({"levels": "8"}, "levels: must be an integer, got str"),
        ({"scale_factor": "1.2"}, "scale_factor: must be a real number, got str"),
        ({"scale_factor": np.True_}, "scale_factor: must be a real number, got bool"),
        ({"fast_threshold": 20.5}, "fast_threshold: must be an integer, got float"),
        ({"min_fast_threshold": None}, "min_fast_threshold: must be an integer, got NoneType"),
    ],
)
def test_options_reject_wrong_types(settings: dict[str, Any], message: str) -> None:
    """A setting of the wrong type is refused, not converted differently by each backend."""
    with pytest.raises(InvalidInputError, match=message):
        orb.Options(**settings)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_numpy_settings_reach_every_backend(backend: str) -> None:
    """NumPy scalar settings are stored as Python numbers, so every backend reads them alike."""
    chosen = require_function(backend, orb.__name__, "extract")
    settings: dict[str, Any] = {
        "n_features": np.int64(300),
        "levels": np.int32(4),
        "scale_factor": np.float32(1.25),
        "fast_threshold": np.uint8(20),
        "min_fast_threshold": np.int16(7),
    }
    options = orb.Options(**settings)
    assert [type(value) for value in asdict(options).values()] == [int, int, float, int, int]
    plain = orb.Options(n_features=300, levels=4, scale_factor=1.25)
    frame = _scene(120, 160, seed=5)
    features, reference = orb.extract(frame, options, backend=chosen), orb.extract(frame, plain)
    _check_keypoints(features.keypoints, reference.keypoints)
    _check_descriptors(features.descriptors, reference.descriptors)


def _fake_backend(monkeypatch: pytest.MonkeyPatch, raw: object) -> None:
    """Make every compiled function return `raw`, as a broken binding would."""
    monkeypatch.setattr(backends, "function", lambda *_: lambda **__: raw)
    monkeypatch.setattr(backends, "implementation", lambda *_: object())


def test_a_malformed_compiled_result_is_a_runtime_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """Features whose pyramid fields have the wrong types are refused, never converted."""
    frame = _scene(80, 90, seed=9)
    found = orb.extract(frame)
    raw = {field: getattr(found.keypoints, field) for field in FIELDS}
    raw.update(descriptors=found.descriptors, scale_factor=1.2, levels=8.0)
    _fake_backend(monkeypatch, raw)
    with pytest.raises(RuntimeError, match="levels float, expected float and int"):
        orb.extract(frame, backend="cpp")
