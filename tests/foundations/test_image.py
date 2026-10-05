"""Image tests: exact parity with the loop fixture, OpenCV, invariants, and failure modes."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import image
from slamlab.foundations.arrays import MAX_IMAGE_SIDE
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import (
    check_rejects,
    load_fixture,
    parity_params,
    protocol_members,
    require_function,
)

IMAGE = load_fixture("foundations/image.json")
BINOMIAL = (1, 6, 15, 20, 15, 6, 1)


def _scene(rows: int, cols: int, seed: int = 0) -> NDArray[np.uint8]:
    """Draw a random image with structure: a gradient, a bright square and noise."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:rows, 0:cols]
    values = 30 + (180 * x) // cols + (40 * y) // rows + rng.integers(0, 25, (rows, cols))
    values[rows // 3 : 2 * rows // 3, cols // 3 : 2 * cols // 3] -= 25
    out: NDArray[np.uint8] = np.clip(values, 0, 255).astype(np.uint8)
    return out


def _check_image(got: object, expected: object) -> None:
    """Check one image: a C-contiguous (H, W) uint8 array equal to `expected`."""
    assert isinstance(got, np.ndarray)
    assert got.dtype == np.uint8
    assert got.flags.c_contiguous
    np.testing.assert_array_equal(got, np.asarray(expected, dtype=np.uint8))


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    assert set(IMAGE["functions"]) == protocol_members(image)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(IMAGE))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend reproduces the per-pixel loops bit for bit."""
    assert IMAGE["functions"][function]["tolerance"]["comparison"] == "exact"
    chosen = require_function(backend, image.__name__, function)
    args = [np.asarray(a, dtype=np.uint8) if i == 0 else a for i, a in enumerate(case["input"])]
    got = getattr(image, function)(*args, backend=chosen)
    if function == "pyramid":
        assert len(got) == len(case["expected"])
        for level, expected in zip(got, case["expected"], strict=True):
            _check_image(level, expected)
    else:
        _check_image(got, case["expected"])


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_backends_agree_on_camera_sized_images(backend: str) -> None:
    """Every operation is bit-identical to the reference on a VGA-sized image."""
    chosen = require_function(backend, image.__name__, "pyramid")
    rgb = np.stack([_scene(480, 640, seed) for seed in range(3)], axis=-1)
    gray = image.to_gray(rgb)
    _check_image(image.to_gray(rgb, backend=chosen), gray)
    _check_image(image.blur(gray, BINOMIAL, backend=chosen), image.blur(gray, BINOMIAL))
    _check_image(image.resize(gray, 333, 517, backend=chosen), image.resize(gray, 333, 517))
    levels = image.pyramid(gray, 8, 1.2, backend=chosen)
    for got, expected in zip(levels, image.pyramid(gray, 8, 1.2), strict=True):
        _check_image(got, expected)
    rng = np.random.default_rng(1)
    coordinates = rng.uniform(-3.0, 645.0, (200, 300, 2))
    _check_image(image.remap(gray, coordinates, backend=chosen), image.remap(gray, coordinates))


def test_gray_is_opencvs() -> None:
    """The gray conversion is OpenCV's ``cvtColor``, bit for bit."""
    rgb = np.random.default_rng(2).integers(0, 256, (97, 131, 3), dtype=np.uint8)
    _check_image(image.to_gray(rgb), cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))


@pytest.mark.parametrize("shape", [(50, 67), (100, 133), (240, 320), (30, 40), (7, 3)])
def test_resize_is_within_one_of_opencv(shape: tuple[int, int]) -> None:
    """Bilinear resizing matches OpenCV's ``INTER_LINEAR`` to within one intensity.

    OpenCV rounds the sample positions in float and its two passes in steps, so a few pixels
    differ by one; the sampling geometry (centres aligned, clamped) is the same.
    """
    gray = _scene(120, 160)
    rows, cols = shape
    ours = image.resize(gray, rows, cols).astype(int)
    theirs = cv2.resize(gray, (cols, rows), interpolation=cv2.INTER_LINEAR).astype(int)
    assert np.abs(ours - theirs).max() <= 1


def test_blur_is_within_one_of_opencv() -> None:
    """The integer blur matches OpenCV's float ``sepFilter2D`` to within one intensity."""
    gray = _scene(60, 80)
    weights = np.asarray(BINOMIAL, dtype=np.float64) / 64.0
    theirs = cv2.sepFilter2D(gray, -1, weights, weights, borderType=cv2.BORDER_REFLECT_101)
    assert np.abs(image.blur(gray, BINOMIAL).astype(int) - theirs.astype(int)).max() <= 1


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_constant_images_stay_constant(backend: str) -> None:
    """Weights that sum to one keep a constant image constant, at every scale."""
    chosen = require_function(backend, image.__name__, "pyramid")
    flat = np.full((37, 53), 173, dtype=np.uint8)
    assert np.all(image.blur(flat, BINOMIAL, backend=chosen) == 173)
    assert np.all(image.resize(flat, 101, 7, backend=chosen) == 173)
    assert all(np.all(level == 173) for level in image.pyramid(flat, 6, 1.2, backend=chosen))
    ys, xs = np.mgrid[0:20, 0:30].astype(np.float64)
    inside = np.stack([xs * 1.7 + 0.3, ys * 1.3 + 0.6], axis=-1)
    assert np.all(image.remap(flat, inside, backend=chosen) == 173)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_identities(backend: str) -> None:
    """Resizing to the same shape, remapping onto the pixel grid and a unit kernel copy."""
    chosen = require_function(backend, image.__name__, "remap")
    gray = _scene(41, 59)
    _check_image(image.resize(gray, 41, 59, backend=chosen), gray)
    _check_image(image.blur(gray, (1,), backend=chosen), gray)
    ys, xs = np.mgrid[0:41, 0:59].astype(np.float64)
    _check_image(image.remap(gray, np.stack([xs, ys], axis=-1), backend=chosen), gray)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_remap_outside_is_zero(backend: str) -> None:
    """A coordinate a pixel or more outside the image samples only the zero border."""
    chosen = require_function(backend, image.__name__, "remap")
    gray = np.full((5, 6), 255, dtype=np.uint8)
    far = np.array([[[-1.0, 2.0], [6.0, 2.0], [2.0, -1.0], [2.0, 5.0], [1e12, -1e12]]])
    _check_image(image.remap(gray, far, backend=chosen), [[0, 0, 0, 0, 0]])
    half = np.array([[[-0.5, 2.0], [5.5, 2.0]]])
    _check_image(image.remap(gray, half, backend=chosen), [[128, 128]])


def test_level_shapes_follow_orb_slam() -> None:
    """A VGA frame's 8 levels at 1.2 have ORB-SLAM's sizes (round half up of n / 1.2^l)."""
    assert image.level_shapes(480, 640, 8, 1.2) == [
        (480, 640),
        (400, 533),
        (333, 444),
        (278, 370),
        (231, 309),
        (193, 257),
        (161, 214),
        (134, 179),
    ]


GRAY = np.zeros((4, 5), dtype=np.uint8)
RGB = np.zeros((4, 5, 3), dtype=np.uint8)
MAP = np.zeros((2, 3, 2))


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("function", "args", "message"),
    [
        ("to_gray", (GRAY,), r"rgb: expected a non-empty \(H, W, 3\) uint8 array"),
        ("to_gray", (RGB[..., :2],), r"rgb: expected a non-empty \(H, W, 3\)"),
        ("blur", (GRAY.astype(np.float64), (1,)), "image: expected a non-empty .* uint8"),
        ("blur", (GRAY[:0], (1,)), "image: expected a non-empty"),
        ("blur", (np.ma.masked_array(GRAY), (1,)), "masked arrays"),
        ("blur", (GRAY, (1, 1)), "odd number of non-negative integers"),
        ("blur", (GRAY, (1, -1, 2)), "odd number of non-negative integers"),
        ("blur", (GRAY, (True,)), "odd number of non-negative integers"),
        ("blur", (GRAY, (1.0,)), "odd number of non-negative integers"),
        ("blur", (GRAY, (1, 1, 1)), r"power of two up to 2\^15, got 3"),
        ("blur", (GRAY, (1 << 16,)), r"power of two up to 2\^15, got 65536"),
        ("blur", (GRAY, (0,)), "power of two"),
        ("resize", (GRAY, 0, 3), "rows: must be from 1 to"),
        ("resize", (GRAY, 3, MAX_IMAGE_SIDE + 1), "cols: must be from 1 to"),
        ("resize", (GRAY, 3.0, 3), "rows: must be an integer"),
        ("pyramid", (GRAY, 0, 1.2), "levels: must be from 1 to"),
        ("pyramid", (GRAY, 2, 1.0), "scale_factor: must be finite and above 1"),
        ("pyramid", (GRAY, 2, float("nan")), "scale_factor: must be finite and above 1"),
        ("pyramid", (GRAY, 2, "1.2"), "scale_factor: must be a real number"),
        ("pyramid", (GRAY, True, 1.2), "levels: must be an integer, got bool"),
        ("pyramid", (GRAY, 5, 2.0), "level 4 of a 4x5 image at scale factor 2.0 would be empty"),
        ("remap", (GRAY, MAP[..., :1]), r"coordinates: expected a non-empty \(H, W, 2\)"),
        ("remap", (GRAY, MAP[0]), r"coordinates: expected a non-empty \(H, W, 2\)"),
        ("remap", (GRAY, np.full((1, 1, 2), np.nan)), "coordinates: expected finite values"),
    ],
)
def test_rejects_invalid_input(
    backend: str, function: str, args: tuple[object, ...], message: str
) -> None:
    """Bad images, kernels, shapes and settings raise InvalidInputError before any backend."""
    check_rejects(image, function, args, message, backend)


def test_full_image_coordinate_inverts_the_pyramid() -> None:
    """A level position maps to where the resizes sampled it; level 0 maps to itself."""
    np.testing.assert_array_equal(image.full_image_coordinate([0, 7, 2.5], 640, 640), [0, 7, 2.5])
    # A level of 179 columns from 640: column 0's centre is at 0.5 * 640 / 179 - 0.5.
    assert image.full_image_coordinate(0, 179, 640) == 0.5 * 640 / 179 - 0.5
    assert image.full_image_coordinate(np.int64(19), 179, 640).shape == ()
    np.testing.assert_array_equal(image.full_image_coordinate([1, 1], [100, 50], 200), [2.5, 5.5])
    with pytest.raises(InvalidInputError, match="level_size"):
        image.full_image_coordinate(1, [0], 200)
    half: Any = 1.5  # a size of the wrong type, on purpose
    with pytest.raises(InvalidInputError, match="size"):
        image.full_image_coordinate(1, 100, half)


def test_full_image_coordinate_is_where_the_pyramid_samples() -> None:
    """Each level's column samples the full image where the exact map says, not at x s^l.

    A noisy ramp (dithered, so every level keeps its sub-pixel shift) goes through the pyramid;
    each level column's mean, inverted through the ramp, is the full-image x it sampled.
    """
    rows, cols, slope, base = 2000, 640, 0.25, 40.0
    x = np.arange(cols, dtype=np.float64)
    noise = np.random.default_rng(0).uniform(-40, 41, (rows, cols))
    frame = np.floor(base + slope * x[None, :] + noise).astype(np.uint8)
    scale, naive_offsets = 1.0, []
    for level, pixels in enumerate(image.pyramid(frame, 8, 1.2)):
        inner = np.arange(4, pixels.shape[1] - 4)
        sampled = (pixels[:, inner].astype(np.float64).mean(axis=0) - base) / slope
        exact = image.full_image_coordinate(inner, pixels.shape[1], cols)
        assert abs(np.mean(sampled - exact)) < 0.1, level
        naive_offsets.append(np.mean(sampled - inner * scale))
        scale *= 1.2
    assert max(naive_offsets) > 1.0  # ORB-SLAM's x s^l, on its worst level


def test_rejects_oversized_images() -> None:
    """A side above MAX_IMAGE_SIDE is refused (a broadcast view, so nothing is allocated)."""
    wide = np.broadcast_to(np.uint8(0), (1, MAX_IMAGE_SIDE + 1))
    check_rejects(image, "blur", (wide, (1,)), "sides must be at most", "python")
