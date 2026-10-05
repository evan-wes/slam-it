"""Stereo tests: OpenCV's rectification, epipolar rows, row matching, failure modes."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import cv2
import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import image, orb, stereo
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from tests.conftest import (
    assert_matches_fixture,
    load_fixture,
    parity_params,
    protocol_members,
    require_function,
)

STEREO = load_fixture("foundations/stereo.json")
EUROC = STEREO["functions"]["rectify"]["cases"][0]["input"]


def _rectification(case_input: list[Any]) -> stereo.Rectification:
    """Build a map case's Rectification."""
    (left_model, left_k), (right_model, right_k), fields = case_input
    return stereo.Rectification(
        rotation_left=np.asarray(fields["rotation_left"]),
        rotation_right=np.asarray(fields["rotation_right"]),
        intrinsics=np.asarray(fields["intrinsics"]),
        baseline=fields["baseline"],
        shape=(fields["shape"][0], fields["shape"][1]),
        left_model=left_model,
        left_intrinsics=np.asarray(left_k, dtype=np.float64),
        right_model=right_model,
        right_intrinsics=np.asarray(right_k, dtype=np.float64),
    )


def _features(fields: dict[str, Any]) -> orb.Features:
    """Build a match case's features (responses and angles, unused, are zero)."""
    count = len(fields["level"])
    return orb.Features(
        keypoints=orb.Keypoints(
            xy=np.asarray(fields["xy"], dtype=np.float64).reshape(-1, 2),
            level=np.asarray(fields["level"], dtype=np.int64),
            level_xy=np.asarray(fields["level_xy"], dtype=np.int64).reshape(-1, 2),
            response=np.zeros(count, dtype=np.int64),
            angle=np.zeros(count, dtype=np.int64),
        ),
        descriptors=np.asarray(fields["descriptors"], dtype=np.uint8).reshape(-1, 32),
        scale_factor=fields["scale_factor"],
        levels=fields["levels"],
    )


def _shifted_pair(disparity: float, seed: int) -> tuple[NDArray[np.uint8], NDArray[np.uint8]]:
    """Draw a textured VGA image and its right view at one constant disparity."""
    rng = np.random.default_rng(seed)
    blocks = rng.integers(20, 236, (480 // 10 + 1, 640 // 10 + 1))
    tiles = np.kron(blocks, np.ones((10, 10), dtype=np.int64))[:480, :640]
    left = np.asarray(cv2.GaussianBlur(tiles.astype(np.uint8), (5, 5), 1.2), dtype=np.uint8)
    v, u = np.mgrid[0:480, 0:640].astype(np.float64)
    return left, image.remap(left, np.stack([u + disparity, v], axis=-1))


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    assert set(STEREO["functions"]) == protocol_members(stereo)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(STEREO))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend matches OpenCV's rotations and the defining formulas."""
    tolerance = STEREO["functions"][function]["tolerance"]
    chosen = require_function(backend, stereo.__name__, function)
    if function == "rectify":
        left_model, left_k, right_model, right_k, transform, shape = case["input"]
        got = stereo.rectify(
            left_model, left_k, right_model, right_k, transform, shape=tuple(shape), backend=chosen
        )
        expected = case["expected"]
        for field in ("rotation_left", "rotation_right", "intrinsics"):
            assert_matches_fixture(getattr(got, field), expected[field], tolerance)
        assert_matches_fixture(got.baseline, expected["baseline"], tolerance)
    elif function == "rectification_map":
        left, right = stereo.rectification_maps(_rectification(case["input"]), backend=chosen)
        assert_matches_fixture(left, case["expected"][0], tolerance)
        assert_matches_fixture(right, case["expected"][1], tolerance)
    else:
        left_image, right_image, left_f, right_f, focal, baseline, options = case["input"]
        found = stereo.match(
            np.asarray(left_image, np.uint8),
            np.asarray(right_image, np.uint8),
            _features(left_f),
            _features(right_f),
            focal=focal,
            baseline=baseline,
            options=stereo.Options(**options),
            backend=chosen,
        )
        for field in ("left", "right", "right_x", "disparity", "depth"):
            assert_matches_fixture(getattr(found, field), case["expected"][field], tolerance)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rectified_rows_are_epipolar_lines(backend: str) -> None:
    """A point projects to one row in both rectified views, f b / Z columns apart."""
    chosen = require_function(backend, stereo.__name__, "rectify")
    left_model, left_k, right_model, right_k, transform, shape = EUROC
    r = stereo.rectify(
        left_model, left_k, right_model, right_k, transform, shape=tuple(shape), backend=chosen
    )
    points = np.random.default_rng(0).uniform([-2, -1.5, 2], [2, 1.5, 9], (50, 3))
    t = np.asarray(transform)
    in_right = points @ t[:3, :3].T + t[:3, 3]
    f, _, cx, cy = r.intrinsics
    left = points @ r.rotation_left.T
    right = in_right @ r.rotation_right.T
    u_left, v_left = f * left[:, 0] / left[:, 2] + cx, f * left[:, 1] / left[:, 2] + cy
    u_right, v_right = f * right[:, 0] / right[:, 2] + cx, f * right[:, 1] / right[:, 2] + cy
    np.testing.assert_allclose(v_left, v_right, rtol=0, atol=1e-9)
    np.testing.assert_allclose(u_left - u_right, f * r.baseline / left[:, 2], rtol=1e-10)


def test_maps_agree_with_opencv() -> None:
    """The maps are OpenCV's initUndistortRectifyMap for the same rotations and camera."""
    left_model, left_k, right_model, right_k, transform, shape = EUROC
    r = stereo.rectify(left_model, left_k, right_model, right_k, transform, shape=tuple(shape))
    f, _, cx, cy = r.intrinsics
    new = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])
    for k, rotation, ours in zip(
        (left_k, right_k), (r.rotation_left, r.rotation_right), stereo.rectification_maps(r),
        strict=True,
    ):  # fmt: skip
        matrix = np.array([[k[0], 0, k[2]], [0, k[1], k[3]], [0, 0, 1.0]])
        map_x, map_y = cv2.initUndistortRectifyMap(
            matrix, np.asarray(k[4:9]), rotation, new, (shape[1], shape[0]), cv2.CV_32FC1
        )
        np.testing.assert_allclose(ours[..., 0], map_x, rtol=0, atol=1e-4)  # float32 maps
        np.testing.assert_allclose(ours[..., 1], map_y, rtol=0, atol=1e-4)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rectified_images_agree_across_backends(backend: str) -> None:
    """Remapping with each backend's maps gives the reference's rectified image exactly."""
    chosen = require_function(backend, stereo.__name__, "rectification_map")
    left_model, left_k, right_model, right_k, transform, shape = EUROC
    r = stereo.rectify(left_model, left_k, right_model, right_k, transform, shape=tuple(shape))
    frame = np.random.default_rng(1).integers(0, 256, tuple(shape), dtype=np.uint8)
    for ours, theirs in zip(
        stereo.rectification_maps(r, backend=chosen), stereo.rectification_maps(r), strict=True
    ):
        np.testing.assert_array_equal(image.remap(frame, ours), image.remap(frame, theirs))


def _pose(translation: list[float]) -> NDArray[np.float64]:
    """Build a pure translation."""
    out = np.eye(4)
    out[:3, 3] = translation
    return out


PINHOLE = [500.0, 500.0, 320.0, 240.0]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("args", "error", "message"),
    [
        ((_pose([0.0, 0.1, 0.0]),), InvalidInputError, "closer to vertical"),
        ((_pose([0.0, 0.0, 0.0]),), DegenerateError, "the camera centres coincide"),
        ((_pose([0.1, 0.0, 0.0]),), InvalidInputError, "lies left of the left one; swap"),
        ((np.eye(4) * 2,), InvalidInputError, "transform"),
        ((np.eye(3),), InvalidInputError, r"T_right_left: expected shape \(\.\.\., 4, 4\)"),
    ],
)
def test_rectify_rejects(backend: str, args: tuple[Any, ...], error: type, message: str) -> None:
    """Vertical, coincident, swapped or non-rigid pairs are refused."""
    chosen = require_function(backend, stereo.__name__, "rectify")
    with pytest.raises(error, match=message):
        stereo.rectify(
            "pinhole", PINHOLE, "pinhole", PINHOLE, *args, shape=(480, 640), backend=chosen
        )


@pytest.mark.parametrize("shape", [(0, 640), (480,), (480, 2.0), (1 << 21, 10)])
def test_rectify_rejects_bad_shapes(shape: Any) -> None:  # noqa: ANN401 -- malformed on purpose
    """The image shape must be two positive integers within MAX_IMAGE_SIDE."""
    with pytest.raises(InvalidInputError, match=r"shape: expected|rows: must|cols: must"):
        stereo.rectify("pinhole", PINHOLE, "pinhole", PINHOLE, _pose([-0.1, 0, 0]), shape=shape)


def _area_sampled_pair(disparity: float, seed: int) -> tuple[NDArray[np.uint8], NDArray[np.uint8]]:
    """Draw a VGA pair at one disparity, both views area-sampled from one 10x texture.

    Neither view is interpolated from the other, so neither is blurred relative to it: the
    sub-pixel refinement is measured, not the test pair's resampling.
    """
    fine = 10
    rng = np.random.default_rng(seed)
    blocks = rng.integers(20, 236, (480 // 10 + 2, 640 // 10 + 3)).astype(np.float32)
    texture = np.kron(blocks, np.ones((10 * fine, 10 * fine), dtype=np.float32))
    smooth = np.asarray(cv2.GaussianBlur(texture, (0, 0), 1.2 * fine))
    shift = round(disparity * fine)
    views = [
        cv2.resize(
            smooth[: 480 * fine, s : s + 640 * fine], (640, 480), interpolation=cv2.INTER_AREA
        )
        for s in (0, shift)
    ]
    left, right = (np.clip(np.rint(v), 0, 255).astype(np.uint8) for v in views)
    return left, right


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("disparity", [9.2, 9.6])
def test_match_recovers_a_known_disparity(backend: str, disparity: float) -> None:
    """Matches of an area-sampled pair find its disparity without bias, on every level.

    Zero-mean SAD with the V fit: the median within a hundredth of a pixel, most matches
    within a tenth of a full-image pixel, and all but a rare false match on the repetitive
    texture within a pixel of their level. (ORB-SLAM2's centre-subtracted SAD with a parabola
    was off by a median 0.27 pixels at 9.6.)
    """
    chosen = require_function(backend, stereo.__name__, "match")
    left, right = _area_sampled_pair(disparity, seed=3)
    features = orb.extract(left), orb.extract(right)
    found = stereo.match(left, right, *features, focal=500.0, baseline=0.1, backend=chosen)
    error = found.disparity - disparity
    assert len(found.left) > 400
    assert np.all(np.diff(found.left) > 0)
    assert abs(np.median(error)) < 0.01
    assert np.mean(np.abs(error) < 0.1) >= 0.75
    level_pixel = 1.2 ** features[0].keypoints.level[found.left].astype(np.float64)
    assert np.mean(np.abs(error) < level_pixel) >= 0.99
    np.testing.assert_allclose(found.depth, 500.0 * 0.1 / found.disparity, rtol=1e-15)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_the_narrowest_search_still_refines(backend: str) -> None:
    """With a search of 1, the three offsets still give sub-pixel disparities, unbiased."""
    chosen = require_function(backend, stereo.__name__, "match")
    left, right = _area_sampled_pair(9.6, seed=5)
    features = orb.extract(left), orb.extract(right)
    narrow = stereo.Options(search=1)
    found = stereo.match(
        left, right, *features, focal=500.0, baseline=0.1, options=narrow, backend=chosen
    )
    assert len(found.left) > 100
    assert abs(np.median(found.disparity - 9.6)) < 0.05


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_match_agrees_across_backends(backend: str) -> None:
    """Every backend finds the reference's matches on a VGA pair, bit for bit."""
    chosen = require_function(backend, stereo.__name__, "match")
    left, right = _shifted_pair(17.3, seed=4)
    features = orb.extract(left), orb.extract(right)
    reference = stereo.match(left, right, *features, focal=450.0, baseline=0.12)
    got = stereo.match(left, right, *features, focal=450.0, baseline=0.12, backend=chosen)
    for field in ("left", "right", "right_x", "disparity", "depth"):
        np.testing.assert_array_equal(getattr(got, field), getattr(reference, field))


def _tiny_features(level_xy: list[list[int]], xy: list[list[float]] | None = None) -> orb.Features:
    """Build one-level features at the given pixels, with zero descriptors."""
    pixels = np.asarray(level_xy, dtype=np.int64).reshape(-1, 2)
    count = len(pixels)
    return orb.Features(
        keypoints=orb.Keypoints(
            xy=pixels.astype(np.float64) if xy is None else np.asarray(xy, dtype=np.float64),
            level=np.zeros(count, dtype=np.int64),
            level_xy=pixels,
            response=np.zeros(count, dtype=np.int64),
            angle=np.zeros(count, dtype=np.int64),
        ),
        descriptors=np.zeros((count, 32), dtype=np.uint8),
        scale_factor=1.2,
        levels=1,
    )


FRAME = np.zeros((60, 80), dtype=np.uint8)
DEEP = replace(_tiny_features([[30, 30]]), levels=5, scale_factor=4.0)
ONE_AS_FLOAT: Any = 1.0  # a level count of the wrong type, on purpose


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"right_image": np.zeros((60, 81), np.uint8)}, "right_image: expected the left image"),
        ({"focal": 0.0}, "focal: must be positive and finite"),
        ({"baseline": float("inf")}, "baseline: must be positive and finite"),
        ({"focal": "100"}, "focal: must be a real number, got str"),
        ({"baseline": True}, "baseline: must be a real number, got bool"),
        ({"left": _tiny_features([[2, 30]])}, "left: keypoints must lie at least 5 pixels"),
        ({"left": DEEP, "right": DEEP}, "would be empty"),
        ({"right": replace(_tiny_features([[25, 30]]), scale_factor=1.3)}, "another pyramid"),
        ({"left": replace(_tiny_features([[30, 30]]), levels=ONE_AS_FLOAT)}, "left.levels"),
        ({"right": replace(_tiny_features([[25, 30]]), scale_factor=1.0)}, "right.scale_factor"),
        ({"right": _tiny_features([[80, 30]])}, "right: level_xy must lie inside its level"),
        ({"left": _tiny_features([[30, 30]], [[1e300, 30.0]])}, "left: xy must be level_xy"),
        ({"right": _tiny_features([[25, 30]], [[-1e19, 30.0]])}, "right: xy must be level_xy"),
        ({"right": _tiny_features([[25, 30]], [[25.5, 30.0]])}, "right: xy must be level_xy"),
        ({"left": _tiny_features([[30, 30]], [[np.nan, 30.0]])}, "left.xy: expected finite"),
    ],
)
@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_match_rejects(backend: str, kwargs: dict[str, Any], message: str) -> None:
    """Mismatched images, bad cameras and keypoints off their pyramid are refused, by all."""
    chosen = require_function(backend, stereo.__name__, "match")
    arguments: dict[str, Any] = {
        "left_image": FRAME,
        "right_image": FRAME,
        "left": _tiny_features([[30, 30]]),
        "right": _tiny_features([[25, 30]]),
        "focal": 100.0,
        "baseline": 0.1,
        "backend": chosen,
    }
    arguments.update(kwargs)
    with pytest.raises(InvalidInputError, match=message):
        stereo.match(**arguments)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"rotation_left": 2.0 * np.eye(3)}, "rotation"),
        ({"right_intrinsics": np.array([-500.0, 500.0, 320.0, 240.0])}, "focal lengths"),
        ({"left_intrinsics": np.array([500.0, 500.0, 320.0])}, "intrinsics"),
        ({"intrinsics": np.array([0.0, 0.0, 319.5, 239.5])}, "focal lengths"),
        ({"left_model": "fisheye"}, "fisheye"),
    ],
)
@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rectification_maps_reject(backend: str, change: dict[str, Any], message: str) -> None:
    """A malformed rotation, camera or rectified camera is refused, by every backend."""
    chosen = require_function(backend, stereo.__name__, "rectification_map")
    good = stereo.rectify(
        "pinhole", PINHOLE, "pinhole", PINHOLE, _pose([-0.1, 0.0, 0.0]), shape=(6, 8)
    )
    with pytest.raises(InvalidInputError, match=message):
        stereo.rectification_maps(replace(good, **change), backend=chosen)


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"max_distance": 0}, "max_distance: must be from 1 to 257"),
        ({"band": 0.0}, "band: must be positive and finite"),
        ({"window": 20}, "window: must be from 1 to 19"),
        ({"search": 0}, "search: must be from 1 to 64"),
        ({"outlier_factor": 0.5}, "outlier_factor: must be at least 1"),
        ({"search": stereo.MAX_SEARCH + 1}, "search: must be from 1 to 64"),
        ({"max_distance": 7.0}, "max_distance: must be an integer, got float"),
        ({"band": "2"}, "band: must be a real number, got str"),
        ({"window": True}, "window: must be an integer, got bool"),
        ({"search": 5.0}, "search: must be an integer, got float"),
        ({"outlier_factor": None}, "outlier_factor: must be a real number, got NoneType"),
    ],
)
def test_options_reject_out_of_range(settings: dict[str, Any], message: str) -> None:
    """Every matching setting is range-checked when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        stereo.Options(**settings)


def test_a_malformed_compiled_rectification_is_a_runtime_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A rectification whose baseline is not a float is refused, never converted."""
    raw = {
        "rotation_left": np.eye(3),
        "rotation_right": np.eye(3),
        "intrinsics": np.array([500.0, 500.0, 319.5, 239.5]),
        "baseline": 1,
    }
    monkeypatch.setattr(backends, "function", lambda *_: lambda **__: raw)
    monkeypatch.setattr(backends, "implementation", lambda *_: object())
    with pytest.raises(RuntimeError, match="stereo baseline: backend returned int"):
        stereo.rectify(
            "pinhole", PINHOLE, "pinhole", PINHOLE, _pose([-0.1, 0, 0]), shape=(480, 640),
            backend="cpp",
        )  # fmt: skip
