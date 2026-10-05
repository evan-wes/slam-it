"""Stereo tests: OpenCV's rectification, epipolar rows, row matching, failure modes."""

from __future__ import annotations

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
        left_image, right_image, left_f, right_f, focal, baseline, extraction, options = case[
            "input"
        ]
        found = stereo.match(
            np.asarray(left_image, np.uint8),
            np.asarray(right_image, np.uint8),
            _features(left_f),
            _features(right_f),
            focal=focal,
            baseline=baseline,
            orb_options=orb.Options(**extraction),
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
        ((np.eye(4) * 2,), InvalidInputError, "transform"),
        ((np.eye(3),), InvalidInputError, r"T_right_left: expected shape \(\.\.\., 4, 4\)"),
    ],
)
def test_rectify_rejects(backend: str, args: tuple[Any, ...], error: type, message: str) -> None:
    """Vertical, coincident or non-rigid pairs are refused."""
    chosen = require_function(backend, stereo.__name__, "rectify")
    with pytest.raises(error, match=message):
        stereo.rectify(
            "pinhole", PINHOLE, "pinhole", PINHOLE, *args, shape=(480, 640), backend=chosen
        )


@pytest.mark.parametrize("shape", [(0, 640), (480,), (480, 2.0), (1 << 21, 10)])
def test_rectify_rejects_bad_shapes(shape: Any) -> None:  # noqa: ANN401 -- malformed on purpose
    """The image shape must be two positive integers within MAX_IMAGE_SIDE."""
    with pytest.raises(InvalidInputError, match="shape: expected"):
        stereo.rectify("pinhole", PINHOLE, "pinhole", PINHOLE, _pose([-0.1, 0, 0]), shape=shape)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_match_recovers_a_known_disparity(backend: str) -> None:
    """Matches of a pair shifted by 9.6 pixels find it to within a pixel of their level.

    All but a rare false match on the repetitive texture (99%); the median within a third of
    a pixel, since a parabola through sums of absolute differences is pulled toward whole
    pixels ("pixel locking"), as in ORB-SLAM.
    """
    chosen = require_function(backend, stereo.__name__, "match")
    left, right = _shifted_pair(9.6, seed=3)
    features = orb.extract(left), orb.extract(right)
    found = stereo.match(left, right, *features, focal=500.0, baseline=0.1, backend=chosen)
    assert len(found.left) > 300
    assert np.all(np.diff(found.left) > 0)
    np.testing.assert_allclose(np.median(found.disparity), 9.6, atol=0.35)
    level_pixel = 1.2 ** features[0].keypoints.level[found.left].astype(np.float64)
    assert np.mean(np.abs(found.disparity - 9.6) < level_pixel) >= 0.99
    np.testing.assert_allclose(found.depth, 500.0 * 0.1 / found.disparity, rtol=1e-15)


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


def _tiny_features(level_xy: list[list[int]]) -> orb.Features:
    """Build features at the given level-0 pixels, with zero descriptors."""
    pixels = np.asarray(level_xy, dtype=np.int64).reshape(-1, 2)
    count = len(pixels)
    return orb.Features(
        keypoints=orb.Keypoints(
            xy=pixels.astype(np.float64),
            level=np.zeros(count, dtype=np.int64),
            level_xy=pixels,
            response=np.zeros(count, dtype=np.int64),
            angle=np.zeros(count, dtype=np.int64),
        ),
        descriptors=np.zeros((count, 32), dtype=np.uint8),
    )


FRAME = np.zeros((60, 80), dtype=np.uint8)
ONE_LEVEL = orb.Options(levels=1)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"right_image": np.zeros((60, 81), np.uint8)}, "right_image: expected the left image"),
        ({"focal": 0.0}, "focal: must be positive and finite"),
        ({"baseline": float("inf")}, "baseline: must be positive and finite"),
        ({"focal": "100"}, "focal: must be a real number, got str"),
        ({"baseline": True}, "baseline: must be a real number, got bool"),
        ({"left": _tiny_features([[2, 30]])}, "left: keypoints must lie at least 5 pixels"),
        ({"orb_options": orb.Options(levels=5, scale_factor=4.0)}, "would be empty"),
    ],
)
def test_match_rejects(kwargs: dict[str, Any], message: str) -> None:
    """Mismatched images, bad cameras and keypoints at the border are refused."""
    arguments: dict[str, Any] = {
        "left_image": FRAME,
        "right_image": FRAME,
        "left": _tiny_features([[30, 30]]),
        "right": _tiny_features([[25, 30]]),
        "focal": 100.0,
        "baseline": 0.1,
        "orb_options": ONE_LEVEL,
    }
    arguments.update(kwargs)
    with pytest.raises(InvalidInputError, match=message):
        stereo.match(**arguments)


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"max_distance": 0}, "max_distance: must be from 1 to 257"),
        ({"band": 0.0}, "band: must be positive and finite"),
        ({"window": 20}, "window: must be from 1 to 19"),
        ({"search": 1}, "search: must be from 2 to 64"),
        ({"outlier_factor": 0.5}, "outlier_factor: must be at least 1"),
        ({"search": stereo.MAX_SEARCH + 1}, "search: must be from 2 to 64"),
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
