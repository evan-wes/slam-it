"""Stereo-rectification tests: parity with OpenCV's rotations, epipolar rows, failure modes."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import image, stereo
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
    else:
        left, right = stereo.rectification_maps(_rectification(case["input"]), backend=chosen)
        assert_matches_fixture(left, case["expected"][0], tolerance)
        assert_matches_fixture(right, case["expected"][1], tolerance)


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
