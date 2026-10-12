"""Frames' tests: depth lookup on every backend, and each mode's frame against the room's truth."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace

import numpy as np
import pytest
from numpy.typing import ArrayLike

from slam_it import backends
from slam_it.edge import frames
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge import room
from tests.edge.conftest import Truth
from tests.edge.support import CAMERA, OPTIONS


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_depth_at_matches_the_reference(backend: str) -> None:
    """Every backend reads the same pixels: half up, clamped, holes and infinities as NaN."""
    chosen = require_function(backend, frames.__name__, "depth_at")
    rng = np.random.default_rng(4)
    depth = rng.uniform(0.5, 5.0, (30, 40))
    depth[rng.random((30, 40)) < 0.2] = 0.0
    depth[0, 0] = np.nan
    xy = np.vstack([rng.uniform(-3.0, 43.0, (200, 2)), [[0.5, 0.5], [0.49, 0.49], [39.5, 29.5]]])
    got = frames.depth_at(xy, depth, backend=chosen)
    np.testing.assert_array_equal(got, frames.depth_at(xy, depth))
    assert np.isnan(frames.depth_at([[0.49, 0.49]], depth)[0])  # pixel (0, 0): NaN
    assert frames.depth_at([[0.5, 0.5]], depth)[0] == depth[1, 1]  # half up


@pytest.mark.parametrize(
    ("xy", "depth", "message"),
    [
        ([[0.0, np.nan]], np.ones((2, 2)), "xy: expected finite"),
        ([[0.0, 0.0]], np.ones(4), "depth_map: expected a non-empty 2D array"),
        ([[0.0, 0.0]], np.full((2, 2), np.inf), "depth_map: expected finite"),
    ],
)
def test_depth_at_rejects_invalid_input(xy: ArrayLike, depth: ArrayLike, message: str) -> None:
    """Pixels must be finite; the map 2D, with no infinities."""
    with pytest.raises(InvalidInputError, match=message):
        frames.depth_at(xy, depth)


def test_rgbd_frames_read_the_true_depth(truth: Truth, rendered: list[room.Frame]) -> None:
    """An RGB-D keypoint's depth is the rendered depth there; its right column follows from it."""
    frame = truth.frames[0]
    xy = frame.features.keypoints.xy
    expected = frames.depth_at(xy, rendered[0].depth)
    np.testing.assert_array_equal(frame.depth, expected)
    bf = CAMERA.focal * CAMERA.baseline
    np.testing.assert_allclose(frame.right, xy[:, 0] - bf / frame.depth)
    np.testing.assert_allclose(np.linalg.norm(frame.bearings, axis=1), 1.0)


def test_stereo_frames_measure_the_true_depth(rendered_stereo: list[room.Frame]) -> None:
    """A stereo frame's depths agree with the rendered depth (to the matching's precision)."""
    f = rendered_stereo[0]
    assert f.right is not None
    frame = frames.stereo_pair(f.gray, f.right, f.timestamp, CAMERA, OPTIONS.extraction())
    has = ~np.isnan(frame.depth)
    assert np.count_nonzero(has) > 0.4 * frame.size
    truth = frames.depth_at(frame.features.keypoints.xy[has], f.depth)
    disparity_error = CAMERA.focal * CAMERA.baseline * np.abs(1 / frame.depth[has] - 1 / truth)
    assert np.median(disparity_error) < 0.1  # pixels of disparity


def test_monocular_frames_have_no_depth(rendered: list[room.Frame]) -> None:
    """A monocular frame has bearings but neither depth nor right columns."""
    f = rendered[0]
    camera = frames.Camera("pinhole", room.SMALL.intrinsics, 320, 240)
    frame = frames.monocular(f.gray, f.timestamp, camera, OPTIONS.extraction())
    assert frame.size > 100
    assert np.all(np.isnan(frame.depth))
    assert np.all(np.isnan(frame.right))
    np.testing.assert_allclose(np.linalg.norm(frame.bearings, axis=1), 1.0)


def test_builders_refuse_a_camera_they_cannot_use(rendered: list[room.Frame]) -> None:
    """RGB-D needs a virtual baseline and a depth map the image's shape; stereo a pinhole pair."""
    f = rendered[0]
    no_baseline = frames.Camera("pinhole", room.SMALL.intrinsics, 320, 240)
    with pytest.raises(InvalidInputError, match="virtual baseline"):
        frames.rgbd(f.gray, f.depth, 0.0, no_baseline, OPTIONS.extraction())
    with pytest.raises(InvalidInputError, match="image's shape"):
        frames.rgbd(f.gray, f.depth[1:], 0.0, CAMERA, OPTIONS.extraction())
    with pytest.raises(InvalidInputError, match="rectified pinhole"):
        frames.stereo_pair(f.gray, f.gray, 0.0, no_baseline, OPTIONS.extraction())


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"model": "fisheye"}, "model"),
        ({"width": 0}, "width: must be from 1"),
        ({"baseline": -0.1}, "baseline: must be finite and non-negative"),
        ({"baseline": np.inf}, "baseline: must be finite and non-negative"),
    ],
)
def test_cameras_are_checked(settings: dict[str, object], message: str) -> None:
    """A camera's model, sides and baseline are checked when it is built."""
    fields: dict[str, object] = {
        "model": "pinhole",
        "intrinsics": room.SMALL.intrinsics,
        "width": 320,
        "height": 240,
        "baseline": 0.1,
    }
    fields.update(settings)
    with pytest.raises(InvalidInputError, match=message):
        frames.Camera(**fields)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("field", "make", "message"),
    [
        ("depth", lambda n: np.zeros(n, dtype=np.float32), r"frame\.depth: .*got float32"),
        ("right", lambda n: [0.0] * n, r"frame\.right: .*got list"),
        ("depth", lambda n: np.zeros(n + 1), r"frame\.depth: expected shape"),
        ("bearings", lambda n: np.zeros((n, 2)), r"frame\.bearings: expected shape"),
    ],
)
def test_a_frame_holds_one_float64_row_per_keypoint(
    field: str, make: Callable[[int], object], message: str, truth: Truth
) -> None:
    """A frame's arrays are checked when it is built, naming what they got (finding H23)."""
    frame = truth.frames[0]
    with pytest.raises(InvalidInputError, match=message):
        replace(frame, **{field: make(frame.size)})  # type: ignore[arg-type]


def test_a_frame_holds_one_descriptor_per_keypoint(truth: Truth) -> None:
    """Descriptors that are not one per keypoint are refused."""
    frame = truth.frames[0]
    fewer = replace(frame.features, descriptors=frame.features.descriptors[:-1])
    with pytest.raises(InvalidInputError, match="one descriptor per keypoint"):
        replace(frame, features=fewer)
