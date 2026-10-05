"""Image-file readers: the shared PNG fixtures, and failure modes."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from slamlab.eval import images, tum
from slamlab.foundations.errors import InvalidInputError

FOLDER = Path(__file__).parents[1] / "fixtures" / "eval"
CASES: dict[str, dict[str, Any]] = json.loads((FOLDER / "png.json").read_text())["cases"]
EIGHT_BIT = sorted(name for name, case in CASES.items() if "gray" in case)


@pytest.mark.parametrize("name", EIGHT_BIT)
def test_gray_and_colour_match_the_fixture(name: str) -> None:
    """Each 8-bit file reads as its expected gray and colour arrays, exactly."""
    gray = images.read_gray(FOLDER / "png" / name)
    rgb = images.read_rgb(FOLDER / "png" / name)
    assert gray.dtype == np.uint8
    assert rgb.dtype == np.uint8
    np.testing.assert_array_equal(gray, CASES[name]["gray"])
    np.testing.assert_array_equal(rgb, CASES[name]["rgb"])


def test_depth_matches_the_fixture() -> None:
    """A 16-bit file reads as its raw values, the extremes included."""
    depth = images.read_depth(FOLDER / "png" / "depth16.png")
    assert depth.dtype == np.uint16
    np.testing.assert_array_equal(depth, CASES["depth16.png"]["depth"])


def test_tum_depth_is_in_metres_with_gaps_as_nan() -> None:
    """TUM's reader divides by DEPTH_SCALE and marks a stored 0 as NaN."""
    raw = np.asarray(CASES["depth16.png"]["depth"], dtype=np.float64)
    depth = tum.read_depth(FOLDER / "png" / "depth16.png")
    assert np.isnan(depth[0, 0])
    np.testing.assert_array_equal(depth[raw > 0], raw[raw > 0] / tum.DEPTH_SCALE)


@pytest.mark.parametrize(
    ("reader", "name", "error", "message"),
    [
        (images.read_depth, "gray.png", InvalidInputError, "expected a 16-bit single-channel"),
        (images.read_gray, "depth16.png", InvalidInputError, "expected an 8-bit image"),
        (images.read_rgb, "depth16.png", InvalidInputError, "expected an 8-bit image"),
        (images.read_gray, "missing.png", FileNotFoundError, "no image at"),
        (images.read_gray, "../png.json", InvalidInputError, "not a readable image"),
    ],
)
def test_rejects(reader: Any, name: str, error: type, message: str) -> None:  # noqa: ANN401 -- a reader
    """Wrong bit depths, missing files and non-images are refused."""
    with pytest.raises(error, match=message):
        reader(FOLDER / "png" / name)
