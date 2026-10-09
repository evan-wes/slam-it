"""Image-file readers: the shared PNG fixtures, and failure modes."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from slam_it_all.eval import images, tum
from slam_it_all.foundations.errors import InvalidInputError

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


@pytest.mark.parametrize("reader", [images.read_gray, images.read_rgb, images.read_depth])
def test_refuses_the_truncated_fixture(reader: Any) -> None:  # noqa: ANN401 -- a reader
    """A truncated PNG is not a readable image, for every reader (as natively)."""
    assert CASES["truncated.png"] == {"unreadable": True}
    with pytest.raises(InvalidInputError, match="not a readable image"):
        reader(FOLDER / "png" / "truncated.png")


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


def test_rejects_a_decompression_bomb(monkeypatch: pytest.MonkeyPatch) -> None:
    """An image past Pillow's pixel limit is refused like any unreadable file."""
    monkeypatch.setattr("PIL.Image.MAX_IMAGE_PIXELS", 1)
    with pytest.raises(InvalidInputError, match="not a readable image"):
        images.read_gray(FOLDER / "png" / "gray.png")
