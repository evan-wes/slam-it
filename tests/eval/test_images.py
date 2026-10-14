"""Image-file readers: the shared PNG fixtures, and failure modes."""

from __future__ import annotations

import io
import json
import zlib
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from PIL import Image

from slam_it.eval import images, tum
from slam_it.foundations.errors import InvalidInputError

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


@pytest.mark.parametrize("shape", [(24, 32), (24, 32, 3)])
def test_encode_jpeg_round_trips_closely(shape: tuple[int, ...], tmp_path: Path) -> None:
    """A gray or RGB image comes back as itself, within JPEG's loss on a smooth image."""
    rows, cols = np.mgrid[0 : shape[0], 0 : shape[1]]
    smooth = (4 * rows + 3 * cols).astype(np.uint8)
    picture = smooth if len(shape) == 2 else np.stack([smooth, smooth // 2, 255 - smooth], -1)
    data = images.encode_jpeg(picture, 95)
    with Image.open(io.BytesIO(data)) as decoded:  # the readers open PNG alone
        assert decoded.format == "JPEG"
        back = np.asarray(decoded.convert("L" if len(shape) == 2 else "RGB"))
    assert back.shape == picture.shape
    assert np.abs(back.astype(int) - picture).mean() < 2.0


@pytest.mark.parametrize(
    ("picture", "quality", "message"),
    [
        (np.zeros((4, 4), dtype=np.uint16), 90, "expected a uint8 array"),
        (np.zeros((4, 4, 4), dtype=np.uint8), 90, r"expected \(H, W\) or \(H, W, 3\)"),
        (np.zeros(4, dtype=np.uint8), 90, r"expected \(H, W\) or \(H, W, 3\)"),
        (np.zeros((4, 4), dtype=np.uint8), 0, "quality: expected 1 to 95"),
        (np.zeros((4, 4), dtype=np.uint8), 96, "quality: expected 1 to 95"),
    ],
)
def test_encode_jpeg_refuses(picture: np.ndarray, quality: int, message: str) -> None:
    """Not 8-bit gray or RGB, or a quality outside Pillow's range."""
    with pytest.raises(InvalidInputError, match=message):
        images.encode_jpeg(picture, quality)


@pytest.mark.parametrize("shape", [(24, 32), (24, 32, 3)])
def test_write_png_round_trips_exactly(shape: tuple[int, ...], tmp_path: Path) -> None:
    """A gray or RGB image written as PNG reads back as itself, its folder made."""
    picture = np.random.default_rng(0).integers(0, 256, shape, dtype=np.uint8)
    path = tmp_path / "new" / "frame.png"
    images.write_png(path, picture)
    back = images.read_gray(path) if len(shape) == 2 else images.read_rgb(path)
    np.testing.assert_array_equal(back, picture)


def test_write_png_refuses_what_is_not_8_bit_gray_or_rgb(tmp_path: Path) -> None:
    """A 16-bit or four-channel array is refused, and nothing is written."""
    wrong: list[np.ndarray] = [np.zeros((4, 4), np.uint16), np.zeros((4, 4, 4), np.uint8)]
    for picture in wrong:
        with pytest.raises(InvalidInputError, match="picture: expected"):
            images.write_png(tmp_path / "frame.png", picture)
    assert not (tmp_path / "frame.png").exists()


def _chunk(kind: bytes, body: bytes) -> bytes:
    """Return a PNG chunk: its length, kind, body and CRC."""
    return len(body).to_bytes(4, "big") + kind + body + zlib.crc32(kind + body).to_bytes(4, "big")


def _gray_png(width: int, height: int) -> bytes:
    """Return a valid 8-bit gray PNG's bytes."""
    buffer = io.BytesIO()
    Image.fromarray(np.zeros((height, width), np.uint8)).save(buffer, format="PNG")
    return buffer.getvalue()


SIGNATURE = b"\x89PNG\r\n\x1a\n"
"""A PNG file's first eight bytes."""

IHDR = _chunk(b"IHDR", (4).to_bytes(4, "big") * 2 + bytes([8, 0, 0, 0, 0]))
"""A 4x4 8-bit gray image's header chunk."""


@pytest.mark.parametrize(
    "data",
    [
        SIGNATURE + IHDR[:12],  # a header cut short
        SIGNATURE
        + IHDR
        + _chunk(b"iCCP", b"p\0\0" + b"x" * (1 << 20 | 1)),  # past Pillow's text cap
        _gray_png(4, 4)[:-12] + _chunk(b"IHDR", b"bad") + _chunk(b"IEND", b""),  # read at the end
    ],
    ids=["truncated-header", "huge-profile", "bad-chunk-after-the-data"],
)
def test_pillows_other_parse_errors_are_invalid_input(data: bytes) -> None:
    """A malformed chunk raises ValueError or SyntaxError in Pillow, not OSError: ours too."""
    with pytest.raises(InvalidInputError, match="image: not a readable image"):
        images.read_gray(data)


def test_bytes_read_as_the_file_does_and_a_side_over_the_bound_is_refused() -> None:
    """From bytes, as from a file; the size bound is checked before the pixels are decoded."""
    data = _gray_png(6, 4)
    assert images.size(data) == (6, 4)
    np.testing.assert_array_equal(images.read_gray(data, max_side=6), np.zeros((4, 6), np.uint8))
    with pytest.raises(InvalidInputError, match=r"image: 6x4, over 5 pixels a side"):
        images.read_gray(data, max_side=5)


def test_a_decompression_bombs_warning_is_an_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """Past Pillow's warning threshold, not only its error one, an image is refused."""
    monkeypatch.setattr(Image, "MAX_IMAGE_PIXELS", 20)  # 24 pixels: a warning, not Pillow's error
    with pytest.raises(InvalidInputError, match="not a readable image"):
        images.read_gray(_gray_png(6, 4))


def _eps(width: int, height: int) -> bytes:
    """Return a small EPS file: PostScript, which Pillow would hand to Ghostscript to draw."""
    return (
        f"%!PS-Adobe-3.0 EPSF-3.0\n%%BoundingBox: 0 0 {width} {height}\n"
        "newpath 0 0 moveto 10 10 lineto stroke showpage\n%%EOF\n"
    ).encode()


def _tiff(width: int, height: int) -> bytes:
    """Return a 16-bit gray TIFF's bytes."""
    buffer = io.BytesIO()
    Image.fromarray(np.zeros((height, width), np.uint16)).save(buffer, format="TIFF")
    return buffer.getvalue()


@pytest.mark.parametrize(
    "data",
    [_eps(64, 48), _tiff(64, 48), images.encode_jpeg(np.zeros((48, 64), np.uint8), 90)],
    ids=["eps", "tiff", "jpeg"],
)
def test_only_png_is_opened_whatever_the_content_says(data: bytes) -> None:
    """PostScript, TIFF and JPEG are refused before they are decoded: PNG alone is opened."""
    for reader in (images.read_gray, images.read_rgb, images.read_depth):
        with pytest.raises(InvalidInputError, match=r"not a readable image \(not a PNG\)"):
            reader(data)
