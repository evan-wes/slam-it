"""Reading and writing images, and encoding them as JPEG: the one module that imports Pillow.

The C++ and Rust cores decode PNGs natively (stb_image in C++'s ``slam_it_eval``, the ``png``
crate in Rust's ``slam-it-eval``); this is Python's side, and the three read a file the same
way:

- **Gray:** an 8-bit gray PNG as stored (a gray-and-alpha one without its alpha; a 1-bit one
  as 0 and 255); an 8-bit colour one (RGB, RGBA, or a palette expanded to RGB) through
  :func:`slam_it.foundations.image.to_gray`, which is OpenCV's conversion bit for bit (not
  Pillow's ``convert("L")``, which rounds differently).
- **Colour:** RGB as stored; gray replicated into three channels; alpha dropped.
- **Depth:** a 16-bit single-channel PNG's raw values; the dataset modules scale them.

Each reader takes a file, or a file's bytes (an upload's entry, never written to disk), and
optionally the longest side it accepts: checked against the size Pillow will decode, every
header chunk parsed, before any pixel is (a PNG may declare one size and decode at another).
Only PNG is opened, as the native decoders open only PNG: Pillow names a format by a file's
content, not its name, and its other formats include ones it decodes by running another program
(EPS, through Ghostscript). A file that is not a PNG, one Pillow cannot parse, and one past its
decompression-bomb threshold are refused as invalid input, never raised as Pillow's own errors
or warnings. :func:`size` reads the size alone.

:func:`encode_jpeg` encodes a frame for the run viewer's bundles
(:mod:`slam_it.orchestration.bundles`); :func:`write_png` writes the frames a phone's video
is imported as (:mod:`slam_it.eval.video`).

An import-linter contract keeps Pillow here: no other slam_it module may import it.
"""

from __future__ import annotations

import contextlib
import io
import warnings
from collections.abc import Iterator
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from PIL import Image, UnidentifiedImageError

from slam_it.foundations import image
from slam_it.foundations.errors import InvalidInputError

_GRAY_MODES = ("L", "LA")
"""Pillow's 8-bit gray modes (with and without alpha)."""

_COLOUR_MODES = ("RGB", "RGBA", "P", "PA")
"""Pillow's 8-bit colour modes (a palette is expanded to RGB)."""

_DEPTH_MODES = ("I;16", "I;16B", "I;16L", "I")
"""Pillow's modes for a 16-bit single-channel PNG (``I`` holds it as 32-bit integers)."""


_Source = str | Path | bytes
"""An image file's path, or its bytes."""

_FORMATS = ("PNG",)
"""The formats Pillow may open: PNG alone, whatever a file's content says it is."""

_PILLOW_ERRORS = (
    OSError, ValueError, SyntaxError, Image.DecompressionBombError,
    Image.DecompressionBombWarning,
)  # fmt: skip
"""What Pillow raises for a file it cannot parse: a truncated or malformed chunk raises
``ValueError`` or ``SyntaxError``, not only ``OSError``; a bomb's warning is made an error."""


def _name(source: _Source) -> str:
    """Return how messages name an image: its path, or ``image`` for bytes."""
    return "image" if isinstance(source, bytes) else str(source)


@contextlib.contextmanager
def _opened(source: _Source, max_side: int | None) -> Iterator[Image.Image]:
    """Open an image, its size checked against `max_side`; Pillow's errors made ours.

    Raises
    ------
    FileNotFoundError
        If a path is not a file.
    slam_it.foundations.errors.InvalidInputError
        If it is not an image Pillow can read, or a side is over `max_side`.
    """
    if not isinstance(source, bytes) and not Path(source).is_file():
        raise FileNotFoundError(f"no image at {source}")
    stream = io.BytesIO(source) if isinstance(source, bytes) else Path(source)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", Image.DecompressionBombWarning)
            with Image.open(stream, formats=_FORMATS) as opened:  # its headers parsed: size final
                width, height = opened.size
                if max_side is not None and max(width, height) > max_side:
                    raise InvalidInputError(
                        f"{_name(source)}: {width}x{height}, over {max_side} pixels a side"
                    )
                yield opened
    except InvalidInputError:
        raise
    except UnidentifiedImageError:  # its message names the stream, not the image
        raise InvalidInputError(f"{_name(source)}: not a readable image (not a PNG)") from None
    except _PILLOW_ERRORS as error:  # a bomb: far above any dataset's
        raise InvalidInputError(f"{_name(source)}: not a readable image ({error})") from None


def _open(source: _Source, max_side: int | None = None) -> Image.Image:
    """Open and decode an image fully, its size checked first (:func:`_opened`)."""
    with _opened(source, max_side) as opened:
        opened.load()
        return opened.copy()


def size(source: _Source) -> tuple[int, int]:
    """Read an image's width and height, as Pillow would decode it, without decoding it.

    Parameters
    ----------
    source : str, pathlib.Path or bytes
        The file, or its bytes.

    Returns
    -------
    tuple of int
        The width and height.

    Raises
    ------
    FileNotFoundError
        If a path is not a file.
    slam_it.foundations.errors.InvalidInputError
        If it is not an image Pillow can read.
    """
    with _opened(source, None) as opened:
        width, height = opened.size
    return int(width), int(height)


def _open8(source: _Source, max_side: int | None = None) -> Image.Image:
    """Open an 8-bit image, a 1-bit one widened to 0 and 255 as the native decoders do."""
    picture = _open(source, max_side)
    return picture.convert("L") if picture.mode == "1" else picture


def _rgb(picture: Image.Image, path: _Source) -> NDArray[np.uint8]:
    """Return an 8-bit picture's colour channels, (H, W, 3)."""
    if picture.mode in _GRAY_MODES:
        gray = np.asarray(picture.getchannel(0), dtype=np.uint8)
        return np.ascontiguousarray(np.repeat(gray[:, :, None], 3, axis=2))
    if picture.mode in _COLOUR_MODES:
        return np.ascontiguousarray(np.asarray(picture.convert("RGB"), dtype=np.uint8))
    raise InvalidInputError(
        f"{_name(path)}: expected an 8-bit image, got Pillow mode {picture.mode}"
    )


def read_gray(path: _Source, max_side: int | None = None) -> NDArray[np.uint8]:
    """Read an 8-bit image as gray.

    Parameters
    ----------
    path : str, pathlib.Path or bytes
        The file, or its bytes.
    max_side : int, optional
        The longest side accepted (default: any Pillow decodes).

    Returns
    -------
    ndarray of uint8, shape (H, W)
        A gray file's values as stored; a colour file's through
        :func:`slam_it.foundations.image.to_gray`.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it is not an 8-bit image, or a side is over `max_side`.
    """
    picture = _open8(path, max_side)
    if picture.mode in _GRAY_MODES:
        return np.ascontiguousarray(np.asarray(picture.getchannel(0), dtype=np.uint8))
    return image.to_gray(_rgb(picture, path))


def read_rgb(path: _Source, max_side: int | None = None) -> NDArray[np.uint8]:
    """Read an 8-bit image as RGB.

    Parameters
    ----------
    path : str, pathlib.Path or bytes
        The file, or its bytes.
    max_side : int, optional
        The longest side accepted (default: any Pillow decodes).

    Returns
    -------
    ndarray of uint8, shape (H, W, 3)
        Red, green and blue; a gray file's value in all three.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it is not an 8-bit image, or a side is over `max_side`.
    """
    return _rgb(_open8(path, max_side), path)


def read_depth(path: _Source, max_side: int | None = None) -> NDArray[np.uint16]:
    """Read a 16-bit single-channel image's raw values.

    Parameters
    ----------
    path : str, pathlib.Path or bytes
        The file, or its bytes.
    max_side : int, optional
        The longest side accepted (default: any Pillow decodes).

    Returns
    -------
    ndarray of uint16, shape (H, W)
        The values as stored (a dataset module gives them units).

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it is not a 16-bit single-channel image, or a side is over `max_side`.
    """
    picture = _open(path, max_side)
    if picture.mode not in _DEPTH_MODES:
        raise InvalidInputError(
            f"{_name(path)}: expected a 16-bit single-channel image, got Pillow mode {picture.mode}"
        )
    values = np.asarray(picture)
    if values.min(initial=0) < 0 or values.max(initial=0) > np.iinfo(np.uint16).max:
        raise InvalidInputError(f"{_name(path)}: values outside 16 bits")
    return np.ascontiguousarray(values, dtype=np.uint16)


def _check_picture(picture: object) -> NDArray[np.uint8]:
    """Return an 8-bit gray or RGB image, refusing anything else."""
    if not isinstance(picture, np.ndarray) or picture.dtype != np.uint8:
        raise InvalidInputError("picture: expected a uint8 array")
    if picture.ndim != 2 and (picture.ndim != 3 or picture.shape[2] != 3):
        raise InvalidInputError(f"picture: expected (H, W) or (H, W, 3), got {picture.shape}")
    return picture


def write_png(path: str | Path, picture: NDArray[np.uint8]) -> None:
    """Write an 8-bit gray or RGB image as a PNG file, its folder created.

    Parameters
    ----------
    path : str or pathlib.Path
        The file to write.
    picture : ndarray of uint8, shape (H, W) or (H, W, 3)
        A gray or RGB image; :func:`read_gray` and :func:`read_rgb` read it back as given.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the image is not 8-bit gray or RGB.
    """
    checked = _check_picture(picture)
    file = Path(path)
    file.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.ascontiguousarray(checked)).save(file, format="PNG")


def write_depth(path: str | Path, depth: NDArray[np.uint16]) -> None:
    """Write a 16-bit single-channel image (a depth map's raw values) as a PNG, its folder made.

    Parameters
    ----------
    path : str or pathlib.Path
        The file to write.
    depth : ndarray of uint16, shape (H, W)
        The values; :func:`read_depth` reads them back as given.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the array is not 2-D uint16.
    """
    if not isinstance(depth, np.ndarray) or depth.dtype != np.uint16 or depth.ndim != 2:
        raise InvalidInputError("depth: expected a 2-D uint16 array")
    file = Path(path)
    file.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.ascontiguousarray(depth)).save(file, format="PNG")


def encode_jpeg(picture: NDArray[np.uint8], quality: int) -> bytes:
    """Encode an 8-bit image as a JPEG file's bytes (baseline, 4:2:0 for colour).

    Parameters
    ----------
    picture : ndarray of uint8, shape (H, W) or (H, W, 3)
        A gray or RGB image.
    quality : int
        The JPEG quality, from 1 to 95 (Pillow's useful range).

    Returns
    -------
    bytes
        The file's bytes.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the image is not 8-bit gray or RGB, or the quality is out of range.
    """
    checked = _check_picture(picture)
    if not 1 <= quality <= 95:  # Pillow's documented range
        raise InvalidInputError(f"quality: expected 1 to 95, got {quality}")
    buffer = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(checked)).save(buffer, format="JPEG", quality=quality)
    return buffer.getvalue()
