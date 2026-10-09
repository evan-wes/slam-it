"""Reading images from disk: the one module that imports Pillow.

The C++ and Rust cores decode PNGs natively (stb_image in C++'s ``slam_it_all_eval``, the ``png``
crate in Rust's ``slam-it-all-eval``); this is Python's side, and the three read a file the same
way:

- **Gray:** an 8-bit gray PNG as stored (a gray-and-alpha one without its alpha; a 1-bit one
  as 0 and 255); an 8-bit colour one (RGB, RGBA, or a palette expanded to RGB) through
  :func:`slam_it_all.foundations.image.to_gray`, which is OpenCV's conversion bit for bit (not
  Pillow's ``convert("L")``, which rounds differently).
- **Colour:** RGB as stored; gray replicated into three channels; alpha dropped.
- **Depth:** a 16-bit single-channel PNG's raw values; the dataset modules scale them.

An import-linter contract keeps Pillow here: no other slam_it_all module may import it.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from PIL import Image

from slam_it_all.foundations import image
from slam_it_all.foundations.errors import InvalidInputError

_GRAY_MODES = ("L", "LA")
"""Pillow's 8-bit gray modes (with and without alpha)."""

_COLOUR_MODES = ("RGB", "RGBA", "P", "PA")
"""Pillow's 8-bit colour modes (a palette is expanded to RGB)."""

_DEPTH_MODES = ("I;16", "I;16B", "I;16L", "I")
"""Pillow's modes for a 16-bit single-channel PNG (``I`` holds it as 32-bit integers)."""


def _open(path: str | Path) -> Image.Image:
    """Open and decode an image file fully.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it_all.foundations.errors.InvalidInputError
        If it is not an image Pillow can read.
    """
    file = Path(path)
    if not file.is_file():
        raise FileNotFoundError(f"no image at {file}")
    try:
        with Image.open(file) as opened:
            opened.load()
            return opened.copy()
    except (OSError, Image.DecompressionBombError) as error:  # a bomb: far above any dataset's
        raise InvalidInputError(f"{file}: not a readable image ({error})") from None


def _open8(path: str | Path) -> Image.Image:
    """Open an 8-bit image, a 1-bit one widened to 0 and 255 as the native decoders do."""
    picture = _open(path)
    return picture.convert("L") if picture.mode == "1" else picture


def _rgb(picture: Image.Image, path: str | Path) -> NDArray[np.uint8]:
    """Return an 8-bit picture's colour channels, (H, W, 3)."""
    if picture.mode in _GRAY_MODES:
        gray = np.asarray(picture.getchannel(0), dtype=np.uint8)
        return np.ascontiguousarray(np.repeat(gray[:, :, None], 3, axis=2))
    if picture.mode in _COLOUR_MODES:
        return np.ascontiguousarray(np.asarray(picture.convert("RGB"), dtype=np.uint8))
    raise InvalidInputError(f"{path}: expected an 8-bit image, got Pillow mode {picture.mode}")


def read_gray(path: str | Path) -> NDArray[np.uint8]:
    """Read an 8-bit image as gray.

    Parameters
    ----------
    path : str or pathlib.Path
        The file.

    Returns
    -------
    ndarray of uint8, shape (H, W)
        A gray file's values as stored; a colour file's through
        :func:`slam_it_all.foundations.image.to_gray`.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it_all.foundations.errors.InvalidInputError
        If it is not an 8-bit image.
    """
    picture = _open8(path)
    if picture.mode in _GRAY_MODES:
        return np.ascontiguousarray(np.asarray(picture.getchannel(0), dtype=np.uint8))
    return image.to_gray(_rgb(picture, path))


def read_rgb(path: str | Path) -> NDArray[np.uint8]:
    """Read an 8-bit image as RGB.

    Parameters
    ----------
    path : str or pathlib.Path
        The file.

    Returns
    -------
    ndarray of uint8, shape (H, W, 3)
        Red, green and blue; a gray file's value in all three.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it_all.foundations.errors.InvalidInputError
        If it is not an 8-bit image.
    """
    return _rgb(_open8(path), path)


def read_depth(path: str | Path) -> NDArray[np.uint16]:
    """Read a 16-bit single-channel image's raw values.

    Parameters
    ----------
    path : str or pathlib.Path
        The file.

    Returns
    -------
    ndarray of uint16, shape (H, W)
        The values as stored (a dataset module gives them units).

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it_all.foundations.errors.InvalidInputError
        If it is not a 16-bit single-channel image.
    """
    picture = _open(path)
    if picture.mode not in _DEPTH_MODES:
        raise InvalidInputError(
            f"{path}: expected a 16-bit single-channel image, got Pillow mode {picture.mode}"
        )
    values = np.asarray(picture)
    if values.min(initial=0) < 0 or values.max(initial=0) > np.iinfo(np.uint16).max:
        raise InvalidInputError(f"{path}: values outside 16 bits")
    return np.ascontiguousarray(values, dtype=np.uint16)
