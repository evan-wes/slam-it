"""Grayscale images: colour conversion, blur, resizing, pyramids and remapping, in integers.

An image is a C-contiguous ``(H, W)`` uint8 array, row-major: pixel ``(x, y)`` is
``image[y, x]``, and the centre of the top-left pixel is ``(0, 0)`` (the design doc's pixel
convention, OpenCV's).

**Exact by construction.** Every operation here is fixed-point integer arithmetic, so the
Python reference and the C++ and Rust cores produce bit-identical images. That is what lets
the front end's corners and descriptors be held to exact parity: a FAST threshold compares
integers, and one intensity off by one changes which corners exist. Floating point would not
give that across languages (a fused multiply-add on ARM rounds once where x86 rounds twice).

- **Colour to gray:** BT.601's ``Y = 0.299 R + 0.587 G + 0.114 B`` as 15-bit integer weights
  (:data:`GRAY_WEIGHTS`), rounded half up: OpenCV's ``cvtColor``, bit for bit.
- **Blur:** a separable filter whose integer weights sum to a power of two ``2^b``; the two
  passes sum exactly in 64 bits and divide by ``4^b`` once, rounding half up. Borders reflect
  without repeating the edge (``gfedcb|abcdefgh|gfedcba``, OpenCV's ``BORDER_REFLECT_101``).
  The caller supplies the weights (the ORB module holds its Gaussian's), so no backend
  evaluates an exponential.
- **Resize:** bilinear with pixel centres aligned: destination pixel ``d`` samples source
  coordinate ``s = (d + ½) n_in / n_out - ½``, clamped to ``[0, n_in - 1]``. ``s`` is an
  exact rational, ``((2d + 1) n_in - n_out) / (2 n_out)``, so its integer part and its
  fraction, rounded half up to :data:`RESIZE_BITS` bits, are computed in integers; the two
  passes then round half up once.
- **Pyramid:** level ``l`` has ``round(n / s^l)`` rows and columns (half up, ``s^l`` by
  repeated multiplication in float64) and is resized from level ``l - 1``, as ORB-SLAM builds
  its pyramid. The resizes' aligned-centre maps compose exactly, so a position ``x`` on level
  ``l`` lies at ``((2 x + 1) n_0 - n_l) / (2 n_l)`` in the full image
  (:func:`full_image_coordinate`), and a full-image position ``u`` at
  ``((2 u + 1) n_l - n_0) / (2 n_0)`` on the level (:func:`level_coordinate`).
- **Remap:** each output pixel samples the input at a coordinate from a float64 map, rounded
  half up to :data:`REMAP_BITS` bits (a 32nd of a pixel, as OpenCV's ``remap``); a neighbour
  outside the image counts as 0.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations.arrays import (
    MAX_IMAGE_SIDE,
    as_count,
    as_image,
    as_number,
    as_real,
    holds_integers,
)
from slamlab.foundations.errors import InvalidInputError

Image = NDArray[np.uint8]
"""A grayscale image: a C-contiguous (H, W) uint8 array."""

GRAY_WEIGHTS = (9798, 19235, 3735)
"""BT.601's red, green and blue weights over ``2^GRAY_BITS`` (0.299, 0.587 and 0.114)."""

GRAY_BITS = 15
"""The fixed-point bits of :data:`GRAY_WEIGHTS`."""

RESIZE_BITS = 11
"""The fixed-point bits of a bilinear resize's weights (OpenCV's ``INTER_RESIZE_COEF_BITS``)."""

REMAP_BITS = 5
"""The fixed-point bits of a remap's coordinates: a 32nd of a pixel (OpenCV's ``INTER_BITS``)."""

MAX_KERNEL_BITS = 15
"""A blur kernel's weights sum to ``2^b`` with ``b`` at most this."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.image`` submodule provides."""

    def to_gray(self, rgb: NDArray[np.uint8]) -> NDArray[np.uint8]:
        """Convert an (H, W, 3) RGB image to (H, W) gray."""
        ...

    def blur(self, image: NDArray[np.uint8], kernel: tuple[int, ...]) -> NDArray[np.uint8]:
        """Filter with a separable integer kernel summing to a power of two."""
        ...

    def resize(self, image: NDArray[np.uint8], rows: int, cols: int) -> NDArray[np.uint8]:
        """Resize bilinearly to (rows, cols)."""
        ...

    def pyramid(
        self, image: NDArray[np.uint8], levels: int, scale_factor: float
    ) -> list[NDArray[np.uint8]]:
        """Return the image and its successively downscaled levels."""
        ...

    def remap(
        self, image: NDArray[np.uint8], coordinates: NDArray[np.float64]
    ) -> NDArray[np.uint8]:
        """Sample the image at an (H', W', 2) map of (x, y) coordinates."""
        ...


def _decode_image(raw: Any, codes: backends.Codes) -> Image:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's image, checked to be (H, W) uint8 (never cast)."""
    del codes  # the image module crosses no codes
    out: Image = backends.checked_array(raw, np.uint8, 2, "image")
    return out


def _decode_images(raw: Any, codes: backends.Codes) -> list[Image]:  # noqa: ANN401 -- a backend's list
    """Return a compiled backend's list of images, each checked to be (H, W) uint8."""
    return [_decode_image(level, codes) for level in raw]


def _reflect101(index: NDArray[np.int64], size: int) -> NDArray[np.int64]:
    """Map indices outside ``[0, size)`` back inside, reflecting without repeating the edge."""
    if size == 1:
        return np.zeros_like(index)
    period = 2 * size - 2
    wrapped = np.mod(index, period)
    return np.where(wrapped >= size, period - wrapped, wrapped)


def kernel_bits(kernel: Sequence[int]) -> int:
    """Check a blur kernel and return ``b`` such that its weights sum to ``2^b``.

    Parameters
    ----------
    kernel : sequence of int
        The weights.

    Returns
    -------
    int
        ``b``, from 0 to :data:`MAX_KERNEL_BITS`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `kernel` does not hold an odd number of non-negative integers summing to such a
        power of two.
    """
    if len(kernel) % 2 != 1 or any(
        isinstance(w, bool) or not isinstance(w, int) or w < 0 for w in kernel
    ):
        raise InvalidInputError("kernel: must be an odd number of non-negative integers")
    total = sum(kernel)
    if total <= 0 or total & (total - 1) != 0 or total.bit_length() - 1 > MAX_KERNEL_BITS:
        raise InvalidInputError(
            f"kernel: must sum to a power of two up to 2^{MAX_KERNEL_BITS}, got {total}"
        )
    return total.bit_length() - 1


def _to_gray(rgb: NDArray[np.uint8]) -> Image:
    """Convert RGB to gray (reference)."""
    channels = rgb.astype(np.int64)
    r, g, b = GRAY_WEIGHTS
    total = r * channels[..., 0] + g * channels[..., 1] + b * channels[..., 2]
    return np.ascontiguousarray((total + (1 << (GRAY_BITS - 1))) >> GRAY_BITS, dtype=np.uint8)


def _blur(image: Image, kernel: tuple[int, ...]) -> Image:
    """Filter with a separable integer kernel (reference)."""
    shift = 2 * kernel_bits(kernel)
    radius = len(kernel) // 2
    rows, cols = image.shape
    pixels = image.astype(np.int64)
    padded = pixels[:, _reflect101(np.arange(-radius, cols + radius, dtype=np.int64), cols)]
    across = np.zeros((rows, cols), dtype=np.int64)
    for k, w in enumerate(kernel):
        across += w * padded[:, k : k + cols]
    padded = across[_reflect101(np.arange(-radius, rows + radius, dtype=np.int64), rows), :]
    total = np.zeros((rows, cols), dtype=np.int64)
    for k, w in enumerate(kernel):
        total += w * padded[k : k + rows, :]
    half = (1 << shift) >> 1
    return np.ascontiguousarray((total + half) >> shift, dtype=np.uint8)


def _axis_samples(
    n_in: int, n_out: int
) -> tuple[NDArray[np.int64], NDArray[np.int64], NDArray[np.int64]]:
    """Each destination index's two source indices, and the second one's weight in 11 bits."""
    one = 1 << RESIZE_BITS
    numerator = (2 * np.arange(n_out, dtype=np.int64) + 1) * n_in - n_out
    denominator = 2 * n_out
    inside = numerator > 0  # s <= 0 clamps to the first pixel
    whole = np.where(inside, numerator // denominator, 0)
    remainder = np.where(inside, numerator - whole * denominator, 0)
    # round(remainder / denominator * 2^bits), half up, in integers; a carry moves to `whole`.
    fraction = (2 * remainder * one + denominator) // (2 * denominator)
    carry = fraction == one
    whole = whole + carry
    fraction = np.where(carry, 0, fraction)
    last = whole >= n_in - 1  # s >= n_in - 1 clamps to the last pixel
    whole = np.where(last, n_in - 1, whole)
    fraction = np.where(last, 0, fraction)
    return whole, np.minimum(whole + 1, n_in - 1), fraction


def _resize(image: Image, rows: int, cols: int) -> Image:
    """Resize bilinearly (reference)."""
    one = 1 << RESIZE_BITS
    x0, x1, fx = _axis_samples(image.shape[1], cols)
    y0, y1, fy = _axis_samples(image.shape[0], rows)
    pixels = image.astype(np.int64)
    across: NDArray[np.int64] = pixels[:, x0] * (one - fx) + pixels[:, x1] * fx
    total: NDArray[np.int64] = across[y0, :] * (one - fy)[:, None] + across[y1, :] * fy[:, None]
    shift = 2 * RESIZE_BITS
    return np.ascontiguousarray((total + (1 << (shift - 1))) >> shift, dtype=np.uint8)


def as_scale_factor(value: object, name: str = "scale_factor") -> float:
    """Check a pyramid's scale factor: a finite real number above 1.

    Parameters
    ----------
    value : object
        The factor: a Python or numpy integer or float (a bool is refused).
    name : str, optional
        Its name, for messages.

    Returns
    -------
    float
        The factor, as a Python float.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `value` is not a real number, or not finite and above 1.
    """
    factor = as_number(value, name)
    if not (math.isfinite(factor) and factor > 1.0):
        raise InvalidInputError(f"{name}: must be finite and above 1, got {factor}")
    return factor


def _as_levels(levels: object, scale_factor: object) -> tuple[int, float]:
    """Check a pyramid's level count and scale factor."""
    return as_count(levels, "levels", 1, MAX_IMAGE_SIDE), as_scale_factor(scale_factor)


def level_shapes(rows: int, cols: int, levels: int, scale_factor: float) -> list[tuple[int, int]]:
    """Return each pyramid level's ``(rows, cols)``: ``round(n / s^l)``, half up.

    Parameters
    ----------
    rows, cols : int
        Level 0's shape.
    levels : int
        The number of levels, at least 1.
    scale_factor : float
        Each level's downscale from the previous one, above 1.

    Returns
    -------
    list of tuple of int
        One shape per level; ``s^l`` is accumulated by repeated multiplication in float64, so
        every backend computes the same shapes.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `rows` or `cols` is not an integer from 1 to
        :data:`slamlab.foundations.arrays.MAX_IMAGE_SIDE`, `levels` is not one from 1 to
        that, `scale_factor` is not a finite number above 1, or a level would be empty.
    """
    rows = as_count(rows, "rows", 1, MAX_IMAGE_SIDE)
    cols = as_count(cols, "cols", 1, MAX_IMAGE_SIDE)
    levels, scale_factor = _as_levels(levels, scale_factor)
    shapes = [(rows, cols)]
    scale = 1.0
    for level in range(1, levels):
        scale *= scale_factor
        shape = (math.floor(rows / scale + 0.5), math.floor(cols / scale + 0.5))
        if min(shape) < 1:
            raise InvalidInputError(
                f"pyramid: level {level} of a {rows}x{cols} image at scale factor "
                f"{scale_factor} would be empty"
            )
        shapes.append(shape)
    return shapes


def full_image_coordinate(x: ArrayLike, level_size: ArrayLike, size: int) -> NDArray[np.float64]:
    """Map positions on a pyramid level to the full image, along one axis.

    Each resize samples source coordinate ``(d + 1/2) n_in / n_out - 1/2`` for destination
    pixel ``d``; these maps compose along the pyramid, so position ``x`` on a level of
    ``n_l`` pixels lies at ``(x + 1/2) n_0 / n_l - 1/2`` in a full image of ``n_0``. That is
    computed as ``((2 x + 1) n_0 - n_l) / (2 n_l)`` in float64, one operation at a time, as
    every backend does: for an integer ``x`` only the division rounds, and level 0 maps to
    itself. (ORB-SLAM and OpenCV scale by ``s^l`` instead, which drops the half-pixel term
    and uses ``s^l`` for ``n_0 / n_l``: up to about 2 pixels off on a coarse level.)

    Parameters
    ----------
    x : array_like of float
        Positions on the level, in its pixels.
    level_size : int or array_like of int
        The level's size along the axis, broadcast against `x`.
    size : int
        The full image's size along it.

    Returns
    -------
    ndarray of float64
        The full-image coordinates, `x`'s shape.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `x` is not finite, or a size is not an integer from 1 to
        :data:`slamlab.foundations.arrays.MAX_IMAGE_SIDE`.
    """
    positions, level, full = _axis_map(x, "x", level_size, size)
    out: NDArray[np.float64] = ((2.0 * positions + 1.0) * full - level) / (2.0 * level)
    return out


def level_coordinate(u: ArrayLike, level_size: ArrayLike, size: int) -> NDArray[np.float64]:
    """Map full-image positions to a pyramid level, along one axis.

    The inverse of :func:`full_image_coordinate`: full-image position ``u`` lies at
    ``(u + 1/2) n_l / n_0 - 1/2`` on a level of ``n_l`` pixels, computed as
    ``((2 u + 1) n_l - n_0) / (2 n_0)`` in float64, one operation at a time, as every backend
    does. Level 0 maps to itself. Composed with :func:`full_image_coordinate`, it maps one
    level onto another: pixel ``x`` of a level of ``n_a`` lies at
    ``((2 x + 1) n_b - n_a) / (2 n_a)`` on a level of ``n_b``, whose nearest pixel is
    ``floor((2 x + 1) n_b / (2 n_a))`` (the column :func:`slamlab.foundations.stereo.match`
    searches from, in integers).

    Parameters
    ----------
    u : array_like of float
        Positions in the full image, in its pixels.
    level_size : int or array_like of int
        The level's size along the axis, broadcast against `u`.
    size : int
        The full image's size along it.

    Returns
    -------
    ndarray of float64
        The level coordinates, `u`'s shape.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `u` is not finite, or a size is not an integer from 1 to
        :data:`slamlab.foundations.arrays.MAX_IMAGE_SIDE`.
    """
    positions, level, full = _axis_map(u, "u", level_size, size)
    out: NDArray[np.float64] = ((2.0 * positions + 1.0) * level - full) / (2.0 * full)
    return out


def _axis_map(
    positions: ArrayLike, name: str, level_size: ArrayLike, size: int
) -> tuple[NDArray[np.float64], NDArray[np.float64], float]:
    """Check a coordinate map's inputs: finite positions, a level's size and the image's."""
    values = as_real(positions, name).reshape(np.shape(positions))  # a scalar stays 0-d
    sizes = np.asarray(level_size)
    if not holds_integers(sizes) or np.any(sizes < 1) or np.any(sizes > MAX_IMAGE_SIDE):
        raise InvalidInputError(f"level_size: must be integers from 1 to {MAX_IMAGE_SIDE}")
    full = float(as_count(size, "size", 1, MAX_IMAGE_SIDE))
    return values, sizes.astype(np.float64), full


def _pyramid(image: Image, levels: int, scale_factor: float) -> list[Image]:
    """Build the pyramid (reference)."""
    out = [image.copy()]
    rows, cols = image.shape
    for shape in level_shapes(rows, cols, levels, scale_factor)[1:]:
        out.append(_resize(out[-1], *shape))
    return out


def _remap(image: Image, coordinates: NDArray[np.float64]) -> Image:
    """Sample at a coordinate map (reference)."""
    rows, cols = image.shape
    one = 1 << REMAP_BITS
    # Clamping keeps the fixed-point values small; past it every neighbour is outside anyway.
    x = np.clip(coordinates[..., 0], -2.0, cols + 1.0)
    y = np.clip(coordinates[..., 1], -2.0, rows + 1.0)
    qx = np.floor(x * one + 0.5).astype(np.int64)
    qy = np.floor(y * one + 0.5).astype(np.int64)
    ix, fx = qx >> REMAP_BITS, qx & (one - 1)
    iy, fy = qy >> REMAP_BITS, qy & (one - 1)
    pixels = image.astype(np.int64)

    def at(r: NDArray[np.int64], c: NDArray[np.int64]) -> NDArray[np.int64]:
        inside = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
        return np.where(inside, pixels[np.clip(r, 0, rows - 1), np.clip(c, 0, cols - 1)], 0)

    total = (
        at(iy, ix) * (one - fx) * (one - fy)
        + at(iy, ix + 1) * fx * (one - fy)
        + at(iy + 1, ix) * (one - fx) * fy
        + at(iy + 1, ix + 1) * fx * fy
    )
    shift = 2 * REMAP_BITS
    return np.ascontiguousarray((total + (1 << (shift - 1))) >> shift, dtype=np.uint8)


def to_gray(rgb: ArrayLike, *, backend: str = "python") -> Image:
    """Convert an RGB image to gray: ``Y = 0.299 R + 0.587 G + 0.114 B`` in 15-bit integers.

    Parameters
    ----------
    rgb : array_like of uint8, shape (H, W, 3)
        Red, green and blue channels, in that order (as Pillow reads a PNG).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of uint8, shape (H, W)
        The gray image, rounded half up.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `rgb` is not a non-empty (H, W, 3) uint8 array.
    """
    image = as_image(rgb, "rgb", channels=3)
    return backends.call(
        backend,
        __name__,
        "to_gray",
        reference=lambda: _to_gray(image),
        arguments={"rgb": image},
        decode=_decode_image,
    )


def blur(image: ArrayLike, kernel: Sequence[int], *, backend: str = "python") -> Image:
    """Filter separably with integer weights that sum to a power of two.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        The image.
    kernel : sequence of int
        An odd number of non-negative weights, applied along rows, then along columns; they
        sum to ``2^b`` with ``b`` at most :data:`MAX_KERNEL_BITS`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of uint8, shape (H, W)
        The filtered image, rounded half up, with borders reflected (OpenCV's
        ``BORDER_REFLECT_101``).

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `image` is not a non-empty (H, W) uint8 array, or `kernel` is not as described.
    """
    pixels = as_image(image, "image")
    weights = tuple(int(w) if isinstance(w, np.integer) else w for w in kernel)
    kernel_bits(weights)
    return backends.call(
        backend,
        __name__,
        "blur",
        reference=lambda: _blur(pixels, weights),
        arguments={"image": pixels, "kernel": weights},
        decode=_decode_image,
    )


def resize(image: ArrayLike, rows: int, cols: int, *, backend: str = "python") -> Image:
    """Resize bilinearly, pixel centres aligned, in 11-bit fixed point.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        The image.
    rows, cols : int
        The output's shape, each from 1 to :data:`slamlab.foundations.arrays.MAX_IMAGE_SIDE`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of uint8, shape (rows, cols)
        The resized image.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `image` is not a non-empty (H, W) uint8 array, or the shape is out of range.
    """
    pixels = as_image(image, "image")
    out_rows = as_count(rows, "rows", 1, MAX_IMAGE_SIDE)
    out_cols = as_count(cols, "cols", 1, MAX_IMAGE_SIDE)
    return backends.call(
        backend,
        __name__,
        "resize",
        reference=lambda: _resize(pixels, out_rows, out_cols),
        arguments={"image": pixels, "rows": out_rows, "cols": out_cols},
        decode=_decode_image,
    )


def pyramid(
    image: ArrayLike, levels: int, scale_factor: float, *, backend: str = "python"
) -> list[Image]:
    """Return the image and its successively downscaled levels.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        Level 0.
    levels : int
        The number of levels, at least 1.
    scale_factor : float
        Each level's downscale from the previous one, above 1 (ORB uses 1.2).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    list of ndarray of uint8
        Level ``l`` has the shape :func:`level_shapes` gives, and is resized from level
        ``l - 1``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `image` is not a non-empty (H, W) uint8 array, `levels` is below 1,
        `scale_factor` is not a finite number above 1, or a level would be empty.
    """
    pixels = as_image(image, "image")
    count, factor = _as_levels(levels, scale_factor)
    level_shapes(pixels.shape[0], pixels.shape[1], count, factor)
    return backends.call(
        backend,
        __name__,
        "pyramid",
        reference=lambda: _pyramid(pixels, count, factor),
        arguments={"image": pixels, "levels": count, "scale_factor": factor},
        decode=_decode_images,
    )


def remap(image: ArrayLike, coordinates: ArrayLike, *, backend: str = "python") -> Image:
    """Sample an image bilinearly at a map of coordinates, in fixed point.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        The image.
    coordinates : array_like, shape (H', W', 2)
        Each output pixel's source coordinate ``(x, y)``, in the input's pixels; rounded half
        up to a 32nd of a pixel.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of uint8, shape (H', W')
        The sampled image; neighbours outside the input count as 0.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `image` is not a non-empty (H, W) uint8 array, or `coordinates` is not a finite,
        non-empty (H', W', 2) array with sides at most
        :data:`slamlab.foundations.arrays.MAX_IMAGE_SIDE`.
    """
    pixels = as_image(image, "image")
    where = as_real(coordinates, "coordinates")
    if where.ndim != 3 or where.shape[2] != 2 or 0 in where.shape:
        raise InvalidInputError(
            f"coordinates: expected a non-empty (H, W, 2) array, got {where.shape}"
        )
    if max(where.shape[:2]) > MAX_IMAGE_SIDE:
        raise InvalidInputError(
            f"coordinates: sides must be at most {MAX_IMAGE_SIDE}, got {where.shape[:2]}"
        )
    return backends.call(
        backend,
        __name__,
        "remap",
        reference=lambda: _remap(pixels, where),
        arguments={"image": pixels, "coordinates": where},
        decode=_decode_image,
    )
