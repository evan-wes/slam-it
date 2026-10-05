"""Input validation shared by every rung: convert and check array arguments and settings.

Public functions validate their inputs here, once, before any backend runs, so all
three language implementations see the same clean data (float64 arrays, uint8 images,
Python integers, booleans and floats for settings) and every bad input fails the same
way, with :class:`slamlab.foundations.errors.InvalidInputError`.

Batched functions accept any number of leading dimensions. :func:`as_batch` checks the
trailing shape and flattens the leading ones, so a compiled backend always receives a
C-contiguous ``(N, *shape)`` array; :func:`unbatch` restores the caller's shape.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab.foundations.errors import InvalidInputError

_NUMERIC_KINDS = frozenset("iuf")
"""dtype kinds accepted as numbers: signed and unsigned integers, floats.

Booleans, complex numbers, strings and objects (e.g. ``None``) are rejected rather
than coerced, because numpy would silently turn them into numbers.
"""


MAX_IMAGE_SIDE = 1 << 20
"""The largest image side, in pixels, that :mod:`slamlab.foundations.image`'s fixed-point
arithmetic is sized for (its intermediate values then fit 64 bits)."""


def as_real(
    value: ArrayLike, name: str = "input", *, allow_nan: bool = False
) -> NDArray[np.float64]:
    """Convert input to a finite, C-contiguous float64 array (a new copy).

    Parameters
    ----------
    value : array_like
        The input. Integer and floating-point values are accepted; integers beyond
        2**53 lose precision in the float64 conversion.
    name : str, optional
        The argument's name, used in error messages.
    allow_nan : bool, optional
        Accept NaN as "no value" (infinity is still rejected).

    Returns
    -------
    ndarray of float64
        A new C-contiguous array with the input's shape.

    Raises
    ------
    InvalidInputError
        If `value` is a masked array (its masked values would be used silently), is not
        numeric (booleans, complex, strings and ``None`` are rejected), or holds NaN or
        infinity.
    """
    if isinstance(value, np.ma.MaskedArray):
        raise InvalidInputError(
            f"{name}: masked arrays are not accepted; fill or compress them first"
        )
    arr = np.asarray(value)
    if arr.dtype.kind not in _NUMERIC_KINDS:
        raise InvalidInputError(f"{name}: expected real numbers, got dtype {arr.dtype}")
    out = np.ascontiguousarray(arr, dtype=np.float64).copy()
    bad = np.isinf(out) if allow_nan else ~np.isfinite(out)
    if np.any(bad):
        raise InvalidInputError(f"{name}: expected finite values")
    return out


def as_indices(value: ArrayLike, length: int, bound: int, name: str = "input") -> NDArray[np.int64]:
    """Convert input to an int64 index array of a given length, each entry in ``[0, bound)``.

    Parameters
    ----------
    value : array_like of int
        The indices.
    length : int
        The expected number of indices.
    bound : int
        One past the largest valid index.
    name : str, optional
        The argument's name, used in error messages.

    Returns
    -------
    ndarray of int64, shape (length,)
        A new C-contiguous copy.

    Raises
    ------
    InvalidInputError
        If `value` is not a 1D integer array of `length`, or an index is out of range.
    """
    arr = np.asarray(value)
    if arr.shape != (length,) or not np.issubdtype(arr.dtype, np.integer):
        raise InvalidInputError(f"{name}: expected {length} integers, got {arr.dtype} {arr.shape}")
    if length and (arr.min() < 0 or arr.max() >= bound):
        raise InvalidInputError(f"{name}: indices must lie in [0, {bound})")
    return np.ascontiguousarray(arr, dtype=np.int64).copy()


def as_mask(value: ArrayLike, shape: tuple[int, ...], name: str = "input") -> NDArray[np.bool_]:
    """Convert input to a boolean mask of a given shape.

    Parameters
    ----------
    value : array_like of bool
        The mask (integers are rejected, not coerced).
    shape : tuple of int
        The expected shape.
    name : str, optional
        The argument's name, used in error messages.

    Returns
    -------
    ndarray of bool
        A new C-contiguous copy.

    Raises
    ------
    InvalidInputError
        If `value` is not boolean or has another shape.
    """
    arr = np.asarray(value)
    if arr.shape != shape or arr.dtype != np.bool_:
        raise InvalidInputError(
            f"{name}: expected booleans of shape {shape}, got {arr.dtype} {arr.shape}"
        )
    return np.ascontiguousarray(arr).copy()


def as_weights(
    value: ArrayLike, shape: tuple[int, ...], name: str = "input"
) -> NDArray[np.float64]:
    """Convert input to finite, non-negative weights of a given shape.

    Parameters
    ----------
    value : array_like
        The weights.
    shape : tuple of int
        The expected shape.
    name : str, optional
        The argument's name, used in error messages.

    Returns
    -------
    ndarray of float64
        A new C-contiguous copy.

    Raises
    ------
    InvalidInputError
        If `value` fails :func:`as_real`, has another shape, or holds a negative value.
    """
    w = as_real(value, name)
    if w.shape != shape or np.any(w < 0.0):
        raise InvalidInputError(f"{name}: expected non-negative values of shape {shape}")
    return w


def as_batch(
    value: ArrayLike, shape: tuple[int, ...], name: str = "input"
) -> tuple[NDArray[np.float64], tuple[int, ...]]:
    """Validate a batch of fixed-shape items and flatten its leading dimensions.

    Parameters
    ----------
    value : array_like, shape (..., *shape)
        The input: one item of shape `shape`, or any batch of them.
    shape : tuple of int
        The shape of one item, e.g. ``(3,)`` for 3-vectors or ``(3, 3)`` for matrices.
    name : str, optional
        The argument's name, used in error messages.

    Returns
    -------
    flat : ndarray of float64, shape (N, *shape)
        A new C-contiguous copy with the leading dimensions flattened.
    lead : tuple of int
        The leading dimensions, for :func:`unbatch` (``()`` for a single item).

    Raises
    ------
    InvalidInputError
        If `value` fails :func:`as_real`, or its trailing dimensions are not `shape`.
    """
    arr = as_real(value, name)
    if arr.ndim < len(shape) or arr.shape[arr.ndim - len(shape) :] != shape:
        expected = ", ".join(map(str, shape))
        raise InvalidInputError(f"{name}: expected shape (..., {expected}), got {arr.shape}")
    lead = arr.shape[: arr.ndim - len(shape)]
    return np.ascontiguousarray(arr.reshape((-1, *shape))), lead


def unbatch(flat: NDArray[Any], lead: tuple[int, ...]) -> NDArray[np.float64]:
    """Restore the leading dimensions that :func:`as_batch` flattened.

    Parameters
    ----------
    flat : ndarray, shape (N, *item_shape)
        A batched result.
    lead : tuple of int
        The leading dimensions returned by :func:`as_batch`.

    Returns
    -------
    ndarray of float64, shape (*lead, *item_shape)
        The result in the caller's shape (one item when `lead` is ``()``).
    """
    out = np.asarray(flat, dtype=np.float64)
    return out.reshape((*lead, *out.shape[1:]))


def group_members(group: NDArray[np.int64], n_groups: int) -> list[NDArray[np.int64]]:
    """Return the positions of each group's members, in one stable sort.

    The masked alternative, ``np.flatnonzero(group == g)`` per group, costs a pass over every
    item for every group: quadratic when there are as many groups as cameras or points.

    Parameters
    ----------
    group : ndarray of int, shape (M,)
        Each item's group, in ``[0, n_groups)``.
    n_groups : int
        The number of groups.

    Returns
    -------
    list of ndarray of int
        For each group ``g``, the positions with ``group == g`` in increasing order (empty for
        a group with no members).

    Raises
    ------
    InvalidInputError
        If a group lies outside ``[0, n_groups)``: it would be silently dropped.
    """
    if group.size and (group.min() < 0 or group.max() >= n_groups):
        bad = group[(group < 0) | (group >= n_groups)][0]
        raise InvalidInputError(f"group: {bad} is outside [0, {n_groups})")
    order = np.argsort(group, kind="stable")
    bounds = np.searchsorted(group[order], np.arange(n_groups + 1))
    return [order[bounds[g] : bounds[g + 1]] for g in range(n_groups)]


BEARING_TOLERANCE = 1e-9
"""Bearing vectors must have unit norm within this."""


def as_bearings(value: ArrayLike, name: str = "bearings") -> NDArray[np.float64]:
    """Validate bearing vectors: finite, shape (..., 3), unit norm.

    Parameters
    ----------
    value : array_like, shape (..., 3)
        The bearings.
    name : str, optional
        The argument's name, used in error messages.

    Returns
    -------
    ndarray of float64, shape (..., 3)
        A new C-contiguous copy.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the input is not finite real (..., 3), or a vector's norm is not 1 within
        :data:`BEARING_TOLERANCE`.
    """
    flat, lead = as_batch(value, (3,), name)
    if np.any(np.abs(np.linalg.norm(flat, axis=-1) - 1.0) > BEARING_TOLERANCE):
        raise InvalidInputError(f"{name}: expected unit vectors within {BEARING_TOLERANCE}")
    return flat.reshape((*lead, 3))


def as_bearing_pairs(
    bearings1: ArrayLike, bearings2: ArrayLike, minimum: int, *, positive_z: bool = True
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Validate (N, 3) unit bearing pairs, N >= `minimum`, by default with positive z.

    Parameters
    ----------
    bearings1, bearings2 : array_like, shape (N, 3)
        The bearings.
    minimum : int
        The fewest pairs accepted.
    positive_z : bool, optional
        Require positive z, as methods working in the normalised image plane do (default).

    Returns
    -------
    tuple of ndarray of float64, shape (N, 3)
        C-contiguous copies.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the shapes differ or are not (N, 3), a vector is not unit, a z is not positive, or
        N < `minimum`.
    """
    b1, b2 = as_bearings(bearings1, "bearings1"), as_bearings(bearings2, "bearings2")
    if b1.ndim != 2 or b1.shape != b2.shape:
        raise InvalidInputError(
            f"bearings: expected equal (N, 3) shapes, got {b1.shape}, {b2.shape}"
        )
    if b1.shape[0] < minimum:
        raise InvalidInputError(f"bearings: need at least {minimum} pairs, got {b1.shape[0]}")
    if positive_z and (np.any(b1[:, 2] <= 0.0) or np.any(b2[:, 2] <= 0.0)):
        raise InvalidInputError("bearings: expected positive z (a pinhole view)")
    return b1, b2


def as_correspondences(
    points: ArrayLike, bearings: ArrayLike, minimum: int, *, positive_z: bool = True
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Validate (N, 3) world points and their (N, 3) unit bearings, N >= `minimum`.

    Parameters
    ----------
    points : array_like, shape (N, 3)
        The points, in the world frame.
    bearings : array_like, shape (N, 3)
        Their unit bearings in the camera.
    minimum : int
        The fewest correspondences accepted.
    positive_z : bool, optional
        Require positive z, as methods working in the normalised image plane do (default).

    Returns
    -------
    tuple of ndarray of float64, shape (N, 3)
        C-contiguous copies of the points and the bearings.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the shapes differ or are not (N, 3), a value is not finite, a bearing is not unit,
        a z is not positive, or N < `minimum`.
    """
    world, lead = as_batch(points, (3,), "points")
    rays = as_bearings(bearings)
    if len(lead) != 1 or rays.shape != world.shape:
        raise InvalidInputError(
            f"points, bearings: expected equal (N, 3) shapes, got {world.shape}, {rays.shape}"
        )
    if world.shape[0] < minimum:
        raise InvalidInputError(f"points: need at least {minimum}, got {world.shape[0]}")
    if positive_z and np.any(rays[:, 2] <= 0.0):
        raise InvalidInputError("bearings: expected positive z (a pinhole view)")
    return world, rays


def as_point_pairs(
    points1: ArrayLike, points2: ArrayLike, minimum: int, name: str = "pixels"
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Validate (N, 2) point pairs, N >= `minimum`.

    Parameters
    ----------
    points1, points2 : array_like, shape (N, 2)
        The points.
    minimum : int
        The fewest pairs accepted.
    name : str, optional
        The arguments' name, used in error messages.

    Returns
    -------
    tuple of ndarray of float64, shape (N, 2)
        C-contiguous copies.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the shapes differ or are not (N, 2), or N < `minimum`.
    """
    p1, lead1 = as_batch(points1, (2,), f"{name}1")
    p2, lead2 = as_batch(points2, (2,), f"{name}2")
    if len(lead1) != 1 or lead1 != lead2:
        raise InvalidInputError(f"{name}: expected equal (N, 2) shapes, got {lead1}, {lead2}")
    if p1.shape[0] < minimum:
        raise InvalidInputError(f"{name}: need at least {minimum} pairs, got {p1.shape[0]}")
    return p1, p2


def as_matrix3(value: ArrayLike, name: str) -> NDArray[np.float64]:
    """Validate one finite (3, 3) matrix.

    Parameters
    ----------
    value : array_like, shape (3, 3)
        The matrix.
    name : str
        The argument's name, used in error messages.

    Returns
    -------
    ndarray of float64, shape (3, 3)
        A C-contiguous copy.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If it is not one finite (3, 3) matrix.
    """
    flat, lead = as_batch(value, (3, 3), name)
    if lead != ():
        raise InvalidInputError(f"{name}: expected one (3, 3) matrix, got {lead}")
    return np.ascontiguousarray(flat[0])


def as_image(value: ArrayLike, name: str = "image", *, channels: int = 1) -> NDArray[np.uint8]:
    """Check an image: a non-empty uint8 array, each side at most :data:`MAX_IMAGE_SIDE`.

    Parameters
    ----------
    value : array_like of uint8
        The image: (H, W) when `channels` is 1, (H, W, channels) otherwise. Other dtypes
        are rejected, not converted: an image's values are its exact intensities.
    name : str, optional
        The argument's name, used in error messages.
    channels : int, optional
        The number of channels (1 for gray, 3 for RGB).

    Returns
    -------
    ndarray of uint8
        A new C-contiguous copy.

    Raises
    ------
    InvalidInputError
        If `value` is a masked array, is not uint8, has another shape, is empty, or has a
        side above :data:`MAX_IMAGE_SIDE`.
    """
    if isinstance(value, np.ma.MaskedArray):
        raise InvalidInputError(f"{name}: masked arrays are not accepted")
    arr = np.asarray(value)
    shape = "(H, W)" if channels == 1 else f"(H, W, {channels})"
    expected_ndim = 2 if channels == 1 else 3
    if (
        arr.dtype != np.uint8
        or arr.ndim != expected_ndim
        or (channels != 1 and arr.shape[2] != channels)
        or 0 in arr.shape
    ):
        raise InvalidInputError(
            f"{name}: expected a non-empty {shape} uint8 array, got {arr.dtype} {arr.shape}"
        )
    if max(arr.shape[:2]) > MAX_IMAGE_SIDE:
        raise InvalidInputError(
            f"{name}: sides must be at most {MAX_IMAGE_SIDE} pixels, got {arr.shape[:2]}"
        )
    return np.ascontiguousarray(arr).copy()


def holds_integers(values: NDArray[Any]) -> bool:
    """Whether an array's dtype is an integer type that fits int64 (booleans are not integers).

    Parameters
    ----------
    values : ndarray
        The array.

    Returns
    -------
    bool
        True for a signed or unsigned integer dtype that casts safely to int64.
    """
    return values.dtype.kind in "iu" and np.can_cast(values.dtype, np.int64)


def as_count(value: object, name: str, low: int, high: int) -> int:
    """Check an integer setting in ``[low, high]``.

    Parameters
    ----------
    value : object
        The setting: a Python or numpy integer (a bool is refused, not read as 0 or 1).
    name : str
        Its name, for messages.
    low, high : int
        The inclusive range.

    Returns
    -------
    int
        The value, as a Python int.

    Raises
    ------
    InvalidInputError
        If `value` is not an integer or lies outside the range.
    """
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise InvalidInputError(f"{name}: must be an integer, got {type(value).__name__}")
    if not low <= int(value) <= high:
        raise InvalidInputError(f"{name}: must be from {low} to {high}, got {value}")
    return int(value)


def as_flag(value: object, name: str) -> bool:
    """Check a boolean setting.

    Parameters
    ----------
    value : object
        The setting: a Python or numpy bool (an integer is refused, not read as a truth value).
    name : str
        Its name, for messages.

    Returns
    -------
    bool
        The value, as a Python bool.

    Raises
    ------
    InvalidInputError
        If `value` is not a boolean.
    """
    if not isinstance(value, (bool, np.bool_)):
        raise InvalidInputError(f"{name}: must be a boolean, got {type(value).__name__}")
    return bool(value)


def as_number(value: object, name: str) -> float:
    """Check a real-number setting.

    Parameters
    ----------
    value : object
        The setting: a Python or numpy integer or float (a bool is refused). Its range,
        finiteness included, is the caller's to check.
    name : str
        Its name, for messages.

    Returns
    -------
    float
        The value, as a Python float.

    Raises
    ------
    InvalidInputError
        If `value` is not a real number.
    """
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise InvalidInputError(f"{name}: must be a real number, got {type(value).__name__}")
    return float(value)
