"""Input validation shared by every rung: convert and check array arguments.

Public functions validate their inputs here, once, before any backend runs, so all
three language implementations see the same clean float64 data and every bad input
fails the same way, with :class:`slamlab.foundations.errors.InvalidInputError`.

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


def as_real(value: ArrayLike, name: str = "input") -> NDArray[np.float64]:
    """Convert input to a finite, C-contiguous float64 array (a new copy).

    Parameters
    ----------
    value : array_like
        The input. Integer and floating-point values are accepted; integers beyond
        2**53 lose precision in the float64 conversion.
    name : str, optional
        The argument's name, used in error messages.

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
    if not np.all(np.isfinite(out)):
        raise InvalidInputError(f"{name}: expected finite values")
    return out


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
