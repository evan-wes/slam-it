"""Input validation shared by every rung: convert and check array arguments.

Public functions validate their inputs here, once, before any backend runs, so all
three language implementations see the same clean float64 data and every bad input
fails the same way, with :class:`slamlab.foundations.errors.InvalidInputError`.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from slamlab.foundations.errors import InvalidInputError

_NUMERIC_KINDS = frozenset("iuf")
"""dtype kinds accepted as numbers: signed and unsigned integers, floats.

Booleans, complex numbers, strings and objects (e.g. ``None``) are rejected rather
than coerced, because numpy would silently turn them into numbers.
"""


def as_vec3(v: Sequence[float] | NDArray[np.integer | np.floating]) -> NDArray[np.float64]:
    """Convert input to a finite, contiguous float64 3-vector.

    Parameters
    ----------
    v : sequence of float or ndarray, shape (3,)
        The input to convert. Integer and floating-point values are accepted.
        Integers beyond 2**53 lose precision in the float64 conversion.

    Returns
    -------
    ndarray of float64, shape (3,)
        A new C-contiguous array holding the vector, ready to pass to a compiled backend.

    Raises
    ------
    InvalidInputError
        If `v` is a masked array (its masked values would be used silently), is not
        numeric (booleans, complex, strings and ``None`` are rejected), does not have
        shape (3,), or holds NaN or infinity.
    """
    if isinstance(v, np.ma.MaskedArray):
        raise InvalidInputError("masked arrays are not accepted; fill or compress them first")
    arr = np.asarray(v)
    if arr.dtype.kind not in _NUMERIC_KINDS:
        raise InvalidInputError(f"expected real numbers, got dtype {arr.dtype}")
    if arr.shape != (3,):
        raise InvalidInputError(f"expected a 3-vector, got shape {arr.shape}")
    out = np.ascontiguousarray(arr, dtype=np.float64).copy()
    if not np.all(np.isfinite(out)):
        raise InvalidInputError(f"expected finite values, got {out.tolist()}")
    return out
