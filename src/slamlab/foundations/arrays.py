"""Input validation shared by every rung: convert and check array arguments.

Public functions validate their inputs here, once, before any backend runs, so all
three language implementations see the same clean float64 data and every bad input
fails the same way.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

_NUMERIC_KINDS = frozenset("iuf")
"""dtype kinds accepted as numbers: signed and unsigned integers, floats.

Booleans, complex numbers, strings and objects (e.g. ``None``) are rejected rather
than coerced, because numpy would silently turn them into numbers.
"""


def as_vec3(v: Sequence[float] | NDArray[np.integer | np.floating]) -> NDArray[np.float64]:
    """Convert input to a finite float64 3-vector.

    Parameters
    ----------
    v : sequence of float or ndarray, shape (3,)
        The input to convert. Integer and floating-point values are accepted.

    Returns
    -------
    ndarray of float64, shape (3,)
        A new array holding the vector.

    Raises
    ------
    ValueError
        If `v` is not numeric (booleans, complex, strings and ``None`` are
        rejected), does not have shape (3,), or holds NaN or infinity.
    """
    arr = np.asarray(v)
    if arr.dtype.kind not in _NUMERIC_KINDS:
        raise ValueError(f"expected real numbers, got dtype {arr.dtype}")
    if arr.shape != (3,):
        raise ValueError(f"expected a 3-vector, got shape {arr.shape}")
    out = arr.astype(np.float64)
    if not np.all(np.isfinite(out)):
        raise ValueError(f"expected finite values, got {out.tolist()}")
    return out
