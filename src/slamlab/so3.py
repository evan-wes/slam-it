"""SO(3) helpers: the Python reference implementation.

A placeholder that proves the three-language pipeline. Rung 0's real SO(3)
work arrives with milestone 1.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray


def as_vec3(v: Sequence[float] | NDArray[np.floating]) -> NDArray[np.float64]:
    """Convert input to a float64 3-vector, rejecting any other shape.

    Parameters
    ----------
    v : sequence of float or ndarray, shape (3,)
        The input to convert.

    Returns
    -------
    ndarray of float64, shape (3,)
        The vector.

    Raises
    ------
    ValueError
        If `v` does not have exactly three elements.
    """
    arr = np.asarray(v, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"expected a 3-vector, got shape {arr.shape}")
    return arr


def hat(v: Sequence[float] | NDArray[np.floating]) -> NDArray[np.float64]:
    """Map a 3-vector to its skew-symmetric matrix.

    The result ``[v]x`` satisfies ``hat(v) @ u == np.cross(v, u)`` for every
    3-vector ``u``. It is the map from R^3 into the Lie algebra so(3).

    Parameters
    ----------
    v : sequence of float or ndarray, shape (3,)
        The vector to map.

    Returns
    -------
    ndarray of float64, shape (3, 3)
        The skew-symmetric matrix ``[v]x``.

    Raises
    ------
    ValueError
        If `v` does not have exactly three elements.

    Examples
    --------
    >>> hat([1.0, 2.0, 3.0])
    array([[ 0., -3.,  2.],
           [ 3.,  0., -1.],
           [-2.,  1.,  0.]])
    """
    x, y, z = as_vec3(v)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )
