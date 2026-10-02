"""SO(3), the group of 3D rotations: the Python reference and backend dispatch.

A placeholder that proves the three-language pipeline: only ``hat`` exists so far.
Rung 0's SO(3) work (``vee``, ``exp``, ``log``, Jacobians) arrives with milestone 1.

Every public function validates its input once, here, then either computes with the
Python reference or calls the chosen compiled backend, whose implementation is
checked against the reference by the parity tests.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol, cast

import numpy as np
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations.arrays import as_vec3


class _Implementation(Protocol):
    """The functions a compiled backend's ``foundations.so3`` submodule provides.

    Bindings take and return plain nested lists (no numpy dependency in the cores).
    ``tests/foundations/test_so3.py`` checks each compiled submodule against this.
    """

    def hat(self, v: list[float]) -> list[list[float]]:
        """Map a 3-vector to its skew-symmetric matrix, as row-major nested lists."""
        ...


def hat(
    v: Sequence[float] | NDArray[np.integer | np.floating], *, backend: str = "python"
) -> NDArray[np.float64]:
    """Map a 3-vector to its skew-symmetric matrix.

    The result ``[v]x`` satisfies ``hat(v) @ u == np.cross(v, u)`` for every
    3-vector ``u``. It is the map from R^3 into the Lie algebra so(3).

    Parameters
    ----------
    v : sequence of float or ndarray, shape (3,)
        The vector to map. Must be real and finite.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result. Default ``"python"``, the
        reference. Input is validated here for every backend.

    Returns
    -------
    ndarray of float64, shape (3, 3)
        The skew-symmetric matrix ``[v]x``.

    Raises
    ------
    ValueError
        If `v` is not a finite real 3-vector, or `backend` is unknown.
    slamlab.backends.BackendNotBuiltError
        If a compiled `backend` is not built in this environment.

    Examples
    --------
    >>> hat([1.0, 2.0, 3.0])
    array([[ 0., -3.,  2.],
           [ 3.,  0., -1.],
           [-2.,  1.,  0.]])
    """
    backends.check_name(backend)
    vec = as_vec3(v)
    if backend == "python":
        x, y, z = vec
        return np.array(
            [
                [0.0, -z, y],
                [z, 0.0, -x],
                [-y, x, 0.0],
            ]
        )
    impl = cast(_Implementation, backends.implementation(backend, __name__))
    return np.asarray(impl.hat(vec.tolist()), dtype=np.float64)
