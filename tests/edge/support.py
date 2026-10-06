"""Helpers shared by the edge rung's tests: comparing maps across backends."""

from __future__ import annotations

from dataclasses import fields

import numpy as np
from numpy.typing import NDArray

from slamlab.edge.map import Map

TOLERANCE = {"rtol": 1e-12, "atol": 1e-12}
"""Floats agree to this across backends (norms and sums may round differently); integers,
ids and the map's structure exactly. NaN equals NaN."""

SOLVER_TOLERANCE = {"rtol": 1e-9, "atol": 1e-9}
"""The same for a stage that runs a bundle adjustment, whose iterations carry last-bit
differences forward (``slamlab.foundations.ba``'s own parity is held to this too); the
structure is still exact."""


def assert_close(
    got: NDArray[np.float64],
    expected: NDArray[np.float64],
    tolerance: dict[str, float] | None = None,
) -> None:
    """Check two float arrays to a tolerance, NaN equal to NaN.

    Parameters
    ----------
    got, expected : ndarray of float64
        The arrays.
    tolerance : dict, optional
        ``rtol`` and ``atol`` (default: :data:`TOLERANCE`).
    """
    chosen = TOLERANCE if tolerance is None else tolerance
    np.testing.assert_allclose(got, expected, rtol=chosen["rtol"], atol=chosen["atol"])


def assert_maps_match(got: Map, expected: Map, tolerance: dict[str, float] | None = None) -> None:
    """Check two maps field by field: integers exactly, floats to a tolerance.

    Parameters
    ----------
    got, expected : Map
        The maps.
    tolerance : dict, optional
        ``rtol`` and ``atol`` (default: :data:`TOLERANCE`).
    """
    chosen = TOLERANCE if tolerance is None else tolerance
    for item in fields(Map):
        a, b = getattr(got, item.name), getattr(expected, item.name)
        if isinstance(b, np.ndarray) and b.dtype == np.float64:
            np.testing.assert_allclose(
                np.asarray(a, dtype=np.float64),
                b,
                rtol=chosen["rtol"],
                atol=chosen["atol"],
                err_msg=item.name,
            )
        elif isinstance(b, np.ndarray):
            np.testing.assert_array_equal(a, b, err_msg=item.name)
        else:
            assert a == b, item.name
