"""Helpers shared by the edge rung's tests: comparing maps across backends."""

from __future__ import annotations

from dataclasses import fields

import numpy as np

from slamlab.edge.map import Map

TOLERANCE = {"rtol": 1e-12, "atol": 1e-12}
"""Floats agree to this across backends (norms and sums may round differently); integers,
ids and the map's structure exactly. NaN equals NaN."""


def assert_maps_match(got: Map, expected: Map) -> None:
    """Check two maps field by field: integers exactly, floats to :data:`TOLERANCE`.

    Parameters
    ----------
    got, expected : Map
        The maps.
    """
    for item in fields(Map):
        a, b = getattr(got, item.name), getattr(expected, item.name)
        if isinstance(b, np.ndarray) and b.dtype == np.float64:
            np.testing.assert_allclose(
                np.asarray(a, dtype=np.float64),
                b,
                rtol=TOLERANCE["rtol"],
                atol=TOLERANCE["atol"],
                err_msg=item.name,
            )
        elif isinstance(b, np.ndarray):
            np.testing.assert_array_equal(a, b, err_msg=item.name)
        else:
            assert a == b, item.name
