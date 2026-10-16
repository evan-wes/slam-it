"""The edge rung's test constants, and helpers comparing maps and summaries across backends.

Shared by the tests and by the tools that run the room outside pytest (the golden record's
generator): the room's frame count, settings and camera.
"""

from __future__ import annotations

from dataclasses import fields
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it.edge import frames
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.eval import vocabularies
from slam_it.foundations import vocabulary
from tests.edge import room

FRAMES = 40
"""Frames rendered for the session's states."""

OPTIONS = Options(
    n_features=600, init_window=50.0, init_min_matches=50, min_init_features_depth=200
)
"""ORB-SLAM's defaults scaled to the room's 320x240 images (its are for 640x480)."""

ATE_BOUND = {"rgbd": 0.02, "stereo": 0.02, "monocular": 0.03}
"""The room's ATE (RMSE, metres) each mode stays under: about half again its measured value
(1.2, 1.4 and 2.0 cm; monocular aligned with a similarity)."""

MONOCULAR_START = 10
"""The monocular map starts within this many frames."""

CAMERA = frames.Camera(
    "pinhole", room.SMALL.intrinsics, room.SMALL.width, room.SMALL.height, room.BASELINE
)
"""The room's camera, with the stereo baseline (RGB-D's virtual one here)."""

TOLERANCE = {"rtol": 1e-12, "atol": 1e-12}
"""Floats agree to this across backends (norms and sums may round differently); integers,
ids and the map's structure exactly. NaN equals NaN."""

SOLVER_TOLERANCE = {"rtol": 1e-9, "atol": 1e-9}
"""The same for a stage that runs a bundle adjustment, whose iterations carry last-bit
differences forward (``slam_it.foundations.ba``'s own parity is held to this too); the
structure is still exact."""


def words_for(options: Options) -> vocabulary.Vocabulary | None:
    """Return the vocabulary a pipeline with these settings needs: the committed one, or none.

    Parameters
    ----------
    options : slam_it.edge.options.Options
        The settings.

    Returns
    -------
    slam_it.foundations.vocabulary.Vocabulary or None
        The committed vocabulary with ``relocalize`` on, None with it off.
    """
    return vocabularies.default() if options.relocalize else None


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


def assert_records_match(
    got: Any,  # noqa: ANN401 -- a summary's JSON values, at any depth
    expected: Any,  # noqa: ANN401 -- a summary's JSON values, at any depth
    tolerance: dict[str, float] | None = None,
) -> None:
    """Check two stage summaries' JSON values: floats to a tolerance, everything else exactly.

    The floats are bundle adjustments' costs, which carry the solves' last-bit differences; the
    counts, names and flags are decisions, and must match exactly, type included.

    Parameters
    ----------
    got, expected : JSON values
        Summaries' ``as_dict()``, or values inside them.
    tolerance : dict, optional
        ``rtol`` and ``atol`` (default: :data:`SOLVER_TOLERANCE`).
    """
    chosen = SOLVER_TOLERANCE if tolerance is None else tolerance
    if isinstance(expected, dict):
        assert isinstance(got, dict)
        assert list(got) == list(expected)
        for key, value in expected.items():
            assert_records_match(got[key], value, chosen)
    elif isinstance(expected, list):
        assert isinstance(got, list)
        assert len(got) == len(expected)
        for a, b in zip(got, expected, strict=True):
            assert_records_match(a, b, chosen)
    elif isinstance(expected, float):
        assert isinstance(got, float)
        np.testing.assert_allclose(got, expected, rtol=chosen["rtol"], atol=chosen["atol"])
    else:
        assert type(got) is type(expected)
        assert got == expected


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
