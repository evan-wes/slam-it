"""Shared test helpers: backend selection that cannot silently lose coverage, and fixtures."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from numpy.typing import ArrayLike

from slamlab import backends

FIXTURES = Path(__file__).parent / "fixtures"

REQUIRED_BACKENDS: frozenset[str] = frozenset(
    name for name in os.environ.get("SLAMLAB_REQUIRE_BACKENDS", "").split(",") if name
)
"""Backends that must be present, from ``SLAMLAB_REQUIRE_BACKENDS`` (comma-separated).

``scripts/check.sh`` (and so CI) sets it to every backend. Then a backend that is not
built, or fails to load, fails the run instead of skipping its parity tests.
"""


def require_backend(name: str) -> str:
    """Return `name` if the backend is usable; otherwise skip, or fail if it is required.

    Every test that needs a particular backend goes through this function, so no test
    can skip a required backend.

    Parameters
    ----------
    name : str
        The backend the test needs.

    Returns
    -------
    str
        `name`, for use as the test's ``backend=`` argument.
    """
    if name not in backends.available():
        message = f"{name} backend not built"
        if name in REQUIRED_BACKENDS:
            pytest.fail(f"{message}, but SLAMLAB_REQUIRE_BACKENDS requires it")
        pytest.skip(message)
    return name


def load_fixture(relative: str) -> dict[str, Any]:
    """Load a JSON parity fixture.

    Parameters
    ----------
    relative : str
        Path under ``tests/fixtures/``, e.g. ``"foundations/so3_hat.json"``.

    Returns
    -------
    dict
        The fixture: ``description``, ``tolerance`` (the policy, see
        :func:`assert_matches_fixture`) and ``cases``.
    """
    fixture: dict[str, Any] = json.loads((FIXTURES / relative).read_text())
    return fixture


def assert_matches_fixture(got: ArrayLike, expected: ArrayLike, tolerance: dict[str, Any]) -> None:
    """Compare a result with a fixture's expected value under the fixture's policy.

    Parameters
    ----------
    got : array_like
        The value a backend produced.
    expected : array_like
        The fixture's expected value.
    tolerance : dict
        The fixture's tolerance policy: ``comparison`` is ``"exact"`` (values and the
        sign of every zero must match; ``rtol`` and ``atol`` must be 0) or ``"close"``
        (``rtol`` and ``atol`` are applied as given, never numpy's defaults).

    Raises
    ------
    AssertionError
        If the values differ under the policy, or the policy is malformed.
    """
    got_arr = np.asarray(got, dtype=np.float64)
    expected_arr = np.asarray(expected, dtype=np.float64)
    comparison = tolerance["comparison"]
    rtol, atol = float(tolerance["rtol"]), float(tolerance["atol"])
    if comparison == "exact":
        assert rtol == 0.0, f"an exact policy cannot set rtol: {tolerance}"
        assert atol == 0.0, f"an exact policy cannot set atol: {tolerance}"
        np.testing.assert_array_equal(got_arr, expected_arr)
        np.testing.assert_array_equal(
            np.signbit(got_arr), np.signbit(expected_arr), err_msg="signs of zeros differ"
        )
    elif comparison == "close":
        np.testing.assert_allclose(got_arr, expected_arr, rtol=rtol, atol=atol)
    else:
        raise AssertionError(f"unknown comparison {comparison!r} in tolerance policy")
