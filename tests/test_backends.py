"""Parity tests: every built backend must match the shared fixtures, and so each other."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from slamlab import backends

FIXTURES = Path(__file__).parent / "fixtures"
HAT_CASES: list[dict[str, Any]] = json.loads((FIXTURES / "so3_hat.json").read_text())["cases"]
BACKENDS = ["python", "cpp", "rust"]


def _require(name: str) -> backends.Backend:
    """Return the named backend, skipping the test if it was not built."""
    if name not in backends.available():
        pytest.skip(f"{name} backend not built")
    return backends.get(name)


def test_python_reference_always_available() -> None:
    """The Python reference is always present and listed first."""
    assert backends.available()[0] == "python"


@pytest.mark.parametrize("name", BACKENDS)
@pytest.mark.parametrize("case", HAT_CASES, ids=lambda c: str(c["v"]))
def test_hat_matches_fixture(name: str, case: dict[str, Any]) -> None:
    """Each backend's hat matches the shared fixture."""
    got = _require(name).hat(case["v"])
    np.testing.assert_allclose(got, case["expected"], atol=1e-12)


@pytest.mark.parametrize("name", BACKENDS)
def test_hat_is_cross_product(name: str) -> None:
    """hat(v) @ u equals cross(v, u) for random vectors."""
    rng = np.random.default_rng(0)
    v, u = rng.normal(size=3), rng.normal(size=3)
    np.testing.assert_allclose(_require(name).hat(v) @ u, np.cross(v, u), atol=1e-12)


@pytest.mark.parametrize("name", BACKENDS)
@pytest.mark.parametrize("bad", [[1.0, 2.0], [1.0, 2.0, 3.0, 4.0], [[1.0, 2.0, 3.0]]])
def test_hat_rejects_wrong_shape(name: str, bad: list[Any]) -> None:
    """Every backend rejects input that is not a 3-vector."""
    with pytest.raises(ValueError, match="3-vector"):
        _require(name).hat(bad)


def test_unknown_backend_is_an_error() -> None:
    """Asking for a backend that does not exist raises ValueError."""
    with pytest.raises(ValueError, match="unknown backend"):
        backends.get("fortran")
