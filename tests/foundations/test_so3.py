"""SO(3) tests: parity of every backend with the fixture, properties and input validation."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any, cast

import numpy as np
import pytest

from slamlab import backends
from slamlab.foundations import so3
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import assert_matches_fixture, load_fixture, require_backend

HAT = load_fixture("foundations/so3_hat.json")
CROSS_TOLERANCE_EPS = 4.0
"""Tolerance, in units of eps * max|v| * max|u|, between ``hat(v) @ u`` and ``np.cross``.

The BLAS matmul may fuse a multiply-add (FMA), so it rounds differently from np.cross.
To first order each is within 4u * max|v| * max|u| of the exact value (u = eps / 2: two
rounded products and one rounded sum), so they differ by at most 4 eps * max|v| * max|u|.
"""


def test_fixture_declares_exact_comparison() -> None:
    """The fixture compares exactly, since hat only copies and negates."""
    assert HAT["tolerance"]["comparison"] == "exact"
    assert HAT["tolerance"]["rtol"] == 0.0
    assert HAT["tolerance"]["atol"] == 0.0


@pytest.mark.parametrize("name", backends.BACKENDS)
@pytest.mark.parametrize("case", HAT["cases"], ids=lambda c: str(c["v"]))
def test_hat_matches_fixture(name: str, case: dict[str, Any]) -> None:
    """Each backend's hat matches the fixture under its policy (a float32 backend fails)."""
    got = so3.hat(case["v"], backend=require_backend(name))
    assert got.dtype == np.float64
    assert got.shape == (3, 3)
    assert_matches_fixture(got, case["expected"], HAT["tolerance"])


@pytest.mark.parametrize("name", backends.BACKENDS)
def test_hat_of_basis_vectors(name: str) -> None:
    """The basis vectors pin every entry of the matrix hat returns."""
    backend = require_backend(name)
    for axis, expected in enumerate(
        [
            [[0, 0, 0], [0, 0, -1], [0, 1, 0]],
            [[0, 0, 1], [0, 0, 0], [-1, 0, 0]],
            [[0, -1, 0], [1, 0, 0], [0, 0, 0]],
        ]
    ):
        np.testing.assert_array_equal(so3.hat(np.eye(3)[axis], backend=backend), expected)


@pytest.mark.parametrize("name", backends.BACKENDS)
def test_hat_is_skew_symmetric_and_the_cross_product(name: str) -> None:
    """hat(v) is skew-symmetric and hat(v) @ u equals cross(v, u)."""
    backend = require_backend(name)
    rng = np.random.default_rng(0)
    for _ in range(100):
        v, u = rng.normal(size=3), rng.normal(size=3)
        h = so3.hat(v, backend=backend)
        np.testing.assert_array_equal(h.T, -h)
        scale = np.abs(v).max() * np.abs(u).max()
        atol = CROSS_TOLERANCE_EPS * np.finfo(np.float64).eps * scale
        np.testing.assert_allclose(h @ u, np.cross(v, u), rtol=0.0, atol=atol)


BAD_INPUTS: list[tuple[object, str]] = [
    ([1.0, 2.0], "3-vector"),
    ([1.0, 2.0, 3.0, 4.0], "3-vector"),
    ([[1.0, 2.0, 3.0]], "3-vector"),
    ([None, 1.0, 2.0], "real numbers"),
    (["1", "2", "3"], "real numbers"),
    ([True, False, True], "real numbers"),
    (np.array([1 + 2j, 0, 0]), "real numbers"),
    ([np.nan, 0.0, 0.0], "finite"),
    ([0.0, np.inf, 0.0], "finite"),
    ([0.0, 0.0, -np.inf], "finite"),
    (np.ma.array([1.0, 2.0, 3.0], mask=[False, True, False]), "masked"),
]


@pytest.mark.parametrize("name", backends.BACKENDS)
@pytest.mark.parametrize(("bad", "message"), BAD_INPUTS, ids=lambda x: repr(x)[:24])
def test_hat_rejects_invalid_input(name: str, bad: object, message: str) -> None:
    """Every backend rejects bad input the same way, with InvalidInputError."""
    with pytest.raises(InvalidInputError, match=message):
        # Deliberately the wrong type: hat must reject it at runtime.
        so3.hat(cast("Sequence[float]", bad), backend=require_backend(name))


def test_hat_accepts_integers() -> None:
    """Integer input is promoted to float64."""
    np.testing.assert_array_equal(so3.hat([1, 2, 3]), so3.hat([1.0, 2.0, 3.0]))


def test_hat_rejects_an_unknown_backend() -> None:
    """An unknown backend name raises ValueError before any work is done."""
    with pytest.raises(ValueError, match="unknown backend"):
        so3.hat([1.0, 2.0, 3.0], backend="fortran")


def test_hat_reports_an_unbuilt_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    """Asking hat for a backend whose extension is absent raises BackendNotBuiltError."""

    def absent(backend: str, module_name: str) -> object:
        raise backends.BackendNotBuiltError(f"backend {backend!r} is not built")

    monkeypatch.setattr(backends, "implementation", absent)
    with pytest.raises(backends.BackendNotBuiltError, match="not built"):
        so3.hat([1.0, 2.0, 3.0], backend="cpp")
