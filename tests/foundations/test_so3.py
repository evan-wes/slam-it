"""SO(3) tests: parity of every backend with the fixture, properties and input validation."""

from __future__ import annotations

import inspect
import re
from collections.abc import Sequence
from typing import Any, cast

import numpy as np
import pytest

from slamlab import backends
from slamlab.foundations import so3
from tests.conftest import load_fixture, require_backend

HAT = load_fixture("foundations/so3_hat.json")
CROSS_TOLERANCE_ULPS = 4.0
"""hat(v) @ u and np.cross(v, u) sum the same products in a different order; allow a few
units in the last place of the largest term."""


def test_fixture_declares_exact_comparison() -> None:
    """The hat fixture's tolerance policy is exact, matching how the parity test compares."""
    assert HAT["tolerance"] == {**HAT["tolerance"], "comparison": "exact", "rtol": 0.0, "atol": 0.0}


@pytest.mark.parametrize("name", backends.BACKENDS)
@pytest.mark.parametrize("case", HAT["cases"], ids=lambda c: str(c["v"]))
def test_hat_matches_fixture_exactly(name: str, case: dict[str, Any]) -> None:
    """Each backend's hat equals the fixture bit for bit (a float32 backend would fail)."""
    got = so3.hat(case["v"], backend=require_backend(name))
    assert got.dtype == np.float64
    np.testing.assert_array_equal(got, case["expected"])


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
        atol = CROSS_TOLERANCE_ULPS * np.finfo(np.float64).eps * scale
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
]


@pytest.mark.parametrize("name", backends.BACKENDS)
@pytest.mark.parametrize(("bad", "message"), BAD_INPUTS, ids=lambda x: repr(x)[:24])
def test_hat_rejects_invalid_input(name: str, bad: object, message: str) -> None:
    """Every backend rejects non-numeric, non-finite and wrongly shaped input the same way."""
    with pytest.raises(ValueError, match=message):
        # Deliberately the wrong type: hat must reject it at runtime.
        so3.hat(cast("Sequence[float]", bad), backend=require_backend(name))


def test_hat_accepts_integers() -> None:
    """Integer input is promoted to float64."""
    np.testing.assert_array_equal(so3.hat([1, 2, 3]), so3.hat([1.0, 2.0, 3.0]))


@pytest.mark.parametrize("name", ["cpp", "rust"])
def test_compiled_module_matches_the_protocol(name: str) -> None:
    """Each compiled foundations.so3 provides the Protocol's functions with its parameters.

    Types are checked at runtime by the parity tests; this catches a missing function or a
    renamed or added parameter, which mypy cannot see through the dynamic lookup.
    """
    impl = backends.implementation(require_backend(name), so3.__name__)
    protocol_members = [m for m in vars(so3._Implementation) if not m.startswith("_")]
    for member in protocol_members:
        expected = [
            p
            for p in inspect.signature(getattr(so3._Implementation, member)).parameters
            if p != "self"
        ]
        assert hasattr(impl, member), f"{name} lacks {member}"
        assert _parameter_names(getattr(impl, member)) == expected


def _parameter_names(func: object) -> list[str]:
    """Return a compiled function's parameter names from its nanobind or PyO3 signature."""
    nb_signature = getattr(func, "__nb_signature__", None)
    if nb_signature:
        text = nb_signature[0][0]
    else:
        text_signature = getattr(func, "__text_signature__", None)
        assert text_signature is not None, f"{func!r} exposes no signature"
        text = f"def f{text_signature}"
    params = re.search(r"\((.*?)\)", text)
    assert params is not None, text
    return [p.split(":")[0].split("=")[0].strip() for p in params.group(1).split(",") if p.strip()]
