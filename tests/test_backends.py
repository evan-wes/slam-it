"""Every built backend must match the shared fixtures, and so each other."""

import json
from pathlib import Path

import numpy as np
import pytest

from slamlab import backends

FIXTURES = Path(__file__).parent / "fixtures"
HAT_CASES = json.loads((FIXTURES / "so3_hat.json").read_text())["cases"]


def test_python_reference_always_available():
    assert backends.available()[0] == "python"


@pytest.mark.parametrize("name", ["python", "cpp", "rust"])
@pytest.mark.parametrize("case", HAT_CASES, ids=lambda c: str(c["v"]))
def test_hat_matches_fixture(name, case):
    if name not in backends.available():
        pytest.skip(f"{name} backend not built")
    got = backends.get(name).hat(case["v"])
    np.testing.assert_allclose(got, case["expected"], atol=1e-12)


@pytest.mark.parametrize("name", ["python", "cpp", "rust"])
def test_hat_is_cross_product(name):
    if name not in backends.available():
        pytest.skip(f"{name} backend not built")
    rng = np.random.default_rng(0)
    v, u = rng.normal(size=3), rng.normal(size=3)
    np.testing.assert_allclose(backends.get(name).hat(v) @ u, np.cross(v, u), atol=1e-12)


def test_unknown_backend_is_an_error():
    with pytest.raises(ValueError):
        backends.get("fortran")
