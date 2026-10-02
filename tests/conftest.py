"""Shared test helpers: backend selection that cannot silently lose coverage."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import pytest

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
        The fixture: ``description``, ``tolerance`` and ``cases``.
    """
    fixture: dict[str, Any] = json.loads((FIXTURES / relative).read_text())
    return fixture
