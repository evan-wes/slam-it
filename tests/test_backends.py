"""Tests for the backend locator: names, availability and failure reporting."""

from __future__ import annotations

import importlib
from types import ModuleType

import pytest

from slamlab import backends
from tests.conftest import REQUIRED_BACKENDS


def test_python_reference_always_available() -> None:
    """The Python reference is always present and listed first."""
    assert backends.available()[0] == "python"


def test_required_backends_are_known() -> None:
    """SLAMLAB_REQUIRE_BACKENDS names only real backends (a typo must not pass silently)."""
    assert set(REQUIRED_BACKENDS) <= set(backends.BACKENDS)


def test_required_backends_are_available() -> None:
    """Every backend in SLAMLAB_REQUIRE_BACKENDS is built and loads (the CI guarantee)."""
    missing = REQUIRED_BACKENDS - set(backends.available())
    assert not missing, f"required backends not available: {sorted(missing)}"


def test_unknown_backend_is_an_error() -> None:
    """A backend name that does not exist raises ValueError."""
    with pytest.raises(ValueError, match="unknown backend"):
        backends.check_name("fortran")


def test_python_has_no_compiled_implementation() -> None:
    """Asking the locator for the python backend is an error: it lives in each module."""
    with pytest.raises(ValueError, match="python reference"):
        backends.implementation("python", "slamlab.foundations.so3")


def test_module_outside_slamlab_is_an_error() -> None:
    """The locator only resolves slamlab modules."""
    with pytest.raises(ValueError, match="not a slamlab module"):
        backends.implementation("cpp", "numpy.linalg")


@pytest.mark.parametrize("name", ["cpp", "rust"])
def test_missing_implementation_is_reported(name: str) -> None:
    """A module the backend does not implement raises NotImplementedError."""
    if name not in backends.available():
        pytest.skip(f"{name} backend not built")
    with pytest.raises(NotImplementedError, match="does not implement"):
        backends.implementation(name, "slamlab.foundations.no_such_module")


def test_absent_extension_counts_as_not_built(monkeypatch: pytest.MonkeyPatch) -> None:
    """Only a ModuleNotFoundError for the extension itself means "not built"."""

    def absent(module_name: str) -> ModuleType:
        raise ModuleNotFoundError(f"No module named {module_name!r}", name=module_name)

    monkeypatch.setattr(importlib, "import_module", absent)
    assert backends.available() == ["python"]
    with pytest.raises(backends.BackendNotBuiltError, match="not built"):
        backends.implementation("cpp", "slamlab.foundations.so3")


@pytest.mark.parametrize(
    "failure",
    [
        ImportError("undefined symbol: _ZN7slamlab3hatE"),
        ModuleNotFoundError("No module named 'numpy.core'", name="numpy.core"),
    ],
    ids=["broken-extension", "missing-dependency"],
)
def test_broken_extension_is_not_hidden(
    monkeypatch: pytest.MonkeyPatch, failure: ImportError
) -> None:
    """An extension that is installed but fails to load raises its real error."""

    def broken(module_name: str) -> ModuleType:
        raise failure

    monkeypatch.setattr(importlib, "import_module", broken)
    with pytest.raises(ImportError) as excinfo:
        backends.available()
    assert excinfo.value is failure
