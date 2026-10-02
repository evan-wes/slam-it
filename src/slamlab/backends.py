"""Locate an algorithm's compiled implementation in the C++ or Rust backend.

slamlab implements each algorithm in up to three languages. The Python reference
lives in the algorithm's own module (e.g. :mod:`slamlab.foundations.so3`), which
also validates input and dispatches. The compiled implementations live in two
optional extensions whose submodules mirror the Python package tree:

- ``cpp``: ``slamlab._cpp`` (nanobind), e.g. ``slamlab._cpp.foundations.so3``;
- ``rust``: ``slamlab_rs`` (PyO3), e.g. ``slamlab_rs.foundations.so3``.

This module only finds implementations. It knows nothing about what they compute,
and it sits below every algorithm package in the dependency layers.
"""

from __future__ import annotations

import importlib
from types import ModuleType

BACKENDS: tuple[str, ...] = ("python", "cpp", "rust")
"""Every backend name, the Python reference first."""

_EXTENSIONS: dict[str, str] = {"cpp": "slamlab._cpp", "rust": "slamlab_rs"}
_PACKAGE_PREFIX = "slamlab."


class BackendNotBuiltError(ImportError):
    """A compiled backend was requested but its extension is not installed."""


def check_name(name: str) -> None:
    """Check that `name` is a known backend.

    Parameters
    ----------
    name : str
        The backend name to check.

    Raises
    ------
    ValueError
        If `name` is not one of :data:`BACKENDS`.
    """
    if name not in BACKENDS:
        raise ValueError(f"unknown backend {name!r}; expected one of {', '.join(BACKENDS)}")


def _extension(name: str) -> ModuleType | None:
    """Import a compiled backend's extension module.

    Parameters
    ----------
    name : {"cpp", "rust"}
        The compiled backend.

    Returns
    -------
    module or None
        The extension, or None when it is not installed.

    Raises
    ------
    ImportError
        If the extension is installed but fails to load (a missing shared library,
        an ABI mismatch, an initialisation error). Only a genuinely absent extension
        counts as "not built"; any other failure is surfaced with its real cause.
    """
    module_name = _EXTENSIONS[name]
    try:
        return importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        if exc.name == module_name:
            return None
        raise


def available() -> list[str]:
    """List the backends usable in this environment.

    Returns
    -------
    list of str
        Backend names in :data:`BACKENDS` order. ``"python"`` is always present.

    Raises
    ------
    ImportError
        If a compiled extension is installed but fails to load.
    """
    return ["python", *(name for name in _EXTENSIONS if _extension(name) is not None)]


def implementation(backend: str, module_name: str) -> ModuleType:
    """Return a compiled backend's implementation of a slamlab module.

    Parameters
    ----------
    backend : {"cpp", "rust"}
        The compiled backend.
    module_name : str
        The Python module whose implementation is wanted, as its ``__name__``
        (e.g. ``"slamlab.foundations.so3"``).

    Returns
    -------
    module
        The extension submodule mirroring `module_name`
        (e.g. ``slamlab._cpp.foundations.so3``).

    Raises
    ------
    ValueError
        If `backend` is unknown, is ``"python"`` (the reference lives in the
        algorithm's own module), or `module_name` is not inside slamlab.
    BackendNotBuiltError
        If the backend's extension is not installed.
    NotImplementedError
        If the backend has no implementation of `module_name`.
    """
    check_name(backend)
    if backend == "python":
        raise ValueError("the python reference lives in the algorithm's own module")
    if not module_name.startswith(_PACKAGE_PREFIX):
        raise ValueError(f"{module_name!r} is not a slamlab module")
    extension = _extension(backend)
    if extension is None:
        raise BackendNotBuiltError(
            f"backend {backend!r} is not built in this environment "
            f"({_EXTENSIONS[backend]} is not installed); run scripts/setup_slamlab.sh"
        )
    found: object = extension
    for part in module_name.removeprefix(_PACKAGE_PREFIX).split("."):
        found = getattr(found, part, None)
        if found is None:
            raise NotImplementedError(f"backend {backend!r} does not implement {module_name}")
    if not isinstance(found, ModuleType):
        raise NotImplementedError(f"backend {backend!r} does not implement {module_name}")
    return found
