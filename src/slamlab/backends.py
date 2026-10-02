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
from collections.abc import Callable
from types import ModuleType
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray

BACKENDS: tuple[str, ...] = ("python", "cpp", "rust")
"""Every backend name, the Python reference first."""

LOCKSTEP_BACKENDS: frozenset[str] = frozenset({"python", "cpp"})
"""Backends that implement every function of every compiled module.

C++ is built in lock-step with the Python reference. Rust implements a curated subset
(design doc, "Languages") and never gates a merge, so a Rust module may lack functions.
"""

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


def function(backend: str, module_name: str, name: str) -> Callable[..., Any]:
    """Return one function of a compiled backend's implementation of a slamlab module.

    Parameters
    ----------
    backend : {"cpp", "rust"}
        The compiled backend.
    module_name : str
        The Python module whose implementation is wanted, as its ``__name__``.
    name : str
        The function's name, as in the module's ``Implementation`` Protocol.

    Returns
    -------
    callable
        The compiled function.

    Raises
    ------
    ValueError
        If `backend` is unknown or ``"python"``, or `module_name` is not a slamlab module.
    BackendNotBuiltError
        If the backend's extension is not installed.
    NotImplementedError
        If the backend does not implement `module_name`, or implements it without `name`
        (Rust implements a curated subset).
    """
    found = getattr(implementation(backend, module_name), name, None)
    if found is None or isinstance(found, ModuleType) or not callable(found):
        raise NotImplementedError(f"backend {backend!r} does not implement {module_name}.{name}")
    return cast("Callable[..., Any]", found)


def dispatch(
    backend: str,
    module_name: str,
    name: str,
    reference: Callable[..., NDArray[np.float64]],
    *args: NDArray[Any],
) -> NDArray[np.float64]:
    """Run one function with the chosen backend: the Python reference or a compiled one.

    Algorithm modules call this after validating their inputs, so every backend receives the
    same clean, batched arrays.

    Parameters
    ----------
    backend : {"python", "cpp", "rust"}
        Which implementation computes the result.
    module_name : str
        The calling module's ``__name__``.
    name : str
        The function's name, as in the module's ``Implementation`` Protocol.
    reference : callable
        The Python reference implementation, called when `backend` is ``"python"``.
    *args : ndarray
        The validated arguments, passed unchanged.

    Returns
    -------
    ndarray of float64
        The result.

    Raises
    ------
    ValueError
        If `backend` is unknown.
    BackendNotBuiltError
        If a compiled `backend` is not built.
    NotImplementedError
        If a compiled `backend` does not implement the function.
    """
    check_name(backend)
    if backend == "python":
        return reference(*args)
    return np.asarray(function(backend, module_name, name)(*args), dtype=np.float64)
