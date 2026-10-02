"""The three implementations of each algorithm, chosen at runtime.

``python`` is the reference implementation and is always present. ``cpp``
(``slamlab._cpp``) and ``rust`` (``slamlab_rs``) are compiled extensions,
each optional: a backend is available only if its extension was built into
the environment.
"""

from __future__ import annotations

import importlib
from collections.abc import Sequence
from types import ModuleType

import numpy as np
from numpy.typing import NDArray

from slamlab import so3

_COMPILED: dict[str, str] = {"cpp": "slamlab._cpp", "rust": "slamlab_rs"}


class Backend:
    """One language's implementations, returning numpy arrays whatever the language.

    Parameters
    ----------
    name : str
        The backend's name: ``"python"``, ``"cpp"`` or ``"rust"``.
    module : module
        The imported module that holds the implementations.
    """

    def __init__(self, name: str, module: ModuleType) -> None:
        self.name = name
        self._module = module

    def hat(self, v: Sequence[float] | NDArray[np.floating]) -> NDArray[np.float64]:
        """Map a 3-vector to its skew-symmetric matrix with this backend.

        Same contract as the reference, :func:`slamlab.so3.hat`; input is checked here so
        every backend rejects a bad shape the same way.
        """
        vec = so3.as_vec3(v)
        return np.asarray(self._module.hat(vec.tolist()), dtype=np.float64)


def _import(name: str) -> ModuleType | None:
    """Import a backend's module.

    Parameters
    ----------
    name : str
        The backend's name.

    Returns
    -------
    module or None
        The module, or None if it is a compiled extension that was not built.
    """
    if name == "python":
        return importlib.import_module("slamlab.so3")
    try:
        return importlib.import_module(_COMPILED[name])
    except ImportError:
        return None


def available() -> list[str]:
    """List the backends built into this environment.

    Returns
    -------
    list of str
        Backend names, the ``"python"`` reference first.
    """
    return [name for name in ("python", *_COMPILED) if _import(name) is not None]


def get(name: str) -> Backend:
    """Return the named backend.

    Parameters
    ----------
    name : {"python", "cpp", "rust"}
        The backend to return.

    Returns
    -------
    Backend
        The backend's implementations.

    Raises
    ------
    ValueError
        If `name` is not a known backend.
    ImportError
        If the backend is a compiled extension that was not built.
    """
    if name != "python" and name not in _COMPILED:
        raise ValueError(f"unknown backend {name!r}; expected python, cpp or rust")
    module = _import(name)
    if module is None:
        raise ImportError(
            f"backend {name!r} is not built in this environment ({_COMPILED[name]} "
            "did not import); run scripts/setup_slamlab.sh"
        )
    return Backend(name, module)
