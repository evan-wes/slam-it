"""The three implementations of each algorithm, chosen at runtime.

``python`` is the reference and always present. ``cpp`` (slamlab._cpp) and
``rust`` (slamlab_rs) are compiled extensions, each optional: a backend is
available only if its extension was built into the environment.
"""

from __future__ import annotations

import importlib
from types import ModuleType

import numpy as np

_COMPILED = {"cpp": "slamlab._cpp", "rust": "slamlab_rs"}


class Backend:
    """One implementation's functions, returning numpy arrays whatever the language."""

    def __init__(self, name: str, module: ModuleType):
        self.name = name
        self._module = module

    def hat(self, v) -> np.ndarray:
        return np.asarray(self._module.hat([float(x) for x in v]), dtype=np.float64)


def _import(name: str) -> ModuleType | None:
    if name == "python":
        return importlib.import_module("slamlab.so3")
    try:
        return importlib.import_module(_COMPILED[name])
    except ImportError:
        return None


def available() -> list[str]:
    """Names of the backends built into this environment, reference first."""
    return [name for name in ("python", *_COMPILED) if _import(name) is not None]


def get(name: str) -> Backend:
    """The named backend; raises if it is unknown or was not built."""
    if name != "python" and name not in _COMPILED:
        raise ValueError(f"unknown backend {name!r}; expected python, cpp or rust")
    module = _import(name)
    if module is None:
        raise ImportError(
            f"backend {name!r} is not built in this environment ({_COMPILED[name]} "
            "did not import); run scripts/setup_slamlab.sh"
        )
    return Backend(name, module)
