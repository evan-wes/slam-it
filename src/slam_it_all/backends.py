"""Locate an algorithm's compiled implementation in the C++ or Rust backend.

SLAM It All! implements each algorithm in up to three languages. The Python reference
lives in the algorithm's own module (e.g. :mod:`slam_it_all.foundations.so3`), which
also validates input and dispatches. The compiled implementations live in two
optional extensions whose submodules mirror the Python package tree:

- ``cpp``: ``slam_it_all._cpp`` (nanobind), e.g. ``slam_it_all._cpp.foundations.so3``;
- ``rust``: ``slam_it_all_rs`` (PyO3), e.g. ``slam_it_all_rs.foundations.so3``.

Pipelines that run a whole sequence also come as native programs (:func:`program`): C++'s
installed with the package, Rust's built in the checkout.

This module only finds implementations. It knows nothing about what they compute,
and it sits below every algorithm package in the dependency layers.
"""

from __future__ import annotations

import importlib
import tomllib
from collections.abc import Callable, Mapping
from pathlib import Path
from types import ModuleType
from typing import Any, SupportsInt, TypeVar, cast

import numpy as np
from numpy.typing import NDArray

BACKENDS: tuple[str, ...] = ("python", "cpp", "rust")
"""Every backend name, the Python reference first."""

COMPILED_BACKENDS: tuple[str, ...] = BACKENDS[1:]
"""Every compiled backend: each implements in full every module it exposes."""

COMPILED_RUNGS: dict[str, tuple[str, ...]] = {
    "cpp": ("foundations", "edge"),
    "rust": ("foundations", "edge"),
}
"""The rungs each compiled backend covers (design doc, "Languages"): C++ rung 0 and the edge
rung, Rust rung 0 and the edge VO. Every module with an ``Implementation`` Protocol in one of
them must be in that backend; the other rungs are Python's (or PyTorch's) alone."""

LOCKSTEP_BACKENDS: frozenset[str] = frozenset(BACKENDS)
"""Backends that implement every compiled module, so every module's tests run on each.

All three since PR 6, which completed rung 0 in Rust: a backend missing a module, or a
function of one, fails the tests (design doc, "Languages per module").
"""

_EXTENSIONS: dict[str, str] = {"cpp": "slam_it_all._cpp", "rust": "slam_it_all_rs"}
_PACKAGE_PREFIX = "slam_it_all."
_RUST = Path(__file__).resolve().parents[2] / "rust"
"""The Rust workspace of the checkout this module was imported from."""

_RUST_PROGRAMS = _RUST / "target" / "release"
"""Where cargo builds the Rust programs."""


class BackendNotBuiltError(ImportError):
    """A compiled backend was requested but its extension is not installed."""


class StaleProgramError(BackendNotBuiltError):
    """A Rust program is older than a file it is built from: its runs are not of this code."""


def _program_inputs(workspace: Path, name: str) -> list[Path]:
    """Return the files a Rust program is built from, read from the workspace's manifests.

    The workspace's ``Cargo.toml`` and ``Cargo.lock``, and, for the crate whose ``[[bin]]`` is
    the program and every crate it depends on by path, its manifest, ``build.rs`` and sources.
    Empty if no crate builds it (not a checkout).
    """
    manifests: dict[Path, dict[str, Any]] = {}
    for manifest in sorted(workspace.glob("*/Cargo.toml")):
        with manifest.open("rb") as f:
            manifests[manifest.parent.resolve()] = tomllib.load(f)
    todo = [
        folder
        for folder, parsed in manifests.items()
        if any(entry.get("name") == name for entry in parsed.get("bin", []))
    ]
    seen: set[Path] = set()
    while todo:
        folder = todo.pop()
        if folder in seen or folder not in manifests:
            continue
        seen.add(folder)
        for spec in manifests[folder].get("dependencies", {}).values():
            if isinstance(spec, dict) and "path" in spec:
                todo.append((folder / spec["path"]).resolve())
    if not seen:
        return []
    files = [workspace / "Cargo.toml", workspace / "Cargo.lock"]
    for folder in sorted(seen):
        files += [folder / "Cargo.toml", folder / "build.rs", *sorted(folder.glob("src/**/*.rs"))]
    return [f for f in files if f.is_file()]


def _check_fresh(program: Path, workspace: Path) -> None:
    """Raise :class:`StaleProgramError` if a file the program is built from is newer than it."""
    built = program.stat().st_mtime
    for source in _program_inputs(workspace, program.name):
        if source.stat().st_mtime > built:
            raise StaleProgramError(
                f"{program} is older than {source}: rebuild it (cargo build --release "
                f"--manifest-path {workspace / 'Cargo.toml'} --target-dir {workspace / 'target'})"
            )


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
    """Return a compiled backend's implementation of a slam_it_all module.

    Parameters
    ----------
    backend : {"cpp", "rust"}
        The compiled backend.
    module_name : str
        The Python module whose implementation is wanted, as its ``__name__``
        (e.g. ``"slam_it_all.foundations.so3"``).

    Returns
    -------
    module
        The extension submodule mirroring `module_name`
        (e.g. ``slam_it_all._cpp.foundations.so3``).

    Raises
    ------
    ValueError
        If `backend` is unknown, is ``"python"`` (the reference lives in the
        algorithm's own module), or `module_name` is not inside `slam_it_all`.
    BackendNotBuiltError
        If the backend's extension is not installed.
    NotImplementedError
        If the backend has no implementation of `module_name`.
    """
    check_name(backend)
    if backend == "python":
        raise ValueError("the python reference lives in the algorithm's own module")
    if not module_name.startswith(_PACKAGE_PREFIX):
        raise ValueError(f"{module_name!r} is not a slam_it_all module")
    extension = _extension(backend)
    if extension is None:
        raise BackendNotBuiltError(
            f"backend {backend!r} is not built in this environment "
            f"({_EXTENSIONS[backend]} is not installed); run scripts/setup_slam_it_all.sh"
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
    """Return one function of a compiled backend's implementation of a slam_it_all module.

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
        If `backend` is unknown or ``"python"``, or `module_name` is not a slam_it_all module.
    BackendNotBuiltError
        If the backend's extension is not installed.
    NotImplementedError
        If the backend does not implement `module_name`, or implements it without `name`
        (both of which the tests make a failure).
    """
    found = getattr(implementation(backend, module_name), name, None)
    if found is None or isinstance(found, ModuleType) or not callable(found):
        raise NotImplementedError(f"backend {backend!r} does not implement {module_name}.{name}")
    return cast("Callable[..., Any]", found)


def program(backend: str, name: str) -> Path:
    """Return a compiled backend's native program.

    C++'s programs are installed with the package, in ``slam_it_all/bin/`` (an editable install
    rebuilds them when the extension is imported, which this does first). Rust's are built by
    cargo in the checkout, in ``rust/target/release/`` (``scripts/setup_slam_it_all.sh`` and
    ``scripts/check.sh`` pass ``--target-dir rust/target``, so ``CARGO_TARGET_DIR`` does not
    move them), and nothing rebuilds them on import: after a change to a Rust crate, rebuild
    (``scripts/check.sh`` does): a Rust program older than a file it is built from (its crates'
    manifests and sources, read from the workspace's manifests, and the lock) is refused.

    Parameters
    ----------
    backend : {"cpp", "rust"}
        The compiled backend.
    name : str
        The program, e.g. ``"slam_it_all_vo"``.

    Returns
    -------
    pathlib.Path
        The program's file.

    Raises
    ------
    ValueError
        If `backend` is unknown or ``"python"`` (whose pipelines run in-process).
    BackendNotBuiltError
        If the program is not built.
    StaleProgramError
        If a Rust program is older than a file it is built from.
    ImportError
        If the C++ extension is installed but fails to load.
    """
    check_name(backend)
    if backend == "python":
        raise ValueError("the python reference runs in-process; it has no program")
    if backend == "cpp":
        _extension(backend)
        package = importlib.import_module(_PACKAGE_PREFIX.rstrip("."))
        candidates = [Path(entry) / "bin" / name for entry in package.__path__]
    else:
        candidates = [_RUST_PROGRAMS / name]
    for candidate in candidates:
        if candidate.is_file():
            if backend == "rust":
                _check_fresh(candidate, _RUST)
            return candidate
    raise BackendNotBuiltError(
        f"backend {backend!r} has no program {name!r} (looked in "
        f"{', '.join(str(c.parent) for c in candidates)}); run scripts/setup_slam_it_all.sh"
    )


def dispatch(
    backend: str,
    module_name: str,
    name: str,
    reference: Callable[..., NDArray[np.float64]],
    *args: object,
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
    *args : ndarray or scalar
        The validated arguments, passed unchanged: arrays, and plain scalars (int, float,
        bool) for options.

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


Codes = dict[str, list[str]]
"""A compiled module's enum tables, from its ``codes()``: each enum's names in code order."""


def decode_name(codes: Codes, table: str, code: SupportsInt) -> str:
    """Return the name a compiled backend's code stands for in one of its enum tables.

    Parameters
    ----------
    codes : Codes
        The backend's tables (its ``codes()``).
    table : str
        The enum's table, e.g. ``"nls_termination"``.
    code : int-like
        The code the backend returned.

    Returns
    -------
    str
        The name.

    Raises
    ------
    RuntimeError
        If the code is outside the table. A negative code would otherwise index from the end
        and read as a plausible wrong name.
    """
    names = codes[table]
    index = int(code)
    if not 0 <= index < len(names):
        raise RuntimeError(f"{table}: backend returned code {index}, outside {names}")
    return names[index]


ResultT = TypeVar("ResultT")


def call(  # noqa: PLR0913 -- the two sides and the decoder are keyword-only
    backend: str,
    module_name: str,
    name: str,
    *,
    reference: Callable[[], ResultT],
    arguments: Mapping[str, object],
    decode: Callable[[Any, Codes], ResultT],
) -> ResultT:
    """Run one function with the chosen backend, decoding a compiled result into Python's.

    :func:`dispatch` serves functions whose two sides take the same arguments and return an
    array. This serves the rest: the reference takes Python objects (an options dataclass, a
    solver's name) where the compiled function takes their crossing form (a dict, a code),
    and the compiled function returns a dict for `decode` to rebuild into the reference's
    result type.

    Parameters
    ----------
    backend : {"python", "cpp", "rust"}
        Which implementation computes the result.
    module_name : str
        The calling module's ``__name__``.
    name : str
        The compiled function's name, as in the module's ``Implementation`` Protocol.
    reference : callable
        Computes the result with the Python reference; called when `backend` is ``"python"``.
    arguments : mapping of str to object
        The compiled function's arguments, passed by keyword.
    decode : callable
        Builds the result from the compiled function's return value and the module's
        :data:`Codes` (empty when the module has no ``codes()``).

    Returns
    -------
    ResultT
        The result, of the same type whichever backend computed it.

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
        return reference()
    raw = function(backend, module_name, name)(**arguments)
    tables = getattr(implementation(backend, module_name), "codes", None)
    return decode(raw, tables() if callable(tables) else {})


def checked_array(raw: Any, dtype: type[np.generic], ndim: int, name: str) -> NDArray[Any]:  # noqa: ANN401 -- a compiled backend's value
    """Return a compiled backend's array, checked (never cast) to the dtype and rank expected.

    Parameters
    ----------
    raw : object
        What the backend returned.
    dtype : numpy scalar type
        The dtype it must have.
    ndim : int
        The number of dimensions it must have.
    name : str
        What it is, for the message.

    Returns
    -------
    ndarray
        `raw` as an array.

    Raises
    ------
    RuntimeError
        If the dtype or rank differs: a binding bug, never the caller's.
    """
    out = np.asarray(raw)
    if out.dtype != dtype or out.ndim != ndim:
        raise RuntimeError(
            f"{name}: backend returned {out.dtype} {out.shape}, expected {ndim}-D {np.dtype(dtype)}"
        )
    return out
