"""Tests for the backend locator, and that every compiled module matches its Protocol."""

from __future__ import annotations

import ast
import importlib
import inspect
import pkgutil
from collections.abc import Iterator
from types import ModuleType

import numpy as np
import pytest

import slamlab
from slamlab import backends
from tests.conftest import REQUIRED_BACKENDS, require_backend

_EXTENSION_ROOTS = {"cpp": "slamlab._cpp", "rust": "slamlab_rs"}


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
@pytest.mark.parametrize(
    "module_name",
    ["slamlab.foundations.no_such_module", "slamlab.foundations.so3.hat"],
    ids=["missing", "not-a-module"],
)
def test_missing_implementation_is_reported(name: str, module_name: str) -> None:
    """A path the backend does not implement as a module raises NotImplementedError."""
    with pytest.raises(NotImplementedError, match="does not implement"):
        backends.implementation(require_backend(name), module_name)


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


def _implementation_modules(root: ModuleType, path: str) -> Iterator[tuple[str, ModuleType]]:
    """Yield every submodule below `root` that holds functions, with its dotted path.

    Parameters
    ----------
    root : module
        A rung submodule of a compiled extension (or one of its submodules).
    path : str
        `root`'s dotted path below the extension, e.g. ``"foundations"``.

    Yields
    ------
    tuple of (str, module)
        The dotted path (e.g. ``"foundations.so3"``) and the submodule.
    """
    members = vars(root)
    if any(callable(v) and not isinstance(v, ModuleType) for v in members.values()):
        yield path, root
    for child_name, child in members.items():
        if isinstance(child, ModuleType):
            yield from _implementation_modules(child, f"{path}.{child_name}")


def _strip_brackets(text: str) -> str:
    """Remove every bracketed group, e.g. ``ndarray[dtype=float64, shape=(3)]`` -> ``ndarray``.

    nanobind writes ndarray annotations with keyword-like contents that are not valid
    Python, so they are removed before parsing; only parameter names are compared.

    Parameters
    ----------
    text : str
        A signature line.

    Returns
    -------
    str
        `text` without any ``[...]`` group (nesting handled).
    """
    out, depth = [], 0
    for char in text:
        if char == "[":
            depth += 1
        elif char == "]":
            depth -= 1
        elif depth == 0:
            out.append(char)
    return "".join(out)


def _parameter_names(func: object) -> list[str]:
    """Return a compiled function's parameter names from its nanobind or PyO3 signature.

    Parameters
    ----------
    func : callable
        A function from a compiled extension.

    Returns
    -------
    list of str
        The parameter names, in order.
    """
    nb_signature = getattr(func, "__nb_signature__", None)
    if nb_signature:
        # nanobind: a full Python `def` line, possibly with nested annotations; parse it.
        tree = ast.parse(_strip_brackets(nb_signature[0][0]) + ": ...")
        definition = tree.body[0]
        assert isinstance(definition, ast.FunctionDef)
        return [arg.arg for arg in (*definition.args.args, *definition.args.kwonlyargs)]
    assert callable(func)
    return list(inspect.signature(func).parameters)


def _public_callables(module: ModuleType) -> set[str]:
    """Return the names of a compiled module's public functions."""
    return {
        name
        for name, value in vars(module).items()
        if callable(value) and not isinstance(value, ModuleType) and not name.startswith("_")
    }


@pytest.mark.parametrize("name", ["cpp", "rust"])
def test_every_compiled_module_matches_its_protocol(name: str) -> None:
    """Every compiled submodule mirrors a slamlab module and matches its Implementation.

    For each submodule of the extension that holds functions, the slamlab module at the same
    path must exist and define an ``Implementation`` Protocol; every compiled function must
    be a Protocol member with the Protocol's parameter names. A lock-step backend (C++)
    must provide every member of every Protocol it implements a module for; Rust implements
    a curated subset, so it may provide fewer. This keeps bindings and their Python contract
    from drifting.
    """
    root_name = _EXTENSION_ROOTS[require_backend(name)]
    extension = importlib.import_module(root_name)
    # Walk from slamlab's rung packages, the tree the extension submodules mirror. Every
    # top-level submodule must be one of them (a misnamed one would otherwise go unchecked);
    # maturin's package re-exports the native module under the extension's own name.
    rungs = [m.name for m in pkgutil.iter_modules(slamlab.__path__) if m.ispkg]
    top_level = {k for k, v in vars(extension).items() if isinstance(v, ModuleType)}
    stray = top_level - set(rungs) - {root_name.rsplit(".", 1)[-1]}
    assert not stray, f"{name} extension submodules match no slamlab rung: {sorted(stray)}"
    found = [
        item
        for rung in rungs
        if isinstance(getattr(extension, rung, None), ModuleType)
        for item in _implementation_modules(getattr(extension, rung), rung)
    ]
    assert found, f"{name} extension exposes no implementation modules"
    for path, compiled in found:
        python_module = importlib.import_module(f"slamlab.{path}")
        protocol = getattr(python_module, "Implementation", None)
        assert protocol is not None, f"slamlab.{path} has no Implementation Protocol"
        members = {m for m in vars(protocol) if not m.startswith("_")}
        provided = _public_callables(compiled)
        assert provided <= members, f"{name}:{path} has functions outside the Protocol"
        if name in backends.LOCKSTEP_BACKENDS:
            assert provided == members, f"{name}:{path} lacks {sorted(members - provided)}"
        for member in provided:
            expected = [
                p for p in inspect.signature(getattr(protocol, member)).parameters if p != "self"
            ]
            got = _parameter_names(getattr(compiled, member))
            assert got == expected, f"{name}:{path}.{member} parameters {got} != {expected}"


@pytest.mark.parametrize("name", ["cpp", "rust"])
@pytest.mark.parametrize(
    "bad",
    [
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        np.arange(6.0).reshape(1, 6)[:, ::2],
        # Contiguous float64 at an odd byte offset: misaligned for double.
        np.zeros(25, dtype=np.uint8)[1:25].view(np.float64).reshape(1, 3),
    ],
    ids=["float32", "strided", "misaligned"],
)
def test_compiled_bindings_refuse_unconverted_input(name: str, bad: object) -> None:
    """Both raw bindings refuse anything but an aligned, contiguous float64 (N, 3) batch.

    So a direct caller, or a benchmark, pays no hidden conversion in either language; the
    Python entry points validate and convert before calling.
    """
    impl = backends.implementation(require_backend(name), "slamlab.foundations.so3")
    with pytest.raises((TypeError, ValueError)):
        impl.hat(bad)
