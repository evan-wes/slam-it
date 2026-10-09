"""Tests for the backend locator, and that every compiled module matches its Protocol."""

from __future__ import annotations

import ast
import importlib
import inspect
import os
import pkgutil
from collections.abc import Iterator
from dataclasses import asdict
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
import pytest

import slamlab
from slamlab import backends
from slamlab.foundations import image, orb, stereo
from slamlab.foundations.errors import InvalidInputError
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


def test_program_is_only_for_compiled_backends() -> None:
    """Python's pipelines run in-process; an unknown backend is an error."""
    with pytest.raises(ValueError, match="in-process"):
        backends.program("python", "slamlab_vo")
    with pytest.raises(ValueError, match="unknown backend"):
        backends.program("fortran", "slamlab_vo")


def test_missing_program_is_reported() -> None:
    """A program that is not built is BackendNotBuiltError, naming where it was looked for."""
    with pytest.raises(backends.BackendNotBuiltError, match="no program 'no_such_program'"):
        backends.program("rust", "no_such_program")


def _workspace(root: Path) -> Path:
    """Write a Rust workspace: a program's crate, the crates it depends on, one it does not."""
    (root / "Cargo.toml").write_text('[workspace]\nmembers = ["app", "core", "util", "other"]\n')
    (root / "Cargo.lock").write_text("# lock\n")
    crates = {
        "app": '[[bin]]\nname = "prog"\npath = "src/main.rs"\n\n[dependencies]\n'
        'core = { path = "../core" }\nnalgebra = "0.33"\n',
        "core": '[dependencies]\nutil = { path = "../util" }\n',
        "util": "",
        "other": "",
    }
    for crate, rest in crates.items():
        (root / crate / "src").mkdir(parents=True)
        (root / crate / "Cargo.toml").write_text(f'[package]\nname = "{crate}"\n\n{rest}')
        (root / crate / "src" / "lib.rs").write_text("// source\n")
    program = root / "target" / "release" / "prog"
    program.parent.mkdir(parents=True)
    program.write_text("")
    for path in root.rglob("*"):
        os.utime(path, (1000.0, 1000.0))
    os.utime(program, (2000.0, 2000.0))
    return program


@pytest.mark.parametrize(
    ("touched", "stale"),
    [
        ("util/src/lib.rs", True),  # a crate it depends on through another
        ("core/Cargo.toml", True),
        ("Cargo.lock", True),
        ("other/src/lib.rs", False),  # a crate it does not depend on
        (None, False),
    ],
)
def test_a_rust_program_older_than_its_inputs_is_stale(
    tmp_path: Path, touched: str | None, stale: bool
) -> None:
    """The inputs come from the manifests: the bin's crate, its path dependencies, the lock."""
    program = _workspace(tmp_path)
    if touched is not None:
        os.utime(tmp_path / touched, (3000.0, 3000.0))
    if stale:
        with pytest.raises(backends.StaleProgramError, match=f"older than .*{touched}"):
            backends._check_fresh(program, tmp_path)
    else:
        backends._check_fresh(program, tmp_path)
    assert isinstance(backends.StaleProgramError("x"), backends.BackendNotBuiltError)


def test_module_outside_slamlab_is_an_error() -> None:
    """The locator only resolves slamlab modules."""
    with pytest.raises(ValueError, match="not a slamlab module"):
        backends.implementation("cpp", "numpy.linalg")


@pytest.mark.parametrize("name", backends.COMPILED_BACKENDS)
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


def _protocol_modules(rungs: tuple[str, ...]) -> set[str]:
    """Return every module of `rungs` that defines an ``Implementation`` Protocol (its path).

    Only those rungs are imported: the PyTorch rungs, which no compiled backend covers, stay
    out of the CPU tests.
    """
    found = set()
    for rung in rungs:
        package = importlib.import_module(f"slamlab.{rung}")
        found |= _protocols_below(package)
    return found


def _protocols_below(package: ModuleType) -> set[str]:
    """Return the modules of `package` (itself included) that define ``Implementation``."""
    found = set()
    infos = pkgutil.walk_packages(package.__path__, f"{package.__name__}.")
    for name in [package.__name__, *(info.name for info in infos)]:
        module = importlib.import_module(name)
        protocol = vars(module).get("Implementation")
        if protocol is not None and protocol.__module__ == module.__name__:
            found.add(module.__name__)
    return found


def test_compiled_rungs_name_every_compiled_backend_and_real_rungs() -> None:
    """``COMPILED_RUNGS`` covers exactly the compiled backends, with slamlab's own rungs."""
    assert set(backends.COMPILED_RUNGS) == set(backends.COMPILED_BACKENDS)
    rungs = {m.name for m in pkgutil.iter_modules(slamlab.__path__) if m.ispkg}
    for name, covered in backends.COMPILED_RUNGS.items():
        assert set(covered) <= rungs, f"{name}: {sorted(set(covered) - rungs)}"


@pytest.mark.parametrize("name", backends.COMPILED_BACKENDS)
def test_every_compiled_backend_has_every_module(name: str) -> None:
    """Each compiled backend implements every Protocol module of the rungs it covers.

    Every backend is lock-step within its rungs (``backends.COMPILED_RUNGS``), so this is one
    assertion rather than a property spread over each module's tests (panel A1 on PR 6). A
    compiled submodule stands for the module that defines its Protocol (``foundations.ba``
    for ``slamlab.foundations.ba.solve``'s).
    """
    root_name = _EXTENSION_ROOTS[require_backend(name)]
    extension = importlib.import_module(root_name)
    rungs = [m.name for m in pkgutil.iter_modules(slamlab.__path__) if m.ispkg]
    provided = {
        importlib.import_module(f"slamlab.{path}").Implementation.__module__
        for rung in rungs
        if isinstance(getattr(extension, rung, None), ModuleType)
        for path, _ in _implementation_modules(getattr(extension, rung), rung)
    }
    missing = _protocol_modules(backends.COMPILED_RUNGS[name]) - provided
    assert not missing, f"{name} lacks {sorted(missing)}"


@pytest.mark.parametrize("name", backends.COMPILED_BACKENDS)
def test_every_compiled_module_matches_its_protocol(name: str) -> None:
    """Every compiled submodule mirrors a slamlab module and matches its Implementation.

    For each submodule of the extension that holds functions, the slamlab module at the same
    path must exist and define an ``Implementation`` Protocol; every compiled function must
    be a Protocol member with the Protocol's parameter names, and every member of the Protocol
    must be there: a backend implements each module it has in full. This keeps bindings and
    their Python contract from drifting; that it has every module is
    :func:`test_every_compiled_backend_has_every_module`.
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
        assert provided == members, f"{name}:{path} lacks {sorted(members - provided)}"
        for member in provided:
            expected = [
                p for p in inspect.signature(getattr(protocol, member)).parameters if p != "self"
            ]
            got = _parameter_names(getattr(compiled, member))
            assert got == expected, f"{name}:{path}.{member} parameters {got} != {expected}"


@pytest.mark.parametrize("name", backends.COMPILED_BACKENDS)
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


def _pyramid_call(backend: str) -> tuple[Any, Any, list[Any], dict[str, Any], dict[str, Any]]:
    """Return both raw bindings that take a pyramid, a good one, and their other arguments.

    Every argument is valid, so the calls succeed with the good pyramid and a level can be the
    only thing refused.
    """
    levels = list(image.pyramid(np.zeros((40, 40), dtype=np.uint8), 2, 1.25))
    features = {
        "xy": np.zeros((0, 2)),
        "level": np.zeros(0, dtype=np.int64),
        "level_xy": np.zeros((0, 2), dtype=np.int64),
        "descriptors": np.zeros((0, 32), dtype=np.uint8),
        "scale_factor": 1.25,
        "levels": 2,
    }
    orb_options = asdict(orb.Options(levels=2, scale_factor=1.25))
    stereo_options = asdict(stereo.Options())
    orb_impl = backends.implementation(backend, "slamlab.foundations.orb")
    stereo_impl = backends.implementation(backend, "slamlab.foundations.stereo")
    assert isinstance(orb_impl.extract_levels(levels, orb_options), dict)
    assert isinstance(stereo_impl.match(levels, levels, features, features, 1.0, 0.1,
                                        stereo_options), dict)  # fmt: skip
    return orb_impl, stereo_impl, levels, features, {"orb": orb_options, "stereo": stereo_options}


@pytest.mark.parametrize("name", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(
    "bad",
    [
        np.full((40, 40), 300, dtype=np.int64),
        np.zeros((40, 40), dtype=np.float64),
        np.zeros((40, 80), dtype=np.uint8)[:, ::2],
    ],
    ids=["int64", "float64", "strided"],
)
def test_compiled_pyramid_bindings_refuse_unconverted_levels(name: str, bad: object) -> None:
    """Both raw bindings refuse a pyramid level that is not a contiguous uint8 image.

    As for a single image: a direct caller pays no hidden conversion, and gets no silent
    truncation of 300 to a byte, in either language. The same calls with the good level
    succeed, so the level is what is refused.
    """
    orb_impl, stereo_impl, levels, features, options = _pyramid_call(require_backend(name))
    given = [bad, levels[1]]
    with pytest.raises((TypeError, ValueError)):  # Rust refuses a strided array as a ValueError
        orb_impl.extract_levels(given, options["orb"])
    with pytest.raises((TypeError, ValueError)):
        stereo_impl.match(given, levels, features, features, 1.0, 0.1, options["stereo"])


@pytest.mark.parametrize("name", backends.COMPILED_BACKENDS)
def test_compiled_pyramid_bindings_name_the_bad_level(name: str) -> None:
    """A malformed level is refused by its index, as Python's as_pyramid names it."""
    orb_impl, stereo_impl, levels, features, options = _pyramid_call(require_backend(name))
    given = [levels[0], np.zeros((8, 0), dtype=np.uint8)]
    with pytest.raises(InvalidInputError, match=r"levels\[1\]: sides"):
        orb_impl.extract_levels(given, options["orb"])
    with pytest.raises(InvalidInputError, match=r"left_levels\[1\]: sides"):
        stereo_impl.match(given, levels, features, features, 1.0, 0.1, options["stereo"])


def test_call_runs_the_reference_for_python() -> None:
    """With the Python backend, ``call`` runs the reference and never touches an extension."""
    result = backends.call(
        "python",
        "slamlab.foundations.nowhere",
        "nothing",
        reference=lambda: 42,
        arguments={},
        decode=lambda raw, codes: -1,
    )
    assert result == 42


def test_call_decodes_a_compiled_result_with_the_module_codes() -> None:
    """A compiled result reaches the decoder with the module's ``codes()`` tables."""
    require_backend("cpp")
    seen: dict[str, object] = {}

    def decode(raw: object, codes: backends.Codes) -> int:
        seen["raw"], seen["codes"] = raw, codes
        return 7

    result = backends.call(
        "cpp",
        "slamlab.foundations.fivepoint",
        "codes",
        reference=lambda: 0,
        arguments={},
        decode=decode,
    )
    assert result == 7
    assert seen["raw"] == seen["codes"] == {"solver": ["nister", "stewenius"]}


def test_call_rejects_an_unknown_backend() -> None:
    """An unknown backend name is an error before anything runs."""
    with pytest.raises(ValueError, match="unknown backend"):
        backends.call(
            "julia",
            "slamlab.foundations.so3",
            "exp",
            reference=lambda: 0,
            arguments={},
            decode=lambda raw, codes: 0,
        )


def test_decode_name_rejects_codes_outside_the_table() -> None:
    """A code outside its table fails loudly; -1 would otherwise read as the last name."""
    codes = {"gauge": ["none", "first_pose", "partial"]}
    assert backends.decode_name(codes, "gauge", 1) == "first_pose"
    for bad in (-1, 3):
        with pytest.raises(RuntimeError, match="outside"):
            backends.decode_name(codes, "gauge", bad)


@pytest.mark.parametrize(
    ("raw", "message"),
    [
        (np.zeros(3, np.float32), "values: backend returned float32"),
        (np.zeros((3, 1)), r"expected 1-D float64"),
        ([1, 2, 3], "backend returned int64"),
    ],
)
def test_checked_array_refuses_a_wrong_dtype_or_rank(raw: object, message: str) -> None:
    """A compiled result of another dtype or rank is a binding bug: RuntimeError, never a cast."""
    with pytest.raises(RuntimeError, match=message):
        backends.checked_array(raw, np.float64, 1, "values")


def test_checked_array_returns_a_good_array_as_is() -> None:
    """A result of the expected dtype and rank comes back unchanged, not copied."""
    values = np.arange(3.0)
    assert backends.checked_array(values, np.float64, 1, "values") is values
