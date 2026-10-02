"""Check the ladder's layering in C++ and Rust against the contracts in pyproject.toml.

import-linter enforces the layering for Python. This script enforces the same rules for
the other two languages, reading the same contracts, so the three can never disagree:

- C++: every ``#include "slamlab/<rung>/..."`` or ``#include <slamlab/<rung>/...>`` in
  ``cpp/{include/slamlab,src,tests,bindings}/<rung>/`` must name the file's own rung or one
  it may depend on; no C++ source may sit outside a rung directory (except the bindings'
  root, which only wires the rungs); and every ``slamlab_add_rung(<rung> DEPENDS ...)`` in
  ``CMakeLists.txt`` may only link the rungs it may depend on.
- Rust: every workspace member except the ``slamlab-py`` bindings crate is a rung crate named
  ``slamlab-<rung>``, and may only depend -- in any dependency table, including
  dev/build/target-specific ones, and through renamed (``package = ...``) entries -- on the
  crates of the rungs it may depend on.

A rung may depend on the rungs in lower layers, never on a sibling in its own layer, and
never on a rung a forbidden contract names. Directories and crates must belong to a rung
listed in the layers contract.

Usage::

    python scripts/check_layers.py [REPO_ROOT]

Exits 1 and lists every violation if any is found.
"""

from __future__ import annotations

import re
import sys
import tomllib
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_INCLUDE = re.compile(r"^\s*#\s*include\s*[<\"]slamlab/([a-z_]+)/", re.MULTILINE)
_ADD_RUNG = re.compile(r"slamlab_add_rung\(\s*([a-z_]+)([^)]*)\)")
_CPP_RUNG_ROOTS = ("cpp/include/slamlab", "cpp/src", "cpp/tests", "cpp/bindings")
_CPP_SUFFIXES = frozenset({".hpp", ".cpp", ".h", ".cc"})
_CPP_ROOT_FILES_ALLOWED = frozenset({"cpp/bindings"})
"""Rung roots whose top level may hold files: the bindings' root only wires the rungs."""
_DEPENDENCY_TABLES = ("dependencies", "dev-dependencies", "build-dependencies")
_BINDINGS_CRATE = "slamlab-py"
_PACKAGE = "slamlab"
_NON_RUNG_PACKAGES = frozenset({"backends"})
"""Layers that are Python plumbing, with no C++ or Rust counterpart."""


@dataclass(frozen=True)
class Violation:
    """One layering violation.

    Attributes
    ----------
    location : str
        Where it was found, as ``path`` or ``path:line``.
    message : str
        What is wrong.
    """

    location: str
    message: str


def allowed_dependencies(pyproject: dict[str, Any]) -> dict[str, frozenset[str]]:
    """Derive, for every rung, the rungs it may depend on from the import-linter contracts.

    Parameters
    ----------
    pyproject : dict
        The parsed ``pyproject.toml``.

    Returns
    -------
    dict of str to frozenset of str
        Each rung named in the layers contract, mapped to the rungs below it, minus any its
        forbidden contracts exclude. A rung's own name is not included.

    Raises
    ------
    ValueError
        If ``pyproject.toml`` has no exhaustive layers contract over the ``slamlab`` container.
    """
    contracts = pyproject["tool"]["importlinter"]["contracts"]
    layers_contract = next(
        (
            c
            for c in contracts
            if c["type"] == "layers" and c.get("containers") == [_PACKAGE] and c.get("exhaustive")
        ),
        None,
    )
    if layers_contract is None:
        raise ValueError("pyproject.toml needs an exhaustive layers contract over 'slamlab'")
    layers = [
        [name.strip() for name in re.split(r"[|:]", layer)] for layer in layers_contract["layers"]
    ]
    allowed: dict[str, set[str]] = {}
    for depth, layer in enumerate(layers):
        below = {name for lower in layers[depth + 1 :] for name in lower}
        for name in layer:
            allowed[name] = set(below)
    prefix = f"{_PACKAGE}."
    for contract in contracts:
        if contract["type"] != "forbidden":
            continue
        forbidden = {
            m.removeprefix(prefix) for m in contract["forbidden_modules"] if m.startswith(prefix)
        }
        for source in contract["source_modules"]:
            rung = source.removeprefix(prefix)
            if rung in allowed:
                allowed[rung] -= forbidden
    return {rung: frozenset(deps) for rung, deps in allowed.items()}


def _check_rung(name: str, location: str, allowed: dict[str, frozenset[str]]) -> Violation | None:
    """Return a violation if `name` is not a rung in the layers contract."""
    if name in allowed and name not in _NON_RUNG_PACKAGES:
        return None
    return Violation(location, f"{name!r} is not a rung in pyproject.toml's layers contract")


def check_cpp(root: Path, allowed: dict[str, frozenset[str]]) -> Iterator[Violation]:
    """Check C++ includes and CMake rung dependencies.

    Parameters
    ----------
    root : pathlib.Path
        The repository root.
    allowed : dict of str to frozenset of str
        Each rung's permitted dependencies, from :func:`allowed_dependencies`.

    Yields
    ------
    Violation
        Each forbidden include, forbidden link dependency, or directory outside any rung.
    """
    yield from _check_cpp_includes(root, allowed)
    yield from _check_cmake(root, allowed)


def _check_cpp_includes(root: Path, allowed: dict[str, frozenset[str]]) -> Iterator[Violation]:
    """Check every slamlab include under the per-rung C++ directories, and stray files."""
    for rung_root in _CPP_RUNG_ROOTS:
        base = root / rung_root
        if not base.is_dir():
            continue
        if rung_root not in _CPP_ROOT_FILES_ALLOWED:
            for stray in sorted(p for p in base.iterdir() if p.suffix in _CPP_SUFFIXES):
                yield Violation(
                    str(stray.relative_to(root)), "C++ source outside any rung directory"
                )
        for rung_dir in sorted(p for p in base.iterdir() if p.is_dir()):
            rung = rung_dir.name
            bad_dir = _check_rung(rung, f"{rung_root}/{rung}", allowed)
            if bad_dir:
                yield bad_dir
                continue
            sources = (p for p in sorted(rung_dir.rglob("*")) if p.suffix in _CPP_SUFFIXES)
            for source in sources:
                text = source.read_text()
                for match in _INCLUDE.finditer(text):
                    target = match.group(1)
                    if target != rung and target not in allowed[rung]:
                        line = text.count("\n", 0, match.start()) + 1
                        yield Violation(
                            f"{source.relative_to(root)}:{line}",
                            f"rung {rung!r} may not include from rung {target!r}",
                        )


def _check_cmake(root: Path, allowed: dict[str, frozenset[str]]) -> Iterator[Violation]:
    """Check every ``slamlab_add_rung(<rung> DEPENDS ...)`` in ``CMakeLists.txt``."""
    cmake = root / "CMakeLists.txt"
    if not cmake.is_file():
        return
    for match in _ADD_RUNG.finditer(cmake.read_text()):
        rung, rest = match.group(1), match.group(2)
        bad_rung = _check_rung(rung, "CMakeLists.txt", allowed)
        if bad_rung:
            yield bad_rung
            continue
        for dep in re.findall(r"slamlab_([a-z_]+)", rest):
            if dep not in allowed[rung]:
                yield Violation("CMakeLists.txt", f"rung {rung!r} may not link slamlab_{dep}")


def check_rust(root: Path, allowed: dict[str, frozenset[str]]) -> Iterator[Violation]:
    """Check the per-rung Rust crates' dependencies.

    Parameters
    ----------
    root : pathlib.Path
        The repository root.
    allowed : dict of str to frozenset of str
        Each rung's permitted dependencies, from :func:`allowed_dependencies`.

    Yields
    ------
    Violation
        Each forbidden crate dependency, or workspace crate outside any rung.
    """
    for manifest_path in _workspace_manifests(root / "rust"):
        manifest = tomllib.loads(manifest_path.read_text())
        crate = manifest["package"]["name"]
        if crate == _BINDINGS_CRATE:  # the bindings crate may use every rung crate
            continue
        location = str(manifest_path.relative_to(root))
        rung = crate.removeprefix("slamlab-").replace("-", "_")
        bad_rung = (
            _check_rung(rung, location, allowed)
            if crate.startswith("slamlab-")
            else (Violation(location, f"crate {crate!r} is not named slamlab-<rung>"))
        )
        if bad_rung:
            yield bad_rung
            continue
        for dep in _dependency_names(manifest):
            if dep.startswith("slamlab-") and dep != crate:
                target = dep.removeprefix("slamlab-").replace("-", "_")
                if target not in allowed[rung]:
                    yield Violation(location, f"rung {rung!r} may not depend on crate {dep!r}")


def _workspace_manifests(rust: Path) -> list[Path]:
    """Return the ``Cargo.toml`` of every workspace member (glob members expanded)."""
    workspace = rust / "Cargo.toml"
    if not workspace.is_file():
        return []
    members = tomllib.loads(workspace.read_text()).get("workspace", {}).get("members", [])
    return sorted(
        path / "Cargo.toml"
        for member in members
        for path in rust.glob(member)
        if (path / "Cargo.toml").is_file()
    )


def _dependency_names(manifest: dict[str, Any]) -> Iterator[str]:
    """Yield the real crate name of every dependency, in every table of a manifest.

    Covers ``[dependencies]``, ``[dev-dependencies]``, ``[build-dependencies]`` and their
    ``[target.<cfg>.*]`` forms; a renamed entry (``alias = { package = "real" }``) yields
    the real name.
    """
    tables = [manifest.get(name, {}) for name in _DEPENDENCY_TABLES]
    for target in manifest.get("target", {}).values():
        tables.extend(target.get(name, {}) for name in _DEPENDENCY_TABLES)
    for table in tables:
        for key, spec in table.items():
            yield spec.get("package", key) if isinstance(spec, dict) else key


def main(argv: list[str]) -> int:
    """Run every check and report violations.

    Parameters
    ----------
    argv : list of str
        Command-line arguments: an optional repository root (default: this repository).

    Returns
    -------
    int
        0 if the layering holds in C++ and Rust, 1 otherwise.
    """
    root = Path(argv[0]) if argv else Path(__file__).resolve().parent.parent
    allowed = allowed_dependencies(tomllib.loads((root / "pyproject.toml").read_text()))
    violations = [*check_cpp(root, allowed), *check_rust(root, allowed)]
    for violation in violations:
        print(f"{violation.location}: {violation.message}")
    if violations:
        print(f"{len(violations)} layering violation(s)")
        return 1
    print("C++ and Rust layering matches pyproject.toml")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
