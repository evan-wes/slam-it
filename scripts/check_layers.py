"""Check the ladder's layering in C++ and Rust against the contracts in pyproject.toml.

import-linter enforces the layering for Python. This script enforces the same rules for
the other two languages, reading the same contracts, so the three can never disagree:

- C++: every ``#include "slamlab/<rung>/..."`` or ``#include <slamlab/<rung>/...>`` in
  ``cpp/{include/slamlab,src,tests,bindings}/<rung>/`` must name the file's own rung or one
  it may depend on; no C++ source may sit outside a rung directory (except the bindings'
  root, which only wires the rungs); and every ``slamlab_add_rung(<rung> DEPENDS ...)`` in
  ``CMakeLists.txt`` may only link the rungs it may depend on.
- Rust: every crate under ``rust/`` except the ``slamlab-py`` bindings crate -- listed
  workspace members and the implicit ones Cargo adds for path dependencies alike -- is a
  rung crate named ``slamlab-<rung>``. It may only depend on the crates of the rungs it may
  depend on: in any dependency table (including dev/build/target-specific ones), through
  renamed (``package = ...``) and ``workspace = true`` entries, and through path
  dependencies on non-rung helper crates, which are followed.

A rung may depend on the rungs in lower layers, never on a sibling in its own layer, and
never on a rung a forbidden contract names. Directories and crates must belong to a rung
listed in the layers contract.

- C++ inside a rung: the layers contracts over a rung or a package inside one (such as
  "Rung 0's internal order" over ``slamlab.foundations``) apply to the C++ includes in
  ``cpp/include/slamlab`` and ``cpp/src`` too. A file is the module its path names:
  ``foundations/so3.hpp`` is ``so3`` in ``foundations``, ``foundations/ba/schur.cpp`` is
  ``ba`` in ``foundations`` and ``schur`` in ``foundations.ba``. Includes resolve the same
  way, relative ones (``..`` too) against the including file's directory. As in
  import-linter, chains count: a module no contract lists (such as ``ba/common.hpp``) is not
  constrained itself, but includes are followed through it, so it cannot bridge a forbidden
  dependency. A package's facade (``foundations/ba.hpp``) stands for the Python modules
  ``_CPP_FACADES`` names. Rust's modules inside a crate are not checked (only crate
  dependencies are).

Usage::

    python scripts/check_layers.py [REPO_ROOT]

Exits 1 and lists every violation if any is found.
"""

from __future__ import annotations

import os
import re
import sys
import tomllib
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_INCLUDE = re.compile(r"^\s*#\s*include\s*[<\"]slamlab/([a-z_]+)/", re.MULTILINE)
_ANY_INCLUDE = re.compile(r"^\s*#\s*include\s*([<\"])([^>\"]+)[>\"]", re.MULTILINE)
_CPP_MODULE_ROOTS = ("cpp/include/slamlab", "cpp/src")
"""Where a C++ file's path names its module; tests and bindings are not modules."""
_CPP_MODULE_ALIASES = {"error": "errors"}
"""C++ files whose Python module is named differently."""
_CPP_FACADES: dict[tuple[str, ...], tuple[str | None, str | None]] = {
    ("foundations", "ba"): ("problem", "solve")
}
"""A package's C++ facade, as the Python modules it stands for: ``ba.hpp`` declares the
problem types (``problem``, the bottom layer) when included from inside the package, and
``ba.cpp`` implements ``solve`` (the top) when it includes the package's modules."""
_ADD_RUNG = re.compile(r"slamlab_add_rung\(\s*([a-z_]+)([^)]*)\)")
_CPP_RUNG_ROOTS = (
    "cpp/include/slamlab",
    "cpp/src",
    "cpp/tests",
    "cpp/bench",
    "cpp/bindings",
)
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


@dataclass(frozen=True)
class Layer:
    """One layer of a layers contract: its modules, and whether they may import each other."""

    modules: frozenset[str]
    independent: bool


def internal_orders(pyproject: dict[str, Any]) -> dict[tuple[str, ...], list[Layer]]:
    """Return every layers contract below the rung level, by its container's path.

    Parameters
    ----------
    pyproject : dict
        The parsed ``pyproject.toml``.

    Returns
    -------
    dict of tuple of str to list of Layer
        Each container (``("foundations",)``, ``("foundations", "ba")``) mapped to its
        layers, top first. Optional modules (in parentheses) lose their parentheses.
    """
    orders: dict[tuple[str, ...], list[Layer]] = {}
    prefix = f"{_PACKAGE}."
    for contract in pyproject["tool"]["importlinter"]["contracts"]:
        if contract["type"] != "layers":
            continue
        for container in contract.get("containers", []):
            if not container.startswith(prefix):
                continue
            layers = []
            for layer in contract["layers"]:
                names = re.split(r"[|:]", layer)
                modules = frozenset(name.strip().strip("()") for name in names)
                layers.append(Layer(modules, independent="|" in layer))
            key = tuple(container.removeprefix(prefix).split("."))
            if key in orders:
                raise ValueError(f"two layers contracts over {container}")
            orders[key] = layers
    return orders


def _may_import(layers: list[Layer], source: str, target: str) -> bool:
    """Return whether module ``source`` may import ``target`` under ``layers``.

    True when either is unlisted, the target sits lower, or both share a non-independent
    layer.
    """
    depth = {name: i for i, layer in enumerate(layers) for name in layer.modules}
    if source not in depth or target not in depth:
        return True
    if depth[source] == depth[target]:
        return not layers[depth[source]].independent
    return depth[target] > depth[source]


def _module_path(path: Path) -> tuple[str, ...]:
    """Return a C++ path's module components (``..`` resolved, extension dropped, aliased)."""
    normal = Path(os.path.normpath(path))
    parts = [*normal.parent.parts, normal.stem]
    return tuple(_CPP_MODULE_ALIASES.get(part, part) for part in parts if part not in {"", "."})


def _include_graph(root: Path) -> dict[tuple[str, ...], list[tuple[tuple[str, ...], str]]]:
    """Map each C++ module to the slamlab modules it includes, each with the include's location.

    A module's header and source (``schur.hpp``, ``schur.cpp``) are one module.
    """
    graph: dict[tuple[str, ...], list[tuple[tuple[str, ...], str]]] = {}
    for module_root in _CPP_MODULE_ROOTS:
        base = root / module_root
        if not base.is_dir():
            continue
        for source in sorted(p for p in base.rglob("*") if p.suffix in _CPP_SUFFIXES):
            relative = source.relative_to(base)
            edges = graph.setdefault(_module_path(relative), [])
            text = source.read_text()
            for match in _ANY_INCLUDE.finditer(text):
                bracket, name = match.groups()
                if name.startswith(f"{_PACKAGE}/"):
                    target = _module_path(Path(name).relative_to(_PACKAGE))
                elif bracket == '"':
                    target = _module_path(relative.parent / name)
                else:
                    continue  # a system or third-party header
                line = text.count("\n", 0, match.start()) + 1
                edges.append((target, f"{source.relative_to(root)}:{line}"))
    return graph


def _listed(module: tuple[str, ...], orders: dict[tuple[str, ...], list[Layer]]) -> bool:
    """Return whether the innermost contract around ``module`` lists it (else it is a bridge).

    A module no contract surrounds is outside the rules, not a bridge.
    """
    around = [c for c in orders if module[: len(c)] == c and len(module) > len(c)]
    if not around:
        return True
    container = max(around, key=len)
    return any(module[len(container)] in layer.modules for layer in orders[container])


def check_cpp_internal(
    root: Path, orders: dict[tuple[str, ...], list[Layer]]
) -> Iterator[Violation]:
    """Check C++ includes inside each rung against the internal layers contracts.

    Like import-linter, a chain counts: an include of a module no contract lists (such as
    ``ba/common.hpp``) is followed through to what that module includes, so an unlisted
    helper cannot bridge a forbidden dependency.

    Parameters
    ----------
    root : pathlib.Path
        The repository root.
    orders : dict of tuple of str to list of Layer
        The internal contracts, from :func:`internal_orders`.

    Yields
    ------
    Violation
        Each include of a module the including module may not import, directly or through
        unlisted modules (located at the first include of the chain).
    """
    graph = _include_graph(root)
    for here, edges in graph.items():
        for first, location in edges:
            for target, via in _reachable(graph, here, first, orders):
                hops = f" (via {', '.join('/'.join(v) for v in via)})" if via else ""
                for violation in _check_include(here, target, orders, location):
                    yield Violation(violation.location, violation.message + hops)


Module = tuple[str, ...]


def _reachable(
    graph: dict[Module, list[tuple[Module, str]]],
    here: Module,
    first: Module,
    orders: dict[Module, list[Layer]],
) -> Iterator[tuple[Module, tuple[Module, ...]]]:
    """Yield what an include of ``first`` reaches from ``here``, with the modules on the way.

    ``first`` itself, then, through every module no contract lists, what it includes in turn.
    A listed module is not followed: its own includes are checked as its own.
    """
    pending: list[tuple[Module, tuple[Module, ...]]] = [(first, ())]
    seen = {first}
    while pending:
        target, via = pending.pop()
        yield target, via
        if target == here or _listed(target, orders):
            continue
        for nxt, _ in graph.get(target, []):
            if nxt not in seen:
                seen.add(nxt)
                pending.append((nxt, (*via, target)))


def _check_include(
    here: tuple[str, ...],
    target: tuple[str, ...],
    orders: dict[tuple[str, ...], list[Layer]],
    location: str,
) -> Iterator[Violation]:
    """Check one include against every container both modules sit inside.

    A container's facade (``foundations/ba.hpp``, ``ba.cpp``) stands for the modules
    :data:`_CPP_FACADES` names: what it declares when included, what it implements when
    including.
    """
    for container, layers in orders.items():
        depth = len(container)
        if here[:depth] != container or target[:depth] != container:
            continue
        included, including = _CPP_FACADES.get(container, (None, None))
        source = here[depth] if len(here) > depth else including
        dest = target[depth] if len(target) > depth else included
        if source is None or dest is None:
            continue  # a facade with no mapping stays outside its container's rules
        if source != dest and not _may_import(layers, source, dest):
            where = ".".join(container)
            yield Violation(location, f"in {where}, {source!r} may not include {dest!r}")


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
        Each forbidden crate dependency (direct, or relayed through a non-rung crate), and
        each crate under ``rust/`` that is not named ``slamlab-<rung>``.
    """
    rust = root / "rust"
    workspace_deps = _workspace_dependencies(rust)
    for manifest_path in _crate_manifests(rust):
        manifest = tomllib.loads(manifest_path.read_text())
        crate = manifest["package"]["name"]
        if crate == _BINDINGS_CRATE:  # the bindings crate may use every rung crate
            continue
        location = str(manifest_path.relative_to(root))
        if not crate.startswith("slamlab-"):
            yield Violation(location, f"crate {crate!r} is not named slamlab-<rung>")
            continue
        rung = crate.removeprefix("slamlab-").replace("-", "_")
        bad_rung = _check_rung(rung, location, allowed)
        if bad_rung:
            yield bad_rung
            continue
        for dep in _reachable_rung_crates(manifest_path, workspace_deps):
            target = dep.removeprefix("slamlab-").replace("-", "_")
            if dep != crate and target not in allowed[rung]:
                yield Violation(location, f"rung {rung!r} may not depend on crate {dep!r}")


def _crate_manifests(rust: Path) -> list[Path]:
    """Return every crate manifest under ``rust/`` (skipping build output), members or not.

    Cargo makes every path dependency inside the workspace a member even when
    ``[workspace] members`` does not list it, so every manifest is checked.
    """
    if not rust.is_dir():
        return []
    return sorted(
        path
        for path in rust.rglob("Cargo.toml")
        if "target" not in path.relative_to(rust).parts
        and "package" in tomllib.loads(path.read_text())
    )


def _workspace_dependencies(rust: Path) -> dict[str, tuple[str, Path | None]]:
    """Map each ``[workspace.dependencies]`` key to its real crate name and resolved path."""
    workspace = rust / "Cargo.toml"
    if not workspace.is_file():
        return {}
    table = tomllib.loads(workspace.read_text()).get("workspace", {}).get("dependencies", {})
    return {key: _resolve_spec(key, spec, rust) for key, spec in table.items()}


def _resolve_spec(key: str, spec: object, base: Path) -> tuple[str, Path | None]:
    """Return a dependency entry's real crate name and its path, if it is a path dependency."""
    if not isinstance(spec, dict):
        return key, None
    path = spec.get("path")
    return spec.get("package", key), (base / path).resolve() if path else None


def _dependencies(
    manifest_path: Path, workspace_deps: dict[str, tuple[str, Path | None]]
) -> Iterator[tuple[str, Path | None]]:
    """Yield the real name and path of every dependency of a crate, in every table.

    Covers ``[dependencies]``, ``[dev-dependencies]``, ``[build-dependencies]`` and their
    ``[target.<cfg>.*]`` forms; renamed entries (``alias = { package = "real" }``) and
    ``workspace = true`` entries are resolved to the real crate.
    """
    manifest = tomllib.loads(manifest_path.read_text())
    tables = [manifest.get(name, {}) for name in _DEPENDENCY_TABLES]
    for target in manifest.get("target", {}).values():
        tables.extend(target.get(name, {}) for name in _DEPENDENCY_TABLES)
    for table in tables:
        for key, spec in table.items():
            if isinstance(spec, dict) and spec.get("workspace") is True:
                yield workspace_deps.get(key, (key, None))
            else:
                yield _resolve_spec(key, spec, manifest_path.parent)


def _reachable_rung_crates(
    manifest_path: Path, workspace_deps: dict[str, tuple[str, Path | None]]
) -> set[str]:
    """Return the slamlab rung crates a crate depends on, directly or via non-rung crates.

    Rung crates are not followed further: each is checked on its own. Path dependencies on
    other crates are followed, so a helper crate cannot relay a forbidden rung.
    """
    found: set[str] = set()
    seen: set[Path] = set()
    pending = [manifest_path]
    while pending:
        current = pending.pop()
        for name, path in _dependencies(current, workspace_deps):
            if name.startswith("slamlab-"):
                found.add(name)
            elif path is not None and (path / "Cargo.toml").is_file() and path not in seen:
                seen.add(path)
                pending.append(path / "Cargo.toml")
    return found


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
    pyproject = tomllib.loads((root / "pyproject.toml").read_text())
    allowed = allowed_dependencies(pyproject)
    violations = [
        *check_cpp(root, allowed),
        *check_cpp_internal(root, internal_orders(pyproject)),
        *check_rust(root, allowed),
    ]
    for violation in violations:
        print(f"{violation.location}: {violation.message}")
    if violations:
        print(f"{len(violations)} layering violation(s)")
        return 1
    print("C++ and Rust layering matches pyproject.toml")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
