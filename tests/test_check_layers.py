"""Tests for scripts/check_layers.py: C++ and Rust layering checked against pyproject.toml."""

from __future__ import annotations

import importlib.util
import sys
import tomllib
from pathlib import Path
from types import ModuleType

import pytest

REPO = Path(__file__).resolve().parent.parent


def _load_script() -> ModuleType:
    """Import scripts/check_layers.py as a module."""
    spec = importlib.util.spec_from_file_location("check_layers", REPO / "scripts/check_layers.py")
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Registered before running it: @dataclass looks its module up in sys.modules.
    sys.modules["check_layers"] = module
    spec.loader.exec_module(module)
    return module


check_layers = _load_script()
ALLOWED = check_layers.allowed_dependencies(tomllib.loads((REPO / "pyproject.toml").read_text()))


def test_allowed_dependencies_follow_the_ladder() -> None:
    """Layers, sibling independence and forbidden contracts all shape what a rung may use."""
    assert ALLOWED["foundations"] == {"backends"}
    assert "foundations" in ALLOWED["edge"]
    assert "dense" not in ALLOWED["edge"]  # siblings are independent
    assert ALLOWED["vi"] >= {"edge", "foundations"}
    assert not ALLOWED["vi"] & {"dense", "classic"}  # forbidden contract
    assert "dense" in ALLOWED["learned"]
    assert not ALLOWED["learned"] & {"edge", "classic"}  # forbidden contract


def test_this_repository_passes() -> None:
    """The repository's own C++ and Rust trees satisfy the contracts."""
    assert check_layers.main([str(REPO)]) == 0


def _write(root: Path, relative: str, text: str) -> None:
    """Write a file under a fake repository root, creating its directories."""
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


@pytest.fixture
def fake_repo(tmp_path: Path) -> Path:
    """Return a fake repository root holding a copy of the real pyproject.toml."""
    _write(tmp_path, "pyproject.toml", (REPO / "pyproject.toml").read_text())
    return tmp_path


def test_cross_rung_include_is_reported(fake_repo: Path) -> None:
    """Foundations including an edge header breaks the layering."""
    _write(fake_repo, "cpp/src/foundations/x.cpp", '#include "slamlab/edge/vo.hpp"\n')
    found = list(check_layers.check_cpp(fake_repo, ALLOWED))
    assert [v.location for v in found] == ["cpp/src/foundations/x.cpp:1"]
    assert "may not include from rung 'edge'" in found[0].message


def test_downward_include_is_allowed(fake_repo: Path) -> None:
    """Edge including a foundations header is fine."""
    _write(fake_repo, "cpp/src/edge/vo.cpp", '#include "slamlab/foundations/so3.hpp"\n')
    assert list(check_layers.check_cpp(fake_repo, ALLOWED)) == []


def test_unknown_rung_directory_is_reported(fake_repo: Path) -> None:
    """A C++ directory that is not a rung in the layers contract is reported."""
    _write(fake_repo, "cpp/src/misc/x.cpp", "int x;\n")
    found = list(check_layers.check_cpp(fake_repo, ALLOWED))
    assert "not a rung" in found[0].message


def test_cmake_link_dependency_is_checked(fake_repo: Path) -> None:
    """A rung linking a sibling's library is reported."""
    _write(fake_repo, "CMakeLists.txt", "slamlab_add_rung(edge DEPENDS slamlab_dense)\n")
    found = list(check_layers.check_cpp(fake_repo, ALLOWED))
    assert "may not link slamlab_dense" in found[0].message


@pytest.mark.parametrize(
    "include", ['#include "slamlab/edge/vo.hpp"', "#include <slamlab/edge/vo.hpp>"]
)
def test_both_include_forms_are_checked(fake_repo: Path, include: str) -> None:
    """Quoted and angle-bracket includes are both checked."""
    _write(fake_repo, "cpp/src/foundations/x.cpp", include + "\n")
    found = list(check_layers.check_cpp(fake_repo, ALLOWED))
    assert len(found) == 1


def test_source_outside_a_rung_is_reported(fake_repo: Path) -> None:
    """A header directly in cpp/include/slamlab/ belongs to no rung; the bindings root may."""
    _write(fake_repo, "cpp/include/slamlab/top.hpp", '#include "slamlab/dense/y.hpp"\n')
    _write(fake_repo, "cpp/bindings/module.cpp", '#include "foundations/foundations.hpp"\n')
    found = list(check_layers.check_cpp(fake_repo, ALLOWED))
    assert [v.location for v in found] == ["cpp/include/slamlab/top.hpp"]


def _crate(root: Path, directory: str, name: str, body: str = "") -> None:
    """Write a crate manifest and register it as a workspace member."""
    _write(root, f"rust/{directory}/Cargo.toml", f'[package]\nname = "{name}"\n{body}')
    workspace = root / "rust/Cargo.toml"
    members = (
        []
        if not workspace.exists()
        else tomllib.loads(workspace.read_text())["workspace"]["members"]
    )
    members.append(directory)
    listed = ", ".join(f'"{m}"' for m in members)
    workspace.write_text(f"[workspace]\nmembers = [{listed}]\n")


@pytest.mark.parametrize(
    "body",
    [
        '[dependencies]\nslamlab-classic = { path = ".." }\n',
        '[dev-dependencies]\nslamlab-classic = { path = ".." }\n',
        '[build-dependencies]\nslamlab-classic = { path = ".." }\n',
        '[target.x86_64-unknown-linux-gnu.dependencies]\nslamlab-classic = { path = ".." }\n',
        '[dependencies]\nbase = { package = "slamlab-classic", path = ".." }\n',
    ],
    ids=["dependencies", "dev", "build", "target", "renamed"],
)
def test_rust_crate_dependency_is_checked(fake_repo: Path, body: str) -> None:
    """A rung crate depending on a sibling's crate is reported, in every dependency form."""
    _crate(fake_repo, "slamlab-edge", "slamlab-edge", body)
    found = list(check_layers.check_rust(fake_repo, ALLOWED))
    assert len(found) == 1
    assert "may not depend on crate 'slamlab-classic'" in found[0].message


def test_bindings_crate_is_exempt_and_other_names_are_reported(fake_repo: Path) -> None:
    """The bindings crate may use every rung; a member not named slamlab-<rung> is reported."""
    _crate(
        fake_repo, "slamlab-py", "slamlab-py", '[dependencies]\nslamlab-edge = { path = ".." }\n'
    )
    _crate(fake_repo, "geometry", "geometry")
    found = list(check_layers.check_rust(fake_repo, ALLOWED))
    assert [v.message for v in found] == ["crate 'geometry' is not named slamlab-<rung>"]


def test_helper_crate_cannot_relay_a_forbidden_rung(fake_repo: Path) -> None:
    """An unlisted helper crate (an implicit member) is reported, and its deps are followed."""
    _crate(
        fake_repo,
        "slamlab-edge",
        "slamlab-edge",
        '[dependencies]\nhelper = { path = "../helper" }\n',
    )
    _write(
        fake_repo,
        "rust/helper/Cargo.toml",
        '[package]\nname = "helper"\n[dependencies]\nslamlab-classic = { path = "../x" }\n',
    )
    messages = sorted(v.message for v in check_layers.check_rust(fake_repo, ALLOWED))
    assert messages == [
        "crate 'helper' is not named slamlab-<rung>",
        "rung 'edge' may not depend on crate 'slamlab-classic'",
    ]


def test_renamed_workspace_dependency_is_resolved(fake_repo: Path) -> None:
    """A `workspace = true` entry renamed in [workspace.dependencies] is resolved."""
    _crate(
        fake_repo, "slamlab-edge", "slamlab-edge", "[dependencies]\nbase = { workspace = true }\n"
    )
    workspace = fake_repo / "rust/Cargo.toml"
    workspace.write_text(
        workspace.read_text()
        + '[workspace.dependencies]\nbase = { package = "slamlab-classic", path = "x" }\n'
    )
    found = list(check_layers.check_rust(fake_repo, ALLOWED))
    assert [v.message for v in found] == ["rung 'edge' may not depend on crate 'slamlab-classic'"]


ORDERS = check_layers.internal_orders(tomllib.loads((REPO / "pyproject.toml").read_text()))


@pytest.mark.parametrize(
    ("path", "include", "message"),
    [
        (
            "cpp/include/slamlab/foundations/so3.hpp",
            '#include "slamlab/foundations/se3.hpp"',
            "in foundations, 'so3' may not include 'se3'",
        ),
        (
            "cpp/src/foundations/camera.cpp",
            "#include <slamlab/foundations/ransac.hpp>",
            "in foundations, 'camera' may not include 'ransac'",
        ),
        (
            "cpp/src/foundations/ba/schur.cpp",
            '#include "bundle.hpp"',
            "in foundations.ba, 'schur' may not include 'bundle'",
        ),
        (
            "cpp/src/foundations/ba/layout.hpp",
            '#include "schur.hpp"',
            "in foundations.ba, 'layout' may not include 'schur'",
        ),
    ],
)
def test_internal_order_is_checked(fake_repo: Path, path: str, include: str, message: str) -> None:
    """Upward and sibling includes inside a rung break its internal order, in either form."""
    _write(fake_repo, path, include + "\n")
    found = list(check_layers.check_cpp_internal(fake_repo, ORDERS))
    assert [v.message for v in found] == [message]
    assert found[0].location == f"{path}:1"


@pytest.mark.parametrize(
    ("path", "include"),
    [
        ("cpp/include/slamlab/foundations/so3.hpp", '#include "slamlab/foundations/error.hpp"'),
        ("cpp/src/foundations/ba/bundle.cpp", '#include "schur.hpp"'),
        ("cpp/src/foundations/ba.cpp", '#include "ba/bundle.hpp"'),
        ("cpp/src/foundations/ba/layout.hpp", '#include "common.hpp"'),
        ("cpp/src/foundations/ba/schur.hpp", "#include <Eigen/Core>"),
    ],
)
def test_internal_order_allows_downward_includes(fake_repo: Path, path: str, include: str) -> None:
    """Downward includes, the facade, unlisted helpers and system headers all pass.

    ``error.hpp`` is the ``errors`` module; ``ba.cpp`` is the package's facade, above its
    modules; ``common.hpp`` is listed in no contract.
    """
    _write(fake_repo, path, include + "\n")
    assert list(check_layers.check_cpp_internal(fake_repo, ORDERS)) == []


def test_internal_chain_through_an_unlisted_module_is_reported(fake_repo: Path) -> None:
    """An unlisted helper cannot bridge a forbidden include: the chain is followed."""
    _write(fake_repo, "cpp/src/foundations/ba/schur.hpp", '#include "common.hpp"\n')
    _write(fake_repo, "cpp/src/foundations/ba/common.hpp", '#include "bundle.hpp"\n')
    found = list(check_layers.check_cpp_internal(fake_repo, ORDERS))
    assert [v.message for v in found] == [
        "in foundations.ba, 'schur' may not include 'bundle' (via foundations/ba/common)"
    ]
    assert found[0].location == "cpp/src/foundations/ba/schur.hpp:1"


def test_internal_parent_relative_include_resolves(fake_repo: Path) -> None:
    """A ``..`` include resolves to the module it names."""
    _write(fake_repo, "cpp/src/foundations/ba/schur.cpp", '#include "../ba/bundle.hpp"\n')
    found = list(check_layers.check_cpp_internal(fake_repo, ORDERS))
    assert [v.message for v in found] == ["in foundations.ba, 'schur' may not include 'bundle'"]


def test_facade_included_from_inside_is_the_problem_layer(fake_repo: Path) -> None:
    """``ba.hpp`` holds the problem types: any module of ``ba`` may include it."""
    for module in ("layout", "gauge", "bundle"):
        _write(
            fake_repo,
            f"cpp/src/foundations/ba/{module}.hpp",
            '#include "slamlab/foundations/ba.hpp"\n',
        )
    assert list(check_layers.check_cpp_internal(fake_repo, ORDERS)) == []


RUST = "rust/slamlab-foundations/src"


@pytest.mark.parametrize(
    ("path", "code", "message"),
    [
        ("so3.rs", "use crate::se3;", "in foundations, 'so3' may not use 'se3'"),
        (
            "nls.rs",
            "use crate::{camera::Model, se3};",
            "in foundations, 'nls' may not use 'camera'",
        ),
        (
            "ba/schur.rs",
            "use super::bundle::adjust;",
            "in foundations.ba, 'schur' may not use 'bundle'",
        ),
        (
            "ba/schur.rs",
            "use super::{Gauge, layout::{self, Layout}};",
            "in foundations.ba, 'schur' may not use 'layout'",
        ),
        (
            "ba/gauge.rs",
            "let x = crate::ba::bundle::f();",
            "in foundations.ba, 'gauge' may not use 'bundle'",
        ),
    ],
)
def test_rust_internal_order_is_checked(
    fake_repo: Path, path: str, code: str, message: str
) -> None:
    """Upward and sibling ``crate::``/``super::`` paths inside a rung crate break its order."""
    _write(fake_repo, f"{RUST}/ba/mod.rs", "mod bundle;\nmod gauge;\nmod layout;\nmod schur;\n")
    for module in ("ba/bundle", "ba/gauge", "ba/layout", "ba/schur", "camera", "se3", "so3"):
        if not (fake_repo / RUST / f"{module}.rs").exists():
            _write(fake_repo, f"{RUST}/{module}.rs", "")
    _write(fake_repo, f"{RUST}/{path}", code + "\n")
    found = list(check_layers.check_rust_internal(fake_repo, ORDERS))
    assert [v.message for v in found] == [message]
    assert found[0].location == f"{RUST}/{path}:1"


def test_rust_internal_order_allows_downward_paths_facades_comments_and_tests(
    fake_repo: Path,
) -> None:
    """Downward paths, the package's facade items, comments and the test module all pass."""
    _write(fake_repo, f"{RUST}/ba/mod.rs", "mod bundle;\nmod schur;\n")
    _write(fake_repo, f"{RUST}/so3.rs", "")
    _write(
        fake_repo,
        f"{RUST}/ba/bundle.rs",
        "use super::{schur::Blocks, Gauge, Problem};\nuse crate::so3;\n",
    )
    _write(
        fake_repo,
        f"{RUST}/ba/schur.rs",
        "use super::Gauge; // super::bundle in a comment\n"
        "#[cfg(test)]\nmod tests {\n    use super::super::bundle;\n}\n",
    )
    assert list(check_layers.check_rust_internal(fake_repo, ORDERS)) == []


def test_relative_include_into_another_rung_is_reported(fake_repo: Path) -> None:
    """A quoted relative include is resolved to the rung directory it lands in (panel A9)."""
    _write(fake_repo, "cpp/tests/foundations/test_x.cpp", '#include "../../bench/edge/foo.hpp"\n')
    _write(
        fake_repo, "cpp/tests/foundations/test_y.cpp", '#include "../../bench/foundations/ok.hpp"\n'
    )
    found = list(check_layers.check_cpp(fake_repo, ALLOWED))
    assert [v.message for v in found] == ["rung 'foundations' may not include from rung 'edge'"]
    assert found[0].location == "cpp/tests/foundations/test_x.cpp:1"


@pytest.mark.parametrize(
    ("path", "code", "message"),
    [
        ("ba/schur.rs", "use super::*;\nfn f() { bundle::adjust(); }", "glob import"),
        ("ba/schur.rs", "use crate::ba::*;", "glob import"),
        ("ba/mod.rs", "mod bundle;\npub use bundle::adjust;", "re-export in a package facade"),
        (
            "ba/gauge.rs",
            "#[cfg(test)]\nfn helper() {}\nuse super::bundle;",
            "#[cfg(test)] outside the trailing test module",
        ),
    ],
)
def test_rust_constructs_the_check_cannot_follow_are_reported(
    fake_repo: Path, path: str, code: str, message: str
) -> None:
    """A glob import, a facade re-export or a stray ``#[cfg(test)]`` fails loudly (panel A8)."""
    _write(fake_repo, f"{RUST}/ba/bundle.rs", "")
    _write(fake_repo, f"{RUST}/{path}", code + "\n")
    found = [v.message for v in check_layers.check_rust_internal(fake_repo, ORDERS)]
    assert any(message in m for m in found), found


def test_rust_literals_and_comments_neither_hide_nor_invent_edges(fake_repo: Path) -> None:
    """Literals and comments neither hide code nor add edges.

    A ``//`` inside a string does not hide the code after it; paths in comments, strings and
    char literals are not edges.
    """
    _write(fake_repo, f"{RUST}/se3.rs", "")
    _write(
        fake_repo,
        f"{RUST}/so3.rs",
        'const U: &str = "http://x"; use crate::se3;\n'
        "/* crate::se3 /* nested */ */ const Q: char = '\"'; // crate::se3\n"
        'const R: &str = r#"crate::se3"#;\n',
    )
    found = [v.message for v in check_layers.check_rust_internal(fake_repo, ORDERS)]
    assert found == ["in foundations, 'so3' may not use 'se3'"]


def test_two_contracts_over_one_container_are_rejected() -> None:
    """Two layers contracts over the same package would silently replace one another."""
    contract = {"type": "layers", "containers": ["slamlab.foundations"], "layers": ["a", "b"]}
    pyproject = {"tool": {"importlinter": {"contracts": [contract, contract]}}}
    with pytest.raises(ValueError, match="two layers contracts"):
        check_layers.internal_orders(pyproject)
