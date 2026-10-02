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


def test_rust_crate_dependency_is_checked(fake_repo: Path) -> None:
    """A rung crate depending on a sibling's crate is reported; bindings are exempt."""
    _write(
        fake_repo,
        "rust/slamlab-edge/Cargo.toml",
        '[package]\nname = "slamlab-edge"\n[dependencies]\nslamlab-classic = { path = ".." }\n',
    )
    _write(
        fake_repo,
        "rust/slamlab-py/Cargo.toml",
        '[package]\nname = "slamlab-py"\n[dependencies]\nslamlab-edge = { path = ".." }\n',
    )
    found = list(check_layers.check_rust(fake_repo, ALLOWED))
    assert len(found) == 1
    assert "may not depend on crate 'slamlab-classic'" in found[0].message
