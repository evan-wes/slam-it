"""The room as dataset folders (:mod:`tests.orchestration.datasets`), and its runs.

The room is rendered once per session and written whole and with its gap; :func:`room_runs`
and :func:`gap_runs` run them through :func:`slamlab.orchestration.vo.run`, each case once.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest

from slamlab import backends
from slamlab.edge.options import Options
from slamlab.eval import runs
from slamlab.orchestration import vo
from tests.conftest import REQUIRED_BACKENDS, require_backend
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS
from tests.orchestration.datasets import (
    GAP_OPTIONS,
    MOVED_OPTIONS,
    Room,
    gap_frames,
    write_room,
)

RUST_PROGRAM_CRATES = (
    "slamlab-foundations",
    "slamlab-edge",
    "slamlab-eval",
    "slamlab-orchestration",
)
"""The crates the Rust program is built from: a source newer than it means it is stale."""


def _newer_source(program: Path) -> Path | None:
    """Return a Rust source of the program newer than the program, if any."""
    built = program.stat().st_mtime
    root = Path(__file__).resolve().parents[2] / "rust"
    for crate in RUST_PROGRAM_CRATES:
        for source in (root / crate / "src").rglob("*.rs"):
            if source.stat().st_mtime > built:
                return source
    return None


def require_program(name: str) -> str:
    """Return backend `name` if its native program is built; otherwise skip, or fail.

    The Rust program is not rebuilt on import (the C++ one is), so a Rust program older than
    one of its crates' sources fails: its runs would not be of the code under test.

    Parameters
    ----------
    name : str
        The backend.

    Returns
    -------
    str
        `name`.
    """
    require_backend(name)
    if name != "python":
        try:
            program = backends.program(name, vo.PROGRAM)
        except backends.BackendNotBuiltError as error:
            if name in REQUIRED_BACKENDS:
                pytest.fail(f"{error}, but SLAMLAB_REQUIRE_BACKENDS requires it")
            pytest.skip(str(error))
        newer = _newer_source(program) if name == "rust" else None
        if newer is not None:
            pytest.fail(
                f"{program} is older than {newer}: rebuild it (cargo build --release "
                "--manifest-path rust/Cargo.toml --target-dir rust/target -p slamlab-orchestration)"
            )
    return name


@pytest.fixture(scope="session")
def rendered_room() -> list[room.Frame]:
    """Render the room with its right views, once per session."""
    return room.sequence(FRAMES, stereo=True)


@pytest.fixture(scope="session")
def room_folders(rendered_room: list[room.Frame], tmp_path_factory: pytest.TempPathFactory) -> Room:
    """Write the room as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("room"), rendered_room)


@pytest.fixture(scope="session")
def gap_folders(rendered_room: list[room.Frame], tmp_path_factory: pytest.TempPathFactory) -> Room:
    """Write the gap room (:func:`gap_frames`) as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("gap"), gap_frames(rendered_room))


RoomRun = Callable[[str, str, str], runs.Run]
"""Runs a room from a dataset's folder in a mode with a backend."""


def _runs(folders: Room, options: Options, out: Path) -> RoomRun:
    """Return a runner of the folders with the settings, each case once."""
    done: dict[tuple[str, str, str], runs.Run] = {}

    def run(dataset: str, mode: str, backend: str) -> runs.Run:
        key = (dataset, mode, require_program(backend))
        if key not in done:
            folder = folders.tum if dataset == "tum" else folders.euroc
            done[key] = vo.run(
                dataset, folder, mode, out / "-".join(key), backend=backend, options=options
            )
        return done[key]

    return run


@pytest.fixture(scope="session")
def room_runs(room_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the room, each (dataset, mode, backend) once per session."""
    return _runs(room_folders, OPTIONS, tmp_path_factory.mktemp("runs"))


@pytest.fixture(scope="session")
def gap_runs(gap_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the gap room with :data:`GAP_OPTIONS`, each case once per session."""
    return _runs(gap_folders, GAP_OPTIONS, tmp_path_factory.mktemp("gap_runs"))


@pytest.fixture(scope="session")
def moved_runs(gap_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the gap room with :data:`MOVED_OPTIONS`, each case once per session."""
    return _runs(gap_folders, MOVED_OPTIONS, tmp_path_factory.mktemp("moved_runs"))
