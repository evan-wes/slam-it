"""The room as dataset folders (:mod:`tests.orchestration.datasets`), and its runs.

The room is rendered once per session and written whole and with its gap; :func:`room_runs`
and :func:`gap_runs` run them through :func:`slam_it_all.orchestration.vo.run`, each case once,
recording their features.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from slam_it_all import backends
from slam_it_all.edge.options import Options
from slam_it_all.eval import runs
from slam_it_all.orchestration import vo
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


def require_program(name: str) -> str:
    """Return backend `name` if its native program is built; otherwise skip, or fail.

    The Rust program is not rebuilt on import (the C++ one is): one older than a file it is
    built from (:class:`slam_it_all.backends.StaleProgramError`) fails, since its runs would not be
    of the code under test.

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
            backends.program(name, vo.PROGRAM)
        except backends.StaleProgramError as error:
            pytest.fail(str(error))
        except backends.BackendNotBuiltError as error:
            if name in REQUIRED_BACKENDS:
                pytest.fail(f"{error}, but SLAM_IT_ALL_REQUIRE_BACKENDS requires it")
            pytest.skip(str(error))
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


class RoomRun:
    """Runs a room from a dataset's folder in a mode with a backend, each case once.

    Each run records its features, which :meth:`features` reads.

    Parameters
    ----------
    folders : tests.orchestration.datasets.Room
        The room's folders.
    options : slam_it_all.edge.options.Options
        The settings.
    out : pathlib.Path
        Where the runs' folders go.
    """

    def __init__(self, folders: Room, options: Options, out: Path) -> None:
        """Hold the room and the settings; nothing runs yet."""
        self._folders = folders
        self._options = options
        self._out = out
        self._done: dict[tuple[str, str, str], runs.Run] = {}

    def __call__(self, dataset: str, mode: str, backend: str) -> runs.Run:
        """Return a case's run, run the first time it is asked for."""
        key = (dataset, mode, require_program(backend))
        if key not in self._done:
            folder = self._folders.tum if dataset == "tum" else self._folders.euroc
            self._done[key] = vo.run(
                dataset, folder, mode, self._folder(key), backend=backend, options=self._options,
                record_features=True,
            )  # fmt: skip
        return self._done[key]

    def features(self, dataset: str, mode: str, backend: str) -> runs.Features:
        """Return a case's feature records."""
        return runs.read_features(self.folder(dataset, mode, backend), self(dataset, mode, backend))

    def folder(self, dataset: str, mode: str, backend: str) -> Path:
        """Return a case's run folder, run the first time it is asked for."""
        self(dataset, mode, backend)
        return self._folder((dataset, mode, backend))

    def sequence(self, dataset: str) -> Path:
        """Return the folder a dataset's runs read."""
        return self._folders.tum if dataset == "tum" else self._folders.euroc

    def _folder(self, key: tuple[str, str, str]) -> Path:
        """Return a case's run folder."""
        return self._out / "-".join(key)


@pytest.fixture(scope="session")
def room_runs(room_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the room, each (dataset, mode, backend) once per session."""
    return RoomRun(room_folders, OPTIONS, tmp_path_factory.mktemp("runs"))


@pytest.fixture(scope="session")
def gap_runs(gap_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the gap room with :data:`GAP_OPTIONS`, each case once per session."""
    return RoomRun(gap_folders, GAP_OPTIONS, tmp_path_factory.mktemp("gap_runs"))


@pytest.fixture(scope="session")
def moved_runs(gap_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the gap room with :data:`MOVED_OPTIONS`, each case once per session."""
    return RoomRun(gap_folders, MOVED_OPTIONS, tmp_path_factory.mktemp("moved_runs"))
