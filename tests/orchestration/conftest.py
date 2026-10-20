"""The room as dataset folders (:mod:`tests.orchestration.datasets`), and its runs.

The room is rendered once per session and written whole, with its gap, lost (frames blanked
to lose the track), found (blanked, then back at the start) and the window room (found, then
lost by the time); the loop room is rendered apart, once, and written whole and blinded as the
merge room and the revisit room (:func:`loop_folders`, :func:`merge_folders`,
:func:`revisit_folders`). :func:`room_runs`, :func:`gap_runs`, :func:`lost_runs`,
:func:`kept_runs`, :func:`found_runs`, :func:`window_runs`, :func:`loop_runs`, :func:`merge_runs`
and :func:`revisit_runs` run them through :func:`slam_it.orchestration.vo.run`, each case once,
recording their features, and each again on views when asked (:meth:`RoomRun.viewed`);
:func:`loop_threaded_runs` runs the loop room threaded.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from slam_it import backends
from slam_it.edge.options import Options
from slam_it.eval import runs
from slam_it.orchestration import vo
from tests.conftest import REQUIRED_BACKENDS, require_backend
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS
from tests.orchestration.datasets import (
    FOUND_OPTIONS,
    GAP_OPTIONS,
    KEPT_OPTIONS,
    MERGE_OPTIONS,
    MOVED_OPTIONS,
    REVISIT_OPTIONS,
    WINDOW_OPTIONS,
    Room,
    found_frames,
    gap_frames,
    loop_frames,
    lost_frames,
    merge_frames,
    revisit_frames,
    window_frames,
    write_room,
)


def require_program(name: str) -> str:
    """Return backend `name` if its native program is built; otherwise skip, or fail.

    The Rust program is not rebuilt on import (the C++ one is): one older than a file it is
    built from (:class:`slam_it.backends.StaleProgramError`) fails, since its runs would not be
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
                pytest.fail(f"{error}, but SLAM_IT_REQUIRE_BACKENDS requires it")
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


@pytest.fixture(scope="session")
def lost_folders(rendered_room: list[room.Frame], tmp_path_factory: pytest.TempPathFactory) -> Room:
    """Write the lost room (:func:`lost_frames`) as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("lost"), lost_frames(rendered_room))


@pytest.fixture(scope="session")
def found_folders(
    rendered_room: list[room.Frame], tmp_path_factory: pytest.TempPathFactory
) -> Room:
    """Write the found room (:func:`found_frames`) as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("found"), found_frames(rendered_room))


@pytest.fixture(scope="session")
def window_folders(
    rendered_room: list[room.Frame], tmp_path_factory: pytest.TempPathFactory
) -> Room:
    """Write the window room (:func:`window_frames`) as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("window"), window_frames(rendered_room))


@pytest.fixture(scope="session")
def rendered_loop() -> list[room.Frame]:
    """Render the loop room with its right views, once per session."""
    return loop_frames()


@pytest.fixture(scope="session")
def loop_folders(rendered_loop: list[room.Frame], tmp_path_factory: pytest.TempPathFactory) -> Room:
    """Write the loop room as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("loop"), rendered_loop)


@pytest.fixture(scope="session")
def merge_folders(
    rendered_loop: list[room.Frame], tmp_path_factory: pytest.TempPathFactory
) -> Room:
    """Write the merge room (:func:`merge_frames`) as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("merge"), merge_frames(rendered_loop))


@pytest.fixture(scope="session")
def revisit_folders(
    rendered_loop: list[room.Frame], tmp_path_factory: pytest.TempPathFactory
) -> Room:
    """Write the revisit room (:func:`revisit_frames`) as both datasets' folders."""
    return write_room(tmp_path_factory.mktemp("revisit"), revisit_frames(rendered_loop))


class RoomRun:
    """Runs a room from a dataset's folder in a mode with a backend, each case once.

    Each run records its features, which :meth:`features` reads.

    Parameters
    ----------
    folders : tests.orchestration.datasets.Room
        The room's folders.
    options : slam_it.edge.options.Options
        The settings.
    out : pathlib.Path
        Where the runs' folders go.
    threaded : bool, optional
        Whether the steps run in threads of their own (default: in order).
    """

    def __init__(
        self, folders: Room, options: Options, out: Path, *, threaded: bool = False
    ) -> None:
        """Hold the room and the settings; nothing runs yet."""
        self._folders = folders
        self._options = options
        self._out = out
        self._threaded = threaded
        self._done: dict[tuple[str, str, str, bool], runs.Run] = {}

    def __call__(self, dataset: str, mode: str, backend: str) -> runs.Run:
        """Return a case's run, run the first time it is asked for."""
        return self._run((dataset, mode, require_program(backend)), views=False)

    def viewed(self, dataset: str, mode: str, backend: str) -> runs.Run:
        """Return a case's run in order, each frame tracked on a view, run the first time asked.

        The view a writer would publish then (``views``).
        """
        return self._run((dataset, mode, require_program(backend)), views=True)

    def features(self, dataset: str, mode: str, backend: str) -> runs.Features:
        """Return a case's feature records."""
        return runs.read_features(self.folder(dataset, mode, backend), self(dataset, mode, backend))

    def viewed_features(self, dataset: str, mode: str, backend: str) -> runs.Features:
        """Return a case's feature records, its frames tracked on views."""
        run = self.viewed(dataset, mode, backend)
        return runs.read_features(self._folder((dataset, mode, backend), views=True), run)

    def _run(self, key: tuple[str, str, str], *, views: bool) -> runs.Run:
        """Return a case's run, on views or not, run the first time it is asked for."""
        dataset, mode, backend = key
        if (*key, views) not in self._done:
            self._done[(*key, views)] = vo.run(
                dataset, self.sequence(dataset), mode, self._folder(key, views=views),
                backend=backend, options=self._options, record_features=True,
                threaded=self._threaded, views=views,
            )  # fmt: skip
        return self._done[(*key, views)]

    def folder(self, dataset: str, mode: str, backend: str) -> Path:
        """Return a case's run folder, run the first time it is asked for."""
        self(dataset, mode, backend)
        return self._folder((dataset, mode, backend))

    def sequence(self, dataset: str) -> Path:
        """Return the folder a dataset's runs read."""
        return self._folders.tum if dataset == "tum" else self._folders.euroc

    def _folder(self, key: tuple[str, str, str], *, views: bool = False) -> Path:
        """Return a case's run folder, on views or not."""
        return self._out / "-".join((*key, "views") if views else key)


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


@pytest.fixture(scope="session")
def lost_runs(lost_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the lost room at the room's settings: its first map is reset."""
    return RoomRun(lost_folders, OPTIONS, tmp_path_factory.mktemp("lost_runs"))


@pytest.fixture(scope="session")
def found_runs(found_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the found room with :data:`FOUND_OPTIONS`: the camera, back at the start, relocalises."""
    return RoomRun(found_folders, FOUND_OPTIONS, tmp_path_factory.mktemp("found_runs"))


@pytest.fixture(scope="session")
def window_runs(window_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the window room with :data:`WINDOW_OPTIONS`: found again, then lost by the time."""
    return RoomRun(window_folders, WINDOW_OPTIONS, tmp_path_factory.mktemp("window_runs"))


@pytest.fixture(scope="session")
def kept_runs(lost_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the lost room with :data:`KEPT_OPTIONS`: its first map is kept."""
    return RoomRun(lost_folders, KEPT_OPTIONS, tmp_path_factory.mktemp("kept_runs"))


@pytest.fixture(scope="session")
def loop_runs(loop_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the loop room at the room's settings: a loop closes in every mode."""
    return RoomRun(loop_folders, OPTIONS, tmp_path_factory.mktemp("loop_runs"))


@pytest.fixture(scope="session")
def loop_threaded_runs(loop_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the loop room threaded, its frames fed at their rate by the native programs."""
    out = tmp_path_factory.mktemp("loop_threaded_runs")
    return RoomRun(loop_folders, OPTIONS, out, threaded=True)


@pytest.fixture(scope="session")
def merge_runs(merge_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the merge room with :data:`MERGE_OPTIONS`: its two maps merge."""
    return RoomRun(merge_folders, MERGE_OPTIONS, tmp_path_factory.mktemp("merge_runs"))


@pytest.fixture(scope="session")
def revisit_runs(revisit_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the revisit room with :data:`REVISIT_OPTIONS`: tracking takes its first map up again."""
    return RoomRun(revisit_folders, REVISIT_OPTIONS, tmp_path_factory.mktemp("revisit_runs"))
