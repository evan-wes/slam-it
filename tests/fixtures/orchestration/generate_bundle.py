"""Generate ``bundle/`` and ``found_bundle/``: the golden bundles the run viewer's tests read.

Run from the repository root after changing the bundle's layout (the fixture is committed)::

    python -m tests.fixtures.orchestration.generate_bundle

The room (:mod:`tests.orchestration.datasets`) as a TUM RGB-D folder, run for its first
:data:`NATIVE_FRAMES` frames by the C++ and Rust programs and its first :data:`PYTHON_FRAMES`
by Python, each recording its features, then baked
(:func:`slam_it.orchestration.bundles.bake`): Python covers fewer frames, so the bundle
holds backends of different spans. The Python tests hold the bake to its layout
(``tests/orchestration/test_bundles.py``); the page's tests read its values
(``viewer/tests/``). Its timings and peak memories are those of the runs that made it.

``found_bundle/`` is the found room (:func:`tests.orchestration.datasets.found_frames`) as a
TUM RGB-D folder, run by Python with :data:`~tests.orchestration.datasets.FOUND_OPTIONS` for
its first :data:`FOUND_FRAMES` frames and baked: its track fails on the blank frames and is
relocalised in map 0, so the page's tests read a relocalised frame as a bake writes it.

Each bundle is baked into a temporary folder and each of its files copied over the fixture's by
name: nothing is deleted, so a file the layout no longer has stays until removed by hand (the
layout tests name it).
"""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

from slam_it import backends
from slam_it.orchestration import bundles, vo
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS
from tests.orchestration.datasets import (
    FOUND_AT,
    FOUND_BLANKS,
    FOUND_OPTIONS,
    found_frames,
    write_room,
)

HERE = Path(__file__).resolve().parent
FOLDER = HERE / "bundle"
FOUND_FOLDER = HERE / "found_bundle"

NATIVE_FRAMES = 6
"""The frames the C++ and Rust programs run."""

PYTHON_FRAMES = 4
"""The frames Python runs."""

FOUND_FRAMES = FOUND_AT + FOUND_BLANKS + 2
"""The found room's frames the found bundle holds: to two past the one relocalised."""


def short_runs(sequence: Path, out: Path) -> list[Path]:
    """Run the room's TUM RGB-D folder briefly with every backend, recording features.

    Parameters
    ----------
    sequence : pathlib.Path
        The room's TUM-format folder.
    out : pathlib.Path
        Where the runs' folders go.

    Returns
    -------
    list of pathlib.Path
        The runs' folders, one per backend.
    """
    folders = []
    for backend in backends.BACKENDS:
        count = PYTHON_FRAMES if backend == "python" else NATIVE_FRAMES
        folder = out / backend
        vo.run(
            "tum", sequence, "rgbd", folder, backend=backend, options=OPTIONS, count=count,
            record_features=True,
        )  # fmt: skip
        folders.append(folder)
    return folders


def found_run(sequence: Path, out: Path) -> Path:
    """Run the found room's TUM RGB-D folder with Python, for its first :data:`FOUND_FRAMES`.

    Parameters
    ----------
    sequence : pathlib.Path
        The found room's TUM-format folder.
    out : pathlib.Path
        Where the run's folder goes.

    Returns
    -------
    pathlib.Path
        The run's folder.
    """
    folder = out / "python"
    vo.run("tum", sequence, "rgbd", folder, options=FOUND_OPTIONS, count=FOUND_FRAMES)
    return folder


def _copy(baked: Path, fixture: Path) -> None:
    """Copy a baked bundle's files over a fixture's, by name."""
    for path in sorted(p for p in baked.rglob("*") if p.is_file()):
        target = fixture / path.relative_to(baked)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)


def main() -> None:
    """Bake the golden bundle, the room briefly with every backend, and the found bundle."""
    with tempfile.TemporaryDirectory() as scratch:
        base = Path(scratch)
        rendered = room.sequence(FRAMES, stereo=True)
        folders = write_room(base / "room", rendered)
        bundles.bake(short_runs(folders.tum, base / "runs"), folders.tum, base / "bundle")
        _copy(base / "bundle", FOLDER)
        found = write_room(base / "found", found_frames(rendered))
        baked = base / "found_bundle"
        bundles.bake([found_run(found.tum, base / "found_runs")], found.tum, baked)
        _copy(baked, FOUND_FOLDER)
    print(f"wrote {FOLDER} and {FOUND_FOLDER}")


if __name__ == "__main__":
    main()
