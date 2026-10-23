"""Generate the golden bundles the run viewer's tests read: the room's, found, merge, revisit.

``bundle/``, ``found_bundle/``, ``merge_bundle/`` and ``revisit_bundle/``.

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

``merge_bundle/`` and ``revisit_bundle/`` are the merge room and the revisit room
(:func:`tests.orchestration.datasets.merge_frames`,
:func:`~tests.orchestration.datasets.revisit_frames`) as TUM RGB-D folders, run whole by the
C++ program at their settings and baked: the first's second map is merged into its first, so
every frame's record names the map it ends in, map 0; the second's first map is taken up again,
so map 0's frames and poses come in two stretches with map 1's between. The page's tests
choose the map shown on both (B4's review panel, A1). These two leave out their frames' JPEGs,
the chunk files (:data:`JPEG_CHUNKS`, about 5.5 MB for the two), which no test reads: the
repository is public, so a fixture holds only what its tests need. Their ``images/spans.npy``
stays, as the page's loader reads it with the header (``viewer/src/bundle.js``'s
``loadBundle``), and the layout tests compare them without the chunks.

Each bundle is baked into a temporary folder and each of its files copied over the fixture's by
name: nothing is deleted, so a file the layout no longer has stays until removed by hand (the
layout tests name it).
"""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

from slam_it import backends
from slam_it.edge.options import Options
from slam_it.orchestration import bundles, vo
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS
from tests.orchestration.datasets import (
    FOUND_AT,
    FOUND_BLANKS,
    FOUND_OPTIONS,
    MERGE_OPTIONS,
    REVISIT_OPTIONS,
    found_frames,
    loop_frames,
    merge_frames,
    revisit_frames,
    write_room,
)

HERE = Path(__file__).resolve().parent
FOLDER = HERE / "bundle"
FOUND_FOLDER = HERE / "found_bundle"
MERGE_FOLDER = HERE / "merge_bundle"
REVISIT_FOLDER = HERE / "revisit_bundle"

NATIVE_FRAMES = 6
"""The frames the C++ and Rust programs run."""

PYTHON_FRAMES = 4
"""The frames Python runs."""

JPEG_CHUNKS = "images/chunk_*.bin"
"""A bundle's JPEG chunk files, as a pattern of their paths in it: ``merge_bundle/`` and
``revisit_bundle/`` leave them out (the module docstring)."""

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


def loop_room_run(sequence: Path, out: Path, options: Options) -> Path:
    """Run a loop room's TUM RGB-D folder whole with the C++ program.

    Parameters
    ----------
    sequence : pathlib.Path
        The room's TUM-format folder: the merge room's, or the revisit room's.
    out : pathlib.Path
        Where the run's folder goes.
    options : slam_it.edge.options.Options
        The room's settings (``MERGE_OPTIONS``, ``REVISIT_OPTIONS``).

    Returns
    -------
    pathlib.Path
        The run's folder.
    """
    folder = out / "cpp"
    vo.run("tum", sequence, "rgbd", folder, backend="cpp", options=options)
    return folder


def _copy(baked: Path, fixture: Path, *, chunks: bool = True) -> None:
    """Copy a baked bundle's files over a fixture's, by name; its JPEG chunks only if `chunks`."""
    for path in sorted(p for p in baked.rglob("*") if p.is_file()):
        name = path.relative_to(baked)
        if not chunks and name.match(JPEG_CHUNKS):
            continue
        target = fixture / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)


def main() -> None:
    """Bake every golden bundle: the room's, the found room's, the merge room's, the revisit's."""
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
        loop = loop_frames()
        for name, frames, options, fixture in (
            ("merge", merge_frames(loop), MERGE_OPTIONS, MERGE_FOLDER),
            ("revisit", revisit_frames(loop), REVISIT_OPTIONS, REVISIT_FOLDER),
        ):
            folders = write_room(base / name, frames)
            run = loop_room_run(folders.tum, base / f"{name}_runs", options)
            bundles.bake([run], folders.tum, base / f"{name}_bundle")
            _copy(base / f"{name}_bundle", fixture, chunks=False)
    print(f"wrote {FOLDER}, {FOUND_FOLDER}, {MERGE_FOLDER} and {REVISIT_FOLDER}")


if __name__ == "__main__":
    main()
