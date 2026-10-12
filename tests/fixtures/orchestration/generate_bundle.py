"""Generate ``bundle/``: the golden bundle the run viewer's tests read.

Run from the repository root after changing the bundle's layout (the fixture is committed)::

    python -m tests.fixtures.orchestration.generate_bundle

The room (:mod:`tests.orchestration.datasets`) as a TUM RGB-D folder, run for its first
:data:`NATIVE_FRAMES` frames by the C++ and Rust programs and its first :data:`PYTHON_FRAMES`
by Python, each recording its features, then baked
(:func:`slam_it.orchestration.bundles.bake`): Python covers fewer frames, so the bundle
holds backends of different spans. The Python tests hold the bake to its layout
(``tests/orchestration/test_bundles.py``); the page's tests read its values
(``viewer/tests/``). Its timings and peak memories are those of the runs that made it.

The bundle is baked into a temporary folder and each of its files copied over the fixture's by
name: nothing is deleted, so a file the layout no longer has stays until removed by hand (the
layout test names it).
"""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

from slam_it import backends
from slam_it.orchestration import bundles, vo
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS
from tests.orchestration.datasets import write_room

HERE = Path(__file__).resolve().parent
FOLDER = HERE / "bundle"

NATIVE_FRAMES = 6
"""The frames the C++ and Rust programs run."""

PYTHON_FRAMES = 4
"""The frames Python runs."""


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


def main() -> None:
    """Run the room briefly with every backend and bake the golden bundle."""
    with tempfile.TemporaryDirectory() as scratch:
        base = Path(scratch)
        folders = write_room(base / "room", room.sequence(FRAMES, stereo=True))
        baked = base / "bundle"
        bundles.bake(short_runs(folders.tum, base / "runs"), folders.tum, baked)
        for path in sorted(p for p in baked.rglob("*") if p.is_file()):
            target = FOLDER / path.relative_to(baked)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
    print(f"wrote {FOLDER}")


if __name__ == "__main__":
    main()
