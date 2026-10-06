"""The room as dataset folders, and its runs through every backend.

The room (:mod:`tests.edge.room`) is rendered once with its right views and written as a
TUM-format folder (gray PNGs, 16-bit depth PNGs, ``rgb.txt``, ``depth.txt`` and
``camera.txt``) and as a EuRoC sequence (``mav0/cam0`` and ``cam1``: PNGs, ``data.csv`` and
``sensor.yaml``, the right camera :data:`tests.edge.room.BASELINE` to the left one's right,
no distortion). :func:`room_runs` runs them through :func:`slamlab.orchestration.vo.run`,
each case once per session.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray
from PIL import Image

from slamlab import backends
from slamlab.eval import runs, tum
from slamlab.orchestration import vo
from tests.conftest import REQUIRED_BACKENDS, require_backend
from tests.edge import room
from tests.edge.conftest import FRAMES, OPTIONS

SENSOR = """sensor_type: camera
comment: the room's {name}
T_BS:
  cols: 4
  rows: 4
  data: [{pose}]
rate_hz: 30
resolution: [{cols}, {rows}]
camera_model: pinhole
intrinsics: [{fx}, {fy}, {cx}, {cy}]
distortion_model: radial-tangential
distortion_coefficients: [0.0, 0.0, 0.0, 0.0]
"""
"""A camera's ``sensor.yaml``, in EuRoC's layout."""


@dataclass(frozen=True)
class Room:
    """The room's folders and its truth."""

    tum: Path
    euroc: Path
    timestamps: NDArray[np.float64]
    poses: NDArray[np.float64]


def require_program(name: str) -> str:
    """Return backend `name` if its native program is built; otherwise skip, or fail.

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
        except backends.BackendNotBuiltError as error:
            if name in REQUIRED_BACKENDS:
                pytest.fail(f"{error}, but SLAMLAB_REQUIRE_BACKENDS requires it")
            pytest.skip(str(error))
    return name


def _png(path: Path, values: NDArray[np.uint8] | NDArray[np.uint16]) -> None:
    """Write a gray PNG, 8- or 16-bit as the array is."""
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(values).save(path)


def _write_tum(folder: Path, rendered: list[room.Frame]) -> None:
    """Write the room as a TUM-format folder that describes its own camera."""
    camera = room.SMALL
    rgb, depth = ["# timestamp filename"], ["# timestamp filename"]
    for f in rendered:
        stamp = f"{f.timestamp:.6f}"
        _png(folder / "rgb" / f"{stamp}.png", f.gray)
        scaled = np.clip(np.round(f.depth * tum.DEPTH_SCALE), 0, np.iinfo(np.uint16).max)
        _png(folder / "depth" / f"{stamp}.png", scaled.astype(np.uint16))
        rgb.append(f"{stamp} rgb/{stamp}.png")
        depth.append(f"{stamp} depth/{stamp}.png")
    (folder / "rgb.txt").write_text("\n".join(rgb) + "\n")
    (folder / "depth.txt").write_text("\n".join(depth) + "\n")
    (folder / "camera.txt").write_text(
        f"model pinhole\nintrinsics {camera.fx} {camera.fy} {camera.cx} {camera.cy}\n"
        f"size {camera.width} {camera.height}\ndepth_scale {tum.DEPTH_SCALE}\n"
    )


def _write_euroc(folder: Path, rendered: list[room.Frame]) -> None:
    """Write the room as a EuRoC sequence: the left camera is the body."""
    camera = room.SMALL
    for name, offset in (("cam0", 0.0), ("cam1", room.BASELINE)):
        pose = np.eye(4)
        pose[0, 3] = offset
        root = folder / "mav0" / name
        rows = ["#timestamp [ns],filename"]
        for f in rendered:
            stamp = round(f.timestamp * 1e9)
            view = f.gray if name == "cam0" else f.right
            assert view is not None
            _png(root / "data" / f"{stamp}.png", view)
            rows.append(f"{stamp},{stamp}.png")
        (root / "data.csv").write_text("\n".join(rows) + "\n")
        (root / "sensor.yaml").write_text(
            SENSOR.format(
                name=name,
                pose=", ".join(repr(float(v)) for v in pose.ravel()),
                cols=camera.width,
                rows=camera.height,
                fx=camera.fx,
                fy=camera.fy,
                cx=camera.cx,
                cy=camera.cy,
            )
        )


@pytest.fixture(scope="session")
def room_folders(tmp_path_factory: pytest.TempPathFactory) -> Room:
    """Render the room with its right views and write it as both datasets' folders."""
    rendered = room.sequence(FRAMES, stereo=True)
    base = tmp_path_factory.mktemp("room")
    tum_folder, euroc_folder = base / "room_tum", base / "room_euroc"
    _write_tum(tum_folder, rendered)
    _write_euroc(euroc_folder, rendered)
    return Room(
        tum_folder,
        euroc_folder,
        np.array([f.timestamp for f in rendered]),
        np.array([f.T_world_cam for f in rendered]),
    )


RoomRun = Callable[[str, str, str], runs.Run]
"""Runs the room from a dataset's folder in a mode with a backend."""


@pytest.fixture(scope="session")
def room_runs(room_folders: Room, tmp_path_factory: pytest.TempPathFactory) -> RoomRun:
    """Run the room, each (dataset, mode, backend) once per session."""
    done: dict[tuple[str, str, str], runs.Run] = {}
    out = tmp_path_factory.mktemp("runs")

    def run(dataset: str, mode: str, backend: str) -> runs.Run:
        key = (dataset, mode, require_program(backend))
        if key not in done:
            folder = room_folders.tum if dataset == "tum" else room_folders.euroc
            done[key] = vo.run(
                dataset, folder, mode, out / "-".join(key), backend=backend, options=OPTIONS
            )
        return done[key]

    return run
