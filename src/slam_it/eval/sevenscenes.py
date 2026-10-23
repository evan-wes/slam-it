"""Microsoft's RGB-D 7-Scenes: a scene's sequence imported to TUM format, monocular.

Shotton et al., "Scene Coordinate Regression Forests for Camera Relocalization in RGB-D
Images", CVPR 2013 (https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/).
Seven indoor scenes recorded with a handheld Kinect at 640x480, each a few sequences
(``seq-01``, ``seq-02``, ...) of 500 to 1000 frames, every frame three files:

- ``frame-XXXXXX.color.png``: the colour image (24-bit RGB);
- ``frame-XXXXXX.depth.png``: the depth in millimetres (16-bit, 65535 where invalid), which
  the import does not read;
- ``frame-XXXXXX.pose.txt``: the camera's pose, camera to world, as a 4x4 homogeneous matrix:
  ``T_world_cam``, from KinectFusion's tracking.

**The camera.** The dataset gives no colour calibration: its page says the RGB and depth cameras
"have not been calibrated", and gives only the depth camera's default intrinsics, which
KinectFusion used: a focal length of 585 pixels and the principal point (320, 240). The colour
images are imported as a pinhole of :data:`INTRINSICS`: the focal length of 525 pixels that the
relocalisation literature uses for 7-Scenes' colour camera (Brachmann and Rother, "Visual Camera
Re-Localization from RGB and RGB-D Images Using DSAC", TPAMI 2021, whose
``datasets/setup_7scenes.py`` writes 525; others rescale the colour images by 585/525 to the
depth camera's), and the principal point at the image's centre, where the page puts the depth
camera's. Depth is not imported: the two cameras are not registered to each other, so an RGB-D
run would pair pixels that do not see the same point. The poses are KinectFusion's, tracked on
depth, so they place the depth camera, a few centimetres from the colour one.

**The clock.** The frames carry no times. The import times them at :data:`FPS`, the Kinect's
rate (the page gives none), from 0: frame ``k`` at ``k / 30`` s, a frame missing from the
numbering leaving its time unused. Every sequence starts at 0, so a run of several sequences in
turn moves each on after the one before (:func:`slam_it.orchestration.vo.session_offsets`).

**The import** (:func:`import_sequence`) writes the TUM-format folder the pipeline already
reads (:mod:`slam_it.eval.tum`), so no backend needs a reader of its own: ``rgb/``, each frame a
relative symbolic link to the sequence's colour image (nothing copied); ``rgb.txt``, stamped to
the microsecond; ``camera.txt``, the pinhole above; ``groundtruth.txt``, each frame's
``T_world_cam`` at its time, its rotation projected onto SO(3) to undo the printed digits'
rounding; and ``source.json``, the scene and sequence imported. The folder is written whole or
not at all (:func:`slam_it.eval.tum.staged`). :func:`import_scene` imports each of a scene's
sequences so, each into ``7scenes-<scene>-seq-XX`` under a folder.

Run as a program, on a scene unzipped from its zip with each ``seq-XX.zip`` in it unzipped::

    python -m slam_it.eval.sevenscenes SCENE OUT

for one, ``python -m slam_it.eval.sevenscenes 7scenes/chess imported`` writes
``imported/7scenes-chess-seq-01`` to ``-seq-06``.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections.abc import Sequence
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import images, runs, tum
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError

INTRINSICS = (525.0, 525.0, 320.0, 240.0)
"""The colour camera's ``fx fy cx cy`` in pixels, at :data:`SIZE` (the module docstring)."""

SIZE = (640, 480)
"""The colour images' width and height, which :data:`INTRINSICS` are for."""

FPS = 30.0
"""The frames' rate, in frames per second: the Kinect's."""

_FRAME = re.compile(r"frame-(\d+)\.color\.png")
"""A colour image's name, its frame's number."""

_RIGID_TOLERANCE = 1e-4
"""How far a printed rotation may be from orthonormal, each entry of ``R R^T - I``."""


def read_pose(path: str | Path) -> NDArray[np.float64]:
    """Read a ``frame-XXXXXX.pose.txt``: the camera's pose, ``T_world_cam``.

    Parameters
    ----------
    path : str or pathlib.Path
        The file: sixteen numbers, a 4x4 matrix row by row.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        The pose, its rotation projected onto SO(3).

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it does not hold sixteen finite numbers, or they are not a rigid transform.
    """
    source = Path(path)
    try:
        values = [float(v) for v in source.read_text().split()]
    except ValueError:
        raise InvalidInputError(f"{source}: holds a non-number") from None
    if len(values) != 16 or not np.all(np.isfinite(values)):
        raise InvalidInputError(f"{source}: expected a 4x4 matrix of 16 finite numbers")
    matrix = np.asarray(values).reshape(4, 4)
    rotation = matrix[:3, :3]
    orthonormal = np.allclose(rotation @ rotation.T, np.eye(3), atol=_RIGID_TOLERANCE)
    if not (
        orthonormal and np.linalg.det(rotation) > 0 and np.array_equal(matrix[3], [0, 0, 0, 1])
    ):
        raise InvalidInputError(f"{source}: not a rigid transform")
    matrix[:3, :3] = so3.project_to_so3(rotation)  # printed digits' rounding, made exact
    return matrix


def read_frames(sequence: str | Path) -> list[tuple[int, Path]]:
    """Return a sequence's frames: each colour image's number and file, in number order.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the folder holds no colour image, or the first is not :data:`SIZE`.
    """
    folder = Path(sequence)
    numbered = sorted(
        (int(match[1]), path)
        for path in folder.glob("frame-*.color.png")
        if (match := _FRAME.fullmatch(path.name))
    )
    if not numbered:
        raise InvalidInputError(f"{folder}: no frame-XXXXXX.color.png images")
    size = images.size(numbered[0][1])
    if size != SIZE:
        raise InvalidInputError(
            f"{numbered[0][1]}: {size[0]}x{size[1]}, where 7-Scenes' camera is {SIZE[0]}x{SIZE[1]}"
        )
    return numbered


def import_sequence(sequence: str | Path, out: str | Path) -> None:
    """Import a 7-Scenes sequence into a TUM-format folder (the module docstring).

    Parameters
    ----------
    sequence : str or pathlib.Path
        The unzipped sequence, e.g. ``.../7scenes/chess/seq-01``.
    out : str or pathlib.Path
        The folder written: new or empty.

    Raises
    ------
    FileNotFoundError
        If a frame's pose file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If `out` is not new or empty, the sequence holds no colour image, its first image is
        not 640x480, or a pose is malformed.
    """
    source = Path(sequence)
    frames = read_frames(source)
    stamps = [f"{number / FPS:.6f}" for number, _ in frames]  # each frame's, its truth's
    poses = np.array(
        [
            read_pose(image.with_name(image.name.replace(".color.png", ".pose.txt")))
            for _, image in frames
        ]
    )
    name = f"{source.resolve().parent.name}/{source.resolve().name}"
    with tum.staged(out) as target:
        (target / "rgb").mkdir()
        listed = []
        for stamp, (_, image) in zip(stamps, frames, strict=True):
            link = target / "rgb" / f"{stamp}.png"
            link.symlink_to(os.path.relpath(image, link.parent))
            listed.append((stamp, f"rgb/{stamp}.png"))
        tum.write_list(target / "rgb.txt", listed)
        camera = tum.Camera("pinhole", np.array(INTRINSICS), *SIZE, tum.UNUSED_DEPTH_SCALE, None)
        tum.write_camera(target, camera, note=f"imported by slam_it.eval.sevenscenes from {name}")
        runs.write_trajectory(target / "groundtruth.txt", [float(t) for t in stamps], poses)
        tum.write_source(target, {"kind": "7scenes", "sequence": name, "depth": None})


def import_scene(scene: str | Path, out: str | Path) -> list[Path]:
    """Import each of a scene's sequences, in order, into ``7scenes-<scene>-seq-XX`` under `out`.

    Parameters
    ----------
    scene : str or pathlib.Path
        The unzipped scene, e.g. ``.../7scenes/chess``, its ``seq-XX`` folders unzipped.
    out : str or pathlib.Path
        The folder the sequences' folders are written in, each new or empty.

    Returns
    -------
    list of pathlib.Path
        The folders written.

    Raises
    ------
    FileNotFoundError
        If a frame's pose file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If the scene holds no ``seq-XX`` folder, or a sequence fails :func:`import_sequence`
        (the sequences before it stay imported).
    """
    folder = Path(scene)
    sequences = sorted(p for p in folder.glob("seq-*") if p.is_dir())
    if not sequences:
        raise InvalidInputError(f"{folder}: no seq-XX folder; unzip its sequences first")
    written = []
    for sequence in sequences:
        target = Path(out) / f"7scenes-{folder.resolve().name}-{sequence.name}"
        import_sequence(sequence, target)
        written.append(target)
    return written


def main(arguments: Sequence[str]) -> int:
    """Import a scene's sequences from the command line (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, with a line on stdout for each sequence imported.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it.eval.sevenscenes")
    parser.add_argument("scene", type=Path, help="the unzipped scene, e.g. 7scenes/chess")
    parser.add_argument("out", type=Path, help="where each sequence's TUM-format folder goes")
    args = parser.parse_args(arguments)
    for folder in import_scene(args.scene, args.out):
        print(f"{folder}: {len(tum.read_frames(folder).images)} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
