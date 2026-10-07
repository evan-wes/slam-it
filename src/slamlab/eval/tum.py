"""The TUM RGB-D benchmark: colour frames, their ground-truth poses, and the cameras' models.

Sturm et al., "A benchmark for the evaluation of RGB-D SLAM systems", IROS 2012
(https://cvg.cit.tum.de/data/datasets/rgbd-dataset). A sequence folder holds ``rgb.txt``
(``timestamp filename`` per colour frame), ``depth.txt`` (the same, per depth map),
``groundtruth.txt`` (``timestamp tx ty tz qx qy qz qw`` from a motion-capture system at about
100 Hz), and the images.

**Poses.** The ground truth is the camera's optical centre and orientation in the
motion-capture world, ``T_world_cam`` with the camera looking along +z (the design doc's
convention). Its quaternions are Hamilton, stored xyzw like ours, and printed to four
decimals, so they are normalised before use.

**Association.** Colour frames and ground truth are sampled independently. Each frame takes
the ground-truth sample nearest in time, if one is within ``max_difference`` seconds (TUM's
own ``associate.py`` uses 0.02 s); frames without one are dropped. Each kept frame also takes
the nearest depth map within ``max_difference``, if there is one.

**Depth.** The depth maps are 16-bit PNGs registered to the colour image: pixel ``(u, v)`` of
the depth map is pixel ``(u, v)`` of the colour frame, and its value over :data:`DEPTH_SCALE`
is the point's z in the colour camera, in metres (0: no measurement). :func:`read_depth`
reads one in metres.

**Intrinsics.** The ROS default for each sensor (``freiburg1``, ``2``, ``3``) is published as a
pinhole with Brown-Conrady (radial-tangential) distortion. :data:`INTRINSICS` holds them in
the ``radtan`` layout ``[fx, fy, cx, cy, k1, k2, p1, p2, k3]``; ``freiburg3`` images are
already undistorted.

**Other TUM-format folders** (a recording exported to this layout) describe their camera in
a ``camera.txt`` instead, which takes precedence over the folder's name (:func:`read_camera`):
one ``key values`` line each for ``model`` (a name from
:data:`slamlab.foundations.camera.MODELS`), ``intrinsics`` (that model's), ``size`` (width,
height) and ``depth_scale`` (a depth map's value per metre), and, for an RGB-D run, ``bf``
(the depth sensor's virtual stereo baseline times the focal length, in pixel-metres:
ORB-SLAM's ``Camera.bf``), ``#`` comments allowed::

    # the recording's camera
    model pinhole
    intrinsics 525.0 525.0 319.5 239.5
    size 640 480
    depth_scale 1000
    bf 40

**Frames and ground truth.** :func:`read_frames` reads what a run consumes: every colour
frame, each with its nearest depth map, and the camera; it never reads the ground truth, so
a folder without one runs the same, and the native readers mirror it. :func:`read_sequence`
reads what an evaluation scores: the frames that have a ground-truth pose.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slamlab.eval import images, timestamps
from slamlab.foundations import camera as camera_model
from slamlab.foundations import so3
from slamlab.foundations.arrays import MAX_IMAGE_SIDE
from slamlab.foundations.errors import InvalidInputError

INTRINSICS: dict[str, NDArray[np.float64]] = {
    "freiburg1": np.array([517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633]),
    "freiburg2": np.array([520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172]),
    "freiburg3": np.array([535.4, 539.2, 320.1, 247.6, 0.0, 0.0, 0.0, 0.0, 0.0]),
}
"""Each sensor's colour camera, as ``radtan`` intrinsics (TUM's published calibration)."""

DEPTH_SCALE = 5000.0
"""A depth map's value per metre of z."""

MAX_DIFFERENCE = 0.02
"""The largest time difference, in seconds, between a frame and its ground-truth pose (and
its depth map)."""

SIZE = (640, 480)
"""The benchmark's images' width and height."""

CAMERA_FILE = "camera.txt"
"""The file a TUM-format folder describes its own camera in."""

CAMERA_KEYS = ("model", "intrinsics", "size", "depth_scale")
"""``camera.txt``'s keys, each on one line."""

OPTIONAL_CAMERA_KEYS = ("bf",)
"""The keys it may hold too."""

SENSOR_BF = 40.0
"""Every freiburg sensor's virtual stereo baseline times focal length, in pixel-metres
(ORB-SLAM2's TUM RGB-D settings' ``Camera.bf``)."""


@dataclass(frozen=True)
class Camera:
    """A TUM-format folder's colour camera.

    Attributes
    ----------
    model : str
        A model from :data:`slamlab.foundations.camera.MODELS`.
    intrinsics : ndarray of float64, shape (P,)
        Its intrinsics.
    width, height : int
        The images' size, in pixels.
    depth_scale : float
        A depth map's value per metre of z.
    bf : float or None
        The depth sensor's virtual stereo baseline times the focal length, in pixel-metres
        (ORB-SLAM's ``Camera.bf``); None if ``camera.txt`` gives none (no RGB-D run).
    """

    model: str
    intrinsics: NDArray[np.float64]
    width: int
    height: int
    depth_scale: float
    bf: float | None


@dataclass(frozen=True)
class Frames:
    """A TUM-format folder's colour frames, as a run reads them (no ground truth).

    Attributes
    ----------
    root : pathlib.Path
        The folder.
    timestamps : ndarray of float64, shape (F,)
        Each frame's timestamp, in seconds, in ``rgb.txt``'s order (non-decreasing).
    images : list of pathlib.Path
        Each frame's colour image.
    depths : list of pathlib.Path or None
        Each frame's nearest depth map within :data:`MAX_DIFFERENCE`, None where there is
        none (every frame, for a folder without ``depth.txt``).
    camera : Camera
        The camera.
    """

    root: Path
    timestamps: NDArray[np.float64]
    images: list[Path]
    depths: list[Path | None]
    camera: Camera


@dataclass(frozen=True)
class Sequence:
    """A TUM sequence's colour frames that have a ground-truth pose.

    Attributes
    ----------
    root : pathlib.Path
        The sequence folder.
    camera : Camera
        Its camera (:func:`read_camera`).
    timestamps : ndarray of float64, shape (F,)
        Each frame's timestamp, in seconds.
    images : list of pathlib.Path
        Each frame's colour image.
    T_world_cam : ndarray of float64, shape (F, 4, 4)
        Each frame's ground-truth pose.
    depths : list of pathlib.Path or None
        Each frame's depth map, None where none is within the association tolerance (every
        frame, for a folder without ``depth.txt``).
    """

    root: Path
    camera: Camera
    timestamps: NDArray[np.float64]
    images: list[Path]
    T_world_cam: NDArray[np.float64]
    depths: list[Path | None]

    @property
    def intrinsics(self) -> NDArray[np.float64]:
        """The camera's intrinsics."""
        return self.camera.intrinsics.copy()


def _table(path: Path, columns: int) -> list[tuple[int, list[str]]]:
    """Return a TUM text file's rows with their line numbers, skipping comments and blanks.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a row has fewer than `columns` fields.
    """
    rows = []
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) < columns:
            raise InvalidInputError(
                f"{path}:{number}: expected {columns} fields, got {len(fields)}"
            )
        rows.append((number, fields))
    return rows


def _poses(path: Path) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return the ground truth's timestamps (ascending) and ``T_world_cam`` matrices.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the file has no rows, or a row is not 8 finite numbers with a non-zero quaternion.
    """
    rows = _table(path, 8)
    if not rows:
        raise InvalidInputError(f"{path}: no ground-truth poses")
    values = np.empty((len(rows), 8))
    for k, (number, fields) in enumerate(rows):
        try:
            values[k] = [float(v) for v in fields[:8]]
        except ValueError as error:
            raise InvalidInputError(f"{path}:{number}: expected numbers ({error})") from None
        if not np.all(np.isfinite(values[k])) or not np.linalg.norm(values[k, 4:8]) > 0.0:
            raise InvalidInputError(
                f"{path}:{number}: expected finite values and a non-zero quaternion"
            )
    values = values[np.argsort(values[:, 0], kind="stable")]
    quaternions = values[:, 4:8] / np.linalg.norm(values[:, 4:8], axis=1, keepdims=True)
    poses = np.tile(np.eye(4), (len(values), 1, 1))
    poses[:, :3, :3] = so3.from_quaternion(quaternions)
    poses[:, :3, 3] = values[:, 1:4]
    return values[:, 0], poses


def _stamped(path: Path) -> tuple[NDArray[np.float64], list[str]]:
    """Return a ``timestamp filename`` file's stamps and file names.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a stamp is not a number (the message names the file and line), or the stamps are
        not finite or do not ascend.
    """
    rows = _table(path, 2)
    stamps = np.empty(len(rows))
    for k, (number, fields) in enumerate(rows):
        try:
            stamps[k] = float(fields[0])
        except ValueError as error:
            raise InvalidInputError(
                f"{path}:{number}: expected a numeric timestamp ({error})"
            ) from None
    return timestamps.as_times(stamps, str(path), strict=False), [f[1] for _, f in rows]


def _camera_file(path: Path) -> Camera:
    """Read a ``camera.txt``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a key is unknown, repeated or missing, or its values are malformed.
    """
    values: dict[str, list[str]] = {}
    known = CAMERA_KEYS + OPTIONAL_CAMERA_KEYS
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        fields = line.split("#", 1)[0].split()
        if not fields:
            continue
        key = fields[0]
        if key not in known or key in values:
            raise InvalidInputError(f"{path}:{number}: unknown or repeated key {key!r}")
        values[key] = fields[1:]
    missing = [key for key in CAMERA_KEYS if key not in values]
    if missing:
        raise InvalidInputError(f"{path}: missing {', '.join(missing)}")
    counts = (len(values["model"]), len(values["size"]), len(values["depth_scale"]))
    if counts != (1, 2, 1) or len(values.get("bf", [0])) != 1:
        raise InvalidInputError(
            f"{path}: expected one model, two sizes, one depth scale, at most one bf"
        )
    model = values["model"][0]
    try:
        intrinsics = np.array([float(v) for v in values["intrinsics"]])
        width, height = (int(v) for v in values["size"])
        depth_scale = float(values["depth_scale"][0])
        bf = float(values["bf"][0]) if "bf" in values else None
    except ValueError as error:
        raise InvalidInputError(f"{path}: expected numbers ({error})") from None
    camera_model.check_intrinsics(model, intrinsics)
    if not (1 <= width <= MAX_IMAGE_SIDE and 1 <= height <= MAX_IMAGE_SIDE):
        raise InvalidInputError(f"{path}: size must lie in [1, {MAX_IMAGE_SIDE}]")
    if not (np.isfinite(depth_scale) and depth_scale > 0.0):
        raise InvalidInputError(f"{path}: depth_scale must be finite and positive")
    if bf is not None and not (np.isfinite(bf) and bf > 0.0):
        raise InvalidInputError(f"{path}: bf must be finite and positive")
    return Camera(model, intrinsics, width, height, depth_scale, bf)


def read_camera(root: str | Path) -> Camera:
    """Read a TUM-format folder's camera: its ``camera.txt``, or the sensor its name says.

    Parameters
    ----------
    root : str or pathlib.Path
        The folder.

    Returns
    -------
    Camera
        From ``camera.txt`` if the folder has one; otherwise the benchmark's published
        ``radtan`` calibration of the ``freiburg1``, ``2`` or ``3`` sensor its name holds, at
        640x480, :data:`DEPTH_SCALE` and :data:`SENSOR_BF`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If ``camera.txt`` is malformed, or there is none and the name holds no sensor.
    """
    folder = Path(root)
    if (folder / CAMERA_FILE).exists():
        return _camera_file(folder / CAMERA_FILE)
    sensor = next((s for s in INTRINSICS if s in folder.name), None)
    if sensor is None:
        raise InvalidInputError(
            f"{folder.name}: expected a {CAMERA_FILE}, or 'freiburg1', '2' or '3' in the "
            "folder's name"
        )
    return Camera("radtan", INTRINSICS[sensor].copy(), *SIZE, DEPTH_SCALE, SENSOR_BF)


def _depth_files(
    folder: Path, stamps: NDArray[np.float64], max_difference: float
) -> list[Path | None]:
    """Each stamp's nearest depth map within `max_difference`, if the folder has depth."""
    if not (folder / "depth.txt").exists():
        return [None] * len(stamps)
    depth_times, depth_names = _stamped(folder / "depth.txt")
    if len(depth_times) == 0:
        return [None] * len(stamps)
    closest = timestamps.nearest(depth_times, stamps)
    return [
        folder / depth_names[d] if abs(depth_times[d] - t) <= max_difference else None
        for t, d in zip(stamps, closest, strict=True)
    ]


def read_frames(root: str | Path, max_difference: float = MAX_DIFFERENCE) -> Frames:
    """Read a TUM-format folder's colour frames as a run consumes them, without ground truth.

    Parameters
    ----------
    root : str or pathlib.Path
        The folder.
    max_difference : float, optional
        The largest frame-to-depth-map time difference accepted, in seconds.

    Returns
    -------
    Frames
        Every row of ``rgb.txt``, each with its nearest depth map, and the camera.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `max_difference` is negative, a file is malformed (named with its line), or the
        camera cannot be read (:func:`read_camera`).
    FileNotFoundError
        If ``rgb.txt`` is missing (``depth.txt`` is optional).
    """
    if not (np.isfinite(max_difference) and max_difference >= 0.0):
        raise InvalidInputError(f"max_difference: must be non-negative, got {max_difference}")
    folder = Path(root)
    camera = read_camera(folder)
    stamps, names = _stamped(folder / "rgb.txt")
    depths = _depth_files(folder, stamps, max_difference)
    return Frames(folder, stamps, [folder / n for n in names], depths, camera)


def read_sequence(root: str | Path, max_difference: float = MAX_DIFFERENCE) -> Sequence:
    """Read a TUM RGB-D sequence's colour frames and associate their ground-truth poses.

    Parameters
    ----------
    root : str or pathlib.Path
        The sequence folder, e.g. ``rgbd_dataset_freiburg1_xyz``.
    max_difference : float, optional
        The largest frame-to-pose time difference accepted, in seconds.

    Returns
    -------
    Sequence
        The frames that have a pose within `max_difference`, in time order.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the camera cannot be read (:func:`read_camera`), `max_difference` is negative, a
        file is malformed (named with its line), or no frame has a pose.
    FileNotFoundError
        If ``rgb.txt`` or ``groundtruth.txt`` is missing (``depth.txt`` is optional).
    """
    frames = read_frames(root, max_difference)
    folder, stamps = frames.root, frames.timestamps
    truth_times, truth_poses = _poses(folder / "groundtruth.txt")
    nearest = timestamps.nearest(truth_times, stamps)
    kept = np.flatnonzero(np.abs(truth_times[nearest] - stamps) <= max_difference)
    if kept.size == 0:
        raise InvalidInputError(
            f"{folder}: no frame has a ground-truth pose within {max_difference} s"
        )
    return Sequence(
        folder,
        frames.camera,
        stamps[kept],
        [frames.images[k] for k in kept],
        truth_poses[nearest[kept]],
        [frames.depths[k] for k in kept],
    )


def read_depth(path: str | Path, depth_scale: float = DEPTH_SCALE) -> NDArray[np.float64]:
    """Read a depth map in metres.

    Parameters
    ----------
    path : str or pathlib.Path
        A 16-bit depth PNG (:attr:`Frames.depths`).
    depth_scale : float, optional
        Its value per metre (:attr:`Camera.depth_scale`; default the benchmark's).

    Returns
    -------
    ndarray of float64, shape (H, W)
        Each pixel's z in the colour camera, in metres: its value over `depth_scale`; NaN
        where the sensor measured nothing (a stored 0).

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slamlab.foundations.errors.InvalidInputError
        If it is not a 16-bit single-channel image, or `depth_scale` is not finite and
        positive.
    """
    if not (np.isfinite(depth_scale) and depth_scale > 0.0):
        raise InvalidInputError(f"depth_scale: must be finite and positive, got {depth_scale}")
    raw = images.read_depth(path)
    depth = raw.astype(np.float64) / depth_scale
    depth[raw == 0] = np.nan
    return depth
