"""The run format: what a pipeline writes, and the evaluation and the run viewer read.

A run is a folder, written by any backend's pipeline (C++ and Rust write the same files):

- ``run.json``: what ran: ``{"format": "slam-it-all-run", "version": 3, "dataset", "sequence",
  "mode", "backend", "settings", "record_features", "peak_memory", "cpus", "memory_cap"}``,
  ``mode`` one of :data:`MODES` and ``backend`` one of ``python``, ``cpp`` and ``rust``;
  ``settings`` holds the pipeline's options as JSON; ``record_features`` whether the run
  recorded its features (below). The last three describe the program that ran it, as it recorded
  them before writing (:mod:`slam_it_all.eval.budgets`), and are null when the run was not a
  program's of its own (Python's in-process run) or the system does not say: ``peak_memory``,
  its largest resident set in bytes; ``cpus``, the CPUs it could run on, ascending (a budget
  pins them); ``memory_cap``, its control group's memory cap in bytes (null without one).
- ``trajectory.txt``: each tracked frame's ``T_world_cam`` in TUM's format, one
  ``timestamp tx ty tz qx qy qz qw`` line per frame, ``#`` comments allowed: what ORB-SLAM and
  the TUM tools write, so other systems' trajectories evaluate the same way. Numbers are
  written to round-trip exactly.
- ``keyframes.npy``: the timestamps of every keyframe the run made, by id (the order it made
  them in), ``(K,)`` float64; ``keyframe_poses.npy``: their ``T_world_cam``, ``(K, 4, 4)``, the
  last each had; ``keyframe_lives.npy``: their lives, ``(K, 3)`` int64 (below).
- ``points.npy``: every point the run made, by id, in the world frame, ``(M, 3)`` float64,
  where each last was; ``point_lives.npy``: their lives, ``(M, 3)`` int64.
- **A life** is three numbers: the frame that made it; the frame that removed it, or -1 if it
  is in the final map; and why it was removed (its index in :data:`REMOVALS`), or -1. Frames
  are counted by their line in ``diagnostics.jsonl``. The final map is the rows still alive
  (:meth:`Run.final_points`, :meth:`Run.final_keyframes`).
- ``feature_pixels.npy`` and ``feature_records.npy``, if the run recorded its features: every
  frame's keypoints, in frame order. Their pixels, ``(N, 2)`` float32; and their records,
  ``(N, 4)`` int32: the frame, the keypoint's pyramid level, its fate (its index in
  :data:`FATES`) and the point concerned (an id in ``points.npy``, -1 if none), as
  :class:`slam_it_all.edge.pipeline.Fates` gives them. Streamed as the run goes
  (:class:`FeatureWriter`), and read mapped from disk (:func:`read_features`).
- ``diagnostics.jsonl``: one JSON object per line, per frame, each with its ``timestamp``;
  otherwise what the pipeline wants to report (tracked points, state). The edge pipeline's
  frames also time their steps, in seconds (:data:`STEPS`), which the budget harness judges
  (:func:`slam_it_all.eval.budgets.judge`), and say how their stages went
  (:meth:`slam_it_all.edge.pipeline.FrameSummary.as_dict`): an object per stage, null where it
  did not run.

:func:`write_run` checks everything before it writes, and replaces each file whole (written
to a temporary sibling, then renamed); :func:`read_run` checks every file's shape;
:func:`evaluate` scores a run against the ground truth.
"""

from __future__ import annotations

import io
import itertools
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it_all.eval import euroc, metrics, tum
from slam_it_all.eval.timestamps import as_times
from slam_it_all.foundations import se3, so3
from slam_it_all.foundations.arrays import as_batch
from slam_it_all.foundations.errors import InvalidInputError

FORMAT = "slam-it-all-run"
"""``run.json``'s ``format``."""

VERSION = 3
"""``run.json``'s ``version``: this module's layout (2 added ``peak_memory``, ``cpus`` and
``memory_cap``; 3 every point and keyframe the run made, with their lives)."""

REMOVALS = ("culled", "fused", "weak")
"""Why a point or keyframe was removed, a life's code being the index: culled (a recent point
tracking rarely found, or a redundant keyframe), fused into another point, or erased with its
views (left seen too little). :class:`slam_it_all.edge.map.Removal` has the same codes."""

FATES = ("unmatched", "rejected", "tracked", "made")
"""What became of a keypoint on its frame, a feature record's code being the index: not matched
by the frame's last tracking stage; matched by it, then not tracked; tracked; or made into a
new point. :class:`slam_it_all.edge.pipeline.Fate` has the same codes."""

PIXELS = "feature_pixels.npy"
"""The file of a run's keypoints' pixels."""

RECORDS = "feature_records.npy"
"""The file of a run's keypoints' records: frame, level, fate and point."""

STEPS = ("read_seconds", "build_seconds", "track_seconds", "map_seconds")
"""The steps each frame of the edge pipeline times, as keys of its diagnostic, in seconds:
delivering its images decoded (and a stereo pair rectified), building the frame from them,
tracking it (initialisation included), and mapping it, 0 exactly when it is not a keyframe
mapped. C++ and Rust write the same keys (:mod:`slam_it_all.orchestration.vo`)."""

MODES = ("monocular", "stereo", "rgbd")
"""A run's sensor modes."""

BACKENDS = ("python", "cpp", "rust")
"""The backends a run may come from."""

MAX_DIFFERENCE = 0.02
"""The largest time difference, in seconds, between a run's frame and its ground-truth pose:
TUM's ``associate.py`` default, and :data:`slam_it_all.eval.tum.MAX_DIFFERENCE`. Pairing is
closest first (:func:`slam_it_all.eval.metrics.associate`), so a denser ground truth (EuRoC's
200 Hz) still pairs each frame with its nearest pose."""

_INFO_KEYS = (
    "dataset",
    "sequence",
    "mode",
    "backend",
    "settings",
    "record_features",
    "peak_memory",
    "cpus",
    "memory_cap",
)
"""``run.json``'s keys besides ``format`` and ``version``."""


@dataclass(frozen=True)
class Run:
    """A pipeline's run.

    Attributes
    ----------
    dataset : str
        The dataset's name, e.g. ``"tum"`` or ``"euroc"``.
    sequence : str
        The sequence's name, e.g. ``"V1_01_easy"``.
    mode : str
        One of :data:`MODES`.
    backend : str
        One of :data:`BACKENDS`.
    timestamps : ndarray of float64, shape (F,)
        Each tracked frame's timestamp, in seconds, ascending.
    poses : ndarray of float64, shape (F, 4, 4)
        Each tracked frame's ``T_world_cam``.
    keyframe_timestamps : ndarray of float64, shape (K,)
        The timestamps of every keyframe the run made, by id.
    keyframe_poses : ndarray of float64, shape (K, 4, 4)
        Their last ``T_world_cam``.
    points : ndarray of float64, shape (M, 3)
        Every point the run made, by id, where it last was, in the world frame.
    settings : dict
        The pipeline's options, as JSON values.
    diagnostics : list of dict
        One JSON object per frame, each with a ``timestamp``.
    peak_memory : int or None
        The largest resident set of the program that ran it, in bytes; None if unknown.
    cpus : list of int or None
        The CPUs the program that ran it could run on, ascending; None if unknown.
    memory_cap : int or None
        Its memory cap, in bytes; None if it had none or it is unknown.
    point_lives : ndarray of int64, shape (M, 3)
        Each point's life: the frame that made it, the frame that removed it (-1 if none), and
        why (its index in :data:`REMOVALS`; -1 if none).
    keyframe_lives : ndarray of int64, shape (K, 3)
        Each keyframe's life, the same way.
    record_features : bool
        Whether the run recorded its features (:func:`read_features` reads them).
    """

    dataset: str
    sequence: str
    mode: str
    backend: str
    timestamps: NDArray[np.float64]
    poses: NDArray[np.float64]
    keyframe_timestamps: NDArray[np.float64] = field(default_factory=lambda: np.zeros(0))
    keyframe_poses: NDArray[np.float64] = field(default_factory=lambda: np.zeros((0, 4, 4)))
    points: NDArray[np.float64] = field(default_factory=lambda: np.zeros((0, 3)))
    settings: dict[str, Any] = field(default_factory=dict)
    diagnostics: list[dict[str, Any]] = field(default_factory=list)
    peak_memory: int | None = None
    cpus: list[int] | None = None
    memory_cap: int | None = None
    point_lives: NDArray[np.int64] = field(default_factory=lambda: np.zeros((0, 3), dtype=np.int64))
    keyframe_lives: NDArray[np.int64] = field(
        default_factory=lambda: np.zeros((0, 3), dtype=np.int64)
    )
    record_features: bool = False

    def final_points(self) -> NDArray[np.float64]:
        """Return the points still in the final map.

        Returns
        -------
        ndarray of float64, shape (P, 3)
            The rows of :attr:`points` whose life has no end, in id order.
        """
        return self.points[self.point_lives[:, 1] < 0]

    def final_keyframes(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Return the keyframes still in the final map.

        Returns
        -------
        timestamps : ndarray of float64, shape (J,)
            Their times, ascending.
        poses : ndarray of float64, shape (J, 4, 4)
            Their ``T_world_cam``.
        """
        alive = self.keyframe_lives[:, 1] < 0
        return self.keyframe_timestamps[alive], self.keyframe_poses[alive]


def _check_process(peak_memory: object, cpus: object, memory_cap: object, prefix: str) -> None:
    """Check what a run records of its program: each None, or of its type and range.

    `prefix` begins each message (the file's path when reading).
    """
    if peak_memory is not None and (type(peak_memory) is not int or peak_memory < 0):
        raise InvalidInputError(f"{prefix}peak_memory: expected a whole number of bytes or null")
    if cpus is not None and not (
        isinstance(cpus, list)
        and len(cpus) > 0
        and all(type(cpu) is int and cpu >= 0 for cpu in cpus)
        and all(a < b for a, b in itertools.pairwise(cpus))
    ):
        raise InvalidInputError(f"{prefix}cpus: expected CPU numbers, ascending, or null")
    if memory_cap is not None and (type(memory_cap) is not int or memory_cap < 1):
        raise InvalidInputError(f"{prefix}memory_cap: expected a positive number of bytes or null")


def _as_stamped(
    timestamps: ArrayLike, poses: ArrayLike, name: str
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Check stamped poses: ascending finite times, one rigid transform each."""
    times = as_times(timestamps, f"{name} timestamps")
    flat, lead = as_batch(poses, (4, 4), f"{name} poses")
    if lead != (len(times),):
        raise InvalidInputError(f"{name}: expected (F,) timestamps and (F, 4, 4) poses")
    if len(flat):
        se3.check_transform(flat)
    return times, flat


def write_trajectory(path: str | Path, timestamps: ArrayLike, poses: ArrayLike) -> None:
    """Write stamped poses in TUM's trajectory format, exactly round-tripping.

    Parameters
    ----------
    path : str or pathlib.Path
        The file to write.
    timestamps : array_like, shape (F,)
        Seconds, strictly ascending.
    poses : array_like, shape (F, 4, 4)
        ``T_world_cam``.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the times do not ascend or a pose is not a rigid transform.
    """
    times, flat = _as_stamped(timestamps, poses, "trajectory")
    _replace(Path(path), _trajectory_text(times, flat).encode())


def _trajectory_text(times: NDArray[np.float64], poses: NDArray[np.float64]) -> str:
    """Return checked stamped poses as TUM-format lines, each number round-tripping."""
    quaternions = so3.to_quaternion(poses[:, :3, :3]) if len(poses) else np.zeros((0, 4))
    lines = ["# timestamp tx ty tz qx qy qz qw"]
    for t, pose, q in zip(times, poses, quaternions, strict=True):
        values = [t, *pose[:3, 3], *q]
        lines.append(" ".join(repr(float(v)) for v in values))
    return "\n".join(lines) + "\n"


def _read_text(path: Path) -> str:
    """Read a text file, refusing one that is not UTF-8."""
    try:
        return path.read_text(encoding="utf-8")
    except UnicodeDecodeError as error:
        raise InvalidInputError(f"{path}: not UTF-8 text ({error})") from None


def _replace(path: Path, data: bytes) -> None:
    """Write a file whole: to a temporary sibling, then renamed over it."""
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_bytes(data)
    temporary.replace(path)


def _npy(array: NDArray[Any]) -> bytes:
    """Return an array as the bytes of a ``.npy`` file."""
    buffer = io.BytesIO()
    np.save(buffer, array, allow_pickle=False)
    return buffer.getvalue()


def _check_lives(lives: ArrayLike, count: int, frames: int, name: str) -> NDArray[np.int64]:
    """Check lives, one per row; return them.

    Each must be made in one of the run's frames, removed in that frame or a later one or never,
    and say why exactly when removed. `name` begins each message.
    """
    out = np.asarray(lives)
    if out.dtype != np.int64 or out.shape != (count, 3):
        raise InvalidInputError(
            f"{name}: expected int64 ({count}, 3), one per row, got {out.dtype} {out.shape}"
        )
    made, removed, how = out.T
    alive = removed == -1
    if np.any((made < 0) | (made >= frames)):
        raise InvalidInputError(f"{name}: a life must start in one of the run's {frames} frames")
    if np.any(~alive & ((removed < made) | (removed >= frames))):
        raise InvalidInputError(f"{name}: a life must end in its start's frame or later, or -1")
    if np.any(alive != (how == -1)) or np.any(~alive & ((how < 0) | (how >= len(REMOVALS)))):
        raise InvalidInputError(f"{name}: why a life ended must be a removal code, -1 if alive")
    return out


def _json(value: Any, name: str) -> str:  # noqa: ANN401 -- a run's free-form JSON values
    """Return a value as JSON text, refusing what JSON cannot hold."""
    try:
        return json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise InvalidInputError(f"{name}: not JSON ({error})") from None


def read_trajectory(path: str | Path) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Read a trajectory in TUM's format.

    Parameters
    ----------
    path : str or pathlib.Path
        The file: ``timestamp tx ty tz qx qy qz qw`` per line, ``#`` comments allowed.

    Returns
    -------
    timestamps : ndarray of float64, shape (F,)
        The times, in the file's order (which must ascend).
    poses : ndarray of float64, shape (F, 4, 4)
        ``T_world_cam``, quaternions normalised.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it_all.foundations.errors.InvalidInputError
        If a line is not 8 finite numbers with a non-zero quaternion, or the times do not
        ascend.
    """
    file = Path(path)
    rows = []
    for number, line in enumerate(_read_text(file).splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        try:
            values = [float(v) for v in stripped.split()]
        except ValueError as error:
            raise InvalidInputError(f"{file}:{number}: expected numbers ({error})") from None
        if len(values) != 8 or not all(
            math.isfinite(v) for v in values
        ):  # t, x, y, z and a quaternion
            raise InvalidInputError(f"{file}:{number}: expected 8 finite numbers")
        if not math.hypot(*values[4:]) > 0.0:
            raise InvalidInputError(f"{file}:{number}: the quaternion is zero")
        rows.append(values)
    table = np.asarray(rows, dtype=np.float64).reshape(-1, 8)
    poses = np.tile(np.eye(4), (len(table), 1, 1))
    if len(table):
        quaternions = table[:, 4:] / np.linalg.norm(table[:, 4:], axis=1, keepdims=True)
        poses[:, :3, :3] = so3.from_quaternion(quaternions)
        poses[:, :3, 3] = table[:, 1:4]
    times, poses = _as_stamped(table[:, 0], poses, str(file))
    return times, poses


def write_run(folder: str | Path, run: Run) -> None:
    """Write a run's folder (created if needed; its files are replaced).

    Parameters
    ----------
    folder : str or pathlib.Path
        The run's folder.
    run : Run
        The run.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the dataset or sequence is not a string, the mode or backend is unknown, an array
        is malformed, the settings or a diagnostic are not JSON (a diagnostic must be an
        object with a ``timestamp``), or what it records of its program is malformed (a peak
        memory or memory cap that is not a whole number of bytes, CPUs not ascending), or
        ``record_features`` is not a bool. Nothing is written then.
    """
    for name in ("dataset", "sequence"):
        if not isinstance(getattr(run, name), str):
            raise InvalidInputError(f"{name}: expected a string")
    _check_names(run.mode, run.backend)
    times, poses = _as_stamped(run.timestamps, run.poses, "trajectory")
    keyframe_times, keyframe_poses = _as_stamped(
        run.keyframe_timestamps, run.keyframe_poses, "keyframes"
    )
    points, lead = as_batch(run.points, (3,), "points")
    if len(lead) != 1:
        raise InvalidInputError(f"points: expected (M, 3), got {lead}")
    if not isinstance(run.settings, dict):
        raise InvalidInputError("settings: expected a dict")
    if not isinstance(run.record_features, bool):
        raise InvalidInputError("record_features: expected a bool")
    _check_process(run.peak_memory, run.cpus, run.memory_cap, "")
    info = {
        "format": FORMAT,
        "version": VERSION,
        "dataset": run.dataset,
        "sequence": run.sequence,
        "mode": run.mode,
        "backend": run.backend,
        "settings": run.settings,
        "record_features": run.record_features,
        "peak_memory": run.peak_memory,
        "cpus": run.cpus,
        "memory_cap": run.memory_cap,
    }
    _json(info, "settings")
    diagnostics = []
    for k, entry in enumerate(run.diagnostics):
        if not isinstance(entry, dict) or "timestamp" not in entry:
            raise InvalidInputError(f"diagnostics[{k}]: expected an object with a timestamp")
        diagnostics.append(_json(entry, f"diagnostics[{k}]") + "\n")
    frames = len(run.diagnostics)
    point_lives = _check_lives(run.point_lives, len(points), frames, "point_lives")
    keyframe_lives = _check_lives(run.keyframe_lives, len(keyframe_times), frames, "keyframe_lives")
    out = Path(folder)
    out.mkdir(parents=True, exist_ok=True)
    _replace(out / "run.json", (json.dumps(info, indent=1) + "\n").encode())
    _replace(out / "trajectory.txt", _trajectory_text(times, poses).encode())
    _replace(out / "keyframes.npy", _npy(keyframe_times))
    _replace(out / "keyframe_poses.npy", _npy(keyframe_poses))
    _replace(out / "keyframe_lives.npy", _npy(keyframe_lives))
    _replace(out / "points.npy", _npy(points))
    _replace(out / "point_lives.npy", _npy(point_lives))
    _replace(out / "diagnostics.jsonl", "".join(diagnostics).encode())


def _check_names(mode: str, backend: str) -> None:
    """Check a run's mode and backend are known."""
    if mode not in MODES:
        raise InvalidInputError(f"mode: expected one of {MODES}, got {mode!r}")
    if backend not in BACKENDS:
        raise InvalidInputError(f"backend: expected one of {BACKENDS}, got {backend!r}")


def _load(
    path: Path,
    shape: tuple[int, ...],
    dtype: type[np.generic] = np.float64,
    *,
    mapped: bool = False,
) -> NDArray[Any]:
    """Load a .npy file of `dtype` whose trailing dimensions are `shape`, or map it read-only."""
    if not path.is_file():
        raise FileNotFoundError(f"no {path.name} in {path.parent}")
    try:
        data: NDArray[Any] = np.load(path, mmap_mode="r" if mapped else None, allow_pickle=False)
    except (ValueError, EOFError) as error:  # not an .npy file, an empty one, or objects
        raise InvalidInputError(f"{path}: not a readable .npy array ({error})") from None
    if not isinstance(data, np.ndarray) or data.dtype != dtype or data.shape[1:] != shape:
        expected = ", ".join(["N", *map(str, shape)])
        raise InvalidInputError(
            f"{path}: expected {np.dtype(dtype).name} ({expected}), got {data.dtype} {data.shape}"
        )
    return data


def _read_info(path: Path) -> dict[str, Any]:
    """Read ``run.json``: this format's version, with every key, each of its type."""
    try:
        info = json.loads(_read_text(path))
    except json.JSONDecodeError as error:
        raise InvalidInputError(f"{path}: not JSON ({error})") from None
    if not isinstance(info, dict):
        raise InvalidInputError(f"{path}: expected a JSON object")
    if info.get("format") != FORMAT or info.get("version") != VERSION:
        raise InvalidInputError(
            f"{path}: expected format {FORMAT!r} version {VERSION}, got "
            f"{info.get('format')!r} version {info.get('version')!r}"
        )
    missing = [key for key in _INFO_KEYS if key not in info]
    if missing:
        raise InvalidInputError(f"{path}: missing {', '.join(missing)}")
    for key in ("dataset", "sequence", "mode", "backend"):
        if not isinstance(info[key], str):
            raise InvalidInputError(f"{path}: {key} must be a string")
    if not isinstance(info["settings"], dict):
        raise InvalidInputError(f"{path}: settings must be an object")
    if not isinstance(info["record_features"], bool):
        raise InvalidInputError(f"{path}: record_features must be true or false")
    _check_process(info["peak_memory"], info["cpus"], info["memory_cap"], f"{path}: ")
    _check_names(info["mode"], info["backend"])
    return info


def _read_diagnostics(path: Path) -> list[dict[str, Any]]:
    """Read ``diagnostics.jsonl``: one object with a ``timestamp`` per line."""
    out = []
    for number, line in enumerate(_read_text(path).splitlines(), start=1):
        try:
            entry = json.loads(line)
        except json.JSONDecodeError as error:
            raise InvalidInputError(f"{path}:{number}: not JSON ({error})") from None
        if not isinstance(entry, dict) or "timestamp" not in entry:
            raise InvalidInputError(f"{path}:{number}: expected an object with a timestamp")
        out.append(entry)
    return out


def read_run(folder: str | Path) -> Run:
    """Read a run's folder.

    Parameters
    ----------
    folder : str or pathlib.Path
        The run's folder.

    Returns
    -------
    Run
        The run.

    Raises
    ------
    FileNotFoundError
        If a file is missing.
    slam_it_all.foundations.errors.InvalidInputError
        If ``run.json`` is not JSON, not this format's version, or lacks a key or has one of
        the wrong type, or another file is malformed (named, with its line where it has one).
    """
    root = Path(folder)
    info = _read_info(root / "run.json")
    times, poses = read_trajectory(root / "trajectory.txt")
    keyframe_times = _load(root / "keyframes.npy", ())
    keyframe_poses = _load(root / "keyframe_poses.npy", (4, 4))
    _as_stamped(keyframe_times, keyframe_poses, "keyframes")
    points = _load(root / "points.npy", (3,))
    diagnostics = _read_diagnostics(root / "diagnostics.jsonl")
    lives = {
        name: _check_lives(
            _load(root / f"{name}.npy", (3,), np.int64), count, len(diagnostics), str(root / name)
        )
        for name, count in (("point_lives", len(points)), ("keyframe_lives", len(keyframe_times)))
    }
    return Run(
        dataset=info["dataset"],
        sequence=info["sequence"],
        mode=info["mode"],
        backend=info["backend"],
        timestamps=times,
        poses=poses,
        keyframe_timestamps=keyframe_times,
        keyframe_poses=keyframe_poses,
        points=points,
        settings=info["settings"],
        diagnostics=diagnostics,
        peak_memory=info["peak_memory"],
        cpus=info["cpus"],
        memory_cap=info["memory_cap"],
        point_lives=lives["point_lives"],
        keyframe_lives=lives["keyframe_lives"],
        record_features=info["record_features"],
    )


def _check_keypoints(
    pixels: NDArray[Any], level: NDArray[Any], fate: NDArray[Any], point: NDArray[Any], name: str
) -> None:
    """Check feature records: finite pixels, levels, fate codes, a point exactly when matched."""
    if not np.all(np.isfinite(pixels)):
        raise InvalidInputError(f"{name}: pixels must be finite")
    if np.any(level < 0):
        raise InvalidInputError(f"{name}: a level must be at least 0")
    if np.any((fate < 0) | (fate >= len(FATES))):
        raise InvalidInputError(f"{name}: a fate must be a code of FATES")
    if np.any(point < -1) or np.any((point == -1) != (fate == FATES.index("unmatched"))):
        raise InvalidInputError(f"{name}: a point must be an id, -1 exactly when unmatched")


def _npy_header(rows: int, columns: int, descr: str) -> bytes:
    """Return a ``(rows, columns)`` .npy file's header, as numpy writes it.

    It is the same length for any row count, so it can be rewritten in place as rows are added.
    """
    buffer = io.BytesIO()
    np.lib.format.write_array_header_1_0(
        buffer, {"descr": descr, "fortran_order": False, "shape": (rows, columns)}
    )
    return buffer.getvalue()


class FeatureWriter:
    """Stream a run's feature records into its folder, a frame at a time.

    Writes :data:`PIXELS` and :data:`RECORDS` (the module docstring) as the run goes, so a long
    run's records never wait in memory. After each frame the files' headers count the rows so
    far, so each reads whole up to the last frame written. Frames are counted from 0, one per
    :meth:`write`: one per line of the run's ``diagnostics.jsonl``. Use it as a context
    manager, or call :meth:`close`.

    Parameters
    ----------
    folder : str or pathlib.Path
        The run's folder (created if needed; the two files are replaced).
    """

    def __init__(self, folder: str | Path) -> None:
        """Open the two files, each with no rows yet."""
        out = Path(folder)
        out.mkdir(parents=True, exist_ok=True)
        self._pixels = (out / PIXELS).open("wb")
        self._records = (out / RECORDS).open("wb")
        self._frames = 0
        self._rows = 0
        self._count()

    def __enter__(self) -> FeatureWriter:
        """Return the writer."""
        return self

    def __exit__(self, *exception: object) -> None:
        """Close the files."""
        self.close()

    def write(self, xy: ArrayLike, level: ArrayLike, fate: ArrayLike, point: ArrayLike) -> None:
        """Append one frame's keypoints.

        Parameters
        ----------
        xy : array_like of float, shape (N, 2)
            Their pixels (written as float32).
        level : array_like of int, shape (N,)
            Their pyramid levels.
        fate : array_like of int, shape (N,)
            Their fates (indices in :data:`FATES`).
        point : array_like of int, shape (N,)
            The point each concerns, -1 exactly where unmatched.

        Raises
        ------
        slam_it_all.foundations.errors.InvalidInputError
            If the columns are not one row per keypoint, or a record is malformed (as
            :func:`read_features` checks it). Nothing is written then.
        """
        name = f"features of frame {self._frames}"
        pixels = np.asarray(xy, dtype=np.float64)
        n = len(pixels)
        columns = [np.asarray(column) for column in (level, fate, point)]
        if pixels.shape != (n, 2) or any(column.shape != (n,) for column in columns):
            raise InvalidInputError(f"{name}: expected xy (N, 2) and level, fate, point (N,)")
        if any(not np.issubdtype(column.dtype, np.integer) for column in columns):
            raise InvalidInputError(f"{name}: level, fate and point must be integers")
        _check_keypoints(pixels, columns[0], columns[1], columns[2], name)
        records = np.stack([np.full(n, self._frames), *columns], axis=1)
        self._pixels.write(pixels.astype("<f4").tobytes())
        self._records.write(records.astype("<i4").tobytes())
        self._frames += 1
        self._rows += n
        self._count()

    def close(self) -> None:
        """Close the files."""
        self._pixels.close()
        self._records.close()

    def _count(self) -> None:
        """Rewrite each header with the rows so far, and flush."""
        for file, columns, descr in ((self._pixels, 2, "<f4"), (self._records, 4, "<i4")):
            file.seek(0)
            file.write(_npy_header(self._rows, columns, descr))
            file.seek(0, io.SEEK_END)
            file.flush()


@dataclass(frozen=True)
class Features:
    """A run's feature records: every frame's keypoints, in frame order.

    Attributes
    ----------
    pixels : ndarray of float32, shape (N, 2)
        Each keypoint's pixel.
    records : ndarray of int32, shape (N, 4)
        Its frame (a line of ``diagnostics.jsonl``), its pyramid level, its fate (its index in
        :data:`FATES`) and the point concerned (an id in the run's points, -1 if none).
    """

    pixels: NDArray[np.float32]
    records: NDArray[np.int32]


def read_features(folder: str | Path, run: Run) -> Features:
    """Read a run's feature records, mapped from disk rather than loaded.

    Parameters
    ----------
    folder : str or pathlib.Path
        The run's folder.
    run : Run
        The run, as :func:`read_run` read it from the folder: the records are checked against
        its frames and points.

    Returns
    -------
    Features
        The records.

    Raises
    ------
    FileNotFoundError
        If the run recorded no features, or a file is missing.
    slam_it_all.foundations.errors.InvalidInputError
        If a file is malformed, the two disagree on the rows, the frames are not ascending
        frames of the run, or a record is malformed (as :meth:`FeatureWriter.write` checks
        it) or names a point the run did not make.
    """
    root = Path(folder)
    if not run.record_features:
        raise FileNotFoundError(f"{root}: the run recorded no features")
    pixels = _load(root / PIXELS, (2,), np.float32, mapped=True)
    records = _load(root / RECORDS, (4,), np.int32, mapped=True)
    name = str(root / RECORDS)
    if len(pixels) != len(records):
        raise InvalidInputError(f"{name}: {len(records)} records for {len(pixels)} pixels")
    frame, level, fate, point = records.T
    if np.any(np.diff(frame) < 0) or np.any((frame < 0) | (frame >= len(run.diagnostics))):
        raise InvalidInputError(f"{name}: frames must ascend, each one of the run's")
    _check_keypoints(pixels, level, fate, point, name)
    if np.any(point >= len(run.points)):
        raise InvalidInputError(f"{name}: a point must be one the run made")
    return Features(pixels=pixels, records=records)


Truth = tuple[NDArray[np.float64], NDArray[np.float64]]
"""A sequence's ground truth: its times, shape ``(G,)``, and true ``T_world_cam``, ``(G, 4,
4)``."""


def ground_truth(dataset: str, sequence: str | Path) -> Truth | None:
    """Return a sequence's ground truth, or None if it has none.

    Parameters
    ----------
    dataset : {"tum", "euroc"}
        The dataset.
    sequence : str or pathlib.Path
        The sequence's folder.

    Returns
    -------
    Truth or None
        Its times and true ``T_world_cam``; None if the sequence has no ground-truth file.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the dataset is neither, or the ground truth is malformed.
    """
    if dataset not in ("tum", "euroc"):
        raise InvalidInputError(f"dataset: expected 'tum' or 'euroc', got {dataset!r}")
    try:
        read = tum.read_sequence(sequence) if dataset == "tum" else euroc.read_sequence(sequence)
    except FileNotFoundError:
        return None
    return np.asarray(read.timestamps), np.asarray(read.T_world_cam)


@dataclass(frozen=True)
class Evaluation:
    """A run scored against the ground truth.

    Attributes
    ----------
    matched : int
        The run's frames that had a ground-truth pose within the tolerance.
    absolute : slam_it_all.eval.metrics.AbsoluteError
        The ATE: Sim(3)-aligned for a monocular run, SE(3) otherwise.
    relative : slam_it_all.eval.metrics.RelativeError
        The RPE over one frame period (or the interval asked for), the estimate scaled by the
        ATE's scale.
    """

    matched: int
    absolute: metrics.AbsoluteError
    relative: metrics.RelativeError


def evaluate(
    run: Run,
    timestamps: ArrayLike,
    poses: ArrayLike,
    *,
    max_difference: float = MAX_DIFFERENCE,
    rpe_delta: float | None = None,
) -> Evaluation:
    """Score a run against the ground truth.

    Parameters
    ----------
    run : Run
        The run.
    timestamps : array_like, shape (G,)
        The ground truth's times, ascending.
    poses : array_like, shape (G, 4, 4)
        Its ``T_world_cam``.
    max_difference : float, optional
        The largest time difference, in seconds, between a run frame and its ground truth
        (default :data:`MAX_DIFFERENCE`).
    rpe_delta : float, optional
        The RPE's interval, in seconds (default: the run's frame period, the median spacing
        of its timestamps). Pairs are sought within half a frame period.

    Returns
    -------
    Evaluation
        The ATE and the RPE over the matched frames.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If an array is malformed or its times do not ascend, fewer than three frames match,
        or no matched frame has another `rpe_delta` seconds later.
    """
    truth_times, truth_poses = _as_stamped(timestamps, poses, "ground truth")
    run_times, run_poses = _as_stamped(run.timestamps, run.poses, "run")
    ours, theirs = metrics.associate(run_times, truth_times, max_difference)
    if len(ours) < 3:  # the alignment's minimum
        raise InvalidInputError(f"only {len(ours)} frames have a ground-truth pose; need 3")
    period = float(np.median(np.diff(run_times)))
    absolute = metrics.absolute_error(
        run_poses[ours], truth_poses[theirs], with_scale=run.mode == "monocular"
    )
    relative = metrics.relative_error(
        run_poses[ours],
        truth_poses[theirs],
        run_times[ours],
        delta=period if rpe_delta is None else rpe_delta,
        tolerance=0.5 * period,
        scale=absolute.scale,
    )
    return Evaluation(matched=len(ours), absolute=absolute, relative=relative)
