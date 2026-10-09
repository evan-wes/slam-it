"""The run format: what a pipeline writes, and the evaluation and the run viewer read.

A run is a folder, written by any backend's pipeline (C++ and Rust write the same files):

- ``run.json``: what ran: ``{"format": "slam-it-all-run", "version": 2, "dataset", "sequence",
  "mode", "backend", "settings", "peak_memory", "cpus", "memory_cap"}``, ``mode`` one of
  :data:`MODES` and ``backend`` one of ``python``, ``cpp`` and ``rust``; ``settings`` holds the
  pipeline's options as JSON. The last three describe the program that ran it, as it recorded
  them before writing (:mod:`slam_it_all.eval.budgets`), and are null when the run was not a
  program's of its own (Python's in-process run) or the system does not say: ``peak_memory``,
  its largest resident set in bytes; ``cpus``, the CPUs it could run on, ascending (a budget
  pins them); ``memory_cap``, its control group's memory cap in bytes (null without one).
- ``trajectory.txt``: each tracked frame's ``T_world_cam`` in TUM's format, one
  ``timestamp tx ty tz qx qy qz qw`` line per frame, ``#`` comments allowed: what ORB-SLAM and
  the TUM tools write, so other systems' trajectories evaluate the same way. Numbers are
  written to round-trip exactly.
- ``keyframes.npy``: the keyframes' timestamps, ``(K,)`` float64; ``keyframe_poses.npy``: their
  ``T_world_cam``, ``(K, 4, 4)``.
- ``points.npy``: the map's points in the world frame, ``(M, 3)`` float64.
- ``diagnostics.jsonl``: one JSON object per line, per frame, each with its ``timestamp``;
  otherwise what the pipeline wants to report (tracked points, state). The edge pipeline's
  frames also time their steps, in seconds (:data:`STEPS`), which the budget harness judges
  (:func:`slam_it_all.eval.budgets.judge`).

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

from slam_it_all.eval import metrics
from slam_it_all.eval.timestamps import as_times
from slam_it_all.foundations import se3, so3
from slam_it_all.foundations.arrays import as_batch
from slam_it_all.foundations.errors import InvalidInputError

FORMAT = "slam-it-all-run"
"""``run.json``'s ``format``."""

VERSION = 2
"""``run.json``'s ``version``: this module's layout (2 added ``peak_memory``, ``cpus`` and
``memory_cap``)."""

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
        The keyframes' timestamps.
    keyframe_poses : ndarray of float64, shape (K, 4, 4)
        The keyframes' ``T_world_cam``.
    points : ndarray of float64, shape (M, 3)
        The map's points, in the world frame.
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


def _npy(array: NDArray[np.float64]) -> bytes:
    """Return an array as the bytes of a ``.npy`` file."""
    buffer = io.BytesIO()
    np.save(buffer, array, allow_pickle=False)
    return buffer.getvalue()


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
        memory or memory cap that is not a whole number of bytes, CPUs not ascending). Nothing
        is written then.
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
    _check_process(run.peak_memory, run.cpus, run.memory_cap, "")
    info = {
        "format": FORMAT,
        "version": VERSION,
        "dataset": run.dataset,
        "sequence": run.sequence,
        "mode": run.mode,
        "backend": run.backend,
        "settings": run.settings,
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
    out = Path(folder)
    out.mkdir(parents=True, exist_ok=True)
    _replace(out / "run.json", (json.dumps(info, indent=1) + "\n").encode())
    _replace(out / "trajectory.txt", _trajectory_text(times, poses).encode())
    _replace(out / "keyframes.npy", _npy(keyframe_times))
    _replace(out / "keyframe_poses.npy", _npy(keyframe_poses))
    _replace(out / "points.npy", _npy(points))
    _replace(out / "diagnostics.jsonl", "".join(diagnostics).encode())


def _check_names(mode: str, backend: str) -> None:
    """Check a run's mode and backend are known."""
    if mode not in MODES:
        raise InvalidInputError(f"mode: expected one of {MODES}, got {mode!r}")
    if backend not in BACKENDS:
        raise InvalidInputError(f"backend: expected one of {BACKENDS}, got {backend!r}")


def _load(path: Path, shape: tuple[int, ...]) -> NDArray[np.float64]:
    """Load a float64 .npy file whose trailing dimensions are `shape`."""
    if not path.is_file():
        raise FileNotFoundError(f"no {path.name} in {path.parent}")
    try:
        data: NDArray[np.float64] = np.load(path, allow_pickle=False)
    except (ValueError, EOFError) as error:  # not an .npy file, an empty one, or objects
        raise InvalidInputError(f"{path}: not a readable .npy array ({error})") from None
    if not isinstance(data, np.ndarray) or data.dtype != np.float64 or data.shape[1:] != shape:
        expected = ", ".join(["N", *map(str, shape)])
        raise InvalidInputError(
            f"{path}: expected float64 ({expected}), got {data.dtype} {data.shape}"
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
    return Run(
        dataset=info["dataset"],
        sequence=info["sequence"],
        mode=info["mode"],
        backend=info["backend"],
        timestamps=times,
        poses=poses,
        keyframe_timestamps=keyframe_times,
        keyframe_poses=keyframe_poses,
        points=_load(root / "points.npy", (3,)),
        settings=info["settings"],
        diagnostics=_read_diagnostics(root / "diagnostics.jsonl"),
        peak_memory=info["peak_memory"],
        cpus=info["cpus"],
        memory_cap=info["memory_cap"],
    )


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
