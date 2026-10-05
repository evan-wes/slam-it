"""The run format: what a pipeline writes, and the evaluation and the run viewer read.

A run is a folder, written by any backend's pipeline (C++ and Rust write the same files):

- ``run.json``: what ran: ``{"format": "slamlab-run", "version": 1, "dataset", "sequence",
  "mode", "backend", "settings"}``, ``mode`` one of :data:`MODES` and ``backend`` one of
  ``python``, ``cpp`` and ``rust``; ``settings`` holds the pipeline's options as JSON.
- ``trajectory.txt``: each tracked frame's ``T_world_cam`` in TUM's format, one
  ``timestamp tx ty tz qx qy qz qw`` line per frame, ``#`` comments allowed: what ORB-SLAM and
  the TUM tools write, so other systems' trajectories evaluate the same way. Numbers are
  written to round-trip exactly.
- ``keyframes.npy``: the keyframes' timestamps, ``(K,)`` float64; ``keyframe_poses.npy``: their
  ``T_world_cam``, ``(K, 4, 4)``.
- ``points.npy``: the map's points in the world frame, ``(M, 3)`` float64.
- ``diagnostics.jsonl``: one JSON object per line, per frame, free-form but each with a
  ``timestamp``; what the pipeline wants to report (tracked points, state, time spent).

:func:`read_run` checks every file's shape; :func:`evaluate` scores a run against the ground
truth.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab.eval import metrics
from slamlab.foundations import se3, so3
from slamlab.foundations.arrays import as_batch, as_real
from slamlab.foundations.errors import InvalidInputError

FORMAT = "slamlab-run"
"""``run.json``'s ``format``."""

VERSION = 1
"""``run.json``'s ``version``: this module's layout."""

MODES = ("monocular", "stereo", "rgbd")
"""A run's sensor modes."""

BACKENDS = ("python", "cpp", "rust")
"""The backends a run may come from."""


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


def _as_stamped(
    timestamps: ArrayLike, poses: ArrayLike, name: str
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Check stamped poses: ascending finite times, one rigid transform each."""
    times = as_real(timestamps, f"{name} timestamps")
    flat, lead = as_batch(poses, (4, 4), f"{name} poses")
    if times.ndim != 1 or lead != (len(times),):
        raise InvalidInputError(f"{name}: expected (F,) timestamps and (F, 4, 4) poses")
    if np.any(np.diff(times) <= 0.0):
        raise InvalidInputError(f"{name}: timestamps must ascend strictly")
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
    slamlab.foundations.errors.InvalidInputError
        If the times do not ascend or a pose is not a rigid transform.
    """
    times, flat = _as_stamped(timestamps, poses, "trajectory")
    quaternions = so3.to_quaternion(flat[:, :3, :3]) if len(flat) else np.zeros((0, 4))
    lines = ["# timestamp tx ty tz qx qy qz qw"]
    for t, pose, q in zip(times, flat, quaternions, strict=True):
        values = [t, *pose[:3, 3], *q]
        lines.append(" ".join(repr(float(v)) for v in values))
    Path(path).write_text("\n".join(lines) + "\n")


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
    slamlab.foundations.errors.InvalidInputError
        If a line is not 8 finite numbers with a non-zero quaternion, or the times do not
        ascend.
    """
    file = Path(path)
    rows = []
    for number, line in enumerate(file.read_text().splitlines(), start=1):
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
    slamlab.foundations.errors.InvalidInputError
        If the mode or backend is unknown, or an array is malformed.
    """
    _check_names(run.mode, run.backend)
    keyframe_times, keyframe_poses = _as_stamped(
        run.keyframe_timestamps, run.keyframe_poses, "keyframes"
    )
    points, lead = as_batch(run.points, (3,), "points")
    if len(lead) != 1:
        raise InvalidInputError(f"points: expected (M, 3), got {lead}")
    out = Path(folder)
    out.mkdir(parents=True, exist_ok=True)
    info = {
        "format": FORMAT,
        "version": VERSION,
        "dataset": run.dataset,
        "sequence": run.sequence,
        "mode": run.mode,
        "backend": run.backend,
        "settings": run.settings,
    }
    (out / "run.json").write_text(json.dumps(info, indent=1) + "\n")
    write_trajectory(out / "trajectory.txt", run.timestamps, run.poses)
    np.save(out / "keyframes.npy", keyframe_times)
    np.save(out / "keyframe_poses.npy", keyframe_poses)
    np.save(out / "points.npy", points)
    lines = [json.dumps(entry) for entry in run.diagnostics]
    (out / "diagnostics.jsonl").write_text("".join(line + "\n" for line in lines))


def _check_names(mode: str, backend: str) -> None:
    """Check a run's mode and backend are known."""
    if mode not in MODES:
        raise InvalidInputError(f"mode: expected one of {MODES}, got {mode!r}")
    if backend not in BACKENDS:
        raise InvalidInputError(f"backend: expected one of {BACKENDS}, got {backend!r}")


def _load(path: Path, shape: tuple[int, ...]) -> NDArray[np.float64]:
    """Load a float64 .npy file whose trailing dimensions are `shape`."""
    data: NDArray[np.float64] = np.load(path, allow_pickle=False)
    if data.dtype != np.float64 or data.shape[1:] != shape:
        raise InvalidInputError(
            f"{path}: expected float64 (N, {shape}), got {data.dtype} {data.shape}"
        )
    return data


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
    slamlab.foundations.errors.InvalidInputError
        If ``run.json`` is not this format's version, or a file is malformed.
    """
    root = Path(folder)
    info = json.loads((root / "run.json").read_text())
    if info.get("format") != FORMAT or info.get("version") != VERSION:
        raise InvalidInputError(
            f"{root / 'run.json'}: expected format {FORMAT!r} version {VERSION}, got "
            f"{info.get('format')!r} version {info.get('version')!r}"
        )
    _check_names(info["mode"], info["backend"])
    times, poses = read_trajectory(root / "trajectory.txt")
    keyframe_times = _load(root / "keyframes.npy", ())
    keyframe_poses = _load(root / "keyframe_poses.npy", (4, 4))
    _as_stamped(keyframe_times, keyframe_poses, "keyframes")
    diagnostics = []
    for number, line in enumerate((root / "diagnostics.jsonl").read_text().splitlines(), start=1):
        entry = json.loads(line)
        if not isinstance(entry, dict) or "timestamp" not in entry:
            raise InvalidInputError(
                f"{root / 'diagnostics.jsonl'}:{number}: expected an object with a timestamp"
            )
        diagnostics.append(entry)
    return Run(
        dataset=str(info["dataset"]),
        sequence=str(info["sequence"]),
        mode=info["mode"],
        backend=info["backend"],
        timestamps=times,
        poses=poses,
        keyframe_timestamps=keyframe_times,
        keyframe_poses=keyframe_poses,
        points=_load(root / "points.npy", (3,)),
        settings=dict(info.get("settings", {})),
        diagnostics=diagnostics,
    )


@dataclass(frozen=True)
class Evaluation:
    """A run scored against the ground truth.

    Attributes
    ----------
    matched : int
        The run's frames that had a ground-truth pose within the tolerance.
    absolute : slamlab.eval.metrics.AbsoluteError
        The ATE: Sim(3)-aligned for a monocular run, SE(3) otherwise.
    relative : slamlab.eval.metrics.RelativeError
        The frame-to-frame RPE, the estimate scaled by the ATE's scale.
    """

    matched: int
    absolute: metrics.AbsoluteError
    relative: metrics.RelativeError


def evaluate(
    run: Run,
    timestamps: ArrayLike,
    poses: ArrayLike,
    *,
    max_difference: float = 0.01,
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
        The largest time difference, in seconds, between a run frame and its ground truth.

    Returns
    -------
    Evaluation
        The ATE and the frame-to-frame RPE over the matched frames.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If an array is malformed, or fewer than three frames match.
    """
    truth_times, truth_poses = _as_stamped(timestamps, poses, "ground truth")
    if not (math.isfinite(max_difference) and max_difference >= 0.0):
        raise InvalidInputError(f"max_difference: must be non-negative, got {max_difference}")
    ours, theirs = metrics.associate(run.timestamps, truth_times, max_difference)
    if len(ours) < 3:  # the alignment's minimum
        raise InvalidInputError(f"only {len(ours)} frames have a ground-truth pose; need 3")
    absolute = metrics.absolute_error(
        run.poses[ours], truth_poses[theirs], with_scale=run.mode == "monocular"
    )
    relative = metrics.relative_error(run.poses[ours], truth_poses[theirs], scale=absolute.scale)
    return Evaluation(matched=len(ours), absolute=absolute, relative=relative)
