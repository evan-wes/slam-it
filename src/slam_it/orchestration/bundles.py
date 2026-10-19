"""The run viewer's bundles: one sequence's runs, baked with its images for the page.

A bundle is a folder any plain file server can serve; the page (``viewer/``) reads nothing else.
It holds one sequence's runs in one mode, at most one per backend, and the frames they ran:

- ``bundle.json``: ``{"format": "slam-it-bundle", "version": 4, "dataset", "sequence",
  "mode", "camera", "frames", "first_time", "extent", "images", "truth", "fates", "steps",
  "backends"}``. ``camera`` is the pipeline's (a stereo pair's rectified left camera):
  ``model``, ``width``, ``height``, ``intrinsics`` and ``baseline``. ``frames`` counts the
  bundle's frames and ``first_time`` is the first one's time, in seconds. ``extent`` is the
  whole sequence's, of which the bundle's frames are the part its runs ran: its ``frames``, and
  its first and last frames' times since the bundle's first, ``start`` and ``end``, in seconds,
  so the page can show what no run reached (:func:`_extent`). ``images`` says how
  they were encoded (``chunk``, ``quality``, and ``colour``). ``truth`` says whether the
  sequence has ground truth. ``fates`` and ``steps`` are the run format's tables the bundle was
  baked with (:data:`slam_it.eval.runs.FATES` and :data:`~slam_it.eval.runs.STEPS`),
  which the page checks against its own. ``backends`` maps each backend to what the page's
  header shows (:func:`_backend_info`).
- ``times.npy``: each frame's time since the first, ``(F,)`` float32.
- ``truth.npy``, with ground truth: the true camera centre at each frame (the truth nearest in
  time, within :data:`slam_it.eval.runs.MAX_DIFFERENCE`), ``(F, 3)`` float32, NaN where
  there is none.
- ``images/spans.npy``: each frame's JPEG as a byte range ``[start, end)`` in its chunk, ``(F,
  2)`` int32; ``images/chunk_NNNN.bin``: the JPEGs of frames ``chunk * N`` on, one after
  another, so that scrubbing fetches a few files rather than thousands.
- A folder per backend, named for it:

  - ``records.json``: the run's diagnostics as columns, one value per frame it ran, with
    ``frame``, the bundle's frame of each; its ``map`` column, each frame's map or null, is
    what the page shows a map by;
  - ``poses.npy``: each tracked frame's final ``T_world_cam`` (the run format's
    ``trajectory.txt``), its top three rows, ``(T, 3, 4)``
    float32; ``pose_frames.npy``: their frames, and ``pose_maps.npy``: their maps, each
    ``(T,)`` int32;
  - ``keyframe_poses.npy``, ``(K, 3, 4)`` float32, ``keyframe_lives.npy``, ``(K, 3)`` int32,
    and ``keyframe_maps.npy``, ``(K,)`` int32; ``keyframe_parents.npy``, ``(K,)`` int32, and
    ``keyframe_links.npy``, ``(L, 3)`` int32, the spanning tree and the strong links as each
    map ended (each keyframe's last parent; a map's links when it closed or the run ended), and
    ``keyframe_loops.npy``, ``(E, 2)`` int32, the loops closed, which the page draws through the
    run while both their keyframes live;
    ``points.npy``, ``(N, 3)`` float32,
    ``point_lives.npy``, ``(N, 3)`` int32, and ``point_maps.npy``, ``(N,)`` int32: every
    keyframe and point the run made, as the run format holds them (:mod:`slam_it.eval.runs`);
  - ``feature_pixels.npy``, ``(M, 2)`` float32, and ``feature_records.npy``, ``(M, 4)``
    int32, if the run recorded its features.

  Every frame number in them is the bundle's (an index into ``times.npy``), not the run's.

**Frames.** The bundle's frames are every frame a run ran, matched by time, in the sequence's
order; each backend covers a contiguous range of them. **Alignment.** With ground truth, each
of a run's maps (its trajectory, keyframes and points) is moved into the truth's frame by its
own ATE alignment (:func:`slam_it.eval.runs.evaluate`: Sim(3) for monocular, SE(3)
otherwise), so the maps and the backends overlay, and the run's header names the alignment
(``"sim3"`` or ``"se3"``). A map that alignment cannot place (fewer than three frames matched
to the truth, as a monocular run that never initialised, or its centres on a line) stays in its
own frame, unscored, as every map does without ground truth; a run none of whose maps is placed
has a null alignment. **Images** are the frames as the pipeline saw
them (:attr:`slam_it.orchestration.vo.Source.view`: a stereo pair's left image
rectified), in colour where the sequence has colour, as JPEG
(:func:`slam_it.eval.images.encode_jpeg`). Every array is little-endian, so the page reads
each ``.npy`` file's data straight into a typed array after its header. **Writing.** The bundle
is written into a hidden folder beside ``out`` and moved onto it once whole, so a bake that
fails leaves nothing behind.

Run as a program::

    python -m slam_it.orchestration.bundles RUN_DIR [RUN_DIR ...] --sequence PATH --out BUNDLE
"""

from __future__ import annotations

import argparse
import io
import json
import shutil
import sys
import uuid
from collections.abc import Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it.eval import budgets, images, metrics, runs
from slam_it.foundations.errors import DegenerateError, InvalidInputError
from slam_it.orchestration import vo

FORMAT = "slam-it-bundle"
"""``bundle.json``'s ``format``."""

VERSION = 4
"""``bundle.json``'s ``version``: this module's layout (2 added the maps; 3 the spanning tree and
the strong links; 4 the loops closed, and the trajectory the run format's final one)."""

CHUNK = 100
"""The frames whose JPEGs share a chunk file."""

QUALITY = 75
"""The JPEG quality the frames are encoded with: 34 to 51 KB a frame of EuRoC's (MH_01 stereo's
3,682 rectified frames in 163 MB) and 35 to 37 KB of TUM's colour ones; 90 would take 65 to 77%
more."""


@dataclass(frozen=True)
class _Baked:
    """A run as it enters the bundle: its folder, what was read from it, and its score."""

    folder: Path
    run: runs.Run
    features: runs.Features | None
    scored: runs.Evaluation | None = None


@dataclass(frozen=True)
class _Frames:
    """The bundle's frames: their times, and each one's index in the sequence."""

    times: NDArray[np.float64]
    at: NDArray[np.int32]


def _score(run: runs.Run, truth: runs.Truth | None) -> runs.Evaluation | None:
    """Score a run against the ground truth; None without one, or if it cannot be placed.

    Each map is placed by its ATE alone, and given an RPE where it has a pair of frames a period
    apart (:func:`slam_it.eval.runs.evaluate`). A run none of whose maps can be placed, each
    with fewer than three frames matched to the truth or its centres on a line
    (:class:`~slam_it.foundations.errors.DegenerateError`), stays in its own frame rather than
    failing the bundle. The run and the truth were checked when read, so any other refusal is a
    fault, and is raised.
    """
    if truth is None:
        return None
    try:
        return runs.evaluate(run, *truth)
    except DegenerateError:
        return None


def _read(folders: Sequence[str | Path]) -> list[_Baked]:
    """Read the runs, refusing any that are not of one sequence and mode, one per backend."""
    if not folders:
        raise InvalidInputError("runs: expected at least one run folder")
    baked = []
    for folder in map(Path, folders):
        run = runs.read_run(folder)
        features = runs.read_features(folder, run) if run.record_features else None
        baked.append(_Baked(folder, run, features))
    first = baked[0].run
    for b in baked[1:]:
        if (b.run.dataset, b.run.sequence, b.run.mode) != (
            first.dataset,
            first.sequence,
            first.mode,
        ):
            raise InvalidInputError(
                f"{b.folder}: a run of {b.run.dataset} {b.run.sequence} ({b.run.mode}), not of "
                f"{first.dataset} {first.sequence} ({first.mode}) as {baked[0].folder}"
            )
    backends = [b.run.backend for b in baked]
    if len(set(backends)) != len(backends):
        raise InvalidInputError(f"runs: expected one run per backend, got {sorted(backends)}")
    return baked


def _frames_of(times: NDArray[np.float64], run_times: ArrayLike, name: str) -> NDArray[np.int32]:
    """Return the bundle's frame at each of a run's times, refusing one the bundle lacks."""
    wanted = np.asarray(run_times, dtype=np.float64)
    at = np.searchsorted(times, wanted).clip(0, max(len(times) - 1, 0))
    if len(wanted) and (len(times) == 0 or np.any(times[at] != wanted)):
        raise InvalidInputError(f"{name}: a frame's time is not one of the sequence's frames")
    return at.astype(np.int32)


def _bundle_frames(baked: Sequence[_Baked], source: vo.Source, name: str) -> _Frames:
    """Return the bundle's frames: every frame a run ran, each run's a range of them."""
    ran = np.unique(np.concatenate([[d["timestamp"] for d in b.run.diagnostics] for b in baked]))
    for b in baked:
        frame = _frames_of(ran, [d["timestamp"] for d in b.run.diagnostics], str(b.folder))
        if len(frame) and frame[-1] - frame[0] + 1 != len(frame):
            raise InvalidInputError(f"{b.folder}: its frames are not a range of the bundle's")
    return _Frames(ran, _frames_of(source.times, ran, name))


Alignments = dict[int, NDArray[np.float64]]
"""Each scored map's alignment, a similarity ``[[s R, t], [0, 1]]``, by the map's number."""


def _aligned(
    poses: NDArray[np.float64], maps: NDArray[np.int64], alignments: Alignments
) -> NDArray[np.float32]:
    """Move ``T_world_cam`` poses each by its map's alignment, if any; return their top rows."""
    out = poses[:, :3].copy()
    for map_id, alignment in alignments.items():
        rows = maps == map_id
        scaled, shift = alignment[:3, :3], alignment[:3, 3]
        rotation = scaled / np.cbrt(np.linalg.det(scaled))
        out[rows, :, :3] = rotation @ poses[rows, :3, :3]
        out[rows, :, 3] = poses[rows, :3, 3] @ scaled.T + shift
    return out.astype(np.float32)


def _moved(
    points: NDArray[np.float64], maps: NDArray[np.int64], alignments: Alignments
) -> NDArray[np.float32]:
    """Move points each by its map's alignment, if any; return them as float32."""
    out = points.copy()
    for map_id, alignment in alignments.items():
        rows = maps == map_id
        out[rows] = points[rows] @ alignment[:3, :3].T + alignment[:3, 3]
    return out.astype(np.float32)


def _lives(lives: NDArray[np.int64], frame: NDArray[np.int32]) -> NDArray[np.int32]:
    """Renumber lives' frames from the run's to the bundle's."""
    out = lives.astype(np.int32)
    out[:, 0] = frame[lives[:, 0]]
    ended = lives[:, 1] >= 0
    out[ended, 1] = frame[lives[ended, 1]]
    return out


def _npy(path: Path, array: NDArray[Any]) -> None:
    """Write an array as a ``.npy`` file, its parents created."""
    path.parent.mkdir(parents=True, exist_ok=True)
    buffer = io.BytesIO()
    np.save(buffer, np.ascontiguousarray(array), allow_pickle=False)
    path.write_bytes(buffer.getvalue())


def _score_info(error: runs.MapError) -> dict[str, Any]:
    """Return a map's score as the header shows it: its ATE, RPE and scale."""
    relative = error.relative
    return {
        "matched": error.matched,
        "ate_rmse": error.absolute.rmse,
        "rpe_translation_rmse": None if relative is None else relative.translation_rmse,
        "rpe_rotation_rmse": None if relative is None else relative.rotation_rmse,
        "scale": error.absolute.scale,
    }


def _maps_info(baked: _Baked, frames: NDArray[np.int32]) -> list[dict[str, Any]]:
    """Return each of a run's maps as the page shows it, in order (see :func:`_backend_info`)."""
    run, scored = baked.run, baked.scored
    errors = {} if scored is None else {e.map: e for e in scored.maps}
    named = np.array([-1 if d.get("map") is None else d["map"] for d in run.diagnostics])
    reset = set(run.reset_maps().tolist())
    out = []
    for map_id in (int(m) for m in run.maps()):
        at = frames[named == map_id]  # read_run holds every map named on a frame
        error = errors.get(map_id)
        out.append(
            {
                "id": map_id,
                "frames": [int(at[0]), int(at[-1]) + 1],
                "tracked": int(np.count_nonzero(run.pose_maps == map_id)),
                "reset": map_id in reset,
                "score": None if error is None else _score_info(error),
            }
        )
    return out


def _backend_info(baked: _Baked, frames: NDArray[np.int32]) -> dict[str, Any]:
    """Return what the page's header shows of a run.

    The frames it covers (``[first, end)``), the frames it tracked, the final maps' keyframes
    and points and how many it made, whether it recorded features, the alignment its maps were
    moved by (``"sim3"``, ``"se3"``, or null if none was), its score over every map placed
    (``matched``, ``ate_rmse``, ``rpe_translation_rmse``, ``rpe_rotation_rmse``, the pooled
    root mean squares, an RPE null where no map has one; null unless a map was placed), its
    maps (``id``, the ``frames`` whose records name it, ``[first, end)``, the frames it
    ``tracked``, whether it was ``reset``, and its own ``score``, with its ``scale``, null
    unless placed), its latency against the frames' deadline (``deadline``, ``median``,
    ``p95``, ``max``, ``late``; null for a run of fewer than two frames) and its peak memory.
    """
    run, scored = baked.run, baked.scored
    latency = None
    if len(run.diagnostics) >= 2:  # a deadline needs a frame spacing
        judged = budgets.judge(run)
        latency = {
            "deadline": judged.deadline,
            "median": judged.latency.median,
            "p95": judged.latency.p95,
            "max": judged.latency.max,
            "late": judged.late,
        }
    alignment = score = None
    if scored is not None:
        alignment = "sim3" if scored.maps[0].absolute.similarity else "se3"
        score = {
            "matched": scored.matched,
            "ate_rmse": scored.ate_rmse,
            "rpe_translation_rmse": scored.rpe_translation_rmse,
            "rpe_rotation_rmse": scored.rpe_rotation_rmse,
        }
    return {
        "frames": [int(frames[0]), int(frames[-1]) + 1] if len(frames) else [0, 0],
        "tracked": len(run.timestamps),
        "keyframes": len(run.final_keyframes()[0]),
        "points": len(run.final_points()),
        "made": {"keyframes": len(run.keyframe_timestamps), "points": len(run.points)},
        "features": baked.features is not None,
        "alignment": alignment,
        "score": score,
        "maps": _maps_info(baked, frames),
        "latency": latency,
        "peak_memory": run.peak_memory,
    }


def _write_backend(out: Path, baked: _Baked, times: NDArray[np.float64]) -> dict[str, Any]:
    """Write a run's folder in the bundle, whose frames are at `times`; return its entry."""
    run = baked.run
    folder = out / run.backend
    frame = _frames_of(times, [d["timestamp"] for d in run.diagnostics], str(baked.folder))
    scored = baked.scored
    alignments = {} if scored is None else {e.map: e.absolute.alignment for e in scored.maps}
    columns: dict[str, list[Any]] = {"frame": frame.tolist()}
    for key in dict.fromkeys(k for d in run.diagnostics for k in d):
        columns[key] = [d.get(key) for d in run.diagnostics]
    folder.mkdir(parents=True)
    (folder / "records.json").write_text(json.dumps(columns, separators=(",", ":")) + "\n")
    _npy(folder / "poses.npy", _aligned(run.poses, run.pose_maps, alignments))
    _npy(folder / "pose_frames.npy", _frames_of(times, run.timestamps, str(baked.folder)))
    _npy(folder / "pose_maps.npy", run.pose_maps.astype(np.int32))
    keyframes = _aligned(run.keyframe_poses, run.keyframe_maps, alignments)
    _npy(folder / "keyframe_poses.npy", keyframes)
    _npy(folder / "keyframe_lives.npy", _lives(run.keyframe_lives, frame))
    _npy(folder / "keyframe_maps.npy", run.keyframe_maps.astype(np.int32))
    _npy(folder / "keyframe_parents.npy", run.keyframe_parents.astype(np.int32))
    _npy(folder / "keyframe_links.npy", run.keyframe_links.astype(np.int32).reshape(-1, 3))
    _npy(folder / "keyframe_loops.npy", run.keyframe_loops.astype(np.int32).reshape(-1, 2))
    _npy(folder / "points.npy", _moved(run.points, run.point_maps, alignments))
    _npy(folder / "point_lives.npy", _lives(run.point_lives, frame))
    _npy(folder / "point_maps.npy", run.point_maps.astype(np.int32))
    if baked.features is not None:
        records = np.array(baked.features.records, dtype=np.int32)
        records[:, 0] = frame[records[:, 0]]
        _npy(folder / runs.PIXELS, np.asarray(baked.features.pixels))
        _npy(folder / runs.RECORDS, records)
    return _backend_info(baked, frame)


def _write_images(out: Path, source: vo.Source, at: NDArray[np.int32]) -> bool:
    """Write the frames' JPEGs in chunks and their spans; return whether they are in colour."""
    spans = np.zeros((len(at), 2), dtype=np.int32)
    colour = False
    (out / "images").mkdir(parents=True)
    for start in range(0, len(at), CHUNK):
        chunk = bytearray()
        for k in range(start, min(start + CHUNK, len(at))):
            picture = source.view(int(at[k]))
            colour = picture.ndim == 3  # (H, W, 3)
            encoded = images.encode_jpeg(picture, QUALITY)
            spans[k] = [len(chunk), len(chunk) + len(encoded)]
            chunk += encoded
        (out / "images" / f"chunk_{start // CHUNK:04d}.bin").write_bytes(bytes(chunk))
    _npy(out / "images" / "spans.npy", spans)
    return colour


def _write_truth(out: Path, times: NDArray[np.float64], truth: runs.Truth) -> None:
    """Write the true camera centre at each of the bundle's frames, NaN where there is none."""
    centres = np.full((len(times), 3), np.nan, dtype=np.float32)
    ours, theirs = metrics.associate(times, truth[0], runs.MAX_DIFFERENCE)
    centres[ours] = truth[1][theirs, :3, 3]
    _npy(out / "truth.npy", centres)


def _extent(times: NDArray[np.float64], first_time: float) -> dict[str, Any]:
    """Return the whole sequence's extent: its frames, its first and last times since `first_time`.

    A run goes on past a lost track, in a new map, but may be given a part of the sequence, so
    the bundle's frames can be a part of the sequence's: the page shows the rest as not run.
    """
    if len(times) == 0:
        return {"frames": 0, "start": 0.0, "end": 0.0}
    return {
        "frames": len(times),
        "start": float(times[0] - first_time),
        "end": float(times[-1] - first_time),
    }


def _write(
    out: Path, baked: Sequence[_Baked], source: vo.Source, frames: _Frames, truth: runs.Truth | None
) -> None:
    """Write a bundle's files into a new folder `out`."""
    first, ran = baked[0].run, frames.times
    first_time = float(ran[0]) if len(ran) else 0.0
    colour = _write_images(out, source, frames.at)
    _npy(out / "times.npy", (ran - ran[0] if len(ran) else ran).astype(np.float32))
    if truth is not None:
        _write_truth(out, ran, truth)
    camera = source.camera
    info: dict[str, Any] = {
        "format": FORMAT,
        "version": VERSION,
        "dataset": first.dataset,
        "sequence": first.sequence,
        "mode": first.mode,
        "camera": {
            "model": camera.model,
            "width": camera.width,
            "height": camera.height,
            "intrinsics": camera.intrinsics.tolist(),
            "baseline": camera.baseline,
        },
        "frames": len(ran),
        "first_time": first_time,
        "extent": _extent(source.times, first_time),
        "images": {"chunk": CHUNK, "quality": QUALITY, "colour": colour},
        "truth": truth is not None,
        "fates": list(runs.FATES),
        "steps": list(runs.STEPS),
        "backends": {b.run.backend: _write_backend(out, b, ran) for b in baked},
    }
    (out / "bundle.json").write_text(json.dumps(info, indent=1) + "\n")


def bake(run_folders: Sequence[str | Path], sequence: str | Path, out: str | Path) -> None:
    """Bake runs of one sequence, one per backend, into a bundle (the module docstring).

    Parameters
    ----------
    run_folders : sequence of str or pathlib.Path
        The runs' folders.
    sequence : str or pathlib.Path
        The sequence's folder, as the runs read it.
    out : str or pathlib.Path
        The bundle's folder: created; refused if it exists and is not empty. The bundle is
        written beside it, in a hidden ``.<name>.<id>.baking`` folder, and moved onto it whole,
        so a bake that fails leaves nothing. (A bake killed outright leaves that folder, which
        nothing reads: it is safe to delete.)

    Raises
    ------
    FileNotFoundError
        If a run's or the sequence's file is missing.
    slam_it.foundations.errors.InvalidInputError
        If there are no runs, they are not of one sequence and mode, two are of one backend,
        the sequence's folder is not theirs, a run's frame is not one of the sequence's, or
        `out` is not empty; or a reader's error. A run its ATE alignment cannot place is not
        an error: it stays in its own frame.
    """
    baked = _read(run_folders)
    first = baked[0].run
    root, target = Path(sequence), Path(out)
    if root.resolve().name != first.sequence:
        raise InvalidInputError(f"sequence: the runs are of {first.sequence}, not {root.name}")
    if target.exists() and any(target.iterdir()):
        raise InvalidInputError(f"{target}: expected a new or empty folder for the bundle")
    source = vo.open_source(first.dataset, root, first.mode)
    frames = _bundle_frames(baked, source, str(root))
    truth = runs.ground_truth(first.dataset, root)
    baked = [replace(b, scored=_score(b.run, truth)) for b in baked]
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.with_name(f".{target.name}.{uuid.uuid4().hex}.baking")
    staging.mkdir()
    try:
        _write(staging, baked, source, frames, truth)
        staging.replace(target)  # onto a new or empty folder
    except BaseException:
        shutil.rmtree(staging)  # the bake's own, made above
        raise


def main(arguments: Sequence[str]) -> int:
    """Bake a bundle from the command line (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, with a one-line summary on stdout.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it.orchestration.bundles")
    parser.add_argument("runs", nargs="+", type=Path, help="the runs' folders, one per backend")
    parser.add_argument("--sequence", type=Path, required=True, help="the sequence's folder")
    parser.add_argument("--out", type=Path, required=True, help="the bundle's new folder")
    args = parser.parse_args(arguments)
    bake(args.runs, args.sequence, args.out)
    info = json.loads((args.out / "bundle.json").read_text())
    print(
        f"{args.out}: {info['sequence']} ({info['mode']}), {info['frames']} frames, "
        f"backends {', '.join(info['backends'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
