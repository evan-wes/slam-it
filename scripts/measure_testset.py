"""Measure the test set: each sequence of a config run, scored against its reference, tabled.

Milestone 2A's test set (``docs/design/2026-10-12-phone-robust-vo.md``) is public datasets'
sequences imported to TUM format (:mod:`slam_it.eval.zju`, :mod:`slam_it.eval.advio`,
:mod:`slam_it.eval.arkitscenes`, and TUM's own folders). A config file lists them. It is kept
outside the repository, since it holds this machine's paths; ``scripts/testset.template.toml``
is its format::

    [[sequence]]
    name = "advio-07"
    folders = ["/data/imported/advio-07"]
    mode = "monocular"
    reference = "ADVIO: IMU and fixed points"

A sequence's ``folders`` are a list of one for a single sequence. A sequence may also name its
``dataset``, ``"tum"`` (the default) or ``"euroc"``: a EuRoC sequence's own folder, run as
EuRoC (monocular or stereo; :func:`slam_it.eval.euroc`). Several ``folders`` are sequences of
one camera run in turn in one run, each a session (:mod:`slam_it.orchestration.vo`): a
session's times are moved on by its offset where they do not come after those before, its
ground truth with them, and the sequences' ground truths are scored as one, so the run's maps
are aligned once each across the sessions (the online-SLAM design's section 15)::

    [[sequence]]
    name = "euroc-MH01-03-stereo"
    folders = ["/data/euroc/MH_01_easy", "/data/euroc/MH_02_easy", "/data/euroc/MH_03_medium"]
    mode = "stereo"
    reference = "EuRoC: Vicon or Leica, processed"
    dataset = "euroc"

Each sequence runs on one backend (:func:`slam_it.orchestration.vo.run`) with ORB-SLAM's
settings and, unless ``--features default``, the features for its frame size, as the demo's
worker sets a run up (:func:`slam_it.orchestration.vo.options_for`), then is measured:

- ``frames``: the sequence's frames for its mode (every session's); ``run``: those run (an
  RGB-D frame without a depth map is skipped); a lost track closes its map and the next frame
  starts a new one, so a run goes on to the sequence's end;
- ``sessions``: each sequence's ``sequence`` (its folder's name), its time ``offset``, its
  ``frames``, those ``tracked``, the ``maps`` they end in (the final trajectory's), and how the
  map tracked as it began was ``closed`` (the run's diagnostics; null for the first); one
  session for one folder. ``maps``: the maps the final trajectory's frames end in, all
  sessions' (section 15's bar: every session ending in one map);
- ``tracked`` and ``share``: the frames tracked, in every map, and their share of the
  sequence's;
- ``segments``: each stretch of frames tracked in one map, ``map``, ``tracked`` and its first
  and last tracked frames' times, ``tracked_from`` and ``tracked_to``
  (:func:`slam_it.eval.runs.tracking`); ``losses``, the frames the track was lost on,
  ``resets``, the maps lost too soon to keep, and ``relocalizations``, the frames that tracking
  failed and relocalisation found, in their map or in a kept one; and ``tracked_from``,
  ``tracked_to`` and ``ran_to`` for the whole run: every time since the sequence's first frame
  (:func:`slam_it.eval.runs.tracking`);
- ``ate``: the RMSE in metres over the tracked frames matched to the reference, each map aligned
  on its own, Sim(3) for monocular and SE(3) otherwise, with ``matched`` and the ``reference``
  named; each map's own in ``map_ate`` (``map``, ``matched``, ``ate``), and in
  ``unscored_maps`` the maps too short or straight to align; ``ate`` null, with ``unscored``
  saying why, if no map can be scored. It is the final trajectory's (run format 7),
  ``ate_tracked`` the poses' as tracked, and ``ate_keyframes`` the final keyframes', scored the
  same way: ORB-SLAM's paper reports its monocular TUM results by its keyframes. It is scored alone
  (:func:`slam_it.eval.runs.evaluate_absolute`): the measure reports no relative error, and a
  reference too sparse for one still scores;
- ``loops``: each loop closed, ``keyframes`` (the older first), judged against the reference
  by :func:`slam_it.eval.loops.judge_loops`, as the loop room's test judges its loops:
  ``distance`` between the two keyframes by the reference and ``angle`` (degrees) between their
  viewing directions, and ``wrong_place``; the two keyframes' relative pose after the run
  against the reference's, ``translation_error`` and ``rotation_error`` (degrees), and
  ``poor_pose`` (the online-SLAM design's section 10). Each judgement is null where the
  reference has no pose within :data:`slam_it.eval.runs.MAX_DIFFERENCE` of a keyframe, or the
  map was not scored; ``merges``: each merge made, its two keyframes judged the same way
  (:func:`slam_it.eval.loops.judge_pairs`);
- ``p95``: the 95th percentile of a frame's latency, building and tracking it, in seconds,
  read against the frame period ``deadline``, with the frames ``late`` past it
  (:func:`slam_it.eval.budgets.judge`); null, with ``unjudged`` saying why, if the run is too
  short to judge; and for the online-SLAM design's section 13, ``on_time``, the share of the
  frames built and tracked within the deadline (its latency bar: at least 99%), the latency's
  ``median``, ``p99`` and ``worst``, ``late_frames`` (each late frame's index, time and
  latency), the time tracking ``waited`` for the map (its total, in seconds), and mapping's
  measures: the ``keyframes`` the run made, those ``asked`` for and ``refused``, the largest
  ``queue``, the local BAs ``aborted``, ``mapping_load`` and ``mapping_peak_load``, and, in
  order, the ``backlog`` a mapping thread of its own would build (null threaded);
- the run's details (:data:`DETAILS`), which ``scripts/compare_testsets.py`` compares, so that
  a batch is compared by what the code that ran it recorded, never by reading its runs with
  code that may no longer read their format (B2's review panel, A9):
  - ``stretches``: each stretch of frames a map did not track, ``[ending, frames, seconds]``
    (:func:`stretches`), and ``lost_frames``, the run's ``loss.lost_frames``;
  - ``latency``: each frame's latency, building and tracking it, in seconds, by what the frame
    needed (:data:`KINDS`, :func:`kind`);
  - ``relocalised_frames``: each relocalised frame against the ground truth, its ``seconds``
    since the first frame, its ``map``, its ``error``, the map's ``median`` error and the
    ``before`` error of the map's last tracked frame before it, in metres (null where
    unmatched); null if no map could be scored (:func:`relocalised_errors`).

``--camera nominal`` runs each sequence with a guess for its camera in place of its own: a
typical phone's field of view (:func:`slam_it.eval.lenses.nominal`), without distortion, on a
folder of links to its frames written beside the run. Its depth's virtual baseline is kept, so
an RGB-D run's split of near and far points does not move with the guess. It measures what a
guessed lens costs.

``--threaded`` runs each sequence's steps in threads of their own, the native programs feeding
its frames at their recorded rate, and ``--locks`` with it shares the map by ORB-SLAM's locks,
C++'s baseline (:func:`slam_it.orchestration.vo.run`); each result says ``threaded`` and
``locks``. ``--budget NAME`` runs each within a compute budget
(:data:`slam_it.eval.budgets.BUDGETS`): pinned to its physical cores, its memory capped.

``--option KEY=VALUE``, given once a setting, replaces one of the edge pipeline's settings
(:class:`slam_it.edge.options.Options`, named by its group and field) in every run, its value
read as JSON: ``--option relocalization.enabled=false`` measures without relocalisation, loop
closing kept, with its keyframe database (B3's review panel, H7: before, the database went with
relocalisation, and loop closing with it); adding ``--option loops.enabled=false`` measures
with neither, as ``relocalization.enabled=false`` alone did before. ``--option seed=1`` measures
at another RANSAC seed.
Each result names the settings it replaced, in ``options``.

The results are one JSON line per sequence in ``OUT/results.jsonl``, and a table on stdout::

    python scripts/measure_testset.py CONFIG OUT [--backend cpp] [--features by-size|default]
        [--camera exact|nominal] [--only NAME ...] [--option KEY=VALUE ...]
        [--threaded [--locks]] [--budget NAME]

A run's latency is read only from a measure made alone, pinned to one core (``taskset -c 2
python scripts/measure_testset.py ...``), or threaded to its budget's cores (``--threaded
--budget laptop-cpu``): runs side by side share the caches, the memory's bandwidth and the
power. ``scripts/measure_sharded.py`` runs them side by side, for the tracking and the ATE.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tomllib
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np
from numpy.typing import NDArray

from slam_it.edge.options import MODES, Options
from slam_it.eval import budgets, lenses, loops, runs, tum
from slam_it.foundations.errors import DegenerateError, InvalidInputError, SlamItError
from slam_it.orchestration import vo

KEYS = ("name", "mode", "reference")
"""Each sequence's keys in the config, every one a string, beside its :data:`FOLDERS`."""

FOLDERS = "folders"
"""Where a sequence is: a list of folders, one for a single sequence, several run in turn."""

OPTIONAL_KEYS = ("dataset",)
"""The keys a sequence may also have: its ``dataset``, ``"tum"`` unless given."""

SETTINGS = frozenset(Options().settings())
"""The settings an ``--option`` may replace, each named by its group and field."""

LINKED = ("rgb", "rgb.txt", "depth", "depth.txt", "groundtruth.txt")
"""What a nominal camera's folder links to in the sequence's, where the sequence has it."""

ENDINGS = ("relocalised", "recovered", "lost", "end")
"""How a stretch of frames a map did not track ended."""

KINDS = ("relocalised", "tried", "placed", "other")
"""What a frame needed: relocalised; relocalisation tried, nothing placed; placed otherwise (by
the motion model, the local fallback or a map's start); none of these."""

DETAILS = ("lost_frames", "stretches", "latency", "relocalised_frames")
"""A result's details of its run (the module docstring), which a comparison reads."""


class Entry(NamedTuple):
    """A sequence of the test set, as its config lists it: one folder, or several in turn."""

    name: str
    folders: tuple[Path, ...]
    mode: str
    reference: str
    dataset: str = "tum"


class Stretch(NamedTuple):
    """Consecutive frames of a map that it did not track, and how they ended (:data:`ENDINGS`)."""

    ending: str
    frames: int
    seconds: float


class Placed(NamedTuple):
    """A relocalised frame against the ground truth, in metres (None where unmatched)."""

    seconds: float
    map: int
    error: float | None
    median: float | None
    before: float | None


def stretches(diagnostics: list[dict[str, Any]]) -> list[Stretch]:
    """Return each stretch of frames a run's maps did not track, in order.

    A frame is untracked when tracking failed it (its diagnostic's ``tracked``, its inliers, is
    0), a frame placed that then failed the local map among them, or when it lost the track; a
    map reset later keeps its record. A stretch ends relocalised (the next frame placed by
    relocalisation), recovered (tracked otherwise), lost (its last frame lost the track) or with
    the sequence (``end``).
    """
    out: list[Stretch] = []
    frames, start = 0, 0.0
    for entry in diagnostics:
        time = float(entry["timestamp"])
        lost = entry.get("state") == runs.LOST
        if lost or (entry.get("map") is not None and entry.get("tracked", 0) == 0):
            start = start if frames else time
            frames += 1
            if lost:
                out.append(Stretch("lost", frames, time - start))
                frames = 0
        elif frames:
            found = entry.get("placed_by") == runs.RELOCALIZED
            out.append(Stretch("relocalised" if found else "recovered", frames, time - start))
            frames = 0
    if frames:
        out.append(Stretch("end", frames, float(diagnostics[-1]["timestamp"]) - start))
    return out


def kind(entry: dict[str, Any]) -> str:
    """Return what a frame needed (:data:`KINDS`), from its diagnostic."""
    if entry.get("placed_by") == runs.RELOCALIZED:
        return "relocalised"
    if entry.get("relocalization") is not None:
        return "tried"
    return "other" if entry.get("placed_by") is None else "placed"


def relocalised_errors(run: runs.Run, truth: runs.Truth, scored: runs.Evaluation) -> list[Placed]:
    """Return each relocalised frame's error against the ground truth, each map aligned as scored.

    `scored` is the run scored against `truth` (:func:`slam_it.eval.runs.evaluate_absolute`). A
    frame's error is its centre's distance from the truth's, its map aligned as the ATE aligns
    it, beside its map's median error and the error of the map's last tracked frame before it,
    so that a frame relocalised into the wrong place stands out from both.
    """
    times, poses = truth
    aligned = {e.map: e.absolute for e in scored.maps}
    first = float(run.diagnostics[0]["timestamp"]) if run.diagnostics else 0.0

    def error(k: int) -> float | None:
        absolute = aligned.get(int(run.pose_maps[k]))
        j = int(np.argmin(np.abs(times - run.timestamps[k])))
        if absolute is None or abs(times[j] - run.timestamps[k]) > runs.MAX_DIFFERENCE:
            return None
        centre = absolute.alignment @ np.append(run.poses[k][:3, 3], 1.0)
        return float(np.linalg.norm(centre[:3] - poses[j][:3, 3]))

    row = {float(t): k for k, t in enumerate(run.timestamps)}
    out = []
    for entry in run.diagnostics:
        k = row.get(float(entry["timestamp"]))
        if entry.get("placed_by") != runs.RELOCALIZED or k is None:
            continue
        m = int(run.pose_maps[k])
        before = next((j for j in range(k - 1, -1, -1) if run.pose_maps[j] == m), None)
        median = aligned[m].median if m in aligned else None
        seconds = float(entry["timestamp"]) - first
        out.append(Placed(seconds, m, error(k), median, None if before is None else error(before)))
    return out


def keyframes_ate(run: runs.Run, truth: runs.Truth) -> float | None:
    """Return the ATE RMSE of the run's final keyframes, each map aligned on its own.

    None if no map's keyframes can be aligned.
    """
    times, poses = run.final_keyframes()
    alive = run.keyframe_lives[:, 1] < 0
    keyframes = replace(
        run, timestamps=times, poses=poses, tracked_poses=poses, pose_maps=run.keyframe_maps[alive]
    )
    try:
        return runs.evaluate_absolute(keyframes, *truth).ate_rmse
    except DegenerateError:
        return None


def details(
    run: runs.Run, truth: runs.Truth | None, scored: runs.Evaluation | None
) -> dict[str, Any]:
    """Return a run's details (:data:`DETAILS`; the module docstring).

    `scored` is the run scored against `truth`, None where there is no truth or no map could
    be aligned to it, which makes ``relocalised_frames`` null.
    """
    latency: dict[str, list[float]] = {k: [] for k in KINDS}
    for entry in run.diagnostics:
        latency[kind(entry)].append(float(entry["build_seconds"]) + float(entry["track_seconds"]))
    placed = (
        None
        if truth is None or scored is None
        else [p._asdict() for p in relocalised_errors(run, truth, scored)]
    )
    return {
        "lost_frames": int(run.settings["loss"]["lost_frames"]),
        "stretches": [list(s) for s in stretches(run.diagnostics)],
        "latency": latency,
        "relocalised_frames": placed,
    }


def read_config(path: Path) -> list[Entry]:
    """Read a config's sequences.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it is not TOML, lists no sequence, a sequence lacks a key or has another, a value is
        not a string, its folders not a list of them, a mode or dataset is unknown, or two
        sequences share a name.
    """
    try:
        data = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as error:
        raise InvalidInputError(f"{path}: not TOML ({error})") from None
    listed = data.get("sequence")
    if not isinstance(listed, list) or not listed:
        raise InvalidInputError(f"{path}: no [[sequence]]")
    entries: list[Entry] = []
    for number, item in enumerate(listed, start=1):
        if not isinstance(item, dict) or not {*KEYS, FOLDERS} <= set(item) <= {
            *KEYS,
            FOLDERS,
            *OPTIONAL_KEYS,
        }:
            raise InvalidInputError(
                f"{path}: sequence {number}: expected the keys {(*KEYS, FOLDERS)}, and maybe "
                f"{OPTIONAL_KEYS}"
            )
        folders = item[FOLDERS]
        if not all(isinstance(item[key], str) for key in item if key != FOLDERS) or not (
            isinstance(folders, list) and folders and all(isinstance(f, str) for f in folders)
        ):
            raise InvalidInputError(
                f"{path}: sequence {number}: expected strings, folders a list of them"
            )
        if item["mode"] not in MODES:
            raise InvalidInputError(f"{path}: sequence {number}: unknown mode {item['mode']!r}")
        dataset = item.get("dataset", "tum")
        if dataset not in runs.DATASETS:
            raise InvalidInputError(f"{path}: sequence {number}: unknown dataset {dataset!r}")
        entries.append(
            Entry(
                item["name"],
                tuple(Path(folder) for folder in folders),
                item["mode"],
                item["reference"],
                dataset,
            )
        )
    names = [e.name for e in entries]
    if len(set(names)) != len(names):
        raise InvalidInputError(f"{path}: two sequences share a name")
    return entries


def read_options(given: list[str]) -> dict[str, Any]:
    """Read ``--option`` arguments: each ``KEY=VALUE``, its value JSON or else a string.

    Returns
    -------
    dict
        The settings to replace, each checked as the pipeline's settings check it.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If one is not ``KEY=VALUE``, names no setting or one given before, or its value is not
        one the setting takes.
    """
    out: dict[str, Any] = {}
    for text in given:
        key, equals, value = text.partition("=")
        if not equals:
            raise InvalidInputError(f"--option {text!r}: expected KEY=VALUE")
        if key not in SETTINGS:
            raise InvalidInputError(f"--option {text!r}: no setting {key!r}")
        if key in out:
            raise InvalidInputError(f"--option {text!r}: {key} given twice")
        try:
            out[key] = json.loads(value)
        except json.JSONDecodeError:
            out[key] = value
    Options().with_settings(out)  # each value checked as a run will check it
    return out


def nominal_folder(sequence: Path, out: Path) -> Path:
    """Write a folder of links to a sequence's frames, its camera a nominal guess; return it.

    The guess is :func:`slam_it.eval.lenses.nominal` at the frames' size, a pinhole. The depth
    scale stays the sequence's, so an RGB-D run still reads its depth, and ``bf`` is scaled with
    the guessed focal length, so its virtual baseline, ``bf`` over the focal length, is kept.
    """
    camera = tum.read_camera(sequence)
    out.mkdir(parents=True)
    for name in LINKED:
        if (sequence / name).exists():
            (out / name).symlink_to(os.path.relpath(sequence / name, out))
    guess = lenses.nominal(camera.width, camera.height)
    tum.write_camera(
        out,
        tum.Camera(
            "pinhole",
            np.array([guess.fx, guess.fy, guess.cx, guess.cy]),
            camera.width,
            camera.height,
            camera.depth_scale,
            None if camera.bf is None else camera.bf * guess.fx / float(camera.intrinsics[0]),
        ),
        note=f"a nominal guess for {sequence.name}'s camera (scripts/measure_testset.py)",
    )
    return out


def offsets_of(entry: Entry) -> list[float]:
    """Return each of a sequence's sessions' clock offsets, as its run moved its times on.

    :func:`slam_it.orchestration.vo.session_offsets` of its folders' frames' times; ``[0.0]``
    for one folder, which keeps its own clock, without reading it.

    Parameters
    ----------
    entry : Entry
        The sequence.

    Returns
    -------
    list of float
        Each session's offset, in seconds.
    """
    if len(entry.folders) == 1:
        return [0.0]
    times = [vo.open_source(entry.dataset, folder, entry.mode).times for folder in entry.folders]
    return vo.session_offsets(times)


def truth_of(dataset: str, folders: Sequence[Path], offsets: Sequence[float]) -> runs.Truth | None:
    """Return sequences' ground truth in turn, each moved on by its session's time offset.

    A session's reference times that do not come after those kept before it are dropped, so the
    times ascend (a reference may outlast its sequence's frames). None if no sequence has one.
    """
    times: list[NDArray[np.float64]] = []
    poses: list[NDArray[np.float64]] = []
    end = -math.inf
    for folder, offset in zip(folders, offsets, strict=True):
        truth = runs.ground_truth(dataset, folder)
        if truth is None:
            continue
        moved = truth[0] + offset
        after = moved > end
        times.append(moved[after])
        poses.append(truth[1][after])
        end = max(end, float(moved[after][-1])) if after.any() else end
    if not times:
        return None
    return np.concatenate(times), np.concatenate(poses)


def session_results(
    run: runs.Run,
    folders: Sequence[Path],
    times: Sequence[NDArray[np.float64]],
    offsets: Sequence[float],
) -> list[dict[str, Any]]:
    """Return each session's account of a run (the module docstring's ``sessions``).

    `times` are each session's frames' own times, `offsets` what moved them on in the run
    (:func:`slam_it.orchestration.vo.session_offsets`). A session's frames tracked are the final
    trajectory's within its first and last times, moved on.
    """
    closed = {
        line["session"]["index"]: line["session"]["closed"]
        for line in run.diagnostics
        if "session" in line
    }
    out: list[dict[str, Any]] = []
    for k, (folder, own, offset) in enumerate(zip(folders, times, offsets, strict=True)):
        inside = np.zeros(len(run.timestamps), dtype=bool)
        if len(own):
            inside = (run.timestamps >= own[0] + offset) & (run.timestamps <= own[-1] + offset)
        out.append(
            {
                "sequence": folder.resolve().name,
                "offset": offset,
                "frames": len(own),
                "tracked": int(inside.sum()),
                "maps": sorted(set(run.pose_maps[inside].tolist())),
                "closed": closed.get(k),
            }
        )
    return out


def measure(  # noqa: PLR0913 -- the sequence and its folder; how it runs keyword-only
    entry: Entry,
    out: Path,
    *,
    backend: str,
    by_size: bool,
    nominal: bool,
    replaced: dict[str, Any] | None = None,
    threaded: bool = False,
    locks: bool = False,
    budget: budgets.Budget | None = None,
) -> dict[str, Any]:
    """Run a sequence and measure it (the module docstring); return its result.

    `replaced` holds the settings to replace in its run (:func:`read_options`); `threaded` and
    `locks` say how its steps run, and `budget` what it runs within.
    """
    replaced = replaced or {}
    if nominal and (entry.dataset != "tum" or len(entry.folders) > 1):
        raise InvalidInputError(
            f"{entry.name}: a nominal camera is guessed for one TUM folder only"
        )
    folders = [nominal_folder(entry.folders[0], out / "sequence")] if nominal else entry.folders
    sources = [vo.open_source(entry.dataset, folder, entry.mode) for folder in folders]
    camera = sources[0].camera
    chosen = vo.options_for(camera.width, camera.height) if by_size else Options()
    options = chosen.with_settings(replaced)
    run = vo.run(
        entry.dataset, folders, entry.mode, out / "run", backend=backend, options=options,
        threaded=threaded, locks=locks, budget=budget,
    )  # fmt: skip
    offsets = offsets_of(entry)
    times = np.concatenate([s.times + offset for s, offset in zip(sources, offsets, strict=True)])
    span = runs.tracking(run, times)
    result: dict[str, Any] = {
        "name": entry.name,
        "dataset": entry.dataset,
        "mode": entry.mode,
        "backend": backend,
        "camera": "nominal" if nominal else "exact",
        "size": [camera.width, camera.height],
        "features": options.features.n_features,
        "options": replaced,
        "threaded": threaded,
        "locks": locks,
        "budget": None if budget is None else budget.name,
        "frames": len(times),
        "sessions": session_results(run, folders, [s.times for s in sources], offsets),
        "maps": sorted(set(run.pose_maps.tolist())),
        "run": span.frames,
        "tracked": span.tracked,
        "share": span.tracked / len(times) if len(times) else 0.0,
        "segments": [
            {
                "map": s.map,
                "tracked": s.tracked,
                "tracked_from": s.tracked_from,
                "tracked_to": s.tracked_to,
            }
            for s in span.segments
        ],
        "losses": span.losses,
        "resets": span.resets,
        "relocalizations": span.relocalizations,
        "tracked_from": span.tracked_from,
        "tracked_to": span.tracked_to,
        "ran_to": span.ran_to,
        "reference": entry.reference,
        "ate": None,
        "ate_tracked": None,
        "ate_keyframes": None,
        "matched": 0,
        "map_ate": [],
        "unscored_maps": [],
        "unscored": None,
        "p95": None,
        "deadline": None,
        "late": None,
        "unjudged": None,
    }
    truth = truth_of(entry.dataset, folders, offsets)
    scored = None
    if truth is None:
        result["unscored"] = "no ground truth"
    else:
        try:
            scored = runs.evaluate_absolute(run, *truth)
            as_tracked = runs.evaluate_absolute(run, *truth, tracked=True)
            result.update(
                ate=scored.ate_rmse,
                ate_tracked=as_tracked.ate_rmse,
                ate_keyframes=keyframes_ate(run, truth),
                matched=scored.matched,
                map_ate=[
                    {"map": e.map, "matched": e.matched, "ate": e.absolute.rmse}
                    for e in scored.maps
                ],
                unscored_maps=list(scored.unscored),
            )
        except DegenerateError as error:  # no map can be aligned: said, never hidden
            result["unscored"] = str(error)
    try:
        judged = budgets.judge(run)
        result.update(p95=judged.latency.p95, deadline=judged.deadline, late=judged.late)
        result.update(realtime(run, judged))
    except InvalidInputError as error:  # a run too short to judge: said, never hidden
        result["unjudged"] = str(error)
    result["loops"] = loops.judge_loops(run, truth, scored)
    result["merges"] = loops.judge_pairs(run.keyframe_merges, run, truth, scored)
    result.update(details(run, truth, scored))
    return result


def realtime(run: runs.Run, judged: budgets.Frames) -> dict[str, Any]:
    """Return a run's measures for the online-SLAM design's section 13 (the module docstring)."""
    latency = np.array([d["build_seconds"] + d["track_seconds"] for d in run.diagnostics])
    late = np.flatnonzero(latency > judged.deadline)
    return {
        "on_time": 1.0 - len(late) / len(latency),
        "median": judged.latency.median,
        "p99": float(np.percentile(latency, 99)),
        "worst": judged.latency.max,
        "late_frames": [[int(k), run.diagnostics[k]["timestamp"], float(latency[k])] for k in late],
        "waited": float(sum(d["wait_seconds"] for d in run.diagnostics)),
        "keyframes": len(run.keyframe_timestamps),
        "asked": judged.asked,
        "refused": judged.refused,
        "queue": judged.queue,
        "aborted": judged.aborted,
        "mapping_load": judged.mapping_load,
        "mapping_peak_load": judged.mapping_peak_load,
        "backlog": judged.backlog,
    }


def row(result: dict[str, Any]) -> str:
    """Return a result as a line of the table."""
    width, height = result["size"]
    span = (
        "-"
        if result["tracked_from"] is None
        else f"{result['tracked_from']:.1f}-{result['tracked_to']:.1f} s"
    )
    ate = "-" if result["ate"] is None else f"{100 * result['ate']:.1f} cm ({result['matched']})"
    p95 = (
        "-"
        if result["p95"] is None
        else f"{1000 * result['p95']:.0f} of {1000 * result['deadline']:.0f} ms"
    )
    maps = f"{len(result['segments'])} ({result['resets']} reset)"
    return (
        f"| {result['name']} | {result['mode']} | {width}x{height} | {result['features']} | "
        f"{result['tracked']}/{result['frames']} ({100 * result['share']:.0f}%) | {span} | "
        f"{maps} | {result['losses']} | {result['relocalizations']} | {ate} | {p95} |"
    )


HEADER = (
    "| sequence | mode | size | features | tracked | tracked from-to | maps tracked | losses | "
    "relocalised | ATE (matched) | p95 latency of the frame period |\n"
    "|---|---|---|---|---|---|---|---|---|---|---|"
)
"""The table's header."""


def add_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the measure's arguments: the config, the results' folder, and how runs are made."""
    parser.add_argument("config", type=Path, help="the test set's config (TOML)")
    parser.add_argument("out", type=Path, help="the folder for the runs and results.jsonl")
    parser.add_argument("--backend", default="cpp", choices=("python", "cpp", "rust"))
    parser.add_argument("--features", default="by-size", choices=("by-size", "default"))
    parser.add_argument("--camera", default="exact", choices=("exact", "nominal"))
    parser.add_argument("--only", nargs="+", metavar="NAME", help="measure these sequences")
    parser.add_argument(
        "--option",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="replace one of the pipeline's settings in every run (repeatable)",
    )
    parser.add_argument(
        "--threaded", action="store_true", help="run the steps in threads of their own"
    )
    parser.add_argument(
        "--locks", action="store_true", help="with --threaded: ORB-SLAM's locks (C++'s baseline)"
    )
    parser.add_argument(
        "--budget", choices=sorted(budgets.BUDGETS), help="run within this compute budget"
    )


def chosen(
    args: argparse.Namespace, parser: argparse.ArgumentParser
) -> tuple[list[Entry], dict[str, Any]]:
    """Return the sequences to measure and the settings to replace, and make the results' folder.

    Refuses, through `parser`, a name not in the config, a bad option, or a folder already used.
    """
    entries = read_config(args.config)
    if args.only:
        unknown = set(args.only) - {e.name for e in entries}
        if unknown:
            parser.error(f"--only: not in the config: {', '.join(sorted(unknown))}")
        entries = [e for e in entries if e.name in args.only]
    try:
        replaced = read_options(args.option)
    except InvalidInputError as error:
        parser.error(str(error))
    if args.out.exists() and any(args.out.iterdir()):
        parser.error(f"{args.out}: expected a new or empty folder")
    args.out.mkdir(parents=True, exist_ok=True)
    return entries, replaced


def measured(
    entry: Entry, out: Path, args: argparse.Namespace, replaced: dict[str, Any]
) -> dict[str, Any]:
    """Measure a sequence as the arguments ask; a sequence that fails is a result saying why."""
    try:
        return measure(
            entry,
            out / entry.name,
            backend=args.backend,
            by_size=args.features == "by-size",
            nominal=args.camera == "nominal",
            replaced=replaced,
            threaded=args.threaded,
            locks=args.locks,
            budget=None if args.budget is None else budgets.BUDGETS[args.budget],
        )
    except (SlamItError, FileNotFoundError) as error:
        return {"name": entry.name, "failed": str(error)}


def shown(result: dict[str, Any]) -> str:
    """Return a result as a line of the table, or the line saying why its sequence failed."""
    if "failed" in result:
        return f"| {result['name']} | failed: {result['failed']} |"
    return row(result)


def main(arguments: list[str]) -> int:
    """Measure the test set from the command line (the module docstring)."""
    parser = argparse.ArgumentParser(prog="python scripts/measure_testset.py")
    add_arguments(parser)
    args = parser.parse_args(arguments)
    entries, replaced = chosen(args, parser)
    print(HEADER, flush=True)
    with (args.out / "results.jsonl").open("w") as results:
        for entry in entries:
            result = measured(entry, args.out, args, replaced)
            results.write(json.dumps(result) + "\n")
            print(shown(result), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
