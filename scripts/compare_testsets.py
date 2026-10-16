"""Compare two measures of the test set: what moved, how failures ended, and relocalisation's cost.

BASE and NEW are ``scripts/measure_testset.py``'s folders (or a seed's of
``scripts/measure_sharded.py``): ``results.jsonl``, and each sequence's run. Printed as markdown:

- **By group**, a group being a sequence's dataset (its name up to the first ``-``) and mode:
  the runs in both, the frames tracked, the median share, the maps, the losses, the relocalised
  frames and the median ATE, base then new. A sequence in one only, or failed in either, is
  named.
- **The runs that moved**: a share by more than a point, an ATE by more than a tenth (or scored
  in one only), or any frame relocalised.
- **How failures ended**: a stretch is consecutive frames of a map that it did not track (a
  frame's diagnostic records ``tracked``, its inliers, 0 when tracking failed it, a frame placed
  that then failed the local map among them; a map reset later keeps its record); it ends
  relocalised (the next frame placed by relocalisation), recovered (tracked otherwise), lost
  (its last frame lost the track) or with the sequence. Each batch's stretches and their frames
  by ending, a group a row, and the frames lost stretches waited past ``lost_frames`` (the
  run's own setting): those a map lost after ``lost_frames`` failures in a row would not have.
- **Latency by frame**: building and tracking a frame, as :func:`slam_it.eval.budgets.judge`
  times it, by what the frame needed: relocalised, relocalisation tried with nothing placed,
  placed otherwise, or none of these. Each kind's frames, median, 95th percentile and slowest,
  and those past the run's deadline. Comparable only between measures made alone, pinned to a
  core (``measure_testset.py`` says how).
- With ``--config``, the measure's config: **each relocalised frame against the ground truth**,
  its error with its map aligned as the ATE aligns it, beside the map's median error and the
  error of the map's last tracked frame before it. A frame relocalised into the wrong place
  stands out from both.

::

    python scripts/compare_testsets.py BASE NEW [--config CONFIG]
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from collections import Counter, defaultdict
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any, NamedTuple

import measure_testset  # this script's neighbour: its folder is on the path when it runs
import numpy as np

from slam_it.eval import budgets, runs
from slam_it.foundations.errors import DegenerateError, InvalidInputError

ENDINGS = ("relocalised", "recovered", "lost", "end")
"""How a stretch of frames a map did not track ended."""

KINDS = ("relocalised", "tried", "placed", "other")
"""What a frame needed: relocalised; relocalisation tried, nothing placed; placed otherwise (by
the motion model, the local fallback or a map's start); none of these."""

SHARE_MOVED = 0.01
"""A run's share tracked moved when it changed by more than this."""

ATE_MOVED = 0.1
"""A run's ATE moved when it changed by more than this fraction of the base's."""


class Batch(NamedTuple):
    """A measure: its folder, its results by sequence, and the sequences that failed."""

    folder: Path
    results: dict[str, dict[str, Any]]
    failed: list[str]


class Stretch(NamedTuple):
    """Consecutive frames of a map that it did not track, and how they ended (:data:`ENDINGS`)."""

    ending: str
    frames: int
    seconds: float


class Placed(NamedTuple):
    """A relocalised frame against the ground truth, in metres (None where unmatched)."""

    name: str
    seconds: float
    map: int
    error: float | None
    median: float | None
    before: float | None


def read_batch(folder: Path) -> Batch:
    """Read a measure's results.

    Raises
    ------
    FileNotFoundError
        If the folder has no ``results.jsonl``.
    """
    rows = [json.loads(line) for line in (folder / "results.jsonl").read_text().splitlines()]
    measured = {row["name"]: row for row in rows if "failed" not in row}
    return Batch(folder, measured, [row["name"] for row in rows if "failed" in row])


def group(result: dict[str, Any]) -> str:
    """Return a result's group: its dataset, the name up to the first ``-``, and its mode."""
    return f"{result['name'].split('-')[0]} {result['mode']}"


def stretches(diagnostics: list[dict[str, Any]]) -> list[Stretch]:
    """Return each stretch of frames a run's maps did not track, in order (the module docstring).

    A frame that lost the track ends its stretch.
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
    """Return what a frame needed (:data:`KINDS`)."""
    if entry.get("placed_by") == runs.RELOCALIZED:
        return "relocalised"
    if entry.get("relocalization") is not None:
        return "tried"
    return "other" if entry.get("placed_by") is None else "placed"


def relocalised_errors(name: str, run: runs.Run, truth: runs.Truth) -> list[Placed]:
    """Return each relocalised frame's error against the ground truth (the module docstring).

    Raises
    ------
    slam_it.foundations.errors.DegenerateError
        If no map of the run can be aligned to the truth.
    """
    times, poses = truth
    scored = runs.evaluate_absolute(run, times, poses)
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
        out.append(
            Placed(name, seconds, m, error(k), median, None if before is None else error(before))
        )
    return out


def _cm(metres: float | None) -> str:
    return "-" if metres is None else f"{100 * metres:.1f}"


def _ms(seconds: Iterable[float]) -> str:
    values = list(seconds)
    return f"{1000 * statistics.median(values):.1f}" if values else "-"


def _median(values: Iterable[float]) -> float | None:
    listed = list(values)
    return statistics.median(listed) if listed else None


def by_group(base: Batch, new: Batch, names: list[str]) -> list[str]:
    """Return the table by group, each column base then new."""
    groups: dict[str, list[str]] = defaultdict(list)
    for name in names:
        groups[group(new.results[name])].append(name)
    columns: list[tuple[str, Callable[[list[dict[str, Any]]], str]]] = [
        ("tracked", lambda rs: str(sum(r["tracked"] for r in rs))),
        ("median share %", lambda rs: f"{100 * statistics.median(r['share'] for r in rs):.1f}"),
        ("maps", lambda rs: str(sum(len(r["segments"]) for r in rs))),
        ("losses", lambda rs: str(sum(r["losses"] for r in rs))),
        ("relocalised", lambda rs: str(sum(r["relocalizations"] for r in rs))),
        ("median ATE cm", lambda rs: _cm(_median(r["ate"] for r in rs if r["ate"] is not None))),
    ]
    lines = [
        "| group | runs | " + " | ".join(f"{title}, base / new" for title, _ in columns) + " |",
        "|---|---|" + "---|" * len(columns),
    ]
    for name, members in sorted(groups.items()):
        before = [base.results[n] for n in members]
        after = [new.results[n] for n in members]
        cells = [f"{read(before)} / {read(after)}" for _, read in columns]
        lines.append(f"| {name} | {len(members)} | " + " | ".join(cells) + " |")
    return lines


def moved(base: Batch, new: Batch, names: list[str]) -> list[str]:
    """Return the table of the runs that moved or relocalised, base then new."""
    lines = [
        "| run | share % | maps | losses | relocalised | ATE cm | each map's ATE cm, new |",
        "|---|---|---|---|---|---|---|",
    ]
    for name in names:
        a, b = base.results[name], new.results[name]
        share = abs(a["share"] - b["share"]) > SHARE_MOVED
        scored = (a["ate"] is None) != (b["ate"] is None)
        ate = scored or (a["ate"] is not None and abs(b["ate"] - a["ate"]) > ATE_MOVED * a["ate"])
        if not (share or ate or a["relocalizations"] or b["relocalizations"]):
            continue
        maps = ", ".join(f"{m['map']}: {_cm(m['ate'])}" for m in b["map_ate"])
        lines.append(
            f"| {name} | {100 * a['share']:.1f} / {100 * b['share']:.1f} "
            f"| {len(a['segments'])} / {len(b['segments'])} | {a['losses']} / {b['losses']} "
            f"| {a['relocalizations']} / {b['relocalizations']} "
            f"| {_cm(a['ate'])} / {_cm(b['ate'])} | {maps} |"
        )
    return lines


def endings(batch: Batch, names: list[str]) -> list[str]:
    """Return the table of how a batch's failures ended, a group a row."""
    count: dict[str, Counter[str]] = defaultdict(Counter)
    frames: dict[str, Counter[str]] = defaultdict(Counter)
    waited: Counter[str] = Counter()
    for name in names:
        run = runs.read_run(batch.folder / name / "run")
        key = group(batch.results[name])
        lost_frames = int(run.settings["loss"]["lost_frames"])
        for stretch in stretches(run.diagnostics):
            count[key][stretch.ending] += 1
            frames[key][stretch.ending] += stretch.frames
            if stretch.ending == "lost":
                waited[key] += max(stretch.frames - lost_frames, 0)
    lines = [
        "| group | stretches, " + " / ".join(ENDINGS) + " | their frames "
        "| frames waited past lost_frames before a loss |",
        "|---|---|---|---|",
    ]
    for key in sorted(count):
        stretched = " / ".join(str(count[key][e]) for e in ENDINGS)
        framed = " / ".join(str(frames[key][e]) for e in ENDINGS)
        lines.append(f"| {key} | {stretched} | {framed} | {waited[key]} |")
    return lines


def latencies(batch: Batch, names: list[str]) -> list[str]:
    """Return the table of a batch's latency by what a frame needed, a group a row per kind."""
    seconds: dict[tuple[str, str], list[float]] = defaultdict(list)
    late: Counter[tuple[str, str]] = Counter()
    for name in names:
        run = runs.read_run(batch.folder / name / "run")
        try:
            deadline = budgets.judge(run).deadline
        except InvalidInputError:  # too short to judge
            continue
        for entry in run.diagnostics:
            key = (group(batch.results[name]), kind(entry))
            latency = float(entry["build_seconds"]) + float(entry["track_seconds"])
            seconds[key].append(latency)
            late[key] += latency > deadline
    lines = [
        "| group | frame | frames | median ms | p95 ms | slowest ms | late |",
        "|---|---|---|---|---|---|---|",
    ]
    for key in sorted(seconds, key=lambda k: (k[0], KINDS.index(k[1]))):
        values = np.asarray(seconds[key])
        lines.append(
            f"| {key[0]} | {key[1]} | {len(values)} | {_ms(values.tolist())} "
            f"| {1000 * np.percentile(values, 95):.1f} | {1000 * values.max():.1f} | {late[key]} |"
        )
    return lines


def against_truth(batch: Batch, names: list[str], folders: dict[str, Path]) -> list[str]:
    """Return the table of a batch's relocalised frames against the ground truth."""
    lines = [
        "| run | time s | map | error cm | map's median cm | last tracked before cm |",
        "|---|---|---|---|---|---|",
    ]
    for name in names:
        truth = runs.ground_truth("tum", folders[name])
        if not batch.results[name]["relocalizations"] or truth is None:
            continue
        try:
            placed = relocalised_errors(name, runs.read_run(batch.folder / name / "run"), truth)
        except DegenerateError:  # no map aligned: its ATE says so
            continue
        lines.extend(
            f"| {p.name} | {p.seconds:.2f} | {p.map} | {_cm(p.error)} | {_cm(p.median)} "
            f"| {_cm(p.before)} |"
            for p in placed
        )
    return lines


def main(arguments: list[str]) -> int:
    """Compare two measures from the command line (the module docstring)."""
    parser = argparse.ArgumentParser(prog="python scripts/compare_testsets.py")
    parser.add_argument("base", type=Path, help="the measure compared against")
    parser.add_argument("new", type=Path, help="the measure compared")
    parser.add_argument("--config", type=Path, help="the measure's config, for the ground truth")
    args = parser.parse_args(arguments)
    base, new = read_batch(args.base), read_batch(args.new)
    names = [n for n in new.results if n in base.results]
    print(f"{len(names)} runs compared, of {args.base} (base) and {args.new} (new).")
    for label, listed in (
        ("In the base only", sorted(set(base.results) - set(new.results))),
        ("In the new only", sorted(set(new.results) - set(base.results))),
        ("Failed in the base", base.failed),
        ("Failed in the new", new.failed),
    ):
        if listed:
            print(f"{label}: {', '.join(listed)}.")
    sections = [
        ("By group", by_group(base, new, names)),
        ("The runs that moved", moved(base, new, names)),
        ("How failures ended, base", endings(base, names)),
        ("How failures ended, new", endings(new, names)),
        ("Latency by frame, base", latencies(base, names)),
        ("Latency by frame, new", latencies(new, names)),
    ]
    if args.config is not None:
        folders = {e.name: e.folder for e in measure_testset.read_config(args.config)}
        sections.append(
            ("Relocalised frames against the ground truth, new", against_truth(new, names, folders))
        )
    for title, lines in sections:
        print(f"\n## {title}\n")
        print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
