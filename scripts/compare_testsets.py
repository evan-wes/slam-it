"""Compare two measures of the test set: what moved, how failures ended, and relocalisation's cost.

BASE and NEW are ``scripts/measure_testset.py``'s folders (or a seed's of
``scripts/measure_sharded.py``). Only their ``results.jsonl`` is read: each result carries its
run's details (``measure_testset.DETAILS``), recorded by the code that ran it, so two measures
compare across a change of the run format, which a newer reader refuses (B2's review panel, A9).
Printed as markdown:

- **By group**, a group being a sequence's dataset (its name up to the first ``-``) and mode:
  the runs in both, the frames tracked, the median share, the maps, the losses, the relocalised
  frames and the median ATE, base then new. A sequence in one only, or failed in either, is
  named.
- **The runs that moved**: a share by more than a point, an ATE by more than a tenth (or scored
  in one only), or any frame relocalised.
- **How failures ended**: each batch's stretches of frames a map did not track
  (``measure_testset.stretches``) and their frames, by how they ended, a group a row, and the
  frames lost stretches waited past ``loss.lost_frames`` (the run's own setting): those a map
  lost after ``loss.lost_frames`` failures in a row would not have.
- **Latency by frame**: building and tracking a frame, as :func:`slam_it.eval.budgets.judge`
  times it, by what the frame needed (``measure_testset.KINDS``). Each kind's frames, median,
  95th percentile and slowest, and those past the run's deadline. Comparable only between
  measures made alone, pinned to a core (``measure_testset.py`` says how).
- **Each relocalised frame against the ground truth**, new: its error with its map aligned as
  the ATE aligns it, beside the map's median error and the error of the map's last tracked
  frame before it. A frame relocalised into the wrong place stands out from both.

A measure made before results carried their run's details is given them by ``summarize``,
which reads its runs and its config's ground truth. Run it with the code that made the measure,
before a change of the run format leaves its runs unreadable; the results it replaces are kept
as ``results.undetailed.jsonl``::

    python scripts/compare_testsets.py compare BASE NEW
    python scripts/compare_testsets.py summarize BATCH CONFIG
"""

from __future__ import annotations

import argparse
import contextlib
import json
import statistics
import sys
from collections import Counter, defaultdict
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any, NamedTuple

import measure_testset  # this script's neighbour: its folder is on the path when it runs
import numpy as np

from slam_it.eval import runs
from slam_it.foundations.errors import DegenerateError, InvalidInputError

SHARE_MOVED = 0.01
"""A run's share tracked moved when it changed by more than this."""

ATE_MOVED = 0.1
"""A run's ATE moved when it changed by more than this fraction of the base's."""

UNDETAILED = "results.undetailed.jsonl"
"""Where ``summarize`` keeps the results it gave their details."""


class Batch(NamedTuple):
    """A measure: its folder, its results by sequence, and the sequences that failed."""

    folder: Path
    results: dict[str, dict[str, Any]]
    failed: list[str]


def _rows(folder: Path) -> list[dict[str, Any]]:
    """Return a measure's results, one per line of its ``results.jsonl``.

    Raises
    ------
    FileNotFoundError
        If the folder has no ``results.jsonl``.
    """
    return [json.loads(line) for line in (folder / "results.jsonl").read_text().splitlines()]


def read_batch(folder: Path) -> Batch:
    """Read a measure's results.

    Raises
    ------
    FileNotFoundError
        If the folder has no ``results.jsonl``.
    slam_it.foundations.errors.InvalidInputError
        If a result lacks its run's details: ``summarize`` gives them.
    """
    rows = _rows(folder)
    measured = {row["name"]: row for row in rows if "failed" not in row}
    bare = [name for name, row in measured.items() if not set(measure_testset.DETAILS) <= set(row)]
    if bare:
        raise InvalidInputError(
            f"{folder}: no run details for {', '.join(bare)}; give them with "
            "`compare_testsets.py summarize`, run by the code that made the measure"
        )
    return Batch(folder, measured, [row["name"] for row in rows if "failed" in row])


def summarize(folder: Path, config: Path) -> int:
    """Give each of a measure's results its run's details (the module docstring).

    Reads each result's run, and its sequence's ground truth through `config`; results that
    have their details, and sequences that failed, are left as they are.

    Returns
    -------
    int
        The results given their details; the measure's ``results.jsonl`` is rewritten, the one
        it replaces kept as ``results.undetailed.jsonl``, only if there were any.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a result's sequence is not in the config, or ``results.undetailed.jsonl`` exists.
    """
    entries = {e.name: e for e in measure_testset.read_config(config)}
    rows = _rows(folder)
    given = 0
    for row in rows:
        if "failed" in row or set(measure_testset.DETAILS) <= set(row):
            continue
        if row["name"] not in entries:
            raise InvalidInputError(f"{config}: no sequence {row['name']!r}")
        run = runs.read_run(folder / row["name"] / "run")
        entry = entries[row["name"]]
        truth = runs.ground_truth(entry.dataset, entry.folder)
        scored = None
        if truth is not None:
            with contextlib.suppress(DegenerateError):  # no map aligned: its ATE says so
                scored = runs.evaluate_absolute(run, *truth)
        row.update(measure_testset.details(run, truth, scored))
        given += 1
    if given:
        if (folder / UNDETAILED).exists():
            raise InvalidInputError(f"{folder / UNDETAILED}: exists, so would be overwritten")
        (folder / "results.jsonl").rename(folder / UNDETAILED)
        (folder / "results.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    return given


def group(result: dict[str, Any]) -> str:
    """Return a result's group: its dataset, the name up to the first ``-``, and its mode."""
    return f"{result['name'].split('-')[0]} {result['mode']}"


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
        result = batch.results[name]
        key = group(result)
        for ending, length, _ in result["stretches"]:
            count[key][ending] += 1
            frames[key][ending] += length
            if ending == "lost":
                waited[key] += max(length - result["lost_frames"], 0)
    lines = [
        "| group | stretches, " + " / ".join(measure_testset.ENDINGS) + " | their frames "
        "| frames waited past lost_frames before a loss |",
        "|---|---|---|---|",
    ]
    for key in sorted(count):
        stretched = " / ".join(str(count[key][e]) for e in measure_testset.ENDINGS)
        framed = " / ".join(str(frames[key][e]) for e in measure_testset.ENDINGS)
        lines.append(f"| {key} | {stretched} | {framed} | {waited[key]} |")
    return lines


def latencies(batch: Batch, names: list[str]) -> list[str]:
    """Return the table of a batch's latency by what a frame needed, a group a row per kind.

    A run too short to judge (no ``deadline``) is left out, as the budgets leave it unjudged.
    """
    seconds: dict[tuple[str, str], list[float]] = defaultdict(list)
    late: Counter[tuple[str, str]] = Counter()
    for name in names:
        result = batch.results[name]
        if result["deadline"] is None:
            continue
        for frame_kind, values in result["latency"].items():
            key = (group(result), frame_kind)
            seconds[key].extend(values)
            late[key] += sum(value > result["deadline"] for value in values)
    lines = [
        "| group | frame | frames | median ms | p95 ms | slowest ms | late |",
        "|---|---|---|---|---|---|---|",
    ]
    for key in sorted(seconds, key=lambda k: (k[0], measure_testset.KINDS.index(k[1]))):
        if not seconds[key]:
            continue
        values = np.asarray(seconds[key])
        lines.append(
            f"| {key[0]} | {key[1]} | {len(values)} | {_ms(values.tolist())} "
            f"| {1000 * np.percentile(values, 95):.1f} | {1000 * values.max():.1f} | {late[key]} |"
        )
    return lines


def against_truth(batch: Batch, names: list[str]) -> list[str]:
    """Return the table of a batch's relocalised frames against the ground truth."""
    lines = [
        "| run | time s | map | error cm | map's median cm | last tracked before cm |",
        "|---|---|---|---|---|---|",
    ]
    for name in names:
        lines.extend(
            f"| {name} | {p['seconds']:.2f} | {p['map']} | {_cm(p['error'])} "
            f"| {_cm(p['median'])} | {_cm(p['before'])} |"
            for p in batch.results[name]["relocalised_frames"] or []
        )
    return lines


def compare(base_folder: Path, new_folder: Path) -> None:
    """Print the comparison of two measures (the module docstring)."""
    base, new = read_batch(base_folder), read_batch(new_folder)
    names = [n for n in new.results if n in base.results]
    print(f"{len(names)} runs compared, of {base_folder} (base) and {new_folder} (new).")
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
        ("Relocalised frames against the ground truth, new", against_truth(new, names)),
    ]
    for title, lines in sections:
        print(f"\n## {title}\n")
        print("\n".join(lines))


def main(arguments: list[str]) -> int:
    """Compare two measures, or give one its runs' details, from the command line."""
    parser = argparse.ArgumentParser(prog="python scripts/compare_testsets.py")
    commands = parser.add_subparsers(dest="command", required=True)
    compared = commands.add_parser("compare", help="compare two measures")
    compared.add_argument("base", type=Path, help="the measure compared against")
    compared.add_argument("new", type=Path, help="the measure compared")
    summarized = commands.add_parser("summarize", help="give a measure's results their details")
    summarized.add_argument("batch", type=Path, help="the measure")
    summarized.add_argument("config", type=Path, help="its config, for the ground truth")
    args = parser.parse_args(arguments)
    if args.command == "compare":
        compare(args.base, args.new)
    else:
        given = summarize(args.batch, args.config)
        print(f"{args.batch}: {given} results given their runs' details.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
