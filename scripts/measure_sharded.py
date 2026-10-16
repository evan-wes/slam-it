"""Measure the test set side by side: a worker a CPU, each pinned to its own, at several seeds.

The edge pipeline runs in one thread and, for a RANSAC seed, decides alike every time, so a run's
tracking and ATE do not depend on what else the machine runs: only its latency does. This runs
``scripts/measure_testset.py``'s measure over the config's sequences at each seed given, a worker
process a CPU, each pinned to its CPU with the programs its runs start, the longest sequences
first. Each seed's results are the measure's, in ``OUT/seed-S``: its runs, and ``results.jsonl``,
each result's ``options`` naming its ``seed``. A result is written there as its run ends, so a
measure stopped part way keeps what it finished, and a run whose worker failed is a result
saying why; once every run has ended, the file is rewritten in the config's order::

    python scripts/measure_sharded.py CONFIG OUT [--seeds 0 1 2] [--jobs N] [--backend cpp]
        [--features by-size|default] [--camera exact|nominal] [--only NAME ...]
        [--option KEY=VALUE ...]

``--jobs`` is the workers, at most the CPUs this process may run on (by default, all of them),
the highest-numbered first, as :func:`slam_it.eval.budgets.cpus` takes them. Read only the
tracking and the ATE from these runs: side by side, they share the caches, the memory's bandwidth
and the power, so their latency is not a run's alone (``measure_testset.py`` says how to measure
it).

The workers are forked from this process once it has found the backend's program, which for C++
imports the extension, and an editable install's import checks the build: each worker has it, so
24 workers do not check one build at once.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import multiprocessing
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing.queues import Queue
from pathlib import Path
from typing import Any

import measure_testset  # this script's neighbour: its folder is on the path when it runs

from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import vo

Job = tuple[int, measure_testset.Entry]
"""A run to make: a seed and a sequence."""


def cpus_for(jobs: int | None, offered: set[int]) -> list[int]:
    """Return the CPUs of `jobs` workers: the highest-numbered of those offered, or all of them.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `jobs` is not positive or is more than the CPUs offered.
    """
    if jobs is None:
        return sorted(offered)
    if not 0 < jobs <= len(offered):
        raise InvalidInputError(f"--jobs: expected 1 to {len(offered)}, got {jobs}")
    return sorted(offered)[-jobs:]


def frames_of(folder: Path) -> int:
    """Return a sequence's frames, as its ``rgb.txt`` lists them; 0 if it has none to read."""
    try:
        lines = (folder / "rgb.txt").read_text().splitlines()
    except OSError:
        return 0  # its run says why
    return sum(1 for line in lines if line.strip() and not line.startswith("#"))


def schedule(
    entries: list[measure_testset.Entry], seeds: list[int], frames: dict[str, int]
) -> list[Job]:
    """Return every seed's run of every sequence, the longest sequences first.

    Within one length, the seeds' order, then the config's.
    """
    jobs = [(seed, entry) for seed in seeds for entry in entries]
    return sorted(jobs, key=lambda job: -frames[job[1].name])


def _pin(cpus: Queue[int]) -> None:
    """Pin a new worker, and the programs it starts, to the next of the CPUs."""
    os.sched_setaffinity(0, {cpus.get()})


def _work(
    seed: int, entry: measure_testset.Entry, out: Path, args: argparse.Namespace,
    replaced: dict[str, Any],
) -> dict[str, Any]:  # fmt: skip
    """Measure one sequence at one seed in a worker; return the result."""
    return measure_testset.measured(entry, out, args, {**replaced, "seed": seed})


def main(arguments: list[str]) -> int:
    """Measure the test set side by side from the command line (the module docstring)."""
    parser = argparse.ArgumentParser(prog="python scripts/measure_sharded.py")
    measure_testset.add_arguments(parser)
    parser.add_argument(
        "--seeds", nargs="+", type=int, default=[0], metavar="SEED", help="the RANSAC seeds"
    )
    parser.add_argument("--jobs", type=int, help="the workers (default: one a CPU)")
    args = parser.parse_args(arguments)
    if any(option.partition("=")[0] == "seed" for option in args.option):
        parser.error("--option seed: the seeds are --seeds")
    if len(set(args.seeds)) != len(args.seeds):
        parser.error("--seeds: a seed given twice")
    try:
        for seed in args.seeds:
            measure_testset.read_options([f"seed={seed}"])
        cpus = cpus_for(args.jobs, os.sched_getaffinity(0))
    except InvalidInputError as error:
        parser.error(str(error))
    entries, replaced = measure_testset.chosen(args, parser)
    jobs = schedule(entries, args.seeds, {e.name: frames_of(e.folder) for e in entries})
    try:
        vo.command(args.backend)  # found once here, before the workers are forked
    except ImportError as error:  # not built, or (Rust) older than its sources
        parser.error(str(error))
    for seed in args.seeds:
        (args.out / f"seed-{seed}").mkdir()
    # Forked: each worker has the measure and the backend's program as this process found them.
    context = multiprocessing.get_context("fork")
    free: Queue[int] = context.Queue()
    for cpu in cpus:
        free.put(cpu)
    print(f"{len(jobs)} runs, on CPUs {', '.join(map(str, cpus))}", flush=True)
    results: dict[int, dict[str, dict[str, Any]]] = {seed: {} for seed in args.seeds}
    with contextlib.ExitStack() as stack:
        streams = {
            seed: stack.enter_context((args.out / f"seed-{seed}" / "results.jsonl").open("w"))
            for seed in args.seeds
        }
        pool = stack.enter_context(
            ProcessPoolExecutor(len(cpus), mp_context=context, initializer=_pin, initargs=(free,))
        )
        futures = {
            pool.submit(_work, seed, e, args.out / f"seed-{seed}", args, replaced): (seed, e)
            for seed, e in jobs
        }
        for future in as_completed(futures):
            seed, entry = futures[future]
            try:
                result = future.result()
            except Exception as error:
                result = {"name": entry.name, "failed": f"{type(error).__name__}: {error}"}
            results[seed][entry.name] = result
            streams[seed].write(json.dumps(result) + "\n")
            streams[seed].flush()
            print(f"seed {seed}: {measure_testset.shown(result)}", flush=True)
    for seed in args.seeds:
        with (args.out / f"seed-{seed}" / "results.jsonl").open("w") as written:
            for entry in entries:
                written.write(json.dumps(results[seed][entry.name]) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
