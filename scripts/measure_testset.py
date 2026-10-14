"""Measure the test set: each sequence of a config run, scored against its reference, tabled.

Milestone 2A's test set (``docs/design/2026-10-12-phone-robust-vo.md``) is public datasets'
sequences imported to TUM format (:mod:`slam_it.eval.zju`, :mod:`slam_it.eval.advio`,
:mod:`slam_it.eval.arkitscenes`, and TUM's own folders). A config file lists them. It is kept
outside the repository, since it holds this machine's paths; ``scripts/testset.template.toml``
is its format::

    [[sequence]]
    name = "advio-07"
    folder = "/data/imported/advio-07"
    mode = "monocular"
    reference = "ADVIO: IMU and fixed points"

Each sequence runs on one backend (:func:`slam_it.orchestration.vo.run`) with ORB-SLAM's
settings and, unless ``--features default``, the features for its frame size, as the demo's
worker sets a run up (:func:`slam_it.orchestration.vo.options_for`), then is measured:

- ``frames``: the sequence's frames for its mode; ``run``: those run, since a run ends at the
  frame it loses the track at;
- ``tracked`` and ``share``: the frames tracked, and their share of the sequence's;
- ``tracked_from``, ``tracked_to`` and ``ran_to``: times since the sequence's first frame, and
  ``lost`` (:func:`slam_it.eval.runs.tracking`);
- ``ate``: the RMSE in metres over the tracked frames matched to the reference, each map aligned
  on its own, Sim(3) for monocular and SE(3) otherwise, with ``matched`` and the ``reference``
  named; each map's own in ``map_ate`` (``map``, ``matched``, ``ate``), and in
  ``unscored_maps`` the maps too short or straight to align; ``ate`` null, with ``unscored``
  saying why, if no map can be scored. It is scored alone
  (:func:`slam_it.eval.runs.evaluate_absolute`): the measure reports no relative error, and a
  reference too sparse for one still scores;
- ``p95``: the 95th percentile of a frame's latency, building and tracking it, in seconds,
  read against the frame period ``deadline``, with the frames ``late`` past it
  (:func:`slam_it.eval.budgets.judge`); null, with ``unjudged`` saying why, if the run is too
  short to judge.

``--camera nominal`` runs each sequence with a guess for its camera in place of its own: a
typical phone's field of view (:func:`slam_it.eval.lenses.nominal`), without distortion, on a
folder of links to its frames written beside the run. Its depth's virtual baseline is kept, so
an RGB-D run's split of near and far points does not move with the guess. It measures what a
guessed lens costs.

The results are one JSON line per sequence in ``OUT/results.jsonl``, and a table on stdout::

    python scripts/measure_testset.py CONFIG OUT [--backend cpp] [--features by-size|default]
        [--camera exact|nominal] [--only NAME ...]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tomllib
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np

from slam_it.edge.options import MODES, Options
from slam_it.eval import budgets, lenses, runs, tum
from slam_it.foundations.errors import InvalidInputError, SlamItError
from slam_it.orchestration import vo

KEYS = ("name", "folder", "mode", "reference")
"""Each sequence's keys in the config, every one a string."""

LINKED = ("rgb", "rgb.txt", "depth", "depth.txt", "groundtruth.txt")
"""What a nominal camera's folder links to in the sequence's, where the sequence has it."""


class Entry(NamedTuple):
    """A sequence of the test set, as its config lists it."""

    name: str
    folder: Path
    mode: str
    reference: str


def read_config(path: Path) -> list[Entry]:
    """Read a config's sequences.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it is not TOML, lists no sequence, a sequence lacks a key or has another, a value is
        not a string, a mode is unknown, or two sequences share a name.
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
        if not isinstance(item, dict) or set(item) != set(KEYS):
            raise InvalidInputError(f"{path}: sequence {number}: expected the keys {KEYS}")
        if not all(isinstance(item[key], str) for key in KEYS):
            raise InvalidInputError(f"{path}: sequence {number}: expected strings")
        if item["mode"] not in MODES:
            raise InvalidInputError(f"{path}: sequence {number}: unknown mode {item['mode']!r}")
        entries.append(Entry(item["name"], Path(item["folder"]), item["mode"], item["reference"]))
    names = [e.name for e in entries]
    if len(set(names)) != len(names):
        raise InvalidInputError(f"{path}: two sequences share a name")
    return entries


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


def measure(
    entry: Entry, out: Path, *, backend: str, by_size: bool, nominal: bool
) -> dict[str, Any]:
    """Run a sequence and measure it (the module docstring); return its result."""
    folder = nominal_folder(entry.folder, out / "sequence") if nominal else entry.folder
    camera = tum.read_camera(folder)
    options = vo.options_for(camera.width, camera.height) if by_size else Options()
    run = vo.run("tum", folder, entry.mode, out / "run", backend=backend, options=options)
    times = vo.open_source("tum", folder, entry.mode).times
    span = runs.tracking(run, times)
    result: dict[str, Any] = {
        "name": entry.name,
        "mode": entry.mode,
        "backend": backend,
        "camera": "nominal" if nominal else "exact",
        "size": [camera.width, camera.height],
        "features": options.n_features,
        "frames": len(times),
        "run": span.frames,
        "tracked": span.tracked,
        "share": span.tracked / len(times) if len(times) else 0.0,
        "tracked_from": span.tracked_from,
        "tracked_to": span.tracked_to,
        "ran_to": span.ran_to,
        "lost": span.lost,
        "reference": entry.reference,
        "ate": None,
        "matched": 0,
        "map_ate": [],
        "unscored_maps": [],
        "unscored": None,
        "p95": None,
        "deadline": None,
        "late": None,
        "unjudged": None,
    }
    truth = runs.ground_truth("tum", folder)
    try:
        if truth is None:
            raise InvalidInputError("no ground truth")
        scored = runs.evaluate_absolute(run, *truth)
        result.update(
            ate=scored.ate_rmse,
            matched=scored.matched,
            map_ate=[
                {"map": e.map, "matched": e.matched, "ate": e.absolute.rmse} for e in scored.maps
            ],
            unscored_maps=list(scored.unscored),
        )
    except InvalidInputError as error:
        result["unscored"] = str(error)
    try:
        judged = budgets.judge(run)
        result.update(p95=judged.latency.p95, deadline=judged.deadline, late=judged.late)
    except InvalidInputError as error:  # a run too short to judge: said, never hidden
        result["unjudged"] = str(error)
    return result


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
    return (
        f"| {result['name']} | {result['mode']} | {width}x{height} | {result['features']} | "
        f"{result['tracked']}/{result['frames']} ({100 * result['share']:.0f}%) | {span} | "
        f"{'yes' if result['lost'] else 'no'} | {ate} | {p95} |"
    )


HEADER = (
    "| sequence | mode | size | features | tracked | tracked from-to | lost | ATE (matched) | "
    "p95 latency of the frame period |\n|---|---|---|---|---|---|---|---|---|"
)
"""The table's header."""


def main(arguments: list[str]) -> int:
    """Measure the test set from the command line (the module docstring)."""
    parser = argparse.ArgumentParser(prog="python scripts/measure_testset.py")
    parser.add_argument("config", type=Path, help="the test set's config (TOML)")
    parser.add_argument("out", type=Path, help="the folder for the runs and results.jsonl")
    parser.add_argument("--backend", default="cpp", choices=("python", "cpp", "rust"))
    parser.add_argument("--features", default="by-size", choices=("by-size", "default"))
    parser.add_argument("--camera", default="exact", choices=("exact", "nominal"))
    parser.add_argument("--only", nargs="+", metavar="NAME", help="measure these sequences")
    args = parser.parse_args(arguments)
    entries = read_config(args.config)
    if args.only:
        unknown = set(args.only) - {e.name for e in entries}
        if unknown:
            parser.error(f"--only: not in the config: {', '.join(sorted(unknown))}")
        entries = [e for e in entries if e.name in args.only]
    if args.out.exists() and any(args.out.iterdir()):
        parser.error(f"{args.out}: expected a new or empty folder")
    args.out.mkdir(parents=True, exist_ok=True)
    print(HEADER, flush=True)
    with (args.out / "results.jsonl").open("w") as results:
        for entry in entries:
            try:
                result = measure(
                    entry,
                    args.out / entry.name,
                    backend=args.backend,
                    by_size=args.features == "by-size",
                    nominal=args.camera == "nominal",
                )
            except (SlamItError, FileNotFoundError) as error:
                print(f"| {entry.name} | failed: {error} |", flush=True)
                results.write(json.dumps({"name": entry.name, "failed": str(error)}) + "\n")
                continue
            results.write(json.dumps(result) + "\n")
            print(row(result), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
