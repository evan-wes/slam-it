"""Run a sequence through the edge pipeline with any backend, score the run, and record it.

Writes the run's folder with :func:`slam_it.orchestration.vo.run` (or reads one already
written, with ``--reuse``), then prints what it holds: frames tracked, keyframes, map points,
its frames judged against their real-time deadline (:func:`slam_it.eval.budgets.judge`), and
what its program recorded of its process (peak memory, CPUs, memory cap). ``--budget
laptop-cpu`` runs the backend's program within that compute budget
(:mod:`slam_it.eval.budgets`), pinned and its memory capped. ``--record-features`` records
each frame's keypoints with their fates, for the run viewer
(:func:`slam_it.eval.runs.read_features`). With the
sequence's ground truth (TUM's ``groundtruth.txt``, EuRoC's ``state_groundtruth_estimate0``) it
prints the ATE (Sim(3)-aligned for monocular, SE(3) otherwise) and the RPE over one frame
period (:func:`slam_it.eval.runs.evaluate`). With ``--rerun FILE.rrd`` it saves a rerun
recording of the trajectory, the keyframes and the map points (open it with
``rerun FILE.rrd``).

Example, the C++ program on 300 frames of a TUM sequence::

    python scripts/run_vo.py --dataset tum --sequence PATH --mode rgbd --out RUN_DIR
        --backend cpp --count 300 --rerun RUN.rrd

``--option KEY=VALUE`` sets a setting of :class:`slam_it.edge.options.Options`, named by its
group and field (repeatable), read as the programs read it
(:func:`slam_it.orchestration.vo.parse_setting`).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from slam_it import backends
from slam_it.edge.options import MODES, Options
from slam_it.eval import budgets, runs
from slam_it.foundations.errors import DegenerateError
from slam_it.orchestration import vo


def parse_options(pairs: list[str]) -> Options:
    """Build the settings from ``KEY=VALUE`` pairs, each value of its setting's type.

    Parameters
    ----------
    pairs : list of str
        The pairs.

    Returns
    -------
    slam_it.edge.options.Options
        The defaults, with the pairs' settings set.

    Raises
    ------
    SystemExit
        For a pair without ``=``, an unknown setting, or a value not of its type
        (:func:`slam_it.orchestration.vo.parse_setting`).
    """
    changes: dict[str, Any] = {}  # each of its setting's type (parse_setting)
    try:
        changes.update(vo.parse_setting(pair) for pair in pairs)
    except vo.UsageError as error:
        raise SystemExit(f"--option: {error}") from None
    return Options().with_settings(changes)


def score(run: runs.Run, truth: runs.Truth | None) -> runs.Evaluation | None:
    """Score a run against the ground truth, if there is one and enough of the run matches it.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run.
    truth : slam_it.eval.runs.Truth or None
        The ground truth.

    Returns
    -------
    slam_it.eval.runs.Evaluation or None
        The ATE and RPE, or None (the reason printed).
    """
    if truth is None:
        print("no ground truth: not scored")
        return None
    try:
        return runs.evaluate(run, *truth)
    except DegenerateError as error:  # no map with three frames matched, off a line
        print(f"not scored: {error}")
        return None


def summarize(run: runs.Run, scored: runs.Evaluation | None) -> None:
    """Print what a run holds and, if it was scored, its errors.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run.
    scored : slam_it.eval.runs.Evaluation or None
        Its errors.
    """
    print(
        f"{run.sequence} ({run.dataset}, {run.mode}, {run.backend}): "
        f"{len(run.timestamps)} of {len(run.diagnostics)} frames tracked, "
        f"{len(run.final_keyframes()[0])} keyframes, {len(run.final_points())} points "
        f"({len(run.keyframe_timestamps)} and {len(run.points)} made)"
    )
    if len(run.diagnostics) >= 2:
        judged = budgets.judge(run)

        def ms(seconds: float) -> str:
            return f"{1000 * seconds:.1f} ms"

        latency = judged.latency
        print(
            f"building and tracking a frame: median {ms(latency.median)}, 95th percentile "
            f"{ms(latency.p95)}, max {ms(latency.max)}; {judged.late} of {judged.count} frames "
            f"past the {ms(judged.deadline)} deadline (reading the images aside: median "
            f"{ms(judged.read.median)})"
        )
        if judged.mapping is not None:
            print(
                f"mapping {judged.mapped} keyframes: median {ms(judged.mapping.median)}, max "
                f"{ms(judged.mapping.max)}; load {judged.mapping_load:.2f} over the run, "
                f"{judged.mapping_peak_load:.2f} over its busiest {judged.window:.1f} s (below "
                f"1, a mapping thread would keep up)"
            )
    peak = "not recorded" if run.peak_memory is None else f"{run.peak_memory / 2**20:.0f} MiB"
    cpus = "not recorded" if run.cpus is None else ",".join(map(str, run.cpus))
    cap = "none recorded" if run.memory_cap is None else f"{run.memory_cap / 2**30:.2f} GiB"
    print(f"the program's peak memory: {peak}; CPUs: {cpus}; memory cap: {cap}")
    if scored is None:
        return
    for error in scored.maps:
        absolute, relative = error.absolute, error.relative
        scale = f", scale {absolute.scale:.4f}" if run.mode == "monocular" else ""
        drift = (
            "no RPE (no frame pair a period apart)"
            if relative is None
            else f"RPE per frame {100 * relative.translation_rmse:.3f} cm, "
            f"{np.degrees(relative.rotation_rmse):.3f} deg"
        )
        print(
            f"map {error.map}: ATE over {error.matched} frames: RMSE {100 * absolute.rmse:.2f} "
            f"cm, median {100 * absolute.median:.2f} cm, max {100 * absolute.max:.2f} cm{scale}; "
            f"{drift}"
        )
    if scored.unscored:
        print(f"maps not aligned (too few frames matched, or on a line): {list(scored.unscored)}")
    if len(scored.maps) > 1:
        rpe = scored.rpe_translation_rmse
        rotation = scored.rpe_rotation_rmse
        drift = (
            ""
            if rpe is None or rotation is None
            else f"; RPE {100 * rpe:.3f} cm, {np.degrees(rotation):.3f} deg"
        )
        print(
            f"every map, each aligned on its own: ATE over {scored.matched} frames, RMSE "
            f"{100 * scored.ate_rmse:.2f} cm{drift}"
        )


def record(
    path: Path, run: runs.Run, truth: runs.Truth | None, scored: runs.Evaluation | None
) -> None:
    """Save a rerun recording of the run: each map's trajectory, keyframes and points.

    Each map is drawn in the ground truth's frame when it was scored (its own ATE alignment
    applied), beside the true trajectory.

    Parameters
    ----------
    path : pathlib.Path
        The ``.rrd`` file to write.
    run : slam_it.eval.runs.Run
        The run.
    truth : slam_it.eval.runs.Truth or None
        The ground truth.
    scored : slam_it.eval.runs.Evaluation or None
        The run's errors, for the alignment.
    """
    import rerun as rr  # noqa: PLC0415 -- optional, only for --rerun

    rr.init("slam_it_vo")
    rr.save(str(path))
    if truth is not None:
        centres = truth[1][:, :3, 3]
        rr.log("world/truth", rr.LineStrips3D([centres], colors=[[80, 200, 80]]), static=True)
    aligned = {} if scored is None else {e.map: e.absolute.alignment for e in scored.maps}
    point_alive = run.point_lives[:, 1] < 0
    keyframe_alive = run.keyframe_lives[:, 1] < 0
    for map_id in (int(m) for m in run.maps()):
        entity = f"world/run/map_{map_id}"
        if map_id in aligned:
            alignment = aligned[map_id]
            rr.log(
                entity,
                rr.Transform3D(translation=alignment[:3, 3], mat3x3=alignment[:3, :3]),
                static=True,
            )
        points = run.points[point_alive & (run.point_maps == map_id)]
        rr.log(f"{entity}/points", rr.Points3D(points, radii=0.005), static=True)
        poses = run.poses[run.pose_maps == map_id]
        rr.log(f"{entity}/trajectory", rr.LineStrips3D([poses[:, :3, 3]]), static=True)
        kept = keyframe_alive & (run.keyframe_maps == map_id)
        for k, pose in zip(np.flatnonzero(kept), run.keyframe_poses[kept], strict=True):
            rr.log(
                f"{entity}/keyframes/{k}",
                rr.Transform3D(translation=pose[:3, 3], mat3x3=pose[:3, :3]),
                static=True,
            )
            rr.log(
                f"{entity}/keyframes/{k}/frustum",
                rr.Pinhole(focal_length=0.5, width=1.0, height=0.75),
                static=True,
            )


def main() -> None:
    """Run the script."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dataset", choices=runs.DATASETS, required=True)
    parser.add_argument("--sequence", type=Path, required=True, help="the sequence's folder")
    parser.add_argument("--mode", choices=MODES, required=True)
    parser.add_argument("--out", type=Path, required=True, help="the run's folder")
    parser.add_argument("--backend", choices=backends.BACKENDS, help="default: cpp")
    parser.add_argument("--first", type=int, help="the first frame run (default: 0)")
    parser.add_argument("--count", type=int, help="the frames run (default: all)")
    parser.add_argument(
        "--option", action="append", default=[], metavar="KEY=VALUE", help="a setting"
    )
    parser.add_argument(
        "--record-features", action="store_true", help="record each frame's keypoints and fates"
    )
    parser.add_argument(
        "--reuse", action="store_true", help="read the run already in --out instead of running"
    )
    parser.add_argument("--rerun", type=Path, help="save a rerun recording to this .rrd file")
    parser.add_argument(
        "--budget", choices=budgets.BUDGETS, help="run the backend's program within this budget"
    )
    args = parser.parse_args()
    if args.reuse:
        running = {
            "--backend": args.backend,
            "--first": args.first,
            "--count": args.count,
            "--option": args.option or None,
            "--budget": args.budget,
            "--record-features": args.record_features or None,
        }
        given = [flag for flag, value in running.items() if value is not None]
        if given:
            parser.error(f"--reuse only reads the run in --out; {', '.join(given)} would not apply")
    budget = None if args.budget is None else budgets.BUDGETS[args.budget]

    if args.reuse:
        run = runs.read_run(args.out)
    else:
        run = vo.run(
            args.dataset,
            args.sequence,
            args.mode,
            args.out,
            backend=args.backend or "cpp",
            options=parse_options(args.option),
            first=args.first or 0,
            count=args.count,
            budget=budget,
            record_features=args.record_features,
        )
        if budget is not None:
            print(f"within {budget.name} ({budget.stands_for})")
    truth = runs.ground_truth(args.dataset, args.sequence)
    scored = score(run, truth)
    summarize(run, scored)
    if args.rerun:
        record(args.rerun, run, truth, scored)
        print(f"recording: {args.rerun}")


if __name__ == "__main__":
    main()
