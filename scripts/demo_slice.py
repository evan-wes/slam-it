"""Bundle adjustment across reifyscene's investigation slice: slamlab against COLMAP's Ceres.

For each scene of the slice (the ten DAVIS scenes and spin, ``runs/slice_*``), reads the
seed reconstruction reifyscene's ``pose_refine`` stage exported (VGGSfM tracks and VGGT
cameras, before reifyscene's own BA) and refines it twice on the identical problem: with
slamlab's BA and with COLMAP's. Focal lengths are free, the principal point fixed, and both
use the same loss. Prints one table; the verdict is the table, not any single scene.

A scene without a seed (spin: its pose_refine takes a different route) is listed, not
skipped silently.

Run from the repository root, in the slamlab environment::

    python scripts/demo_slice.py --runs $REIFYSCENE_DATA_DIR/runs
    python scripts/demo_slice.py --loss huber --loss-scale 2.0
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from slamlab.eval import colmap_reference
from slamlab.eval.reprojection import in_front, rms
from slamlab.foundations import ba, nls

SLICE = (
    "lab-coat",
    "blackswan",
    "crossing",
    "dogs-scale",
    "gold-fish",
    "india",
    "judo",
    "lindy-hop",
    "miami-surf",
    "motocross-bumps",
    "spin",
)
"""reifyscene's investigation slice (``runs/slice_configs/rebuild_slice.sh``)."""

SEED = Path("pose_refine") / "colmap_seed"
"""Where a run keeps its seed reconstruction."""

_CENTRE = {"pinhole": slice(2, 4), "radial": slice(1, 3), "radtan": slice(2, 4)}
"""Each model's principal-point entries (held fixed, as reifyscene's BA does)."""


def focal_free(model: ba.ColmapModel) -> ba.BAProblem:
    """Free every refinable intrinsic except the principal point."""
    fixed = ~model.refinable
    fixed[:, _CENTRE[model.problem.camera_model]] = True
    return replace(model.problem, fixed_intrinsics=fixed)


def run_scene(seed: Path, loss: nls.Loss, iterations: int) -> str:
    """Refine one scene's seed with both solvers; return its table row."""
    problem = focal_free(ba.read_colmap_text(seed))
    options = ba.BAOptions(solver=nls.Options(max_iterations=iterations, loss=loss))
    start = time.perf_counter()
    ours = ba.solve(problem, options, backend="cpp")
    ours_seconds = time.perf_counter() - start
    reference = colmap_reference.solve(
        problem,
        colmap_reference.ColmapOptions(
            refine_extra=False, loss=loss, tolerance=1e-6, max_iterations=iterations
        ),
    )
    ours_solved = replace(problem, poses=ours.poses, intrinsics=ours.intrinsics, points=ours.points)
    colmap_solved = replace(
        problem, poses=reference.poses, intrinsics=reference.intrinsics, points=reference.points
    )
    front = in_front(problem)
    ours_front, colmap_front = in_front(ours_solved), in_front(colmap_solved)
    # One common set: in front at the start and in both results.
    used = front & ours_front & colmap_front
    ours_rms, colmap_rms = rms(ours_solved, used), rms(colmap_solved, used)
    return (
        f"{problem.poses.shape[0]:>4} {problem.points.shape[0]:>6} "
        f"{np.count_nonzero(~front):>5} {np.count_nonzero(front & ~ours_front):>5} "
        f"{np.count_nonzero(front & ~colmap_front):>5} {rms(problem, used):>8.4f} "
        f"{ours_rms:>8.4f} {colmap_rms:>8.4f} {ours_rms - colmap_rms:>+9.2e} "
        f"{ours.summary.solve.iterations:>4} {ours.summary.solve.termination:>14} "
        f"{ours.summary.unobservable_focal:>3} {ours.intrinsics_std[0, 0]:>8.2f} "
        f"{ours.intrinsics[0, 0]:>8.2f} {reference.intrinsics[0, 0]:>8.2f} "
        f"{ours_seconds:>6.2f} {reference.seconds:>6.2f}"
    )


def main() -> None:
    """Parse arguments, run every scene, print the table."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--runs", type=Path, default=Path("$REIFYSCENE_DATA_DIR/runs").resolve()
    )
    parser.add_argument("--loss", choices=nls.LOSSES, default="trivial")
    parser.add_argument("--loss-scale", type=float, default=1.0)
    parser.add_argument("--iterations", type=int, default=100, help="cap for both solvers")
    args = parser.parse_args()
    loss = nls.Loss(args.loss, args.loss_scale)
    print(
        f"loss: {loss.kind} (scale {loss.scale}). RMS in px over the observations in front at "
        "the start and in both results; 'behind' counts observations behind their camera at "
        "the start, and pushed behind by each solver"
    )
    print(
        f"{'scene':<16} {'imgs':>4} {'points':>6} {'bhnd0':>5} {'b-our':>5} {'b-col':>5} "
        f"{'start':>8} {'slamlab':>8} {'colmap':>8} {'diff':>9} {'its':>4} {'termination':>14} "
        f"{'unf':>3} {'sigma_f':>8} "
        f"{'f ours':>8} {'f colmap':>8} {'t ours':>6} {'t col':>6}"
    )
    for scene in SLICE:
        seed = args.runs / f"slice_{scene}" / SEED
        row = (
            run_scene(seed, loss, args.iterations)
            if seed.is_dir()
            else "no seed reconstruction in this run"
        )
        print(f"{scene:<16} {row}", flush=True)


if __name__ == "__main__":
    main()
