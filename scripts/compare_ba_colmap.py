"""Compare slamlab's bundle adjustment with COLMAP's on COLMAP text models, one table.

Each argument names a COLMAP text model (``cameras.txt``, ``images.txt``, ``points3D.txt``)
as ``NAME=DIRECTORY`` or just ``DIRECTORY``. Each model is refined twice on the identical
problem, by slamlab (C++) and by COLMAP's Ceres bundle adjuster, with the principal point
fixed and the chosen intrinsics free, under the same loss. The table lists, per model: the
RMS reprojection error at the start and after each solver, how many observations each solver
pushed behind their camera, our iterations, termination and focal-length deviation, both
focal lengths, and both run times. A directory without a model is listed, not skipped.

Run from anywhere, in the slamlab environment::

    python scripts/compare_ba_colmap.py lab-coat=/path/to/colmap_model other=/path/to/another
    python scripts/compare_ba_colmap.py --loss huber --loss-scale 2 --iterations 300 DIR...

``docs/notes/bundle-adjustment.md`` records the invocation and table for reifyscene's
investigation slice.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from slamlab.eval import colmap_reference, readers
from slamlab.eval.reprojection import in_front, rms
from slamlab.foundations import ba, camera, nls

REFINE = ("none", "focal", "focal+distortion")
"""What the comparison frees (the principal point always stays fixed)."""


def freed(model: readers.ColmapModel, refine: str) -> ba.BAProblem:
    """Free the model's refinable focal lengths (and distortion), never the principal point."""
    problem = model.problem
    fixed = np.ones_like(model.refinable)
    focal = list(camera.FOCAL_INDICES[problem.camera_model])
    centre = list(camera.CENTRE_INDICES[problem.camera_model])
    if refine != "none":
        fixed[:, focal] = ~model.refinable[:, focal]
    if refine == "focal+distortion":
        rest = [k for k in range(fixed.shape[1]) if k not in focal + centre]
        fixed[:, rest] = ~model.refinable[:, rest]
    return replace(problem, fixed_intrinsics=fixed)


def compare(directory: Path, refine: str, loss: nls.Loss, iterations: int) -> str:
    """Refine one model with both solvers; return its table row."""
    problem = freed(readers.read_colmap_text(directory), refine)
    options = ba.BAOptions(solver=nls.Options(max_iterations=iterations, loss=loss))
    start = time.perf_counter()
    ours = ba.solve(problem, options, backend="cpp")
    ours_seconds = time.perf_counter() - start
    reference = colmap_reference.solve(
        problem,
        colmap_reference.ColmapOptions(loss=loss, tolerance=1e-6, max_iterations=iterations),
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
        f"{ours.summary.unobservable_focal:>3} {np.sqrt(ours.summary.variance_factor):>6.3f} "
        f"{ours.intrinsics_std[0, 0]:>8.2f} "
        f"{ours.intrinsics[0, 0]:>8.2f} {reference.intrinsics[0, 0]:>8.2f} "
        f"{ours_seconds:>6.2f} {reference.seconds:>6.2f}"
    )


def main() -> None:
    """Parse arguments, compare every model, print the table."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("models", nargs="+", help="NAME=DIRECTORY or DIRECTORY")
    parser.add_argument("--refine", choices=REFINE, default="focal")
    parser.add_argument("--loss", choices=nls.LOSSES, default="trivial")
    parser.add_argument("--loss-scale", type=float, default=1.0)
    parser.add_argument("--iterations", type=int, default=100, help="cap for both solvers")
    args = parser.parse_args()
    loss = nls.Loss(args.loss, args.loss_scale)
    print(
        f"refine: {args.refine}; loss: {loss.kind} (scale {loss.scale}). RMS in px over the "
        "observations in front at the start and in both results; bhnd0 counts observations "
        "behind their camera at the start, b-our and b-col those each solver pushed behind; "
        "s0 is the a-posteriori noise in px, which sigma_f is scaled by"
    )
    print(
        f"{'model':<16} {'imgs':>4} {'points':>6} {'bhnd0':>5} {'b-our':>5} {'b-col':>5} "
        f"{'start':>8} {'slamlab':>8} {'colmap':>8} {'diff':>9} {'its':>4} {'termination':>14} "
        f"{'unf':>3} {'s0':>6} {'sigma_f':>8} {'f ours':>8} {'f colmap':>8} "
        f"{'t ours':>6} {'t col':>6}"
    )
    for spec in args.models:
        name, _, path = spec.rpartition("=")
        directory = Path(path)
        label = name or directory.name
        if (directory / "cameras.txt").is_file():
            row = compare(directory, args.refine, loss, args.iterations)
        else:
            row = f"no COLMAP model at {directory}"
        print(f"{label:<16} {row}", flush=True)


if __name__ == "__main__":
    main()
