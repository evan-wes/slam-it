"""Compare SLAM It All!'s bundle adjustment with COLMAP's on COLMAP text models, one table.

Each argument names a COLMAP text model (``cameras.txt``, ``images.txt``, ``points3D.txt``)
as ``NAME=DIRECTORY`` or just ``DIRECTORY``. Each model is refined twice on the identical
problem, by SLAM It All! (C++) and by COLMAP's Ceres bundle adjuster, with the principal point
fixed and the chosen intrinsics free, under the same loss. The table lists, per model: the
RMS reprojection error at the start and after each solver, how many observations each solver
pushed behind their camera, our iterations, termination and focal-length deviation, both
focal lengths, and both run times. The focal columns are intrinsics group 0's (a model with
several cameras shows its first). A directory without a model is listed, not skipped.

Run from anywhere, in the slam-it-all environment::

    python scripts/compare_ba_colmap.py lab-coat=/path/to/colmap_model other=/path/to/another
    python scripts/compare_ba_colmap.py --loss huber --loss-scale 2 --iterations 300 DIR...

``docs/notes/bundle-adjustment.md`` records the invocation and table for reifyscene's
investigation slice.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from slam_it_all.eval import colmap_reference, readers
from slam_it_all.eval.reprojection import in_front, rms
from slam_it_all.foundations import ba, camera, nls

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


COLUMNS = [
    ("imgs", 4, ""),
    ("points", 6, ""),
    ("bhnd0", 5, ""),
    ("b-our", 5, ""),
    ("b-col", 5, ""),
    ("start", 8, ".4f"),
    ("ours", 8, ".4f"),
    ("colmap", 8, ".4f"),
    ("diff", 9, "+.2e"),
    ("its", 4, ""),
    ("termination", 14, ""),
    ("unf", 3, ""),
    ("s0", 6, ".3f"),
    ("sigma_f", 8, ".2f"),
    ("f ours", 8, ".2f"),
    ("f colmap", 8, ".2f"),
    ("t ours", 6, ".2f"),
    ("t col", 6, ".2f"),
]
"""The table's columns: title, width and format, in :func:`row`'s order."""

LEGEND = (
    "RMS in px over the observations in front at the start and in both results; bhnd0 counts "
    "observations behind their camera at the start, b-our and b-col those each solver "
    "pushed behind; s0 is the a-posteriori noise in px, which sigma_f is scaled by; sigma_f "
    "and the focal lengths are intrinsics group 0's (its first focal length; NaN if fixed)"
)
"""What the columns mean, printed above the table."""


@dataclass(frozen=True)
class Comparison:
    """One model refined by both solvers, on the identical problem."""

    problem: ba.BAProblem
    ours: ba.BAResult
    ours_seconds: float
    reference: colmap_reference.ColmapResult


def compare(directory: Path, refine: str, loss: nls.Loss, iterations: int) -> Comparison:
    """Refine one model with both solvers, under the same loss, cap and tolerance.

    COLMAP's Ceres tolerances are set to SLAM It All!'s relative cost-decrease tolerance, so both
    stop on the same criterion (Ceres also applies it to the gradient and the step).
    """
    problem = freed(readers.read_colmap_text(directory), refine)
    options = ba.BAOptions(solver=nls.Options(max_iterations=iterations, loss=loss))
    start = time.perf_counter()
    ours = ba.solve(problem, options, backend="cpp")
    ours_seconds = time.perf_counter() - start
    reference = colmap_reference.solve(
        problem,
        colmap_reference.ColmapOptions(
            loss=loss, tolerance=options.solver.function_tolerance, max_iterations=iterations
        ),
    )
    return Comparison(problem, ours, ours_seconds, reference)


def row(result: Comparison) -> str:
    """Format a comparison as its table row (focal lengths and σ_f of intrinsics group 0)."""
    problem, ours, reference = result.problem, result.ours, result.reference
    ours_solved = replace(problem, poses=ours.poses, intrinsics=ours.intrinsics, points=ours.points)
    colmap_solved = replace(
        problem, poses=reference.poses, intrinsics=reference.intrinsics, points=reference.points
    )
    front = in_front(problem)
    ours_front, colmap_front = in_front(ours_solved), in_front(colmap_solved)
    # One common set: in front at the start and in both results.
    used = front & ours_front & colmap_front
    ours_rms, colmap_rms = rms(ours_solved, used), rms(colmap_solved, used)
    focal = camera.FOCAL_INDICES[problem.camera_model][0]
    values = [
        problem.poses.shape[0],
        problem.points.shape[0],
        np.count_nonzero(~front),
        np.count_nonzero(front & ~ours_front),
        np.count_nonzero(front & ~colmap_front),
        rms(problem, used),
        ours_rms,
        colmap_rms,
        ours_rms - colmap_rms,
        ours.summary.solve.iterations,
        str(ours.summary.solve.termination),
        ours.summary.unobservable_focal,
        np.sqrt(ours.summary.variance_factor),
        ours.intrinsics_std[0, focal],
        ours.intrinsics[0, focal],
        reference.intrinsics[0, focal],
        result.ours_seconds,
        reference.seconds,
    ]
    return " ".join(
        _cell(value, width, spec) for (_, width, spec), value in zip(COLUMNS, values, strict=True)
    )


def _cell(value: object, width: int, spec: str) -> str:
    """Format one cell right-aligned in ``width``; a leading ``+`` in ``spec`` signs it."""
    sign, rest = ("+", spec[1:]) if spec.startswith("+") else ("", spec)
    return f"{value:>{sign}{width}{rest}}"


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
    print(f"refine: {args.refine}; loss: {loss.kind} (scale {loss.scale}). {LEGEND}")
    print(f"{'model':<16} " + " ".join(f"{title:>{width}}" for title, width, _ in COLUMNS))
    for spec in args.models:
        name, _, path = spec.rpartition("=")
        directory = Path(path)
        label = name or directory.name
        if (directory / "cameras.txt").is_file():
            line = row(compare(directory, args.refine, loss, args.iterations))
        else:
            line = f"no COLMAP model at {directory}"
        print(f"{label:<16} {line}", flush=True)


if __name__ == "__main__":
    main()
