"""Bundle adjustment on a real BAL problem: slamlab's BA against COLMAP's Ceres BA.

Downloads a problem from the Bundle Adjustment in the Large site (once, into ``--data``),
solves it with slamlab (every backend, or one) and with COLMAP's bundle adjuster, and prints
the RMS reprojection error, the iterations and the time of each. With ``--rerun FILE.rrd`` it
also saves a rerun recording of the cameras and points before and after (open it with
``rerun FILE.rrd``).

Run from the repository root, in the slamlab environment::

    python scripts/demo_bal.py --problem ladybug/problem-49-7776-pre
    python scripts/demo_bal.py --backend all
    python scripts/demo_bal.py --backend rust --rerun /tmp/ladybug.rrd

The data stays outside the repository (default ``~/datasets/bal``).
"""

from __future__ import annotations

import argparse
import time
import urllib.request
from dataclasses import replace
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slamlab.backends import BACKENDS
from slamlab.eval import colmap_reference, readers
from slamlab.eval.reprojection import in_front, rms
from slamlab.foundations import ba, nls

BAL_URL = "https://grail.cs.washington.edu/projects/bal/data/{problem}.txt.bz2"
"""Where BAL problems are published."""


def fetch(problem: str, data: Path) -> Path:
    """Return the local path of a BAL problem, downloading it if it is missing.

    The download goes to a ``.part`` file renamed on success, so an interrupted one is
    never mistaken for the problem.
    """
    path = data / f"{problem}.txt.bz2"
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        url = BAL_URL.format(problem=problem)
        print(f"downloading {url}")
        partial = path.with_name(path.name + ".part")
        urllib.request.urlretrieve(url, partial)
        partial.replace(path)
    return path


def record(path: Path, stages: dict[str, tuple[NDArray[np.float64], NDArray[np.float64]]]) -> None:
    """Save a rerun recording of each stage's camera centres and points."""
    import rerun as rr  # noqa: PLC0415 -- optional, only for --rerun

    rr.init("slamlab BAL demo")
    for name, (poses, points) in stages.items():
        rr.log(f"{name}/points", rr.Points3D(points, radii=0.01))
        rr.log(f"{name}/cameras", rr.Points3D(poses[:, :3, 3], radii=0.05, colors=[255, 120, 0]))
    rr.save(str(path))
    print(f"saved {path}")


def main() -> None:
    """Parse arguments, solve, print the comparison."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--problem", default="ladybug/problem-49-7776-pre")
    parser.add_argument("--data", type=Path, default=Path.home() / "datasets" / "bal")
    parser.add_argument("--backend", choices=[*BACKENDS, "all"], default="cpp")
    parser.add_argument("--rerun", type=Path, default=None, help="save a .rrd recording")
    args = parser.parse_args()
    problem = readers.read_bal(fetch(args.problem, args.data))
    used = in_front(problem)
    print(
        f"{args.problem}: {problem.poses.shape[0]} cameras, {problem.points.shape[0]} points, "
        f"{problem.observations.shape[0]} observations ({np.count_nonzero(~used)} behind their "
        f"camera, left out of every RMS); start RMS {rms(problem, used):.4f} px"
    )
    options = ba.BAOptions(solver=nls.Options(max_iterations=100), gauge="none")
    chosen = list(BACKENDS) if args.backend == "all" else [args.backend]
    stages = {"start": (problem.poses, problem.points)}
    for backend in chosen:
        start = time.perf_counter()
        result = ba.solve(problem, options, backend=backend)
        seconds = time.perf_counter() - start
        solved = replace(
            problem, poses=result.poses, intrinsics=result.intrinsics, points=result.points
        )
        summary = result.summary.solve
        print(
            f"slamlab {backend:>6}: RMS {rms(solved, used):.4f} px, "
            f"{summary.iterations} iterations ({summary.termination}), {seconds:.2f} s"
        )
        stages[f"slamlab_{backend}"] = (result.poses, result.points)
    reference = colmap_reference.solve(
        problem, colmap_reference.ColmapOptions(tolerance=1e-6, max_iterations=100)
    )
    colmap = replace(
        problem, poses=reference.poses, intrinsics=reference.intrinsics, points=reference.points
    )
    print(f"COLMAP (Ceres): RMS {rms(colmap, used):.4f} px, {reference.seconds:.2f} s")
    stages["colmap"] = (reference.poses, reference.points)
    if args.rerun is not None:
        record(args.rerun, stages)


if __name__ == "__main__":
    main()
