"""Generate ``global_ba_slide.json``: a monocular global BA whose scale GTSAM's damping lets slide.

Run from the repository root with the dumped problem's files (they are not in the repository)::

    python -m tests.fixtures.edge.generate_global_ba_slide DUMP_PREFIX

The source is a global BA dumped from a failing run of the loop room (its TUM monocular case,
C++, threaded with ORB-SLAM's locks, ``mapping.redundant_observers`` at 4, on 2026-10-22; its
final ATE was 4.33 cm): 103 keyframes, the first held, 5,372 points and 39,160 observations,
written by a temporary probe as ``.npy`` files (``DUMP_PREFIX`` + ``poses.npy``,
``points.npy``, ``cameras.npy``, ``point_ids.npy``, ``pixels.npy``, ``information.npy``,
``fixed.npy``, ``intrinsics.npy``; each pose ``T_world_cam``, row-major, as
:func:`slam_it.foundations.graphs.bundle_adjust` takes it). Solved whole by GTSAM's own damping
(10 iterations from an absolute ``λ`` of 1e-5, divided by 10 per step), the similarity that
carries its keyframes' centres onto where they started has a scale of 0.5508, in all three
languages alike, at a final cost of 5091.76; by global BA's solve since 0.3.18.0
(:func:`slam_it.edge.loops.global_ba_options`), 0.9887 at 5091.82.

The fixture is a tenth of its points, drawn with seed 4 (:data:`FRACTION`, :data:`SEED`), and
every observation of them, 537 points and 3,930 observations: the smallest of the subsets tried
(3, 5, 7, 10, 15, 20 and 25% of the points, 6 to 12 seeds each) on which GTSAM's damping still
slides, to 0.7322 in every language, where global BA's solve holds it at 0.9968 (C++, Python
and Rust alike, 2026-10-23).
Every keyframe is still observed, and the first still held. ``tests/edge/test_loops.py`` holds
global BA's solve on it to its scale (0.3.17.2's review panel, L4).
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np

from tests.fixtures.foundations._layout import dumps

HERE = Path(__file__).parent

FRACTION = 0.1
"""The share of the dumped problem's points kept."""

SEED = 4
"""The seed that draws them."""


def subset(prefix: str) -> dict[str, Any]:
    """Return the fixture's problem: the dumped BA's drawn points, their observations, its cameras.

    Parameters
    ----------
    prefix : str
        The dumped files' common prefix.

    Returns
    -------
    dict
        ``bundle_adjust``'s inputs, as lists: ``poses`` (C, 4, 4), ``points`` (P, 3),
        ``cameras``, ``point_ids``, ``pixels`` (M, 2), ``information``, ``intrinsics`` and
        ``fixed``.
    """

    def read(name: str) -> np.ndarray[Any, Any]:
        return np.asarray(np.load(f"{prefix}{name}.npy", allow_pickle=False))

    poses = read("poses").reshape(-1, 4, 4)
    points = read("points").reshape(-1, 3)
    cameras = read("cameras").reshape(-1)
    point_ids = read("point_ids").reshape(-1)
    rng = np.random.default_rng(SEED)
    kept = np.sort(rng.choice(points.shape[0], int(FRACTION * points.shape[0]), replace=False))
    rows = np.isin(point_ids, kept)
    seen = np.unique(cameras[rows])
    if seen.shape[0] != poses.shape[0]:
        raise SystemExit("the subset leaves a keyframe unobserved")
    renumber = np.full(points.shape[0], -1, dtype=np.int64)
    renumber[kept] = np.arange(kept.shape[0])
    return {
        "poses": poses.tolist(),
        "points": points[kept].tolist(),
        "cameras": cameras[rows].astype(np.int64).tolist(),
        "point_ids": renumber[point_ids[rows]].tolist(),
        "pixels": read("pixels").reshape(-1, 2)[rows].tolist(),
        "information": read("information").reshape(-1)[rows].tolist(),
        "intrinsics": read("intrinsics").reshape(-1).tolist(),
        "fixed": read("fixed").reshape(-1).astype(np.int64).tolist(),
    }


def main() -> None:
    """Write ``global_ba_slide.json`` beside this file from the dump named on the command line."""
    if len(sys.argv) != 2:  # the program and its one argument
        raise SystemExit("usage: python -m tests.fixtures.edge.generate_global_ba_slide PREFIX")
    doc = {
        "description": (
            "A monocular global BA whose scale GTSAM's damping lets slide: a tenth of the points "
            "of the loop room's failing global BA (generate_global_ba_slide.py)."
        ),
        "camera": "pinhole",
        "problem": subset(sys.argv[1]),
    }
    (HERE / "global_ba_slide.json").write_text(dumps(doc))


if __name__ == "__main__":
    main()
