"""Generate ``graphs.json``: the parity fixture of rung 0's factor graphs on GTSAM.

Run from the repository root after changing the cases or the reference (the fixture is
committed)::

    python -m tests.fixtures.foundations.generate_graphs

Unlike the closed-form fixtures, this one imports slam_it: a nonlinear least-squares optimum on
noisy data has no independent closed form, so the expected values are the Python reference's
results (:mod:`slam_it.foundations.graphs`), against which the stage comparisons already hold
the compiled backends. What this fixture adds is rung 0's own check of every backend against
them, the solves' summaries included (B3's review panel, A2), under :data:`TOLERANCES`: each
field's policy, applied by ``tests.conftest.assert_matches_fixture`` (run 2, H22 = A19). The
cases:

- ``optimize_pose_graph``: a ring of 8 poses with noisy relatives, in SE(3) and in Sim(3), pose 0
  held; and each again with poses 0 to 3 held, so that edges join two held poses, a held pose to a
  free one, and a free pose to a held one;
- ``refine_similarity``: ORB-SLAM's two rounds, monocular (scaled) and stereo (rigid): a first
  with a Huber loss on noisy matches and a few outliers, then, the matches past the bound
  dropped, a second with none, from the first's result;
- ``bundle_adjust``: a monocular BA through a radtan camera (each language's own factor), two
  cameras held so its scale is fixed, and a stereo BA through a pinhole camera (GTSAM's own
  factors in Python, each core's elsewhere); half their cameras held (as a merge's welding BA
  holds the matched map's), the monocular radtan BA, a monocular pinhole one and the stereo one,
  the weld's typical stereo case (0.3.17.2's review panel, L8), and the stereo one with every
  camera held, its points alone adjusted; the monocular radtan BA solved as loop closing's global
  BA solves (0.3.18.0): g2o's scaled damping and ORB-SLAM3's stop after three small steps; and
  the stereo BA with a point 1 mm in front of a camera and one behind two, the observations near
  their camera's plane left out (``near`` 1e-3).

Regenerated in 0.3.18.0 with the new cases. Until then every case holding a camera or a pose
held the results of solves that held them through GTSAM's ``NonlinearEquality``, before they were
held as constants (0.3.17.2); regenerated from the constant form, they moved by at most 3.8e-11
in a pose and 1.9e-10 m in a point (the stereo BA, camera 0 held; the others 5.9e-13 and 1.6e-14
at most), their errors by 1.1e-14 relative, their iterations and terminations not at all; the
similarity refinements, which hold nothing, not at all. The stereo case with half its cameras
held was left out until then: with seed 11, the constrained C++ solve differed from Python's by
2.4e-8 m in a point; with the cameras constants, the backends agree within 7.1e-15.

NaN (a match image 2 does not observe, a monocular observation's right column) is written as
null.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from slam_it.foundations import camera, graphs, se3
from tests.fixtures.foundations import _layout

HERE = Path(__file__).parent

_STEPS = "GTSAM's C++ factors, Python's custom factors and Rust's bridge differ in the last bits"

TOLERANCES = {
    "poses": {
        "comparison": "close", "rtol": 0.0, "atol": 1e-8,
        "reason": f"rotation entries and metres, O(1); {_STEPS}",
    },
    "points": {
        "comparison": "close", "rtol": 0.0, "atol": 1e-8,
        "reason": f"metres, every coordinate within 12 m; {_STEPS}",
    },
    "similarity": {
        "comparison": "close", "rtol": 0.0, "atol": 1e-8,
        "reason": f"a scaled rotation's entries and a translation in metres, O(1); {_STEPS}",
    },
    "chi2": {
        "comparison": "close", "rtol": 1e-8, "atol": 1e-8,
        "reason": (
            "squared whitened pixels, 1e-3 to 6e3 (the outliers'): 1e-8 absolute for the "
            "inliers, 1e-8 relative for the outliers"
        ),
    },
    "errors": {
        "comparison": "close", "rtol": 1e-6, "atol": 1e-12,
        "reason": "GTSAM's initial and final errors, 1e-4 to 2e4: summed over every factor",
    },
    "codes": {
        "comparison": "exact", "rtol": 0.0, "atol": 0.0,
        "reason": "iterations, termination and counts: the backends' solves step alike",
    },
}  # fmt: skip
"""Each field's policy (``tests.conftest.assert_matches_fixture``)."""

PINHOLE = [520.0, 515.0, 320.0, 240.0]
"""A pinhole camera's intrinsics."""

RADTAN = [517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633]
"""A radtan camera's intrinsics (TUM fr1's): fx, fy, cx, cy, k1, k2, p1, p2, k3."""

BF = 40.0
"""A stereo camera's baseline times its focal length."""

GLOBAL_BA = graphs.Options(max_iterations=10, scaled_damping=True, small_steps=3)
"""Loop closing's global BA's solve (0.3.18.0; ``slam_it.edge.loops.global_ba_options``): g2o's
damping from ``τ`` 1e-5, and ORB-SLAM3's stop after three decreases in a row under 0.1%."""


def _json(value: Any) -> Any:  # noqa: ANN401 -- any JSON-able value
    """Return a value as JSON holds it: arrays as lists, NaN as null."""
    if isinstance(value, dict):
        return {key: _json(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return _json(value.tolist())
    if isinstance(value, list):
        return [_json(v) for v in value]
    if isinstance(value, float) and math.isnan(value):
        return None
    return value


def _summary(summary: graphs.Summary) -> dict[str, Any]:
    """Return a solve's summary as the fixture holds it: its errors and its codes (the counts)."""
    return {
        "errors": [summary.initial_error, summary.final_error],
        "codes": {
            "iterations": summary.iterations,
            "termination": str(summary.termination),
            "left_out": summary.left_out,
            "ended_near": summary.ended_near,
        },
    }


def _ring(similarity: bool, seed: int, held: int = 1) -> dict[str, Any]:
    """Return a ring of 8 poses, its relatives noisy, a loop closing it; the first `held` held."""
    rng = np.random.default_rng(seed)
    count = 8
    truth = []
    for k in range(count):
        angle = 2 * math.pi * k / count
        pose = se3.exp([2.0 * math.cos(angle), 2.0 * math.sin(angle), 0.1 * k, 0.0, 0.0, angle])
        if similarity:
            pose = pose.copy()
            pose[:3, :3] *= 1.0 + 0.02 * k
        truth.append(pose)
    edges = [[k, (k + 1) % count] for k in range(count)] + [[0, 4]]
    relatives = []
    for i, j in edges:
        relative = np.linalg.inv(truth[i]) @ truth[j]
        noise = se3.exp(rng.normal(0.0, [0.01, 0.01, 0.01, 0.005, 0.005, 0.005]))
        relatives.append(relative @ noise)
    start = [pose @ se3.exp(rng.normal(0.0, 0.05, 6)) for pose in truth]
    start[0] = truth[0]
    inputs: dict[str, Any] = {
        "poses": np.array(start), "edges": edges, "relatives": np.array(relatives),
        "fixed": list(range(held)), "similarity": similarity,
    }  # fmt: skip
    solved = graphs.optimize_pose_graph(**inputs)
    return {
        "name": f"ring, {'Sim(3)' if similarity else 'SE(3)'}"
        + (f", poses 0 to {held - 1} held" if held > 1 else ""),
        "input": _json(inputs),
        "expected": {"poses": _json(solved.poses), **_summary(solved.summary)},
    }


def _matches(fixed_scale: bool, seed: int) -> dict[str, Any]:
    """Return noisy matches through a similarity, a few outliers, a third unseen in image 2."""
    rng = np.random.default_rng(seed)
    count = 40
    rotation = se3.exp([0.0, 0.0, 0.0, 0.02, -0.04, 0.03])[:3, :3]
    scale = 1.0 if fixed_scale else 1.15
    truth = np.eye(4)
    truth[:3, :3] = scale * rotation
    truth[:3, 3] = [0.1, -0.05, 0.2]
    points2 = np.column_stack([rng.uniform(-1, 1, (count, 2)), rng.uniform(2, 6, count)])
    points1 = points2 @ truth[:3, :3].T + truth[:3, 3]
    pixels1 = camera.project("pinhole", np.array(PINHOLE), points1) + rng.normal(0, 0.7, (count, 2))
    pixels2 = camera.project("pinhole", np.array(PINHOLE), points2) + rng.normal(0, 0.7, (count, 2))
    pixels1[:4] += rng.uniform(30, 60, (4, 2))  # four outliers
    pixels2[::3] = np.nan  # a third of the matches unseen in image 2
    start = truth @ se3.exp([0.02, -0.01, 0.01, 0.01, 0.0, -0.01])
    if not fixed_scale:
        start = start.copy()
        start[:3, :] *= 1.03
        start[3] = [0.0, 0.0, 0.0, 1.0]
    inputs: dict[str, Any] = {
        "similarity": start, "points1": points1, "points2": points2, "pixels1": pixels1,
        "pixels2": pixels2, "information": np.ones((count, 2)), "model": "pinhole",
        "intrinsics": np.array(PINHOLE), "fixed_scale": fixed_scale, "huber": math.sqrt(10.0),
    }  # fmt: skip
    return inputs


def _refinements(fixed_scale: bool, seed: int) -> list[dict[str, Any]]:
    """Return ORB-SLAM's two rounds: Huber, then the matches past 10 dropped and no loss."""
    first = _matches(fixed_scale, seed)
    one = graphs.refine_similarity(**first)
    keep = ~np.any(one.chi2 > 10.0, axis=1)
    second = {**first, "similarity": one.similarity, "huber": None}
    for key in ("points1", "points2", "pixels1", "pixels2", "information"):
        second[key] = first[key][keep]
    two = graphs.refine_similarity(**second)
    kind = "stereo (rigid)" if fixed_scale else "monocular (scaled)"
    return [
        {
            "name": f"{kind}, round 1 (Huber)",
            "input": _json(first),
            "expected": {
                "similarity": _json(one.similarity),
                "chi2": _json(one.chi2),
                **_summary(one.summary),
            },
        },
        {
            "name": f"{kind}, round 2 (no robust loss, {int(keep.sum())} matches kept)",
            "input": _json(second),
            "expected": {
                "similarity": _json(two.similarity),
                "chi2": _json(two.chi2),
                **_summary(two.summary),
            },
        },
    ]


def _bundle(
    stereo: bool,
    seed: int,
    held: int | None = None,
    model: str | None = None,
    *,
    options: graphs.Options | None = None,
    near: float = 0.0,
) -> dict[str, Any]:
    """Return six cameras round a cloud of 40 points, the observations noisy.

    Stereo holds camera 0; monocular holds cameras 0 and 1, fixing the scale too: with one
    camera held, a monocular BA's scale is free, and the backends' solutions differ along it at
    the same error (measured: 3e-5 in the points, the final errors equal to 1e-15). `held`, if
    given, holds the first `held` cameras instead; `model` overrides the camera (stereo's pinhole,
    monocular's radtan). `options`, if given, is the solve's (written to the input as a dict);
    `near` above 0 adds two points, 1 mm in front of camera 3 and 1 m behind camera 0, each seen
    by cameras 0, 3 and 5, and leaves out the observations near their camera's plane.
    """
    rng = np.random.default_rng(seed)
    model = model or ("pinhole" if stereo else "radtan")
    intrinsics = PINHOLE if model == "pinhole" else RADTAN
    poses = np.stack([se3.exp([0.3 * k, 0.05 * k, 0.0, 0.0, 0.03 * k, 0.0]) for k in range(6)])
    points = np.column_stack(
        [rng.uniform(-1.5, 1.5, (40, 2)) + np.array([0.8, 0.0]), rng.uniform(4, 8, 40)]
    )
    cameras, point_ids, pixels, right = [], [], [], []
    for c, pose in enumerate(poses):
        local = (points - pose[:3, 3]) @ pose[:3, :3]
        seen = camera.project(model, np.array(intrinsics), local) + rng.normal(0, 0.5, (40, 2))
        for p in range(40):
            cameras.append(c)
            point_ids.append(p)
            pixels.append(seen[p])
            right.append(seen[p, 0] - BF / local[p, 2] + rng.normal(0, 0.5) if stereo else math.nan)
    start_poses = np.stack([pose @ se3.exp(rng.normal(0, 0.01, 6)) for pose in poses])
    start_poses[0] = poses[0]
    start_points = points + rng.normal(0, 0.05, points.shape)
    if near:
        degenerate = [
            poses[3][:3, :3] @ np.array([0.0, 0.0, 1e-3]) + poses[3][:3, 3],
            poses[0][:3, :3] @ np.array([0.1, 0.0, -1.0]) + poses[0][:3, 3],
        ]
        start_points = np.concatenate([start_points, np.array(degenerate)])
        for p in (40, 41):
            for c in (0, 3, 5):
                cameras.append(c)
                point_ids.append(p)
                pixels.append(np.array([300.0, 250.0]))
                right.append(math.nan)
    inputs: dict[str, Any] = {
        "poses": start_poses, "points": start_points,
        "cameras": cameras, "point_ids": point_ids, "pixels": np.array(pixels),
        "right": np.array(right), "information": np.ones(len(cameras)), "model": model,
        "intrinsics": np.array(intrinsics), "bf": BF if stereo else 0.0,
        "fixed": list(range(held)) if held else [0] if stereo else [0, 1],
    }  # fmt: skip
    if near:
        inputs["near"] = near
    solved = graphs.bundle_adjust(**inputs, options=options)
    if options is not None:
        inputs["options"] = options.as_dict()
    which = "every camera" if held == len(poses) else f"cameras 0 to {held - 1}" if held else ""
    how = ", global BA's damping and stop" if options is not None else ""
    how += f", near {near:g}" if near else ""
    return {
        "name": f"{'stereo' if stereo else 'monocular'}, {model}"
        + (f", {which} held" if held else "")
        + how,
        "input": _json(inputs),
        "expected": {
            "poses": _json(solved.poses),
            "points": _json(solved.points),
            **_summary(solved.summary),
        },
    }


def main() -> None:
    """Write ``graphs.json`` beside this file."""
    doc = {
        "description": (
            "Rung 0's factor graphs on GTSAM, the Python reference's results on noisy data "
            "(generate_graphs.py)."
        ),
        "oracle": "the Python reference; see the generator's docstring",
        "tolerance": TOLERANCES,
        "functions": {
            "optimize_pose_graph": {
                "cases": [_ring(False, 1), _ring(True, 2), _ring(False, 7, 4), _ring(True, 8, 4)]
            },
            "refine_similarity": {"cases": _refinements(False, 3) + _refinements(True, 4)},
            "bundle_adjust": {
                "cases": [
                    _bundle(False, 5),
                    _bundle(True, 6),
                    _bundle(False, 9, 3),
                    _bundle(False, 10, 3, "pinhole"),
                    _bundle(True, 11, 3, "pinhole"),
                    _bundle(True, 12, 6),
                    _bundle(False, 13, options=GLOBAL_BA),
                    _bundle(True, 14, near=1e-3),
                ]
            },
        },
    }
    (HERE / "graphs.json").write_text(_layout.dumps(doc))


if __name__ == "__main__":
    main()
