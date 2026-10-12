"""Generate ``ba.json``, the bundle-adjustment parity fixture, from the Python reference.

Run from the repository root after changing BA or the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_ba

Unlike the geometry fixtures, there is no closed-form oracle for a bundle adjustment's
optimum. The reference is the Python implementation, whose correctness the BA tests check
on their own terms: zero cost on noise-free data, the Schur step against the dense solve,
Jacobians against finite differences, Ceres on a BAL problem. This fixture pins what it
computes on scenes that between them exercise every gauge, both methods, robust losses,
degenerate drops, the radial model, priors and anchored points, so a compiled backend is
held to the reference case by case, and a change to the reference shows up as a diff.

Each scene is small (five cameras, two dozen points) and seeded, so the file stays small.
NaN (fixed intrinsics' deviations, plain points' inverse depths) is written as ``null``.
"""

from __future__ import annotations

import math
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it.foundations import ba, camera, nls, se3
from tests.fixtures.foundations._layout import dumps

OUT = Path(__file__).with_name("ba.json")
CAMERAS = 5
POINTS = 24
INTRINSICS = {
    "pinhole": [500.0, 505.0, 320.0, 240.0],
    "radtan": [500.0, 505.0, 320.0, 240.0, -0.08, 0.01, 0.001, -0.0005, 0.0],
    "radial": [499.4, 320.0, 240.0, -0.31, 0.12],
}

TOLERANCES = {
    "poses": {"comparison": "close", "rtol": 0.0, "atol": 1e-9, "reason": "metres and radians"},
    "points": {"comparison": "close", "rtol": 0.0, "atol": 1e-8, "reason": "depths 5-9 m"},
    "intrinsics": {
        "comparison": "close",
        "rtol": 1e-8,
        "atol": 0.0,
        "reason": "focal lengths ~500 px next to distortion terms ~0.01",
    },
    "depth_affine": {"comparison": "close", "rtol": 0.0, "atol": 1e-9, "reason": "O(1)"},
    "inverse_depth": {"comparison": "close", "rtol": 1e-9, "atol": 0.0, "reason": "~0.1-0.2"},
    "intrinsics_std": {
        "comparison": "close",
        "rtol": 1e-6,
        "atol": 0.0,
        "reason": "the diagonal of an inverse, from a system solved to 1e-9",
    },
    "cost": {"comparison": "close", "rtol": 1e-9, "atol": 1e-20, "reason": "relative"},
    "codes": {
        "comparison": "exact",
        "rtol": 0.0,
        "atol": 0.0,
        "reason": "gauge, termination, iterations and counts: the two cores step alike",
    },
}
"""Each output's policy."""


def _scene(
    model: str,
    *,
    seed: int,
    noise: float = 0.5,
    rotation: float = 0.3,
    **create: Any,  # noqa: ANN401 -- BAProblem.create's keywords
) -> tuple[ba.BAProblem, NDArray[np.float64], NDArray[np.float64]]:
    """Return a seeded scene's problem, its true camera-to-world poses and inverse depths.

    Cameras step along x with ``rotation`` radians of orientation spread; every point is
    seen by every camera; the start perturbs poses (0.01) and points (0.1 m).
    """
    rng = np.random.default_rng(seed)
    world = rng.uniform([-2.0, -2.0, 5.0], [2.0, 2.0, 9.0], (POINTS, 3))
    steps = np.arange(CAMERAS)
    twists = np.concatenate(
        [
            np.stack([0.4 * steps, 0.1 * np.sin(steps), np.zeros(CAMERAS)], 1),
            rng.normal(0.0, rotation, (CAMERAS, 3)),
        ],
        axis=1,
    )
    poses = se3.exp(twists)
    t_cam_world = se3.inverse(poses)
    in_camera = (
        np.einsum("cij,nj->cni", t_cam_world[:, :3, :3], world) + t_cam_world[:, None, :3, 3]
    ).reshape(-1, 3)
    pixels = camera.project(model, INTRINSICS[model], in_camera)
    pixels = pixels + rng.normal(0.0, noise, pixels.shape)
    cam_idx, pt_idx = np.divmod(np.arange(CAMERAS * POINTS), POINTS)
    problem = ba.BAProblem.create(
        model,
        create.pop("intrinsics", INTRINSICS[model]),
        se3.compose(poses, se3.exp(rng.normal(0.0, 0.01, (CAMERAS, 6)))),
        world + rng.normal(0.0, 0.1, world.shape),
        pixels,
        cam_idx,
        pt_idx,
        **create,
    )
    return problem, poses, 1.0 / in_camera[:, 2]


def _similarity() -> tuple[ba.BAProblem, dict[str, Any]]:
    problem, _, _ = _scene("pinhole", seed=1)
    return problem, {}


def _first_pose() -> tuple[ba.BAProblem, dict[str, Any]]:
    problem, _, inverse_depth = _scene("pinhole", seed=2)
    metric = replace(
        problem,
        depth_prior=inverse_depth,
        depth_prior_weights=np.full(inverse_depth.size, 100.0),
    )
    return metric, {}


def _scale() -> tuple[ba.BAProblem, dict[str, Any]]:
    problem, poses, _ = _scene("pinhole", seed=3, fixed_intrinsics=[False])
    weights = np.zeros((CAMERAS, 2))
    weights[0] = 1e4
    return replace(problem, pose_prior=poses, pose_prior_weights=weights), {}


def _none() -> tuple[ba.BAProblem, dict[str, Any]]:
    problem, poses, _ = _scene("pinhole", seed=4)
    prior = se3.compose(poses, se3.exp(np.full((CAMERAS, 6), 0.002)))
    return replace(problem, pose_prior=prior, pose_prior_weights=np.full((CAMERAS, 2), 1e3)), {}


def _gauss_newton() -> tuple[ba.BAProblem, dict[str, Any]]:
    problem, _, _ = _scene("pinhole", seed=5)
    return problem, {"method": "gauss_newton", "gauge": "similarity"}


def _degenerate() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Add a point seen once (dropped) and one behind its cameras (its views left out)."""
    p, _, _ = _scene("pinhole", seed=6)
    points = np.vstack([p.points, [[0.0, 0.0, 7.0], [0.0, 0.0, -3.0]]])
    lonely, behind = POINTS, POINTS + 1
    extra_obs = np.array([[320.0, 240.0], [330.0, 250.0], [300.0, 230.0]])
    problem = ba.BAProblem.create(
        "pinhole",
        p.intrinsics,
        p.poses,
        points,
        np.vstack([p.observations, extra_obs]),
        np.concatenate([p.cam_idx, [0, 0, 1]]),
        np.concatenate([p.pt_idx, [lonely, behind, behind]]),
    )
    return problem, {}


def _radial() -> tuple[ba.BAProblem, dict[str, Any]]:
    """BAL's camera: focal length and both radial terms free, the centre fixed."""
    problem, _, _ = _scene(
        "radial", seed=7, fixed_intrinsics=[[False, True, True, False, False]], noise=0.3
    )
    return problem, {}


def _huber() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Five observations moved 20 px: Huber keeps them from pulling the solution."""
    p, _, _ = _scene("pinhole", seed=8)
    observations = p.observations.copy()
    observations[::23] += 20.0
    return replace(p, observations=observations), {"loss": ["huber", 2.0]}


def _every_kind() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Radtan with two groups, Cauchy, depth priors with free affines, pose priors, anchors."""
    groups = np.array(INTRINSICS["radtan"])
    p, poses, inverse_depth = _scene(
        "radtan",
        seed=9,
        intrinsics=np.stack([groups, groups * 1.001]),
        intrinsics_idx=[0, 0, 1, 1, 1],
        fixed_intrinsics=[False, False],
    )
    n_obs = p.cam_idx.size
    pose_weights = np.zeros((CAMERAS, 2))
    pose_weights[1] = [4.0, 0.0]
    pose_weights[2] = [1.0, 9.0]
    problem = replace(
        p,
        depth_prior=np.where(np.arange(n_obs) % 2 == 0, inverse_depth * 1.02, np.nan),
        depth_prior_weights=np.full(n_obs, 25.0),
        depth_affine=np.tile([1.1, 0.01], (CAMERAS, 1)),
        fixed_depth_affine=np.arange(CAMERAS) >= 2,
        pose_prior=se3.compose(poses, se3.exp(np.full((CAMERAS, 6), 0.01))),
        pose_prior_weights=pose_weights,
        point_anchor=np.where(np.arange(POINTS) % 3 == 0, ba.anchor_points(p).point_anchor, -1),
    )
    return problem, {"loss": ["cauchy", 2.0], "depth_loss": ["huber", 0.5]}


def _rotation_priors() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Attitude priors pin rotation: translation and scale are left, two poses take them."""
    problem, poses, _ = _scene("pinhole", seed=10)
    weights = np.zeros((CAMERAS, 2))
    weights[:, 1] = 1e4
    return replace(problem, pose_prior=poses, pose_prior_weights=weights), {}


def _translation_priors() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Two GNSS-style priors (centres only) pin all but the rotation about their baseline.

    The stored rotation is identity: the decoupled residual compares centres alone.
    """
    problem, poses, _ = _scene("pinhole", seed=11)
    weights = np.zeros((CAMERAS, 2))
    weights[[0, 3], 0] = 1e4
    centres_only = np.tile(np.eye(4), (CAMERAS, 1, 1))
    centres_only[:, :3, 3] = poses[:, :3, 3]
    return replace(problem, pose_prior=centres_only, pose_prior_weights=weights), {}


def _explicit_first_pose() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Use the explicit SE(3) recipe: the first pose fixed, metric depth pinning the scale."""
    problem, _, inverse_depth = _scene("pinhole", seed=12)
    metric = replace(
        problem, depth_prior=inverse_depth, depth_prior_weights=np.full(inverse_depth.size, 100.0)
    )
    return metric, {"gauge": "first_pose"}


def _explicit_scale() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Use the explicit scale recipe, about the one pose with a full prior."""
    problem, poses, _ = _scene("pinhole", seed=13)
    weights = np.zeros((CAMERAS, 2))
    weights[2] = 1e4
    return replace(problem, pose_prior=poses, pose_prior_weights=weights), {"gauge": "scale"}


def _explicit_none() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Use no gauge: the caller's two fixed poses apart pin the similarity."""
    problem, _, _ = _scene("pinhole", seed=14)
    fixed = np.zeros(CAMERAS, dtype=bool)
    fixed[[0, 4]] = True
    return replace(problem, fixed_poses=fixed), {"gauge": "none"}


def _far_prior() -> tuple[ba.BAProblem, dict[str, Any]]:
    """Give one camera a full prior 1.2 rad and 0.6 m from its truth: Jr⁻¹ far from I.

    The prior is the only anchor, so the whole scene turns 1.2 rad to meet it (39 steps).
    """
    problem, poses, _ = _scene("pinhole", seed=15)
    axis = np.array([0.666, -0.5, 0.5]) / np.linalg.norm([0.666, -0.5, 0.5])
    offset = np.zeros((CAMERAS, 6))
    offset[2] = [0.5, -0.3, 0.2, *(1.2 * axis)]
    weights = np.zeros((CAMERAS, 2))
    weights[2] = 1e4
    prior = se3.compose(poses, se3.exp(offset))
    return replace(problem, pose_prior=prior, pose_prior_weights=weights), {}


CASES = {
    "similarity: auto gauge with nothing anchored": _similarity,
    "first_pose: metric depth priors pin the scale": _first_pose,
    "scale: one full pose prior pins rigid motion; focal free": _scale,
    "none: pose priors on every camera": _none,
    "gauss-newton with an explicit similarity gauge": _gauss_newton,
    "degenerate: a point seen once, and one behind its cameras": _degenerate,
    "radial: f, k1, k2 free": _radial,
    "huber: five 20 px outliers": _huber,
    "every kind of block": _every_kind,
    "partial: rotation priors confine two poses": _rotation_priors,
    "partial: translation priors on two cameras": _translation_priors,
    "explicit first_pose": _explicit_first_pose,
    "explicit scale about a full prior": _explicit_scale,
    "explicit none with two fixed poses": _explicit_none,
    "a pose prior 1.2 rad from the truth": _far_prior,
}


def _options(spec: dict[str, Any]) -> ba.BAOptions:
    """Build a case's options: tight tolerances plus its overrides.

    The cost tolerance is 1e-10, not lower: the cores must stop on the same iteration, and
    at 1e-12 a cost creeping down by about that much per step lets last-bit rounding decide
    which step crosses it (Python 9 iterations, C++ 12, the same optimum).
    """
    solver = nls.Options(
        max_iterations=100,
        function_tolerance=1e-10,
        gradient_tolerance=1e-10,
        method=spec.get("method", "levenberg_marquardt"),
        loss=nls.Loss(*spec.get("loss", ["trivial", 1.0])),
    )
    return ba.BAOptions(
        solver=solver,
        depth_loss=nls.Loss(*spec.get("depth_loss", ["trivial", 1.0])),
        gauge=spec.get("gauge", "auto"),
    )


def options_from(spec: dict[str, Any]) -> ba.BAOptions:
    """Rebuild a case's options from its JSON input, which lists every field."""
    solver = dict(spec["solver"])
    solver["loss"] = nls.Loss(**solver["loss"])
    return ba.BAOptions(
        **{
            **spec,
            "solver": nls.Options(**solver),
            "depth_loss": nls.Loss(**spec["depth_loss"]),
        }
    )


def problem_from(spec: dict[str, Any]) -> ba.BAProblem:
    """Rebuild a case's problem from its JSON input (``null`` is NaN)."""
    fields = {
        name: value if name == "camera_model" else np.array(value, dtype=_dtype(name))
        for name, value in spec.items()
    }
    return ba.BAProblem(**fields)


def _dtype(name: str) -> type:
    """Return the dtype a problem field is stored with."""
    if name.startswith("fixed_"):
        return bool
    if name in {"cam_idx", "pt_idx", "intrinsics_idx", "point_anchor"}:
        return np.int64
    return float


def _json(value: Any) -> Any:  # noqa: ANN401 -- any JSON-serialisable value
    """Convert arrays and floats for JSON, writing NaN as null."""
    if isinstance(value, dict):
        return {key: _json(v) for key, v in value.items()}
    if isinstance(value, np.ndarray):
        return _json(value.tolist())
    if isinstance(value, list | tuple):
        return [_json(v) for v in value]
    if isinstance(value, float) and math.isnan(value):
        return None
    if isinstance(value, np.generic):
        return _json(value.item())
    return value


def _expected(result: ba.BAResult) -> dict[str, Any]:
    """Return the outputs a backend must reproduce."""
    summary = result.summary
    return {
        "poses": result.poses,
        "intrinsics": result.intrinsics,
        "points": result.points,
        "depth_affine": result.depth_affine,
        "inverse_depth": result.inverse_depth,
        "intrinsics_std": result.intrinsics_std,
        "initial_cost": summary.solve.initial_cost,
        "final_cost": summary.solve.final_cost,
        "variance_factor": summary.variance_factor,
        "codes": {
            "gauge": summary.gauge,
            "termination": str(summary.solve.termination),
            "iterations": summary.solve.iterations,
            **{name: getattr(summary, name) for name in ba.COUNT_NAMES},
        },
    }


def main() -> None:
    """Solve every case with the reference and write ``ba.json``."""
    cases = []
    for name, build in CASES.items():
        problem, spec = build()
        options = _options(spec)
        result = ba.solve(problem, options)
        if not result.summary.solve.converged:
            raise RuntimeError(f"{name}: the reference did not converge")
        cases.append(
            {
                "name": name,
                "input": {"problem": asdict(problem), "options": asdict(options)},
                "expected": _expected(result),
            }
        )
    doc = {
        "description": (
            "Bundle adjustment: parity fixture from the Python reference (generate_ba.py)."
        ),
        "oracle": "the Python reference; see the generator's docstring",
        "tolerance": TOLERANCES,
        "cases": _json(cases),
    }
    OUT.write_text(dumps(doc))
    print(f"wrote {OUT.name} ({len(cases)} cases)")


if __name__ == "__main__":
    main()
