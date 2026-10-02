"""Reprojection error of a bundle-adjustment problem's variables, for comparing solvers.

Two solvers' results are compared on one cost: each result is written back into the
problem and measured here with this package's camera model, over the same observations.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from slamlab.foundations import camera, se3
from slamlab.foundations.ba import BAProblem


def in_front(problem: BAProblem) -> NDArray[np.bool_]:
    """Return which observations' points lie in front of their camera.

    Parameters
    ----------
    problem : BAProblem
        The problem, at its current variables.

    Returns
    -------
    ndarray of bool, shape (M,)
        True where the observation's point has ``Z > MIN_DEPTH`` in its camera.
    """
    t_cw = se3.inverse(problem.poses)
    cam, pt = problem.cam_idx, problem.pt_idx
    z = np.einsum("mj,mj->m", t_cw[cam, 2, :3], problem.points[pt]) + t_cw[cam, 2, 3]
    return np.asarray(z > camera.MIN_DEPTH)


def rms(problem: BAProblem, used: NDArray[np.bool_]) -> float:
    """Return the RMS reprojection error (px) over the ``used`` observations.

    Parameters
    ----------
    problem : BAProblem
        The problem, at the variables to measure.
    used : ndarray of bool, shape (M,)
        The observations to include (all must be in front of their camera).

    Returns
    -------
    float
        ``sqrt(mean |π(X) - u|²)`` over the used observations.
    """
    t_cw = se3.inverse(problem.poses)
    cam, pt = problem.cam_idx[used], problem.pt_idx[used]
    in_cam = np.einsum("mij,mj->mi", t_cw[cam, :3, :3], problem.points[pt]) + t_cw[cam, :3, 3]
    pixels = np.zeros((cam.size, 2))
    for c in np.unique(cam):
        sel = cam == c
        k = problem.intrinsics[problem.intrinsics_idx[c]]
        pixels[sel] = camera.project(problem.camera_model, k, in_cam[sel])
    errors = pixels - problem.observations[used]
    return float(np.sqrt(np.mean(np.sum(errors**2, axis=1))))
