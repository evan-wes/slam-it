"""COLMAP's Ceres bundle adjuster as a reference for :mod:`slamlab.foundations.ba`.

Builds a pycolmap reconstruction from a :class:`~slamlab.foundations.ba.BAProblem`, runs
COLMAP's bundle adjustment (Ceres underneath), and reads the result back in this package's
conventions, so the two solvers can be compared on exactly the same problem.

pycolmap is a development dependency, imported only when these functions run; the library
itself never needs it. Only problems that COLMAP can represent are supported: intrinsics groups
(shared or per camera), the ``"radial"`` model (COLMAP's ``RADIAL``) or ``"pinhole"``, no priors
and no anchored points.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slamlab.foundations import nls, se3
from slamlab.foundations.ba import BAProblem
from slamlab.foundations.errors import InvalidInputError

COLMAP_MODELS = {"radial": "RADIAL", "pinhole": "PINHOLE"}
"""This package's camera models COLMAP can represent, and their COLMAP names."""

PIXEL_OFFSET = 0.5
"""COLMAP's pixel-centre coordinate minus this package's (OpenCV's)."""

_CENTRE = {"radial": slice(1, 3), "pinhole": slice(2, 4)}
"""Each model's principal-point entries."""


@dataclass(frozen=True)
class ColmapResult:
    """COLMAP's refined variables, in this package's conventions.

    Attributes
    ----------
    poses : ndarray of float64, shape (C, 4, 4)
        Camera-to-world poses.
    intrinsics : ndarray of float64, shape (K, P)
        Refined intrinsics groups.
    points : ndarray of float64, shape (N, 3)
        World points (those COLMAP filtered out keep their starting position).
    seconds : float
        Wall-clock time of COLMAP's solve.
    """

    poses: NDArray[np.float64]
    intrinsics: NDArray[np.float64]
    points: NDArray[np.float64]
    seconds: float


def _pycolmap() -> Any:  # noqa: ANN401 -- the optional module, untyped
    """Import pycolmap, explaining how to get it if it is missing."""
    try:
        return importlib.import_module("pycolmap")
    except ImportError as error:  # pragma: no cover - the dev environment has it
        raise ImportError(
            "pycolmap is needed for the COLMAP reference: pip install pycolmap"
        ) from error


def _check(problem: BAProblem) -> None:
    """Reject a problem COLMAP cannot represent as-is."""
    if problem.camera_model not in COLMAP_MODELS:
        raise InvalidInputError(f"COLMAP reference: model {problem.camera_model!r} unsupported")
    if np.any(problem.point_anchor >= 0) or not np.all(np.isnan(problem.depth_prior)):
        raise InvalidInputError("COLMAP reference: anchored points and depth priors unsupported")


def to_reconstruction(problem: BAProblem) -> tuple[Any, list[int]]:
    """Build a pycolmap reconstruction of a problem (COLMAP's pixel centres).

    Parameters
    ----------
    problem : BAProblem
        A problem COLMAP can represent (see the module docstring).

    Returns
    -------
    reconstruction : pycolmap.Reconstruction
        Intrinsics group g is COLMAP camera ``g + 1``; pose i is image ``i + 1``.
    point_ids : list of int
        COLMAP's id for each of the problem's points.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the problem uses something COLMAP cannot represent.
    """
    _check(problem)
    pycolmap = _pycolmap()
    reconstruction = pycolmap.Reconstruction()
    t_cam_world = se3.inverse(problem.poses)
    centre = _CENTRE[problem.camera_model]
    for g, group in enumerate(problem.intrinsics):
        params = group.copy()
        params[centre] += PIXEL_OFFSET
        width, height = (2.0 * params[centre]).round().astype(int).clip(1)
        reconstruction.add_camera_with_trivial_rig(
            pycolmap.Camera(
                model=COLMAP_MODELS[problem.camera_model],
                width=int(width),
                height=int(height),
                params=params,
                camera_id=g + 1,
            )
        )
    slots = []
    for c in range(problem.poses.shape[0]):
        sel = np.flatnonzero(problem.cam_idx == c)
        slots.append({int(m): k for k, m in enumerate(sel)})
        image = pycolmap.Image(
            name=str(c),
            keypoints=problem.observations[sel] + PIXEL_OFFSET,
            camera_id=int(problem.intrinsics_idx[c]) + 1,
            image_id=c + 1,
        )
        reconstruction.add_image_with_trivial_frame(image, pycolmap.Rigid3d(t_cam_world[c, :3, :]))
    point_ids = []
    for p in range(problem.points.shape[0]):
        track = [
            pycolmap.TrackElement(int(problem.cam_idx[m]) + 1, slots[problem.cam_idx[m]][int(m)])
            for m in np.flatnonzero(problem.pt_idx == p)
        ]
        point_ids.append(int(reconstruction.add_point3D(problem.points[p], pycolmap.Track(track))))
    return reconstruction, point_ids


@dataclass(frozen=True)
class ColmapOptions:
    """Settings for COLMAP's bundle adjustment.

    Attributes
    ----------
    refine_focal, refine_extra : bool
        Refine the focal lengths and the distortion terms (the principal point stays fixed).
    loss : slamlab.foundations.nls.Loss
        The robust loss per observation. Ceres's Huber and Cauchy losses on the squared norm
        are the same as this package's.
    tolerance : float
        Ceres's function, gradient and parameter tolerances.
    max_iterations : int
        Ceres's iteration cap.
    """

    refine_focal: bool = True
    refine_extra: bool = True
    loss: nls.Loss = field(default_factory=nls.Loss)
    tolerance: float = 1e-10
    max_iterations: int = 200


def solve(problem: BAProblem, options: ColmapOptions | None = None) -> ColmapResult:
    """Run COLMAP's bundle adjustment on a problem.

    Parameters
    ----------
    problem : BAProblem
        A problem COLMAP can represent (see the module docstring).
    options : ColmapOptions, optional
        What to refine, the loss and Ceres's tolerances (default: ``ColmapOptions()``).

    Returns
    -------
    ColmapResult
        The refined variables.
    """
    import time  # noqa: PLC0415 -- only the reference's timing needs it

    pycolmap = _pycolmap()
    reconstruction, point_ids = to_reconstruction(problem)
    settings = options if options is not None else ColmapOptions()
    colmap = pycolmap.BundleAdjustmentOptions()
    colmap.refine_principal_point = False
    colmap.refine_focal_length = settings.refine_focal
    colmap.refine_extra_params = settings.refine_extra
    colmap.print_summary = False
    colmap.ceres.loss_function_type = getattr(pycolmap.LossFunctionType, settings.loss.kind.upper())
    colmap.ceres.loss_function_scale = settings.loss.scale
    solver = colmap.ceres.solver_options
    solver.function_tolerance = settings.tolerance
    solver.gradient_tolerance = settings.tolerance
    solver.parameter_tolerance = settings.tolerance
    solver.max_num_iterations = settings.max_iterations
    start = time.perf_counter()
    pycolmap.bundle_adjustment(reconstruction, colmap)
    seconds = time.perf_counter() - start
    n_cams = problem.poses.shape[0]
    n_groups = problem.intrinsics.shape[0]
    intrinsics = np.array([reconstruction.cameras[g + 1].params for g in range(n_groups)])
    intrinsics[:, _CENTRE[problem.camera_model]] -= PIXEL_OFFSET
    t_cam_world = np.tile(np.eye(4), (n_cams, 1, 1))
    for c in range(n_cams):
        t_cam_world[c, :3, :] = reconstruction.images[c + 1].cam_from_world().matrix()
    # COLMAP drops points it filters (e.g. short tracks); those keep their starting position.
    points = problem.points.copy()
    for p, point_id in enumerate(point_ids):
        if reconstruction.exists_point3D(point_id):
            points[p] = reconstruction.points3D[point_id].xyz
    return ColmapResult(se3.inverse(t_cam_world), intrinsics, points, seconds)
