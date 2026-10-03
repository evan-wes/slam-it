"""COLMAP's Ceres bundle adjuster as a reference for :mod:`slamlab.foundations.ba`.

Builds a pycolmap reconstruction from a :class:`~slamlab.foundations.ba.BAProblem`, runs
COLMAP's bundle adjustment (Ceres underneath), and reads the result back in this package's
conventions, so the two solvers can be compared on exactly the same problem.

pycolmap is an optional dependency (the ``colmap`` extra, which the development setup
installs), imported only when these functions run; the library itself never needs it. Only
problems that COLMAP can represent are supported: intrinsics groups (shared or per camera),
the ``"radial"`` model (COLMAP's ``RADIAL``) or ``"pinhole"``, no priors and no anchored
points.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slamlab.eval.readers import COLMAP_PIXEL_OFFSET
from slamlab.foundations import camera, nls, se3
from slamlab.foundations.arrays import group_members
from slamlab.foundations.ba import BAProblem
from slamlab.foundations.errors import InvalidInputError

COLMAP_MODELS = {"radial": "RADIAL", "pinhole": "PINHOLE"}
"""This package's camera models COLMAP can represent, and their COLMAP names."""


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
    """Import pycolmap (the ``colmap`` extra), explaining how to get it if it is missing."""
    try:
        import pycolmap  # noqa: PLC0415 -- optional: only the reference needs it
    except ImportError as error:  # pragma: no cover - the dev environment has it
        raise ImportError(
            "the COLMAP reference needs pycolmap: pip install slamlab[colmap]"
        ) from error
    return pycolmap


def _check(problem: BAProblem) -> None:
    """Reject a problem COLMAP cannot represent as-is, rather than solving a different one.

    COLMAP's bundle adjuster has no priors, anchored points or observation weights here, and
    this adapter does not pass fixed poses, fixed points or per-parameter intrinsics masks
    beyond "principal point fixed". A problem using any of them is rejected.
    """
    if problem.camera_model not in COLMAP_MODELS:
        raise InvalidInputError(f"COLMAP reference: model {problem.camera_model!r} unsupported")
    unsupported = {
        "anchored points": np.any(problem.point_anchor >= 0),
        "depth priors": not np.all(np.isnan(problem.depth_prior)),
        "pose priors": np.any(problem.pose_prior_weights > 0.0),
        "observation weights other than 1": not np.all(problem.weights == 1.0),
        "fixed poses": np.any(problem.fixed_poses),
        "fixed points": np.any(problem.fixed_points),
    }
    named = [name for name, used in unsupported.items() if used]
    if named:
        raise InvalidInputError(f"COLMAP reference: unsupported here: {', '.join(named)}")


def refinement(problem: BAProblem) -> tuple[bool, bool]:
    """Return what COLMAP should refine to solve the problem's own intrinsics masks.

    COLMAP refines all focal lengths or none, all distortion terms or none, and here never
    the principal point. A problem whose masks say anything else is rejected.

    Parameters
    ----------
    problem : BAProblem
        The problem.

    Returns
    -------
    refine_focal, refine_extra : bool
        Whether the focal lengths, and the distortion terms, are free.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the principal point is free, or focal or distortion entries are free in some
        places and fixed in others.
    """
    model = problem.camera_model
    free = ~problem.fixed_intrinsics
    focal = list(camera.FOCAL_INDICES[model])
    centre = list(camera.CENTRE_INDICES[model])
    extra = [k for k in range(free.shape[1]) if k not in focal + centre]
    if np.any(free[:, centre]):
        raise InvalidInputError("COLMAP reference: the principal point must be fixed")
    for name, columns in (("focal", focal), ("distortion", extra)):
        if columns and np.any(free[:, columns]) and not np.all(free[:, columns]):
            raise InvalidInputError(
                f"COLMAP reference: {name} entries must be all free or all fixed"
            )
    return bool(np.all(free[:, focal])), bool(extra) and bool(np.all(free[:, extra]))


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
    centre = list(camera.CENTRE_INDICES[problem.camera_model])
    for g, group in enumerate(problem.intrinsics):
        params = group.copy()
        params[centre] += COLMAP_PIXEL_OFFSET
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
    by_camera = group_members(problem.cam_idx, problem.poses.shape[0])
    slot = np.zeros(problem.cam_idx.size, dtype=np.int64)  # each observation's keypoint index
    for c, sel in enumerate(by_camera):
        slot[sel] = np.arange(sel.size)
        image = pycolmap.Image(
            name=str(c),
            keypoints=problem.observations[sel] + COLMAP_PIXEL_OFFSET,
            camera_id=int(problem.intrinsics_idx[c]) + 1,
            image_id=c + 1,
        )
        reconstruction.add_image_with_trivial_frame(image, pycolmap.Rigid3d(t_cam_world[c, :3, :]))
    point_ids = []
    for p, views in enumerate(group_members(problem.pt_idx, problem.points.shape[0])):
        track = [pycolmap.TrackElement(int(problem.cam_idx[m]) + 1, int(slot[m])) for m in views]
        point_ids.append(int(reconstruction.add_point3D(problem.points[p], pycolmap.Track(track))))
    return reconstruction, point_ids


@dataclass(frozen=True)
class ColmapOptions:
    """Settings for COLMAP's bundle adjustment.

    What COLMAP refines is not an option: it follows the problem's ``fixed_intrinsics``
    (see :func:`refinement`), so both solvers solve the same problem.

    Attributes
    ----------
    loss : slamlab.foundations.nls.Loss
        The robust loss per observation. Ceres's Huber and Cauchy losses on the squared norm
        are the same as this package's.
    tolerance : float
        Ceres's function, gradient and parameter tolerances.
    max_iterations : int
        Ceres's iteration cap.
    """

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
        The loss and Ceres's tolerances (default: ``ColmapOptions()``).

    Returns
    -------
    ColmapResult
        The refined variables.
    """
    pycolmap = _pycolmap()
    reconstruction, point_ids = to_reconstruction(problem)
    settings = options if options is not None else ColmapOptions()
    colmap = pycolmap.BundleAdjustmentOptions()
    colmap.refine_principal_point = False
    colmap.refine_focal_length, colmap.refine_extra_params = refinement(problem)
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
    intrinsics[:, list(camera.CENTRE_INDICES[problem.camera_model])] -= COLMAP_PIXEL_OFFSET
    t_cam_world = np.tile(np.eye(4), (n_cams, 1, 1))
    for c in range(n_cams):
        t_cam_world[c, :3, :] = reconstruction.images[c + 1].cam_from_world().matrix()
    # COLMAP drops points it filters (e.g. short tracks); those keep their starting position.
    points = problem.points.copy()
    for p, point_id in enumerate(point_ids):
        if reconstruction.exists_point3D(point_id):
            points[p] = reconstruction.points3D[point_id].xyz
    return ColmapResult(se3.inverse(t_cam_world), intrinsics, points, seconds)
