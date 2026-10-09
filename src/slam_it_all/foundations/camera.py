"""Camera models: projection, its Jacobians, and unprojection to bearing vectors.

Conventions (design doc, "Conventions"): points are in the camera frame (x right, y down,
z forward); pixels follow OpenCV, with the centre of the top-left pixel at ``(0, 0)``;
intrinsics are flat vectors per model:

- ``"pinhole"``: ``[fx, fy, cx, cy]``;
- ``"radtan"``: ``[fx, fy, cx, cy, k1, k2, p1, p2, k3]`` -- Brown-Conrady radial (k1, k2, k3)
  and tangential (p1, p2) distortion, as in OpenCV, TUM RGB-D and EuRoC;
- ``"radial"``: ``[f, cx, cy, k1, k2]`` -- one focal length and two radial terms, COLMAP's
  ``RADIAL`` and the Bundle Adjustment in the Large model. It is radtan with ``fx = fy = f``
  and ``p1 = p2 = k3 = 0``, so it is computed through radtan by the fixed linear map
  :data:`RADIAL_TO_RADTAN`, and its intrinsics Jacobian is radtan's times that map.

Projection: normalise ``(x, y) = (X/Z, Y/Z)``; distort (radtan); then
``u = fx x_d + cx``, ``v = fy y_d + cy``. Points must lie in front of the camera
(``Z > MIN_DEPTH``).

Unprojection returns unit **bearing vectors**, the input every calibrated solver takes
(so wide-angle models can join later without changing them). Undistortion inverts the radtan
map by Newton's method with an analytic 2x2 Jacobian; a pixel outside the region where the
distortion is invertible, or where Newton fails to converge, raises
:class:`slam_it_all.foundations.errors.NotConvergedError` rather than returning a wrong ray.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it_all import backends
from slam_it_all.foundations.arrays import as_batch, as_real
from slam_it_all.foundations.errors import InvalidInputError, NotConvergedError

MODELS: dict[str, tuple[int, int]] = {"pinhole": (0, 4), "radtan": (1, 9), "radial": (2, 5)}
"""Each model's integer code (as it crosses the bindings) and intrinsics length."""

FOCAL_INDICES: dict[str, tuple[int, ...]] = {"pinhole": (0, 1), "radtan": (0, 1), "radial": (0,)}
"""Each model's focal-length entries in its intrinsics vector."""

CENTRE_INDICES: dict[str, tuple[int, ...]] = {"pinhole": (2, 3), "radtan": (2, 3), "radial": (1, 2)}
"""Each model's principal-point entries ``(cx, cy)`` in its intrinsics vector."""

RADIAL_TO_RADTAN = np.zeros((9, 5))
"""The linear map from radial ``[f, cx, cy, k1, k2]`` to radtan
``[f, f, cx, cy, k1, k2, 0, 0, 0]``."""
RADIAL_TO_RADTAN[[0, 1, 2, 3, 4, 5], [0, 0, 1, 2, 3, 4]] = 1.0

_RADTAN = 1
_RADIAL = 2

MIN_DEPTH = 1e-9
"""Points with ``Z`` at or below this are behind (or at) the camera and cannot project."""

UNDISTORT_MAX_ITERATIONS = 30
"""Newton iterations allowed when inverting the radtan distortion."""

UNDISTORT_TOLERANCE = 1e-15
"""Newton stops when the distortion residual is below this (normalised units)."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.camera`` submodule provides.

    ``model`` is the integer code from :data:`MODELS`; arrays are float64 C-contiguous.
    """

    def project(
        self, model: int, intrinsics: NDArray[np.float64], points: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Project (N, 3) camera-frame points to (N, 2) pixels."""
        ...

    def project_jacobians(
        self, model: int, intrinsics: NDArray[np.float64], points: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Return (N, 2, 3 + K): d(pixel)/d(point) then d(pixel)/d(intrinsics)."""
        ...

    def unproject(
        self, model: int, intrinsics: NDArray[np.float64], pixels: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Map (N, 2) pixels to (N, 3) unit bearing vectors."""
        ...


# --- validation ---------------------------------------------------------------------------


def as_model(model: str, intrinsics: ArrayLike) -> tuple[int, NDArray[np.float64]]:
    """Check a model name and its intrinsics vector.

    Parameters
    ----------
    model : str
        A model from :data:`MODELS`.
    intrinsics : array_like
        The intrinsics vector.

    Returns
    -------
    code : int
        The model's code, as it crosses to a compiled backend.
    intrinsics : ndarray of float64
        A checked copy of the intrinsics.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the model is unknown, the vector has the wrong length or is not finite, or a
        focal length is not positive.
    """
    if model not in MODELS:
        raise InvalidInputError(f"model: unknown camera model {model!r}; expected {sorted(MODELS)}")
    code, length = MODELS[model]
    params = as_real(intrinsics, "intrinsics")
    if params.shape != (length,):
        raise InvalidInputError(
            f"intrinsics: {model} expects shape ({length},), got {params.shape}"
        )
    if np.any(params[list(FOCAL_INDICES[model])] <= 0.0):
        raise InvalidInputError("intrinsics: focal lengths must be positive")
    return code, params


def check_intrinsics(model: str, intrinsics: ArrayLike) -> None:
    """Raise unless ``intrinsics`` is a valid vector for ``model``.

    Parameters
    ----------
    model : str
        A model from :data:`MODELS`.
    intrinsics : array_like
        The intrinsics vector.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the model is unknown, the vector has the wrong length or is not finite, or a
        focal length is not positive.
    """
    as_model(model, intrinsics)


def _points_in_front(points: ArrayLike) -> tuple[NDArray[np.float64], tuple[int, ...]]:
    """Validate points in the camera frame, all in front of the camera."""
    flat, lead = as_batch(points, (3,), "points")
    if np.any(flat[:, 2] <= MIN_DEPTH):
        raise InvalidInputError(f"points: Z must exceed {MIN_DEPTH} (in front of the camera)")
    return flat, lead


# --- reference implementations (flat batches) ---------------------------------------------


def _radial_terms(
    params: NDArray[np.float64], xy: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Return ``x``, ``y``, ``r²`` and the radial factor ``1 + k1 r² + k2 r⁴ + k3 r⁶``."""
    x, y = xy[:, 0], xy[:, 1]
    r2 = x * x + y * y
    k1, k2, k3 = params[4], params[5], params[8]
    return x, y, r2, 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))


def _distort(params: NDArray[np.float64], xy: NDArray[np.float64]) -> NDArray[np.float64]:
    """Apply radtan distortion to (N, 2) normalised coordinates."""
    p1, p2 = params[6], params[7]
    x, y, r2, radial = _radial_terms(params, xy)
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return np.stack([xd, yd], axis=-1)


def _distortion_jacobian(
    params: NDArray[np.float64], xy: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Compute d(distorted)/d(normalised), shape (N, 2, 2)."""
    k1, k2, p1, p2, k3 = params[4:9]
    x, y, r2, radial = _radial_terms(params, xy)
    d_radial = k1 + r2 * (2.0 * k2 + 3.0 * k3 * r2)  # d(radial)/d(r2)
    cross = 2.0 * x * y * d_radial + 2.0 * p1 * x + 2.0 * p2 * y
    j = np.empty((xy.shape[0], 2, 2))
    j[:, 0, 0] = radial + 2.0 * x * x * d_radial + 2.0 * p1 * y + 6.0 * p2 * x
    j[:, 0, 1] = cross
    j[:, 1, 0] = cross
    j[:, 1, 1] = radial + 2.0 * y * y * d_radial + 6.0 * p1 * y + 2.0 * p2 * x
    return j


def _project(
    model: int, params: NDArray[np.float64], points: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Project (N, 3) points (reference)."""
    if model == _RADIAL:
        return _project(_RADTAN, RADIAL_TO_RADTAN @ params, points)
    xy = points[:, :2] / points[:, 2:3]
    if model == MODELS["radtan"][0]:
        xy = _distort(params, xy)
    return np.stack([params[0] * xy[:, 0] + params[2], params[1] * xy[:, 1] + params[3]], axis=-1)


def _project_jacobians(
    model: int, params: NDArray[np.float64], points: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Compute d(pixel)/d(point) and d(pixel)/d(intrinsics), shape (N, 2, 3 + K) (reference)."""
    if model == _RADIAL:
        full = _project_jacobians(_RADTAN, RADIAL_TO_RADTAN @ params, points)
        return np.concatenate([full[:, :, :3], full[:, :, 3:] @ RADIAL_TO_RADTAN], axis=2)
    n = points.shape[0]
    z = points[:, 2]
    xy = points[:, :2] / z[:, None]
    d_xy_d_point = np.zeros((n, 2, 3))
    d_xy_d_point[:, 0, 0] = 1.0 / z
    d_xy_d_point[:, 1, 1] = 1.0 / z
    d_xy_d_point[:, 0, 2] = -xy[:, 0] / z
    d_xy_d_point[:, 1, 2] = -xy[:, 1] / z
    focal = np.array([[params[0], 0.0], [0.0, params[1]]])
    radtan = model == MODELS["radtan"][0]
    xd = _distort(params, xy) if radtan else xy
    d_xd_d_xy = (
        _distortion_jacobian(params, xy) if radtan else np.broadcast_to(np.eye(2), (n, 2, 2))
    )
    out = np.zeros((n, 2, 3 + params.shape[0]))
    out[:, :, :3] = focal @ d_xd_d_xy @ d_xy_d_point
    out[:, 0, 3] = xd[:, 0]
    out[:, 1, 4] = xd[:, 1]
    out[:, 0, 5] = 1.0
    out[:, 1, 6] = 1.0
    if radtan:
        x, y = xy[:, 0], xy[:, 1]
        r2 = x * x + y * y
        # d(xd, yd)/d(k1, k2, p1, p2, k3), scaled by the focal lengths.
        d_dist = np.stack(
            [
                np.stack([x * r2, x * r2 * r2, 2.0 * x * y, r2 + 2.0 * x * x, x * r2**3], -1),
                np.stack([y * r2, y * r2 * r2, r2 + 2.0 * y * y, 2.0 * x * y, y * r2**3], -1),
            ],
            axis=1,
        )
        out[:, :, 7:] = focal @ d_dist
    return out


def _undistort(params: NDArray[np.float64], xy_d: NDArray[np.float64]) -> NDArray[np.float64]:
    """Invert radtan distortion by Newton's method; raise if any point fails to converge."""
    xy = xy_d.copy()
    for _ in range(UNDISTORT_MAX_ITERATIONS):
        residual = _distort(params, xy) - xy_d
        if np.all(np.abs(residual) <= UNDISTORT_TOLERANCE * np.maximum(1.0, np.abs(xy_d))):
            return xy
        jac = _distortion_jacobian(params, xy)
        if np.any(np.linalg.det(jac) <= 0.0):
            raise NotConvergedError(
                "undistort: pixel outside the invertible region of the distortion"
            )
        xy = xy - np.linalg.solve(jac, residual[..., None])[..., 0]
    residual = _distort(params, xy) - xy_d
    if np.all(np.abs(residual) <= UNDISTORT_TOLERANCE * np.maximum(1.0, np.abs(xy_d))):
        return xy
    raise NotConvergedError(
        f"undistort: Newton did not converge in {UNDISTORT_MAX_ITERATIONS} iterations"
    )


def _unproject(
    model: int, params: NDArray[np.float64], pixels: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Map (N, 2) pixels to unit bearing vectors (reference)."""
    if model == _RADIAL:
        return _unproject(_RADTAN, RADIAL_TO_RADTAN @ params, pixels)
    xy = np.stack(
        [(pixels[:, 0] - params[2]) / params[0], (pixels[:, 1] - params[3]) / params[1]], -1
    )
    if model == MODELS["radtan"][0]:
        xy = _undistort(params, xy)
    rays = np.concatenate([xy, np.ones((xy.shape[0], 1))], axis=-1)
    return np.asarray(rays / np.linalg.norm(rays, axis=-1, keepdims=True), dtype=np.float64)


# --- public, batched API -------------------------------------------------------------------


def project(
    model: str, intrinsics: ArrayLike, points: ArrayLike, *, backend: str = "python"
) -> NDArray[np.float64]:
    """Project camera-frame points to pixels.

    Parameters
    ----------
    model : {"pinhole", "radtan", "radial"}
        The camera model.
    intrinsics : array_like, shape (K,)
        The model's intrinsics (see the module docstring for the layout).
    points : array_like, shape (..., 3)
        Points in the camera frame, all with ``Z > MIN_DEPTH``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 2)
        Pixels ``(u, v)``.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the model is unknown, the intrinsics have the wrong length or non-positive focal
        lengths, or a point is not in front of the camera.
    """
    code, params = as_model(model, intrinsics)
    flat, lead = _points_in_front(points)
    result = backends.dispatch(backend, __name__, "project", _project, code, params, flat)
    return np.asarray(result.reshape((*lead, 2)), dtype=np.float64)


def project_jacobians(
    model: str, intrinsics: ArrayLike, points: ArrayLike, *, backend: str = "python"
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Differentiate the projection with respect to the point and to the intrinsics.

    Parameters
    ----------
    model : {"pinhole", "radtan", "radial"}
        The camera model.
    intrinsics : array_like, shape (K,)
        The model's intrinsics.
    points : array_like, shape (..., 3)
        Points in the camera frame, all with ``Z > MIN_DEPTH``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    d_point : ndarray of float64, shape (..., 2, 3)
        ``d(u, v) / d(X, Y, Z)``.
    d_intrinsics : ndarray of float64, shape (..., 2, K)
        ``d(u, v) / d(intrinsics)``, in the intrinsics vector's order.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        As :func:`project`.
    """
    code, params = as_model(model, intrinsics)
    flat, lead = _points_in_front(points)
    both = backends.dispatch(
        backend, __name__, "project_jacobians", _project_jacobians, code, params, flat
    )
    both = both.reshape((*lead, 2, 3 + params.shape[0]))
    return both[..., :3].copy(), both[..., 3:].copy()


def unproject(
    model: str, intrinsics: ArrayLike, pixels: ArrayLike, *, backend: str = "python"
) -> NDArray[np.float64]:
    """Map pixels to unit bearing vectors in the camera frame.

    Parameters
    ----------
    model : {"pinhole", "radtan", "radial"}
        The camera model.
    intrinsics : array_like, shape (K,)
        The model's intrinsics.
    pixels : array_like, shape (..., 2)
        Pixels ``(u, v)``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3)
        Unit vectors along each pixel's ray (``Z > 0``).

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the model or intrinsics are invalid, or `pixels` is not (..., 2).
    slam_it_all.foundations.errors.NotConvergedError
        If a pixel lies outside the region where the distortion can be inverted.
    """
    code, params = as_model(model, intrinsics)
    flat, lead = as_batch(pixels, (2,), "pixels")
    result = backends.dispatch(backend, __name__, "unproject", _unproject, code, params, flat)
    return np.asarray(result.reshape((*lead, 3)), dtype=np.float64)


def unproject_or_nan(
    model: str, intrinsics: ArrayLike, pixels: ArrayLike, *, backend: str = "python"
) -> NDArray[np.float64]:
    """Map pixels to unit bearings as :func:`unproject` does, with NaN rows where it cannot.

    A pixel beyond the region where the distortion can be inverted has no ray. Where
    :func:`unproject` fails the whole batch, this answers per pixel, as the C++ core's
    per-item result does: callers that must go on (an anchor whose ray is lost, a pixel to
    skip) get NaN for those pixels alone.

    Parameters
    ----------
    model : {"pinhole", "radtan", "radial"}
        The camera model.
    intrinsics : array_like, shape (K,)
        The model's intrinsics.
    pixels : array_like, shape (N, 2)
        Pixels ``(u, v)``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (N, 3)
        Unit vectors along each pixel's ray, or NaN rows.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the model or intrinsics are invalid, or `pixels` is not (N, 2).
    """
    flat = as_real(pixels, "pixels")
    if flat.ndim != 2 or flat.shape[1] != 2:
        raise InvalidInputError(f"pixels: expected shape (N, 2), got {flat.shape}")
    try:
        return unproject(model, intrinsics, flat, backend=backend)
    except NotConvergedError:
        out = np.full((flat.shape[0], 3), np.nan)
        for i, pixel in enumerate(flat):
            try:
                out[i] = unproject(model, intrinsics, pixel, backend=backend)
            except NotConvergedError:
                continue
        return out
