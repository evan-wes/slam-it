"""Rectified stereo: Bouguet's rectification of a calibrated pair.

**Rectification** turns both cameras about their centres so that their image rows become
epipolar lines: a point at row ``v`` in the left image lies on row ``v`` in the right one,
and its depth follows from the column difference alone. It follows Bouguet's algorithm, as
OpenCV's ``stereoRectify`` does:

1. Split the relative rotation in half: each camera turns half the way toward the other, so
   both share an orientation and the distortion each image suffers stays small.
2. Turn both about the axis that brings their baseline onto the x axis.
3. Give both one pinhole camera: focal length the smallest of the four input focal lengths
   (so the rectified views keep the narrower field), principal point at the image centre
   ``((cols - 1) / 2, (rows - 1) / 2)``. (OpenCV averages the focal lengths and moves the
   principal point to keep the valid region; the centre is simpler and enough here.)

With ``T_right_left`` mapping left-camera to right-camera coordinates
(``p_right = R p_left + t``), the rectified right camera sits at ``(baseline, 0, 0)`` in the
rectified left frame, so a point at depth ``Z`` appears ``f baseline / Z`` columns further
left in the right image.

:func:`rectification_maps` gives, for each rectified pixel, where to sample the original
image (through each camera's own model, distortion included), for
:func:`slamlab.foundations.image.remap`. The maps are float64 and agree across the three
languages to rounding (the rotations come from SO(3)'s exponential and logarithm); the
remap's rounding to a 32nd of a pixel makes the rectified images agree exactly but for a map
value within rounding of a 64th of a pixel, which is vanishingly rare.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import camera, se3, so3
from slamlab.foundations.arrays import MAX_IMAGE_SIDE, as_batch, as_real
from slamlab.foundations.errors import DegenerateError, InvalidInputError

MIN_BASELINE = 1e-9
"""Camera centres closer than this (in the translation's units) count as coinciding."""

OUTSIDE = -2.0
"""Where a rectification map sends a pixel whose ray points behind the original camera: a
coordinate outside every image, which :func:`slamlab.foundations.image.remap` reads as 0."""


@dataclass(frozen=True)
class Rectification:
    """A rectified stereo pair.

    Attributes
    ----------
    rotation_left : ndarray of float64, shape (3, 3)
        ``R_rect_left``: turns left-camera coordinates into the rectified frame.
    rotation_right : ndarray of float64, shape (3, 3)
        ``R_rect_right``: turns right-camera coordinates into the rectified right frame.
    intrinsics : ndarray of float64, shape (4,)
        Both rectified cameras' pinhole intrinsics ``[f, f, cx, cy]``.
    baseline : float
        The distance between the camera centres, in the translation's units.
    shape : tuple of int
        The images' ``(rows, cols)``, original and rectified alike.
    left_model, right_model : str
        The original cameras' models.
    left_intrinsics, right_intrinsics : ndarray of float64
        The original cameras' intrinsics.
    """

    rotation_left: NDArray[np.float64]
    rotation_right: NDArray[np.float64]
    intrinsics: NDArray[np.float64]
    baseline: float
    shape: tuple[int, int]
    left_model: str
    left_intrinsics: NDArray[np.float64]
    right_model: str
    right_intrinsics: NDArray[np.float64]


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.stereo`` submodule provides.

    Camera models cross as their :data:`slamlab.foundations.camera.MODELS` codes.
    """

    def rectify(  # noqa: PLR0913 -- two cameras, their pose, the image shape
        self,
        *,
        model_left: int,
        intrinsics_left: NDArray[np.float64],
        model_right: int,
        intrinsics_right: NDArray[np.float64],
        T_right_left: NDArray[np.float64],  # noqa: N803 -- the frame convention's name
        rows: int,
        cols: int,
    ) -> dict[str, Any]:
        """Rectify a pair; return the rotations, the rectified intrinsics and the baseline."""
        ...

    def rectification_map(  # noqa: PLR0913 -- one camera, its rectification, the shape
        self,
        *,
        model: int,
        intrinsics: NDArray[np.float64],
        rotation: NDArray[np.float64],
        rectified: NDArray[np.float64],
        rows: int,
        cols: int,
    ) -> NDArray[np.float64]:
        """Return the (rows, cols, 2) map from rectified pixels to the original image."""
        ...


def _rectify(
    intrinsics: tuple[NDArray[np.float64], NDArray[np.float64]],
    focal_indices: tuple[tuple[int, ...], tuple[int, ...]],
    transform: NDArray[np.float64],
    shape: tuple[int, int],
) -> dict[str, Any]:
    """Bouguet's rectification (reference)."""
    rotation, translation = transform[:3, :3], transform[:3, 3]
    baseline = math.sqrt(float(translation @ translation))
    if baseline <= MIN_BASELINE:
        raise DegenerateError("T_right_left: the camera centres coincide")
    half = so3.exp(-0.5 * so3.log(rotation))  # each camera turns half the way
    t = half @ translation
    if abs(t[0]) <= abs(t[1]):
        raise InvalidInputError(
            "T_right_left: the baseline is closer to vertical; only side-by-side pairs rectify"
        )
    target = np.array([math.copysign(1.0, t[0]), 0.0, 0.0])
    axis = np.cross(t, target)
    norm = math.sqrt(float(axis @ axis))
    norm_t = math.sqrt(float(t @ t))
    if norm > 0.0:
        axis = axis * (math.acos(min(abs(float(t[0])) / norm_t, 1.0)) / norm)
    onto_x = so3.exp(axis)
    focal = min(
        float(params[i])
        for params, indices in zip(intrinsics, focal_indices, strict=True)
        for i in indices
    )
    rows, cols = shape
    return {
        "rotation_left": onto_x @ half.T,
        "rotation_right": onto_x @ half,
        "intrinsics": np.array([focal, focal, (cols - 1) / 2.0, (rows - 1) / 2.0]),
        "baseline": baseline,
    }


def _rectification_map(
    model: str,
    intrinsics: NDArray[np.float64],
    rotation: NDArray[np.float64],
    rectified: NDArray[np.float64],
    shape: tuple[int, int],
) -> NDArray[np.float64]:
    """Each rectified pixel's source in the original image (reference)."""
    rows, cols = shape
    f, _, cx, cy = rectified
    v, u = np.mgrid[0:rows, 0:cols].astype(np.float64)
    x, y = (u - cx) / f, (v - cy) / f
    r = rotation
    # The ray in the original camera, R^T (x, y, 1), one product and sum at a time (no BLAS,
    # whose fused multiply-adds would round differently from the compiled cores).
    ray = np.stack(
        [
            r[0, 0] * x + r[1, 0] * y + r[2, 0],
            r[0, 1] * x + r[1, 1] * y + r[2, 1],
            r[0, 2] * x + r[1, 2] * y + r[2, 2],
        ],
        axis=-1,
    )
    out = np.full((rows, cols, 2), OUTSIDE)
    front = ray[..., 2] > camera.MIN_DEPTH
    if np.any(front):
        out[front] = camera.project(model, intrinsics, ray[front])
    return out


def _as_shape(shape: tuple[int, int]) -> tuple[int, int]:
    """Check an image shape: two integers from 1 to MAX_IMAGE_SIDE."""
    values = tuple(shape)
    if len(values) != 2 or any(
        isinstance(n, bool) or not isinstance(n, (int, np.integer)) or not 1 <= n <= MAX_IMAGE_SIDE
        for n in values
    ):
        raise InvalidInputError(
            f"shape: expected (rows, cols), each from 1 to {MAX_IMAGE_SIDE}, got {shape!r}"
        )
    return int(values[0]), int(values[1])


def _decode_rectification(raw: dict[str, Any], codes: backends.Codes) -> dict[str, Any]:
    """Return a compiled backend's rectification as float64 arrays and a float."""
    del codes  # the stereo module crosses no codes
    return {
        "rotation_left": np.asarray(raw["rotation_left"], dtype=np.float64).reshape(3, 3),
        "rotation_right": np.asarray(raw["rotation_right"], dtype=np.float64).reshape(3, 3),
        "intrinsics": np.asarray(raw["intrinsics"], dtype=np.float64).reshape(4),
        "baseline": float(raw["baseline"]),
    }


def _decode_map(raw: Any, codes: backends.Codes) -> NDArray[np.float64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's map as a float64 (rows, cols, 2) array."""
    del codes
    out = np.asarray(raw, dtype=np.float64)
    if out.ndim != 3 or out.shape[2] != 2:
        raise RuntimeError(f"stereo: backend returned a map of shape {out.shape}")
    return out


def rectify(  # noqa: PLR0913 -- two cameras, their pose, the image shape
    left_model: str,
    left_intrinsics: ArrayLike,
    right_model: str,
    right_intrinsics: ArrayLike,
    T_right_left: ArrayLike,  # noqa: N803 -- the frame convention's name
    *,
    shape: tuple[int, int],
    backend: str = "python",
) -> Rectification:
    """Rectify a calibrated stereo pair (Bouguet's algorithm).

    Parameters
    ----------
    left_model, right_model : {"pinhole", "radtan", "radial"}
        The cameras' models.
    left_intrinsics, right_intrinsics : array_like
        Their intrinsics (:mod:`slamlab.foundations.camera` gives each model's layout).
    T_right_left : array_like, shape (4, 4)
        The rigid transform from left-camera to right-camera coordinates.
    shape : tuple of int
        The images' ``(rows, cols)``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Rectification
        The rectifying rotations, the shared pinhole camera and the baseline.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a model or its intrinsics are invalid, `T_right_left` is not one rigid transform,
        the shape is out of range, or the baseline is closer to vertical than horizontal.
    slamlab.foundations.errors.DegenerateError
        If the camera centres coincide.
    """
    left_code, left_params = camera.as_model(left_model, left_intrinsics)
    right_code, right_params = camera.as_model(right_model, right_intrinsics)
    flat, lead = as_batch(T_right_left, (4, 4), "T_right_left")
    if lead != ():
        raise InvalidInputError(f"T_right_left: expected one (4, 4) transform, got {lead}")
    se3.check_transform(flat)
    transform = flat[0]
    rows, cols = _as_shape(shape)
    result = backends.call(
        backend,
        __name__,
        "rectify",
        reference=lambda: _rectify(
            (left_params, right_params),
            (camera.FOCAL_INDICES[left_model], camera.FOCAL_INDICES[right_model]),
            transform,
            (rows, cols),
        ),
        arguments={
            "model_left": left_code,
            "intrinsics_left": left_params,
            "model_right": right_code,
            "intrinsics_right": right_params,
            "T_right_left": transform,
            "rows": rows,
            "cols": cols,
        },
        decode=_decode_rectification,
    )
    return Rectification(
        **result,
        shape=(rows, cols),
        left_model=left_model,
        left_intrinsics=left_params,
        right_model=right_model,
        right_intrinsics=right_params,
    )


def _map(  # noqa: PLR0913 -- one camera, its rotation, the rectified camera and shape
    model: str,
    intrinsics: ArrayLike,
    rotation: ArrayLike,
    rectified: NDArray[np.float64],
    *,
    shape: tuple[int, int],
    backend: str,
) -> NDArray[np.float64]:
    """Check one side of a rectification and compute its map on the chosen backend."""
    code, params = camera.as_model(model, intrinsics)
    turn = as_real(rotation, "rotation")
    if turn.shape != (3, 3):
        raise InvalidInputError(f"rotation: expected (3, 3), got {turn.shape}")
    so3.check_rotation(turn.reshape(1, 3, 3))
    return backends.call(
        backend,
        __name__,
        "rectification_map",
        reference=lambda: _rectification_map(model, params, turn, rectified, shape),
        arguments={
            "model": code,
            "intrinsics": params,
            "rotation": turn,
            "rectified": rectified,
            "rows": shape[0],
            "cols": shape[1],
        },
        decode=_decode_map,
    )


def rectification_maps(
    rectification: Rectification, *, backend: str = "python"
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return where each rectified pixel samples the original left and right images.

    Parameters
    ----------
    rectification : Rectification
        The pair, from :func:`rectify`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    left, right : ndarray of float64, shape (rows, cols, 2)
        For each rectified pixel, the ``(x, y)`` to sample in that original image with
        :func:`slamlab.foundations.image.remap`; :data:`OUTSIDE` where the pixel's ray points
        behind the camera.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the rectification's rotations or intrinsics are malformed.
    """
    rows, cols = _as_shape(rectification.shape)
    rectified = as_real(rectification.intrinsics, "intrinsics")
    camera.check_intrinsics("pinhole", rectified)
    left = _map(
        rectification.left_model, rectification.left_intrinsics, rectification.rotation_left,
        rectified, shape=(rows, cols), backend=backend,
    )  # fmt: skip
    right = _map(
        rectification.right_model, rectification.right_intrinsics, rectification.rotation_right,
        rectified, shape=(rows, cols), backend=backend,
    )  # fmt: skip
    return left, right
