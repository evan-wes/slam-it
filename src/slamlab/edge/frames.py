"""Frames: one input image's features, each keypoint's bearing, depth and right column.

Every mode ends in the same :class:`Frame`, so tracking and mapping never ask which mode
they run in:

- **Monocular** (:func:`monocular`): ORB features and their bearings; no depth.
- **Stereo** (:func:`stereo`): rectified left and right images, matched along rows
  (:func:`slamlab.foundations.stereo.match`); a matched left keypoint has the depth
  ``f b / d`` and its right column.
- **RGB-D** (:func:`rgbd`): the depth map's value at each keypoint's nearest pixel, and a
  *virtual* right column ``u - f b / z`` from a virtual baseline, as ORB-SLAM gives RGB-D
  keypoints, so a depth measurement is handled exactly as a stereo one.

Keypoints stay where ORB found them, in the distorted image: bearings come from
:func:`slamlab.foundations.camera.unproject_or_nan`, and projections go through the full
camera model, so nothing is undistorted.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import camera as camera_model
from slamlab.foundations import orb, stereo
from slamlab.foundations.arrays import MAX_IMAGE_SIDE, as_count, as_number, as_real
from slamlab.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class Camera:
    """The camera a sequence was taken with.

    Attributes
    ----------
    model : str
        A model from :data:`slamlab.foundations.camera.MODELS`.
    intrinsics : ndarray of float64, shape (P,)
        Its intrinsics.
    width, height : int
        The image's size, in pixels.
    baseline : float
        The stereo baseline in metres, or RGB-D's virtual one (ORB-SLAM's ``bf / fx``); 0 for
        monocular.
    """

    model: str
    intrinsics: Array
    width: int
    height: int
    baseline: float = 0.0

    def __post_init__(self) -> None:
        """Check the camera.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If the model or intrinsics are invalid, a side is not a positive integer, or the
            baseline is negative or not finite.
        """
        camera_model.check_intrinsics(self.model, self.intrinsics)
        set_ = object.__setattr__  # frozen: store the checked values
        set_(self, "intrinsics", as_real(self.intrinsics, "intrinsics"))
        set_(self, "width", as_count(self.width, "width", 1, MAX_IMAGE_SIDE))
        set_(self, "height", as_count(self.height, "height", 1, MAX_IMAGE_SIDE))
        baseline = as_number(self.baseline, "baseline")
        if not (np.isfinite(baseline) and baseline >= 0.0):
            raise InvalidInputError(f"baseline: must be finite and non-negative, got {baseline}")
        set_(self, "baseline", baseline)

    def as_dict(self) -> dict[str, object]:
        """Return the camera as it crosses the bindings.

        Returns
        -------
        dict
            ``model``, ``intrinsics``, ``width``, ``height`` and ``baseline``.
        """
        return {
            "model": self.model,
            "intrinsics": self.intrinsics,
            "width": self.width,
            "height": self.height,
            "baseline": self.baseline,
        }

    @property
    def focal(self) -> float:
        """The horizontal focal length, in pixels."""
        return float(self.intrinsics[camera_model.FOCAL_INDICES[self.model][0]])

    def project(self, points: Array) -> Array:
        """Project camera-frame points to pixels, through the full model.

        Parameters
        ----------
        points : ndarray of float64, shape (N, 3)
            Points in the camera's frame, in front of it.

        Returns
        -------
        ndarray of float64, shape (N, 2)
            Their pixels.
        """
        return camera_model.project(self.model, self.intrinsics, points)

    def bearings(self, pixels: Array) -> Array:
        """Return unit bearings of pixels; NaN rows where the model cannot unproject.

        Parameters
        ----------
        pixels : ndarray of float64, shape (N, 2)
            The pixels.

        Returns
        -------
        ndarray of float64, shape (N, 3)
            Their bearings.
        """
        return camera_model.unproject_or_nan(self.model, self.intrinsics, pixels)

    def inside(self, pixels: Array) -> NDArray[np.bool_]:
        """Which pixels lie in the image: ``[0, width) x [0, height)``.

        Parameters
        ----------
        pixels : ndarray of float64, shape (N, 2)
            The pixels.

        Returns
        -------
        ndarray of bool, shape (N,)
            Whether each is inside.
        """
        return np.asarray(
            (pixels[:, 0] >= 0.0)
            & (pixels[:, 0] < self.width)
            & (pixels[:, 1] >= 0.0)
            & (pixels[:, 1] < self.height)
        )


@dataclass(frozen=True)
class Frame:
    """One frame's features, as tracking and mapping read them.

    Attributes
    ----------
    timestamp : float
        Its time, in seconds.
    features : slamlab.foundations.orb.Features
        Its ORB features (N keypoints).
    bearings : ndarray of float64, shape (N, 3)
        Each keypoint's unit bearing in the camera; NaN where it cannot be unprojected.
    depth : ndarray of float64, shape (N,)
        Each keypoint's depth (its point's z in the camera), in metres; NaN where unknown.
    right : ndarray of float64, shape (N,)
        Each keypoint's right-image column (real for stereo, virtual for RGB-D); NaN where
        its depth is unknown.
    """

    timestamp: float
    features: orb.Features
    bearings: Array
    depth: Array
    right: Array

    @property
    def size(self) -> int:
        """The number of keypoints."""
        return len(self.features.descriptors)


def _frame(
    timestamp: float, features: orb.Features, camera: Camera, depth: Array, right: Array
) -> Frame:
    """Assemble a frame from its features, with their bearings."""
    return Frame(
        timestamp=float(timestamp),
        features=features,
        bearings=camera.bearings(features.keypoints.xy),
        depth=depth,
        right=right,
    )


def monocular(
    image: ArrayLike,
    timestamp: float,
    camera: Camera,
    options: orb.Options,
    *,
    backend: str = "python",
) -> Frame:
    """Build a monocular frame: features and bearings, no depth.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        The gray image.
    timestamp : float
        Its time, in seconds.
    camera : Camera
        The camera.
    options : slamlab.foundations.orb.Options
        The extraction settings.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation extracts the features.

    Returns
    -------
    Frame
        The frame.
    """
    features = orb.extract(image, options, backend=backend)
    n = len(features.descriptors)
    return _frame(timestamp, features, camera, np.full(n, np.nan), np.full(n, np.nan))


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.frames`` submodule provides."""

    def depth_at(
        self, xy: NDArray[np.float64], depth_map: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Read a depth map at each keypoint's nearest pixel."""
        ...


def _depth_at(xy: Array, depth_map: Array) -> Array:
    """Read a depth map at keypoints (reference)."""
    h, w = depth_map.shape
    col = np.clip(np.floor(xy[:, 0] + 0.5).astype(np.int64), 0, w - 1)
    row = np.clip(np.floor(xy[:, 1] + 0.5).astype(np.int64), 0, h - 1)
    values = depth_map[row, col]
    return np.where(np.isfinite(values) & (values > 0.0), values, np.nan)


def depth_at(xy: ArrayLike, depth_map: ArrayLike, *, backend: str = "python") -> Array:
    """Read a depth map at each keypoint's nearest pixel (half up); NaN where it holds none.

    Parameters
    ----------
    xy : array_like, shape (N, 2)
        The keypoints, in pixels, finite (clamped to the map's edge).
    depth_map : array_like, shape (H, W)
        Depths in metres; 0 or NaN where the sensor measured nothing.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (N,)
        The depths: positive, or NaN.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the keypoints are not a finite (N, 2) array, or the map is not a non-empty 2D
        array of real numbers (infinities refused).
    """
    pixels = as_real(xy, "xy").reshape(-1, 2) if np.size(xy) else np.zeros((0, 2))
    depths = as_real(depth_map, "depth_map", allow_nan=True)
    if depths.ndim != 2 or depths.size == 0:
        raise InvalidInputError(f"depth_map: expected a non-empty 2D array, got {depths.shape}")
    return backends.dispatch(backend, __name__, "depth_at", _depth_at, pixels, depths)


def rgbd(  # noqa: PLR0913 -- the image and its depth, its time, the camera, the settings
    image: ArrayLike,
    depth_map: ArrayLike,
    timestamp: float,
    camera: Camera,
    options: orb.Options,
    *,
    backend: str = "python",
) -> Frame:
    """Build an RGB-D frame: features, and each keypoint's depth and virtual right column.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        The gray image.
    depth_map : array_like of float, shape (H, W)
        The registered depth, in metres (0 or NaN for none).
    timestamp : float
        Its time, in seconds.
    camera : Camera
        The camera; its ``baseline`` is the virtual one (positive).
    options : slamlab.foundations.orb.Options
        The extraction settings.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation extracts the features.

    Returns
    -------
    Frame
        The frame.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the depth map's shape differs from the image's, or the baseline is 0.
    """
    features = orb.extract(image, options, backend=backend)
    depths = as_real(depth_map, "depth_map", allow_nan=True)
    if depths.shape != np.shape(image):
        raise InvalidInputError(
            f"depth_map: expected the image's shape {np.shape(image)}, got {depths.shape}"
        )
    if camera.baseline <= 0.0:
        raise InvalidInputError("camera.baseline: an RGB-D camera needs a virtual baseline")
    xy = features.keypoints.xy
    depth = _depth_at(xy, depths)
    right = xy[:, 0] - camera.focal * camera.baseline / depth
    return _frame(timestamp, features, camera, depth, right)


def stereo_pair(  # noqa: PLR0913 -- the pair, its time, the camera, both settings
    left: ArrayLike,
    right: ArrayLike,
    timestamp: float,
    camera: Camera,
    options: orb.Options,
    *,
    stereo_options: stereo.Options | None = None,
    backend: str = "python",
) -> Frame:
    """Build a stereo frame from a rectified pair: left features, matched along rows.

    Parameters
    ----------
    left, right : array_like of uint8, shape (H, W)
        The rectified pair.
    timestamp : float
        Its time, in seconds.
    camera : Camera
        The rectified left camera (``pinhole``); its ``baseline`` is the pair's.
    options : slamlab.foundations.orb.Options
        The extraction settings.
    stereo_options : slamlab.foundations.stereo.Options, optional
        The row matching's settings.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation extracts and matches.

    Returns
    -------
    Frame
        The frame (the left image's keypoints).

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the camera is not ``pinhole`` or the baseline is 0.
    """
    if camera.model != "pinhole" or camera.baseline <= 0.0:
        raise InvalidInputError(
            "camera: a stereo pair needs a rectified pinhole camera and baseline"
        )
    features = orb.extract(left, options, backend=backend)
    other = orb.extract(right, options, backend=backend)
    found = stereo.match(
        left,
        right,
        features,
        other,
        focal=camera.focal,
        baseline=camera.baseline,
        options=stereo_options,
        backend=backend,
    )
    n = len(features.descriptors)
    depth, columns = np.full(n, np.nan), np.full(n, np.nan)
    depth[found.left] = found.depth
    columns[found.left] = found.right_x
    return _frame(timestamp, features, camera, depth, columns)


@dataclass(frozen=True)
class Tracked:
    """A frame placed against the map.

    Attributes
    ----------
    pose : ndarray of float64, shape (4, 4)
        Its ``T_cam_world``.
    points : ndarray of int64, shape (N,)
        The id of the map point each keypoint matched, -1 where none (outliers removed).
    """

    pose: Array
    points: IntArray

    @property
    def inliers(self) -> int:
        """How many keypoints matched a map point."""
        return int(np.count_nonzero(self.points >= 0))
