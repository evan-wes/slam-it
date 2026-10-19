"""Frames: one input image's features, each keypoint's bearing, depth and right column.

Every mode ends in the same :class:`Frame`, so tracking and mapping never ask which mode
they run in:

- **Monocular** (:func:`monocular`): ORB features and their bearings; no depth.
- **Stereo** (:func:`stereo_pair`): rectified left and right images, matched along rows
  (:func:`slam_it.foundations.stereo.match`); a matched left keypoint has the depth
  ``f b / d`` and its right column.
- **RGB-D** (:func:`rgbd`): the depth map's value at each keypoint's nearest pixel, and a
  *virtual* right column ``u - f b / z`` from a virtual baseline, as ORB-SLAM gives RGB-D
  keypoints, so a depth measurement is handled exactly as a stereo one.

Keypoints stay where ORB found them, in the distorted image: bearings come from
:func:`slam_it.foundations.camera.unproject_or_nan`, and projections go through the full
camera model, so nothing is undistorted.

**A mask** (each builder's ``mask``: uint8, the image's shape, 0 where nothing is to be
tracked, such as a camera's burned-in overlay or the car's own body) drops every feature whose
keypoint falls on a 0, at its nearest pixel, half up (:func:`unmasked`), after extraction: the
extractor and its parity are unchanged, and the features a mask drops are not replaced. A
stereo pair's mask is the left image's: the right image's features are matched unmasked.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations import camera as camera_model
from slam_it.foundations import matching, orb, stereo
from slam_it.foundations.arrays import MAX_IMAGE_SIDE, as_count, as_image, as_number, as_real
from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.image import pyramid

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


@dataclass(frozen=True)
class Camera:
    """The camera a sequence was taken with.

    Attributes
    ----------
    model : str
        A model from :data:`slam_it.foundations.camera.MODELS`.
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
        slam_it.foundations.errors.InvalidInputError
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

    def project_in_view(self, points: Array) -> tuple[Array, NDArray[np.bool_]]:
        """Project camera-frame points, and say which are in front of the camera and in the image.

        Parameters
        ----------
        points : ndarray of float64, shape (N, 3)
            Points in the camera's frame.

        Returns
        -------
        pixels : ndarray of float64, shape (N, 2)
            Their pixels; NaN for a point not in front of the camera.
        seen : ndarray of bool, shape (N,)
            Which are in front of it (``z > 0``) and project into the image (:meth:`inside`).
        """
        front = points[:, 2] > 0.0
        pixels = np.full((len(points), 2), np.nan)
        pixels[front] = self.project(points[front])
        return pixels, front & self.inside(pixels)

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
    features : slam_it.foundations.orb.Features
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

    def __post_init__(self) -> None:
        """Check that every array holds one float64 row per keypoint.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If the descriptors, bearings, depths or right columns are not one per keypoint.
        """
        n = len(self.features.keypoints.xy)
        if len(self.features.descriptors) != n:
            raise InvalidInputError(
                f"frame: expected one descriptor per keypoint ({n}), got "
                f"{len(self.features.descriptors)}"
            )
        for name, value, shape in (
            ("bearings", self.bearings, (n, 3)),
            ("depth", self.depth, (n,)),
            ("right", self.right, (n,)),
        ):
            if not isinstance(value, np.ndarray) or value.dtype != np.float64:
                received = getattr(value, "dtype", type(value).__name__)
                raise InvalidInputError(f"frame.{name}: expected a float64 array, got {received}")
            if value.shape != shape:
                raise InvalidInputError(
                    f"frame.{name}: expected shape {shape}, one row per keypoint, got {value.shape}"
                )

    @property
    def size(self) -> int:
        """The number of keypoints."""
        return len(self.features.descriptors)

    def as_dict(self) -> dict[str, Any]:
        """Return the frame as it crosses the bindings: its features' fields and its arrays.

        Returns
        -------
        dict
            ``timestamp``, the keypoints' ``xy``, ``level``, ``level_xy``, ``response`` and
            ``angle``, ``descriptors``, ``scale_factor``, ``levels``, and ``bearings``,
            ``depth`` and ``right``.
        """
        keypoints = self.features.keypoints
        return {
            "timestamp": float(self.timestamp),
            "xy": keypoints.xy,
            "level": keypoints.level,
            "level_xy": keypoints.level_xy,
            "response": keypoints.response,
            "angle": keypoints.angle,
            "descriptors": self.features.descriptors,
            "scale_factor": float(self.features.scale_factor),
            "levels": int(self.features.levels),
            "bearings": self.bearings,
            "depth": self.depth,
            "right": self.right,
        }


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


def monocular(  # noqa: PLR0913 -- the image, its time, the camera, the settings; mask, backend
    image: ArrayLike,
    timestamp: float,
    camera: Camera,
    options: orb.Options,
    *,
    mask: ArrayLike | None = None,
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
    options : slam_it.foundations.orb.Options
        The extraction settings.
    mask : array_like of uint8, shape (H, W), optional
        The features to drop: those on a 0 (the module docstring); None keeps them all.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation extracts the features.

    Returns
    -------
    Frame
        The frame.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mask's shape differs from the image's.
    """
    features = _masked(orb.extract(image, options, backend=backend), mask, np.shape(image), backend)
    n = len(features.descriptors)
    return _frame(timestamp, features, camera, np.full(n, np.nan), np.full(n, np.nan))


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.frames`` submodule provides."""

    def depth_at(
        self, xy: NDArray[np.float64], depth_map: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Read a depth map at each keypoint's nearest pixel."""
        ...

    def unmasked(self, xy: NDArray[np.float64], mask: NDArray[np.uint8]) -> NDArray[np.int64]:
        """Return the indices of the keypoints whose nearest pixel the mask keeps."""
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
    slam_it.foundations.errors.InvalidInputError
        If the keypoints are not a finite (N, 2) array, or the map is not a non-empty 2D
        array of real numbers (infinities refused).
    """
    pixels = as_real(xy, "xy").reshape(-1, 2) if np.size(xy) else np.zeros((0, 2))
    depths = as_real(depth_map, "depth_map", allow_nan=True)
    if depths.ndim != 2 or depths.size == 0:
        raise InvalidInputError(f"depth_map: expected a non-empty 2D array, got {depths.shape}")
    return backends.dispatch(backend, __name__, "depth_at", _depth_at, pixels, depths)


def _unmasked(xy: Array, mask: NDArray[np.uint8]) -> IntArray:
    """Return the keypoints the mask keeps (reference)."""
    h, w = mask.shape
    col = np.clip(np.floor(xy[:, 0] + 0.5).astype(np.int64), 0, w - 1)
    row = np.clip(np.floor(xy[:, 1] + 0.5).astype(np.int64), 0, h - 1)
    return np.flatnonzero(mask[row, col] != 0).astype(np.int64)


def _decode_rows(raw: Any, codes: backends.Codes) -> IntArray:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's kept rows, checked (never cast)."""
    del codes  # crosses no codes
    out: IntArray = backends.checked_array(raw, np.int64, 1, "frames unmasked")
    return out


def unmasked(xy: ArrayLike, mask: ArrayLike, *, backend: str = "python") -> IntArray:
    """Return the keypoints a mask keeps: those whose nearest pixel (half up) is not 0.

    Parameters
    ----------
    xy : array_like, shape (N, 2)
        The keypoints, in pixels, finite (clamped to the mask's edge, as :func:`depth_at`
        clamps them).
    mask : array_like of uint8, shape (H, W)
        0 where features are dropped, anything else where they are kept.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64
        The kept keypoints' indices, ascending.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the keypoints are not a finite (N, 2) array, or the mask is not a non-empty (H, W)
        uint8 array.
    """
    pixels = as_real(xy, "xy").reshape(-1, 2) if np.size(xy) else np.zeros((0, 2))
    keep = as_image(mask, "mask")
    return backends.call(
        backend,
        __name__,
        "unmasked",
        reference=lambda: _unmasked(pixels, keep),
        arguments={"xy": pixels, "mask": keep},
        decode=_decode_rows,
    )


def _take(features: orb.Features, rows: IntArray) -> orb.Features:
    """Return the features' rows `rows`, every array kept together."""
    k = features.keypoints
    return orb.Features(
        keypoints=orb.Keypoints(
            xy=k.xy[rows], level=k.level[rows], level_xy=k.level_xy[rows],
            response=k.response[rows], angle=k.angle[rows],
        ),
        descriptors=features.descriptors[rows],
        scale_factor=features.scale_factor,
        levels=features.levels,
    )  # fmt: skip


def _masked(
    features: orb.Features, mask: ArrayLike | None, shape: tuple[int, ...], backend: str
) -> orb.Features:
    """Drop the features a mask of the image's `shape` covers; all of them kept without one."""
    if mask is None:
        return features
    if np.shape(mask) != shape:
        raise InvalidInputError(f"mask: expected the image's shape {shape}, got {np.shape(mask)}")
    return _take(features, unmasked(features.keypoints.xy, mask, backend=backend))


def rgbd(  # noqa: PLR0913 -- the image and its depth, its time, the camera, the settings
    image: ArrayLike,
    depth_map: ArrayLike,
    timestamp: float,
    camera: Camera,
    options: orb.Options,
    *,
    mask: ArrayLike | None = None,
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
    options : slam_it.foundations.orb.Options
        The extraction settings.
    mask : array_like of uint8, shape (H, W), optional
        The features to drop: those on a 0 (the module docstring); None keeps them all.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation extracts the features.

    Returns
    -------
    Frame
        The frame.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the depth map's or the mask's shape differs from the image's, or the baseline is 0.
    """
    depths = as_real(depth_map, "depth_map", allow_nan=True)
    if depths.shape != np.shape(image):
        raise InvalidInputError(
            f"depth_map: expected the image's shape {np.shape(image)}, got {depths.shape}"
        )
    if camera.baseline <= 0.0:
        raise InvalidInputError("camera.baseline: an RGB-D camera needs a virtual baseline")
    features = _masked(orb.extract(image, options, backend=backend), mask, np.shape(image), backend)
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
    mask: ArrayLike | None = None,
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
    options : slam_it.foundations.orb.Options
        The extraction settings.
    stereo_options : slam_it.foundations.stereo.Options, optional
        The row matching's settings.
    mask : array_like of uint8, shape (H, W), optional
        The left image's features to drop: those on a 0 (the module docstring), before the
        row matching; the right image's are matched unmasked. None keeps them all.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation extracts and matches.

    Returns
    -------
    Frame
        The frame (the left image's keypoints).

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the camera is not ``pinhole`` or the baseline is 0, or the mask's shape differs from
        the left image's.
    """
    if camera.model != "pinhole" or camera.baseline <= 0.0:
        raise InvalidInputError(
            "camera: a stereo pair needs a rectified pinhole camera and baseline"
        )
    # Each image's pyramid, built once: extraction and the row matching both read it.
    left_levels = pyramid(left, options.levels, options.scale_factor, backend=backend)
    right_levels = pyramid(right, options.levels, options.scale_factor, backend=backend)
    features = _masked(
        orb.extract_levels(left_levels, options, backend=backend), mask, np.shape(left), backend
    )
    other = orb.extract_levels(right_levels, options, backend=backend)
    found = stereo.match(
        left_levels,
        right_levels,
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


def search(
    frame: Frame,
    projections: matching.Projections,
    options: matching.SearchOptions,
    *,
    taken: Mask | None = None,
) -> matching.Matches:
    """Match projected map points to a frame's keypoints, its right columns checked.

    :func:`slam_it.foundations.matching.search_by_projection` with the frame's features and
    right columns (NaN for a monocular frame: no check).

    Parameters
    ----------
    frame : Frame
        The frame.
    projections : slam_it.foundations.matching.Projections
        The projected points.
    options : slam_it.foundations.matching.SearchOptions
        The filters.
    taken : ndarray of bool, shape (N,), optional
        The keypoints already matched, never candidates (default: none).

    Returns
    -------
    slam_it.foundations.matching.Matches
        ``(projection, keypoint)`` pairs.
    """
    return matching.search_by_projection(
        projections, frame.features, options, right=frame.right, taken=taken
    )


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

    def as_dict(self) -> dict[str, Any]:
        """Return the tracked frame as it crosses the bindings.

        Returns
        -------
        dict
            ``pose`` (4, 4) and ``points`` (N,).
        """
        return {"pose": self.pose, "points": self.points}


def as_pose(pose: ArrayLike, name: str = "pose") -> Array:
    """Check a pose: a finite 4x4 matrix.

    Parameters
    ----------
    pose : array_like, shape (4, 4)
        The pose.
    name : str, optional
        Its name, for messages.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        The pose.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If it is not a finite 4x4 matrix.
    """
    matrix = as_real(pose, name)
    if matrix.shape != (4, 4):
        raise InvalidInputError(f"{name}: expected shape (4, 4), got {matrix.shape}")
    return matrix


def checked_tracked(tracked: Tracked, frame: Frame, name: str = "tracked") -> Tracked:
    """Check a tracked frame: a finite 4x4 pose and one point id (or -1) per keypoint.

    Parameters
    ----------
    tracked : Tracked
        The tracked frame.
    frame : Frame
        The frame it tracks.
    name : str, optional
        Its name, for messages.

    Returns
    -------
    Tracked
        The tracked frame, its arrays contiguous.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the pose is not a finite 4x4 matrix, or the matches are not one per keypoint.
    """
    ids = np.asarray(tracked.points)
    if ids.shape != (frame.size,) or (
        ids.size and (not np.issubdtype(ids.dtype, np.integer) or ids.min() < -1)
    ):
        raise InvalidInputError(f"{name}.points: expected one point id (or -1) per keypoint")
    return Tracked(
        pose=as_pose(tracked.pose, f"{name}.pose"),
        points=np.ascontiguousarray(ids, dtype=np.int64),
    )


def decode_tracked(raw: dict[str, Any] | None, codes: backends.Codes) -> Tracked | None:
    """Rebuild a tracked frame from a compiled backend's dict, its arrays checked.

    Parameters
    ----------
    raw : dict or None
        ``pose`` and ``points``, or None.
    codes : dict
        The backend module's code tables (a tracked frame crosses none).

    Returns
    -------
    Tracked or None
        The tracked frame.
    """
    del codes
    if raw is None:
        return None
    return Tracked(
        pose=backends.checked_array(raw["pose"], np.float64, 2, "tracked pose"),
        points=backends.checked_array(raw["points"], np.int64, 1, "tracked points"),
    )
