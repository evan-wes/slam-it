"""A phone's LiDAR depth, made the depth maps a TUM-format folder holds for an RGB-D run.

Apple's LiDAR (ARKit's scene depth), as Stray Scanner and ARKitScenes record it: a 16-bit depth
map in millimetres at 256x192 and an 8-bit confidence map beside it, ``0`` low, ``1`` medium and
``2`` high, both registered to the wide camera's whole field of view, so they are the colour
frame's own pixels at a coarser grid, of the same 4:3 aspect.

:func:`registered_depth` keeps the depth whose confidence is at least a threshold (high, by
default: low confidence is noisy at objects' edges), zero elsewhere, and brings it to the
colour frame's size by nearest neighbour: interpolating would invent depth across an edge,
between a near surface and a far one. The result is what :mod:`slam_it.eval.tum` reads: a
depth map registered to the colour frame, in millimetres, so a folder's ``camera.txt`` says
``depth_scale`` :data:`DEPTH_SCALE` and ``bf`` :func:`bf` of the colour camera's focal length.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from slam_it.foundations.errors import InvalidInputError

HIGH = 2
"""ARKit's high confidence."""

MEDIUM = 1
"""ARKit's medium confidence."""

DEPTH_SCALE = 1000.0
"""A depth map's value per metre: millimetres."""

BASELINE = 0.08
"""The virtual stereo baseline a depth sensor is given for an RGB-D run, in metres: what
ORB-SLAM's RGB-D settings give TUM's Kinect (``Camera.bf`` 40 at a focal length of about 517
pixels). It decides which keypoints count as near; no stereo is computed."""


def bf(fx: float) -> float:
    """Return ``camera.txt``'s ``bf`` for a colour camera of focal length `fx`, in pixels."""
    return BASELINE * fx


def registered_depth(
    depth: NDArray[np.uint16],
    confidence: NDArray[np.uint8],
    size: tuple[int, int],
    min_confidence: int = HIGH,
) -> NDArray[np.uint16]:
    """Return LiDAR depth kept by its confidence, at a colour frame's size by nearest neighbour.

    Parameters
    ----------
    depth : ndarray of uint16, shape (h, w)
        The depth map, in millimetres; 0 where there is none.
    confidence : ndarray of uint8, shape (h, w)
        Its confidence, 0 to 2.
    size : tuple of int
        The colour frame's width and height: the depth map's aspect, at least its size.
    min_confidence : int, optional
        The least confidence kept (:data:`HIGH` by default).

    Returns
    -------
    ndarray of uint16, shape (height, width)
        The depth, each colour pixel taking the depth pixel its centre falls in; 0 where the
        confidence is below `min_confidence`.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the maps differ in shape or are not 2-D, the threshold is not 0 to 2, or the colour
        frame is not the depth map's aspect, at least its size.
    """
    if depth.ndim != 2 or depth.shape != confidence.shape:
        raise InvalidInputError(
            f"depth {depth.shape} and confidence {confidence.shape}: expected one 2-D shape"
        )
    if min_confidence not in (0, MEDIUM, HIGH):
        raise InvalidInputError(f"min_confidence: expected 0, 1 or 2, got {min_confidence}")
    height, width = depth.shape
    out_width, out_height = size
    if out_width * height != out_height * width or out_width < width:
        raise InvalidInputError(
            f"size {out_width}x{out_height}: expected the depth map's {width}x{height} aspect, "
            "at least its size"
        )
    kept = np.where(confidence >= min_confidence, depth, 0).astype(np.uint16)
    rows = ((np.arange(out_height) + 0.5) * height / out_height).astype(np.int64)
    columns = ((np.arange(out_width) + 0.5) * width / out_width).astype(np.int64)
    out: NDArray[np.uint16] = kept[np.ix_(rows, columns)]
    return out
