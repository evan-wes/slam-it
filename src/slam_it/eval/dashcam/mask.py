"""A dashcam's static mask: the pixels that never move with the scene.

A dashcam's frames hold things that ride with the camera: its burned-in overlay, the car's hood
or rear shelf, a wire across the rear glass. Their features never move, so a run would track
them as a scene that does not move. :func:`measure` finds them from a drive: an edge that sits
on a pixel in more than ``share`` of frames sampled a second apart is the car's or the overlay's,
whatever the light, where the scene's edges move on; small specks are dropped, the rest closed
and grown a little. :func:`finish` scales it to the video's size, adds what is better drawn
than measured (a band across the bottom, a thin out-of-focus line, :class:`Line`), and gives a
``mask.png`` as SLAM It! reads it: 0 where features are dropped, 255 elsewhere
(:mod:`slam_it.eval.tum`).
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import video
from slam_it.eval.dashcam.drives import skip
from slam_it.foundations.errors import InvalidInputError

Mask = NDArray[np.bool_]


@dataclass(frozen=True)
class Line:
    """A line drawn into the mask: a wire, or the edge of a wiper, too soft to measure.

    Attributes
    ----------
    x0, y0, x1, y1 : float
        Its ends, as fractions of the frame's width and height.
    width : float
        Its width, as a fraction of the frame's width.
    """

    x0: float
    y0: float
    x1: float
    y1: float
    width: float


def _shifted_or(mask: Mask, radius: int, axis: int) -> Mask:
    """Return the OR of `mask` shifted by -radius..radius along an axis (edges padded False)."""
    out = mask.copy()
    n = mask.shape[axis]
    for k in range(1, radius + 1):
        if k >= n:
            break
        lead = [slice(None)] * 2
        trail = [slice(None)] * 2
        lead[axis], trail[axis] = slice(k, None), slice(None, -k)
        out[tuple(lead)] |= mask[tuple(trail)]
        out[tuple(trail)] |= mask[tuple(lead)]
    return out


def dilate(mask: Mask, radius: int) -> Mask:
    """Return `mask` grown by a square of half-side `radius` pixels."""
    return _shifted_or(_shifted_or(mask, radius, 0), radius, 1)


def erode(mask: Mask, radius: int) -> Mask:
    """Return `mask` shrunk by a square of half-side `radius` pixels.

    Outside the frame counts as in the mask, so a part touching the frame's edge (a hood) keeps
    its edge through a closing.
    """
    padded = np.pad(~mask, radius, constant_values=False)
    return ~dilate(padded, radius)[radius : -radius or None, radius : -radius or None]


def drop_small(mask: Mask, min_size: int) -> Mask:
    """Return `mask` without its 4-connected parts of fewer than `min_size` pixels."""
    if min_size <= 1:
        return mask.copy()
    h, w = mask.shape
    seen = np.zeros_like(mask)
    keep = np.zeros_like(mask)
    for start in zip(*np.nonzero(mask), strict=True):
        if seen[start]:
            continue
        part = [start]
        seen[start] = True
        stack = [start]
        while stack:
            r, c = stack.pop()
            for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = True
                    stack.append((nr, nc))
                    part.append((nr, nc))
        if len(part) >= min_size:
            rows, cols = zip(*part, strict=True)
            keep[list(rows), list(cols)] = True
    return keep


def edges(gray: NDArray[np.uint8], threshold: float) -> Mask:
    """Return a gray frame's edges: its gradient, after a 3x3 blur, steeper than `threshold`.

    The gradient is the central difference, in gray levels a pixel.
    """
    g = gray.astype(np.float64)
    p = np.pad(g, 1, mode="edge")
    blurred = sum(p[r : r + g.shape[0], c : c + g.shape[1]] for r in range(3) for c in range(3)) / 9
    q = np.pad(blurred, 1, mode="edge")
    dx = (q[1:-1, 2:] - q[1:-1, :-2]) / 2
    dy = (q[2:, 1:-1] - q[:-2, 1:-1]) / 2
    return np.asarray(np.hypot(dx, dy) > threshold)


def _edge_counts(
    frames: Iterable[NDArray[np.uint8]], threshold: float, total: NDArray[np.float64] | None
) -> tuple[NDArray[np.float64] | None, int]:
    """Return `total` plus each frame's edges, as a new array, and the frames counted."""
    out = None if total is None else total.copy()
    count = 0
    for frame in frames:
        e = edges(frame, threshold)
        if out is None:
            out = np.zeros(e.shape)
        elif e.shape != out.shape:
            raise InvalidInputError("frames: expected one size")
        out += e
        count += 1
    return out, count


def persistence(frames: Iterable[NDArray[np.uint8]], threshold: float) -> NDArray[np.float64]:
    """Return the share of frames with an edge at each pixel.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If there are no frames, or they differ in size.
    """
    total, count = _edge_counts(frames, threshold, None)
    if total is None or not count:
        raise InvalidInputError("frames: none")
    return total / count


def measure(  # noqa: PLR0913 -- the segments; how they are judged, keyword-only
    segments: Sequence[str | Path],
    *,
    every: float = 1.0,
    width: int = 640,
    share: float = 0.4,
    threshold: float = 20.0,
    min_size: int = 150,
    close: int = 4,
    grow: int = 3,
) -> Mask:
    """Measure the static pixels of a camera's segments, at a small analysis size.

    Parameters
    ----------
    segments : sequence of str or pathlib.Path
        One camera's segments: minutes of driving, so the scene's edges move on.
    every : float, optional
        The time between the frames sampled, in seconds.
    width : int, optional
        The analysis width, in pixels, the aspect kept.
    share : float, optional
        The share of frames, in (0, 1), an edge must sit at a pixel in for the pixel to be static.
    threshold : float, optional
        An edge's least gradient, in gray levels a pixel (:func:`edges`).
    min_size : int, optional
        The fewest pixels a static part keeps (at the analysis size).
    close : int, optional
        The half-side, in pixels, of the closing that joins a part's pieces.
    grow : int, optional
        The pixels each part is grown by.

    Returns
    -------
    ndarray of bool, shape (H, W)
        The static pixels, at the analysis size.

    A damaged segment is skipped whole, with a warning (:func:`.drives.skip`).

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If there are no segments, every one is damaged, or a setting is out of range.
    """
    if not segments:
        raise InvalidInputError("segments: none given")
    if not 0.0 < share < 1.0:
        raise InvalidInputError(f"share: expected a fraction in (0, 1), got {share}")
    if min(close, grow) < 0 or min_size < 0:
        raise InvalidInputError("close, grow, min_size: expected values >= 0")
    total: NDArray[np.float64] | None = None
    count = 0
    for segment in segments:
        sampled = (f for _, f in video.sample_frames(segment, every=every, width=width, gray=True))
        try:
            added, more = _edge_counts(sampled, threshold, total)
            if not more:
                raise video.DamagedSegmentError(f"{Path(segment).name}: no frames")
        except video.DamagedSegmentError as error:
            skip(segment, error)
            continue
        total, count = added, count + more
    if total is None:
        raise InvalidInputError("segments: every one is damaged")
    static = total / count > share
    if close:
        static = erode(dilate(static, close), close)
    static = drop_small(static, min_size)
    return dilate(static, grow) if grow else static


def _resize_nearest(mask: Mask, width: int, height: int) -> Mask:
    """Return `mask` resampled to ``(height, width)``, each pixel its nearest source pixel."""
    h, w = mask.shape
    rows = np.minimum(((np.arange(height) + 0.5) * h / height).astype(np.int64), h - 1)
    cols = np.minimum(((np.arange(width) + 0.5) * w / width).astype(np.int64), w - 1)
    return np.asarray(mask[rows[:, None], cols[None, :]], dtype=bool)


def scaled(mask: NDArray[np.uint8], width: int, height: int) -> NDArray[np.uint8]:
    """Return a ``mask.png``'s mask at another size: a pixel drops if any pixel under it does.

    Scaled down, a thin line stays in the mask, where a nearest pixel's value could miss it.

    Parameters
    ----------
    mask : ndarray of uint8, shape (h, w)
        The mask, 0 where features are dropped.
    width, height : int
        The size wanted.

    Returns
    -------
    ndarray of uint8, shape (height, width)
        The mask, 0 or 255.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the size is not positive.
    """
    if width <= 0 or height <= 0:
        raise InvalidInputError(f"size: expected a positive size, got {width}x{height}")
    dropped = np.asarray(mask) == 0
    h, w = dropped.shape
    integral = np.zeros((h + 1, w + 1), dtype=np.int64)
    integral[1:, 1:] = dropped.cumsum(axis=0).cumsum(axis=1)

    def spans(n: int, source: int) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
        k = np.arange(n)
        lo = np.minimum(k * source // n, source - 1)
        hi = np.maximum(-(-(k + 1) * source // n), lo + 1)
        return lo, hi

    r0, r1 = spans(height, h)
    c0, c1 = spans(width, w)
    count = integral[r1][:, c1] - integral[r0][:, c1] - integral[r1][:, c0] + integral[r0][:, c0]
    return np.where(count > 0, 0, 255).astype(np.uint8)


def draw_line(mask: Mask, line: Line) -> Mask:
    """Return `mask` with a line drawn in, its ends and width as :class:`Line` gives them."""
    h, w = mask.shape
    a = np.array([line.x0 * w, line.y0 * h])
    b = np.array([line.x1 * w, line.y1 * h])
    rows, cols = np.mgrid[0:h, 0:w]
    p = np.stack([cols + 0.5, rows + 0.5], axis=-1)
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    distance = np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)
    return np.asarray(mask | (distance <= line.width * w / 2), dtype=bool)


def finish(
    static: Mask,
    width: int,
    height: int,
    *,
    bottom: float | None = None,
    lines: Sequence[Line] = (),
) -> NDArray[np.uint8]:
    """Return the mask at the video's size, as SLAM It! reads one: 0 drops a feature, 255 keeps.

    Parameters
    ----------
    static : ndarray of bool, shape (h, w)
        The static pixels (:func:`measure`), at any size of the video's aspect.
    width, height : int
        The video's stored size.
    bottom : float, optional
        Drop everything below this fraction of the height too (the hood, a dark shelf).
    lines : sequence of Line, optional
        Lines drawn in.

    Returns
    -------
    ndarray of uint8, shape (height, width)
        The mask, 0 or 255.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the size is not positive, or `bottom` is not a fraction.
    """
    if width <= 0 or height <= 0:
        raise InvalidInputError(f"size: expected a positive size, got {width}x{height}")
    if bottom is not None and not 0.0 < bottom <= 1.0:
        raise InvalidInputError(f"bottom: expected a fraction in (0, 1], got {bottom}")
    out = _resize_nearest(np.asarray(static, dtype=bool), width, height)
    if bottom is not None:
        out[round(bottom * height) :, :] = True
    for line in lines:
        out = draw_line(out, line)
    return np.where(out, 0, 255).astype(np.uint8)
