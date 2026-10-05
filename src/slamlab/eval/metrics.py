"""Trajectory metrics: absolute trajectory error (ATE) and relative pose error (RPE).

Both follow Sturm et al.'s TUM RGB-D benchmark (IROS 2012), the convention ORB-SLAM, DROID and
most papers report. They compare an estimated trajectory with the ground truth, pose for pose:
the two arrays hold ``T_world_cam`` at the same instants (associate them by timestamp first).

- **ATE:** the estimate is aligned to the truth by the least-squares transform between their
  camera centres (Umeyama, :func:`slamlab.foundations.alignment.align`): Sim(3) for monocular
  runs, whose scale is unobservable, SE(3) for stereo and RGB-D. The error of each pose is the
  distance between its aligned centre and the true one; the headline is their RMSE.
- **RPE:** the drift over a fixed interval. For each pose ``i`` and the pose ``delta`` steps
  later, the error is ``(Q_i^-1 Q_j)^-1 (P_i^-1 P_j)``, ``Q`` the truth and ``P`` the estimate
  (its translations multiplied by the ATE's scale for a monocular run); its translation's norm
  and rotation's angle are reported, with their RMSE. No alignment is needed: relative motion
  does not depend on the world frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab.foundations import alignment, se3, so3
from slamlab.foundations.arrays import as_batch
from slamlab.foundations.errors import InvalidInputError


@dataclass(frozen=True)
class AbsoluteError:
    """An absolute trajectory error.

    Attributes
    ----------
    errors : ndarray of float64, shape (N,)
        Each pose's centre error after alignment, in the truth's units.
    rmse, mean, median, max : float
        Their root mean square, mean, median and maximum.
    alignment : ndarray of float64, shape (4, 4)
        The transform applied to the estimate's centres: ``[[s R, t], [0, 1]]``.
    scale : float
        Its scale ``s`` (1 for an SE(3) alignment).
    """

    errors: NDArray[np.float64]
    rmse: float
    mean: float
    median: float
    max: float
    alignment: NDArray[np.float64]
    scale: float


@dataclass(frozen=True)
class RelativeError:
    """A relative pose error over a fixed interval.

    Attributes
    ----------
    translation : ndarray of float64, shape (M,)
        Each pair's translational drift, in the truth's units.
    rotation : ndarray of float64, shape (M,)
        Each pair's rotational drift, in radians.
    translation_rmse : float
        The translational drifts' root mean square.
    rotation_rmse : float
        The rotational drifts' root mean square, in radians.
    delta : int
        The interval, in poses.
    """

    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]
    translation_rmse: float
    rotation_rmse: float
    delta: int


def _as_trajectories(
    estimate: ArrayLike, truth: ArrayLike
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Check two trajectories: equal-length (N, 4, 4) batches of rigid transforms."""
    est, lead = as_batch(estimate, (4, 4), "estimate")
    tru, lead_t = as_batch(truth, (4, 4), "truth")
    if len(lead) != 1 or lead != lead_t:
        raise InvalidInputError(
            f"estimate and truth: expected equal (N, 4, 4) shapes, got {lead} vs {lead_t}"
        )
    se3.check_transform(est)
    se3.check_transform(tru)
    return est, tru


def _rms(values: NDArray[np.float64]) -> float:
    """Return the root mean square of a non-empty array."""
    return math.sqrt(float(np.mean(values * values)))


def absolute_error(
    estimate: ArrayLike, truth: ArrayLike, *, with_scale: bool = False
) -> AbsoluteError:
    """Return the absolute trajectory error after aligning the estimate to the truth.

    Parameters
    ----------
    estimate, truth : array_like, shape (N, 4, 4)
        ``T_world_cam`` at the same instants, N >= 3.
    with_scale : bool, optional
        Align with a similarity (Sim(3)), for monocular estimates (default: rigid, SE(3)).

    Returns
    -------
    AbsoluteError
        The per-pose centre errors, their statistics, and the alignment.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the trajectories are not equal-length batches of rigid transforms.
    slamlab.foundations.errors.DegenerateError
        If fewer than three poses are given, or the centres are collinear, so the alignment's
        rotation is undetermined.
    """
    est, tru = _as_trajectories(estimate, truth)
    fit = alignment.align(est[:, :3, 3], tru[:, :3, 3], with_scale=with_scale)
    aligned = fit.scale * est[:, :3, 3] @ fit.rotation.T + fit.translation
    errors = np.linalg.norm(aligned - tru[:, :3, 3], axis=1)
    transform = np.eye(4)
    transform[:3, :3] = fit.scale * fit.rotation
    transform[:3, 3] = fit.translation
    return AbsoluteError(
        errors=errors,
        rmse=_rms(errors),
        mean=float(np.mean(errors)),
        median=float(np.median(errors)),
        max=float(np.max(errors)),
        alignment=transform,
        scale=fit.scale,
    )


def relative_error(
    estimate: ArrayLike, truth: ArrayLike, *, delta: int = 1, scale: float = 1.0
) -> RelativeError:
    """Return the relative pose error over `delta` poses.

    Parameters
    ----------
    estimate, truth : array_like, shape (N, 4, 4)
        ``T_world_cam`` at the same instants, N > `delta`.
    delta : int, optional
        The interval, in poses (default 1: frame to frame).
    scale : float, optional
        Multiplies the estimate's relative translations: the ATE's Sim(3) scale for a
        monocular estimate (default 1).

    Returns
    -------
    RelativeError
        The per-pair drifts and their RMSEs.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the trajectories are not equal-length batches of rigid transforms, `delta` is not a
        positive integer below their length, or `scale` is not positive and finite.
    """
    est, tru = _as_trajectories(estimate, truth)
    if isinstance(delta, bool) or not isinstance(delta, int) or not 1 <= delta < len(est):
        raise InvalidInputError(f"delta: must be from 1 to {len(est) - 1}, got {delta}")
    if not (math.isfinite(scale) and scale > 0.0):
        raise InvalidInputError(f"scale: must be positive and finite, got {scale}")
    moved = se3.compose(se3.inverse(est[:-delta]), est[delta:])
    moved[:, :3, 3] *= scale
    true_moved = se3.compose(se3.inverse(tru[:-delta]), tru[delta:])
    error = se3.compose(se3.inverse(true_moved), moved)
    translation = np.linalg.norm(error[:, :3, 3], axis=1)
    rotation = np.linalg.norm(so3.log(error[:, :3, :3]), axis=1)
    return RelativeError(
        translation=translation,
        rotation=rotation,
        translation_rmse=_rms(translation),
        rotation_rmse=_rms(rotation),
        delta=delta,
    )


def associate(
    times: ArrayLike, reference: ArrayLike, max_difference: float
) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
    """Pair each time with the nearest reference time, as TUM's ``associate.py`` does.

    Parameters
    ----------
    times : array_like, shape (F,)
        The times to pair, ascending.
    reference : array_like, shape (G,)
        The reference times, ascending.
    max_difference : float
        The largest time difference accepted.

    Returns
    -------
    ours, theirs : ndarray of int64
        The paired indices into `times` and `reference`, ascending. A reference time is used
        once, by its nearest time (ties: the earlier).

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If an array is not 1D and finite, or `max_difference` is negative.
    """
    ours = np.asarray(times, dtype=np.float64)
    theirs = np.asarray(reference, dtype=np.float64)
    if (
        ours.ndim != 1
        or theirs.ndim != 1
        or not np.all(np.isfinite(ours))
        or not np.all(np.isfinite(theirs))
    ):
        raise InvalidInputError("times and reference: expected finite 1D arrays")
    if not (math.isfinite(max_difference) and max_difference >= 0.0):
        raise InvalidInputError(f"max_difference: must be non-negative, got {max_difference}")
    if len(ours) == 0 or len(theirs) == 0:
        empty = np.zeros(0, dtype=np.int64)
        return empty, empty
    after = np.clip(np.searchsorted(theirs, ours), 1, max(len(theirs) - 1, 1))
    before = np.maximum(after - 1, 0)
    after = np.minimum(after, len(theirs) - 1)
    nearest = np.where(np.abs(theirs[before] - ours) <= np.abs(theirs[after] - ours), before, after)
    gap = np.abs(theirs[nearest] - ours)
    candidates = np.flatnonzero(gap <= max_difference)
    # Each reference time keeps its closest pairing (ties: the earlier time).
    order = np.lexsort((candidates, gap[candidates], nearest[candidates]))
    ranked = candidates[order]
    first = np.ones(len(ranked), dtype=bool)
    first[1:] = nearest[ranked][1:] != nearest[ranked][:-1]
    kept = np.sort(ranked[first])
    return kept.astype(np.int64), nearest[kept].astype(np.int64)
