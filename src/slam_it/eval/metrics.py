"""Trajectory metrics: absolute trajectory error (ATE) and relative pose error (RPE).

Both follow Sturm et al.'s TUM RGB-D benchmark (IROS 2012), the convention ORB-SLAM, DROID and
most papers report. They compare an estimated trajectory with the ground truth, pose for pose:
the two arrays hold ``T_world_cam`` at the same instants (associate them by timestamp first).

- **ATE:** the estimate is aligned to the truth by the least-squares transform between their
  camera centres (Umeyama, :func:`slam_it.foundations.alignment.align`): Sim(3) for monocular
  runs, whose scale is unobservable, SE(3) for stereo and RGB-D. The error of each pose is the
  distance between its aligned centre and the true one; the headline is their RMSE.
- **RPE:** the drift over a fixed interval of time. Each pose ``i`` pairs with the pose ``j``
  nearest ``delta`` seconds later, as TUM's ``evaluate_rpe.py`` pairs them with a fixed delta.
  The tolerance is SLAM It!'s own: a pose with no pose within it of ``delta`` later (near the
  end, or before a tracking gap) is skipped, so a gap never counts as one step, where TUM's
  script pairs across it. The error is Sturm et al.'s ``(Q_i^-1 Q_j)^-1 (P_i^-1 P_j)``, ``Q``
  the truth and ``P`` the estimate (its translations multiplied by the ATE's scale for a
  monocular run), as evo computes it; TUM's script composes ``P_rel Q_rel^-1`` instead, the
  same rotation but a translation that differs at long intervals. Its translation's norm and
  rotation's angle are reported, with their RMSE. No alignment is needed: relative motion
  does not depend on the world frame. :func:`interval_pairs` gives the pairs alone.

:func:`associate` pairs an estimate's timestamps with the ground truth's first, as TUM's
``associate.py`` does.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it.eval import timestamps
from slam_it.foundations import alignment, se3, so3
from slam_it.foundations.arrays import as_batch, as_number
from slam_it.foundations.errors import InvalidInputError


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
    similarity : bool
        Whether it is a similarity, its scale estimated (Sim(3)), rather than rigid (SE(3)).
    """

    errors: NDArray[np.float64]
    rmse: float
    mean: float
    median: float
    max: float
    alignment: NDArray[np.float64]
    scale: float
    similarity: bool


@dataclass(frozen=True)
class RelativeError:
    """A relative pose error over a fixed interval of time.

    Attributes
    ----------
    pairs : ndarray of int64, shape (M, 2)
        The poses ``(i, j)`` compared, ``j`` about `delta` seconds after ``i``.
    translation : ndarray of float64, shape (M,)
        Each pair's translational drift, in the truth's units.
    rotation : ndarray of float64, shape (M,)
        Each pair's rotational drift, in radians.
    translation_rmse : float
        The translational drifts' root mean square.
    rotation_rmse : float
        The rotational drifts' root mean square, in radians.
    delta : float
        The interval, in seconds.
    """

    pairs: NDArray[np.int64]
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]
    translation_rmse: float
    rotation_rmse: float
    delta: float


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
    slam_it.foundations.errors.InvalidInputError
        If the trajectories are not equal-length batches of rigid transforms.
    slam_it.foundations.errors.DegenerateError
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
        similarity=with_scale,
    )


def interval_pairs(
    times: ArrayLike, *, delta: float, tolerance: float | None = None
) -> NDArray[np.int64]:
    """Return the pairs of poses `delta` seconds apart, which the relative pose error compares.

    Pose ``i`` pairs with the pose nearest ``times[i] + delta``, if that one is later and lies
    within `tolerance` of it, so no pair spans a gap in the poses.

    Parameters
    ----------
    times : array_like, shape (N,)
        The poses' instants, in seconds, strictly ascending, N >= 2.
    delta : float
        The interval, in seconds.
    tolerance : float, optional
        How far, in seconds, a pose may lie from ``times[i] + delta`` and still pair with
        pose ``i`` (default: half the median spacing of `times`).

    Returns
    -------
    ndarray of int64, shape (P, 2)
        Each pair's two indices into `times`, the first ascending; none if no pose has another
        `delta` seconds later.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `times` has fewer than two instants or does not ascend strictly, or `delta` or
        `tolerance` is out of range.
    """
    stamps = timestamps.as_times(times, "times")
    if len(stamps) < 2:  # an interval needs two poses
        raise InvalidInputError(f"times: expected at least 2, got {len(stamps)}")
    interval = as_number(delta, "delta")
    if not (math.isfinite(interval) and interval > 0.0):
        raise InvalidInputError(f"delta: must be positive and finite, got {interval}")
    window = 0.5 * float(np.median(np.diff(stamps))) if tolerance is None else tolerance
    window = as_number(window, "tolerance")
    if not (math.isfinite(window) and window >= 0.0):
        raise InvalidInputError(f"tolerance: must be non-negative and finite, got {window}")
    targets = stamps + interval
    later = timestamps.nearest(stamps, targets)
    first = np.flatnonzero(
        (np.abs(stamps[later] - targets) <= window) & (later > np.arange(len(stamps)))
    )
    return np.column_stack([first, later[first]]).astype(np.int64)


def relative_error(  # noqa: PLR0913 -- two trajectories, their times, the interval
    estimate: ArrayLike,
    truth: ArrayLike,
    times: ArrayLike,
    *,
    delta: float,
    tolerance: float | None = None,
    scale: float = 1.0,
) -> RelativeError:
    """Return the relative pose error over an interval of `delta` seconds.

    Parameters
    ----------
    estimate, truth : array_like, shape (N, 4, 4)
        ``T_world_cam`` at the same instants, N >= 2.
    times : array_like, shape (N,)
        Those instants, in seconds, strictly ascending.
    delta : float
        The interval, in seconds: one frame period gives the frame-to-frame drift.
    tolerance : float, optional
        How far, in seconds, a pose may lie from ``times[i] + delta`` and still pair with
        pose ``i`` (default: half the median spacing of `times`); see :func:`interval_pairs`.
    scale : float, optional
        Multiplies the estimate's relative translations: the ATE's Sim(3) scale for a
        monocular estimate (default 1).

    Returns
    -------
    RelativeError
        The per-pair drifts and their RMSEs.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the trajectories are not equal-length batches of rigid transforms, `times` does
        not ascend strictly or differs in length, `delta`, `tolerance` or `scale` is out of
        range, or no pose has another `delta` seconds later (:func:`interval_pairs` finds
        none).
    """
    est, tru = _as_trajectories(estimate, truth)
    stamps = timestamps.as_times(times, "times")
    if len(stamps) != len(est) or len(est) < 2:  # an interval needs two poses
        raise InvalidInputError(
            f"times: expected one per pose and at least 2, got {len(stamps)} for {len(est)}"
        )
    pairs = interval_pairs(stamps, delta=delta, tolerance=tolerance)
    factor = as_number(scale, "scale")
    if not (math.isfinite(factor) and factor > 0.0):
        raise InvalidInputError(f"scale: must be positive and finite, got {factor}")
    if len(pairs) == 0:
        raise InvalidInputError(f"no pose has another {delta} s later, within the tolerance")
    first, second = pairs[:, 0], pairs[:, 1]
    moved = se3.compose(se3.inverse(est[first]), est[second])
    moved[:, :3, 3] *= factor
    true_moved = se3.compose(se3.inverse(tru[first]), tru[second])
    error = se3.compose(se3.inverse(true_moved), moved)
    translation = np.linalg.norm(error[:, :3, 3], axis=1)
    rotation = np.linalg.norm(so3.log(error[:, :3, :3]), axis=1)
    return RelativeError(
        pairs=pairs,
        translation=translation,
        rotation=rotation,
        translation_rmse=_rms(translation),
        rotation_rmse=_rms(rotation),
        delta=float(delta),
    )


def associate(
    times: ArrayLike, reference: ArrayLike, max_difference: float
) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
    """Pair times with reference times as TUM's ``associate.py`` does: closest first.

    Every pair of a time and a reference time less than `max_difference` apart is a
    candidate. Candidates are taken in order of their gap (ties: the earlier time, then the
    earlier reference), each time and each reference time once, so a time whose nearest
    reference is taken still pairs with its next nearest within reach.

    Parameters
    ----------
    times : array_like, shape (F,)
        The times to pair, strictly ascending.
    reference : array_like, shape (G,)
        The reference times, strictly ascending.
    max_difference : float
        The largest time difference accepted, exclusive (TUM's ``<``).

    Returns
    -------
    ours, theirs : ndarray of int64
        The paired indices into `times` and `reference`, `ours` ascending.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If an array is not a finite 1D array that ascends strictly, or `max_difference` is
        negative.
    """
    ours = timestamps.as_times(times, "times")
    theirs = timestamps.as_times(reference, "reference")
    reach = as_number(max_difference, "max_difference")
    if not (math.isfinite(reach) and reach >= 0.0):
        raise InvalidInputError(f"max_difference: must be non-negative, got {reach}")
    # Each time's candidates are a run of references; widen each run by one on both sides so
    # the rounding of t -/+ reach loses none, then keep the gaps below reach exactly.
    low = np.maximum(np.searchsorted(theirs, ours - reach, side="right") - 1, 0)
    high = np.minimum(np.searchsorted(theirs, ours + reach, side="left") + 1, len(theirs))
    counts = np.maximum(high - low, 0)
    i = np.repeat(np.arange(len(ours)), counts)
    j = np.arange(counts.sum()) - np.repeat(np.cumsum(counts) - counts - low, counts)
    gap = np.abs(theirs[j] - ours[i])
    close = gap < reach
    i, j, gap = i[close], j[close], gap[close]
    used_ours = np.zeros(len(ours), dtype=bool)
    used_theirs = np.zeros(len(theirs), dtype=bool)
    pairs = []
    for k in np.lexsort((j, i, gap)):  # by gap, then time, then reference
        if not (used_ours[i[k]] or used_theirs[j[k]]):
            used_ours[i[k]] = used_theirs[j[k]] = True
            pairs.append((int(i[k]), int(j[k])))
    pairs.sort()
    matched = np.asarray(pairs, dtype=np.int64).reshape(-1, 2)
    return matched[:, 0].copy(), matched[:, 1].copy()
