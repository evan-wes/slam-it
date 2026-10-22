"""The loops and merges a run made, judged against a reference: at a wrong place, or poor.

The online-SLAM design's section 10 holds a loop to one bar on the synthetic rooms and on real
data, so the judgement lives here, once, and the loop and merge rooms' tests and the test-set
measure (``scripts/measure_testset.py``) all call it (B3's review panel, A7). Each loop closed,
:attr:`slam_it.eval.runs.Run.keyframe_loops` (the older keyframe first), is judged by
:func:`judge_loops`, and each merge made, :attr:`slam_it.eval.runs.Run.keyframe_merges`, the same
way (:func:`judge_pairs`, milestone 2B's B4):

- **At a wrong place:** its two keyframes by the reference more than :data:`WRONG_DISTANCE`
  apart, or their viewing directions (their z axes) more than :data:`WRONG_ANGLE` apart: they
  cannot have seen the same place. A turn about the viewing direction leaves the place seen
  (TUM's fr2_360_hemisphere closes a loop across 88 degrees of turn, 18 between the views).
- **With a poor pose:** the map aligned on the reference as its ATE aligns it, the two
  keyframes' relative pose after the run, against the reference's, off by more than
  :data:`POOR_FACTOR` times the map's median error in translation, or by more than
  :data:`POOR_ANGLE` in rotation.

A judgement is null where the reference has no pose within
:data:`slam_it.eval.runs.MAX_DIFFERENCE` of a keyframe, or (the pose) where the map was not
scored.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import runs

WRONG_DISTANCE = 3.0
"""A loop's keyframes farther apart than this by the reference, in metres, are at a wrong place."""

WRONG_ANGLE = 60.0
"""A loop's keyframes looking further apart than this, in degrees, are at a wrong place."""

POOR_FACTOR = 5.0
"""A loop's relative pose missing by more than this many times its map's median error is poor."""

POOR_ANGLE = 5.0
"""A loop's relative pose turned by more than this, in degrees, from the reference's is poor."""


def _turn(rotation: NDArray[np.float64]) -> float:
    """Return a rotation's angle, in degrees."""
    return math.degrees(math.acos(float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))))


def _looking_apart(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    """Return the angle between two cameras' viewing directions (their z axes), in degrees."""
    return math.degrees(math.acos(float(np.clip(a[:3, 2] @ b[:3, 2], -1.0, 1.0))))


def judge_loops(
    run: runs.Run, truth: runs.Truth | None, scored: runs.Evaluation | None
) -> list[dict[str, Any]]:
    """Return each loop the run closed, judged against the reference (:func:`judge_pairs`).

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run.
    truth : slam_it.eval.runs.Truth or None
        The reference's times and ``T_world_cam``; None where there is none.
    scored : slam_it.eval.runs.Evaluation or None
        The run scored against `truth`; None where there is no truth or no map could be
        aligned to it.

    Returns
    -------
    list of dict
        Per loop, in the run's order, as :func:`judge_pairs` gives it.
    """
    return judge_pairs(run.keyframe_loops, run, truth, scored)


def judge_pairs(
    pairs: NDArray[np.int64],
    run: runs.Run,
    truth: runs.Truth | None,
    scored: runs.Evaluation | None,
) -> list[dict[str, Any]]:
    """Return each pair of keyframes a loop or a merge joined, judged against the reference.

    Parameters
    ----------
    pairs : ndarray of int64, shape (E, 2)
        The pairs, by the run's keyframe ids, the older first: its ``keyframe_loops`` or its
        ``keyframe_merges``.
    run : slam_it.eval.runs.Run
        The run.
    truth : slam_it.eval.runs.Truth or None
        The reference's times and ``T_world_cam``; None where there is none, which leaves
        every judgement null.
    scored : slam_it.eval.runs.Evaluation or None
        The run scored against `truth`; None where there is no truth or no map could be
        aligned to it, which leaves the poses' judgements null.

    Returns
    -------
    list of dict
        Per pair, in the given order: ``keyframes`` (the older first); ``distance``, in metres,
        and ``angle``, in degrees, between the two keyframes by the reference, and
        ``wrong_place``; ``translation_error``, in metres, and ``rotation_error``, in degrees,
        of their relative pose, and ``poor_pose`` (the module docstring).
    """
    aligned = {} if scored is None else {e.map: e.absolute for e in scored.maps}
    out: list[dict[str, Any]] = []
    for older, newer in np.asarray(pairs, dtype=np.int64).reshape(-1, 2).tolist():
        judged: dict[str, Any] = {
            "keyframes": [older, newer],
            "distance": None,
            "angle": None,
            "wrong_place": None,
            "translation_error": None,
            "rotation_error": None,
            "poor_pose": None,
        }
        out.append(judged)
        if truth is None:
            continue
        times, poses = truth
        rows = [int(np.argmin(np.abs(times - run.keyframe_timestamps[k]))) for k in (older, newer)]
        if any(
            abs(times[j] - run.keyframe_timestamps[k]) > runs.MAX_DIFFERENCE
            for j, k in zip(rows, (older, newer), strict=True)
        ):
            continue
        a, b = poses[rows[0]], poses[rows[1]]
        distance = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
        angle = _looking_apart(a, b)
        judged.update(
            distance=distance,
            angle=angle,
            wrong_place=distance > WRONG_DISTANCE or angle > WRONG_ANGLE,
        )
        absolute = aligned.get(int(run.keyframe_maps[older]))
        if absolute is None:
            continue
        # Each keyframe placed on the reference: its centre carried by the alignment, its
        # rotation turned by the alignment's; the pair's relative pose in the first's frame.
        turn = absolute.alignment[:3, :3] / absolute.scale
        centres = [
            absolute.alignment[:3, :3] @ run.keyframe_poses[k][:3, 3] + absolute.alignment[:3, 3]
            for k in (older, newer)
        ]
        rotations = [turn @ run.keyframe_poses[k][:3, :3] for k in (older, newer)]
        moved = float(
            np.linalg.norm(
                rotations[0].T @ (centres[1] - centres[0]) - a[:3, :3].T @ (b[:3, 3] - a[:3, 3])
            )
        )
        turned = _turn((rotations[0].T @ rotations[1]).T @ (a[:3, :3].T @ b[:3, :3]))
        judged.update(
            translation_error=moved,
            rotation_error=turned,
            poor_pose=moved > POOR_FACTOR * absolute.median or turned > POOR_ANGLE,
        )
    return out
