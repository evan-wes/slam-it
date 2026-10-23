"""Tests for slam_it.eval.loops: a run's loops and merges judged against its reference.

Moved from the test-set measure's tests with the judge (B3's review panel, run 2, H26 = A23).
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from slam_it.eval import loops, runs
from slam_it.foundations import so3
from tests.eval.synthetic_runs import line_run


def test_a_loop_is_judged_by_the_reference() -> None:
    """A loop over 3 m apart is at a wrong place; one whose keyframe turned 10 degrees is poor.

    The keyframes' poses are the run's own after it, placed on the reference by the trajectory's
    alignment; a loop closed between keyframes 2 m apart, the second turned, is in the right
    place with a poor pose.
    """
    run, truth = line_run(30)
    turned = run.keyframe_poses.copy()
    turned[1, :3, :3] = so3.exp([0.0, 0.0, np.radians(10.0)])
    run = replace(run, keyframe_poses=turned, keyframe_loops=np.array([[0, 1], [0, 3]]))
    scored = runs.evaluate_absolute(run, *truth)
    near, far = loops.judge_loops(run, truth, scored)
    assert near["keyframes"] == [0, 1]
    assert (near["wrong_place"], near["poor_pose"]) == (False, True)
    _, poses = truth
    assert near["distance"] == pytest.approx(np.linalg.norm(poses[10, :3, 3] - poses[0, :3, 3]))
    assert near["rotation_error"] == pytest.approx(10.0, abs=1e-6)
    assert (far["wrong_place"], far["poor_pose"]) == (True, False)
    assert far["distance"] > loops.WRONG_DISTANCE
    unjudged = loops.judge_loops(run, None, None)
    assert unjudged[0]["wrong_place"] is None
    assert unjudged[0]["poor_pose"] is None


def test_a_loop_turned_about_its_view_is_in_the_right_place() -> None:
    """Keyframes 1 m apart, one rolled 90 degrees about its viewing direction, saw one place.

    The judge measures the angle between the viewing directions, not the whole turn between
    the two keyframes (TUM's fr2_360_hemisphere: 88 degrees of turn, 18 between the views).
    """
    run, (times, poses) = line_run(30)
    rolled = poses.copy()
    rolled[10, :3, :3] = so3.exp([0.0, 0.0, np.radians(90.0)])
    run = replace(run, keyframe_loops=np.array([[0, 1]]))
    scored = runs.evaluate_absolute(run, times, rolled)
    (judged,) = loops.judge_loops(run, (times, rolled), scored)
    assert judged["angle"] == pytest.approx(0.0, abs=1e-6)
    assert judged["wrong_place"] is False


def test_a_merge_is_judged_as_a_loop_is() -> None:
    """A run's merges are judged by the same rules as its loops (milestone 2B's B4)."""
    run, truth = line_run(30)
    run = replace(run, keyframe_merges=np.array([[0, 1], [0, 3]], dtype=np.int64))
    scored = runs.evaluate_absolute(run, *truth)
    merges = loops.judge_pairs(run.keyframe_merges, run, truth, scored)
    as_loops = loops.judge_loops(replace(run, keyframe_loops=run.keyframe_merges), truth, scored)
    assert merges == as_loops
    assert [merge["wrong_place"] for merge in merges] == [False, True]
    assert loops.judge_pairs(np.zeros((0, 2), dtype=np.int64), run, truth, scored) == []
