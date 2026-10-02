"""Unit tests for slamlab.foundations.ba.gauge: the remaining gauge, its name, its removal."""

from __future__ import annotations

import numpy as np
import pytest

from slamlab.foundations.ba import gauge
from slamlab.foundations.ba.gauge import Anchors, Frame

CENTRES = np.array(
    [[0.0, 0.0, 0.0], [0.4, 0.1, 0.0], [0.8, -0.1, 0.05], [1.2, 0.2, -0.05], [1.6, 0.0, 0.1]]
)
NONE = np.zeros(0, dtype=np.int64)


def _anchors(
    centres: np.ndarray,
    *,
    points: np.ndarray | None = None,
    centre_cams: tuple[int, ...] = (),
    orientation_cams: tuple[int, ...] = (),
    metric: bool = False,
) -> Anchors:
    """Return anchors over ``centres`` in the frame the layout would build."""
    pts = np.zeros((0, 3)) if points is None else points
    return Anchors(
        pts,
        np.array(centre_cams, dtype=np.int64),
        np.array(orientation_cams, dtype=np.int64),
        metric,
        Frame.of(np.concatenate([centres, pts])),
    )


@pytest.mark.parametrize(
    ("kwargs", "dimension", "name"),
    [
        ({}, 7, "similarity"),
        ({"metric": True}, 6, "first_pose"),
        ({"centre_cams": (0,), "orientation_cams": (0,)}, 1, "scale"),
        ({"centre_cams": (0,), "orientation_cams": (0,), "metric": True}, 0, "none"),
        ({"points": np.array([[0.0, 0.0, 6.0]])}, 4, "partial"),
        ({"points": np.array([[0.0, 0.0, 6.0], [1.0, 0.0, 6.0]])}, 1, "partial"),
        ({"orientation_cams": (0, 1)}, 4, "partial"),
    ],
    ids=[
        "nothing",
        "metric depth",
        "one fixed pose",
        "fixed pose and metric depth",
        "one fixed point",
        "two fixed points",
        "attitude priors",
    ],
)
def test_remaining_gauge(kwargs: dict[str, object], dimension: int, name: str) -> None:
    """Each kind of anchor removes the similarity's directions it pins, and no more."""
    null = gauge.remaining(_anchors(CENTRES, **kwargs), CENTRES)  # type: ignore[arg-type]
    assert null.shape == (gauge.GENERATORS, dimension)
    np.testing.assert_allclose(null.T @ null, np.eye(dimension), atol=1e-12)
    assert gauge.classify(null) == name


def test_complement_is_an_orthonormal_complement() -> None:
    """The basis is orthonormal and orthogonal to every normal."""
    rng = np.random.default_rng(0)
    q = np.linalg.qr(rng.normal(size=(6, 2)))[0]
    basis = gauge._complement([q[:, 0], q[:, 1]], 6)
    assert basis.shape == (6, 4)
    np.testing.assert_allclose(basis.T @ basis, np.eye(4), atol=1e-14)
    np.testing.assert_allclose(basis.T @ q, 0.0, atol=1e-14)


def test_plan_fixes_one_pose_and_confines_the_farthest() -> None:
    """Nothing anchored: the first pose is fixed, the farthest from it keeps its baseline."""
    chosen = gauge.plan(_anchors(CENTRES), CENTRES, np.arange(5))
    assert list(chosen.fixed) == [0]
    assert (list(chosen.confined), list(chosen.ranks), chosen.free) == ([4], [1], 0)


def test_plan_reports_what_no_pose_can_take() -> None:
    """A pose at the centre of the free scale cannot see it: the scale is left, and counted."""
    centres = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    anchors = _anchors(centres, centre_cams=(0,), orientation_cams=(0,))
    chosen = gauge.plan(anchors, centres, np.array([1]))
    assert (chosen.fixed.size, chosen.confined.size, chosen.free) == (0, 0, 1)
    assert gauge.plan(_anchors(CENTRES), CENTRES, NONE).free == gauge.GENERATORS


@pytest.mark.parametrize("offset", [1e3, 1e6])
def test_far_from_the_origin_nothing_changes(offset: float) -> None:
    """The rows are built in the scene's own frame: a geo-referenced offset changes nothing.

    In world coordinates the rotation and scale directions shrink like 1/|X|; the first
    version lost the gauge from about 5e4 m.
    """
    shifted = CENTRES + offset * np.array([1.0, 0.3, 0.0])
    near = gauge.plan(_anchors(CENTRES), CENTRES, np.arange(5))
    far = gauge.plan(_anchors(shifted), shifted, np.arange(5))
    assert (list(far.fixed), list(far.confined), list(far.ranks), far.free) == (
        list(near.fixed),
        list(near.confined),
        list(near.ranks),
        near.free,
    )
