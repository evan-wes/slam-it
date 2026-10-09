"""Input validation across rung 0: every backend rejects bad input the same way.

Python validates before dispatching, so one table covers every module and backend.
"""

from __future__ import annotations

from types import ModuleType

import numpy as np
import pytest

from slam_it_all import backends
from slam_it_all.foundations import arrays, camera, se3, so3
from slam_it_all.foundations.errors import InvalidInputError
from tests.conftest import check_rejects

PINHOLE = [500.0, 500.0, 320.0, 240.0]
INVALID: list[tuple[ModuleType, str, tuple[object, ...], str]] = [
    (so3, "exp", ([1.0, 2.0],), "shape"),
    (so3, "exp", ([[1.0, 2.0, 3.0, 4.0]],), "shape"),
    (so3, "exp", ([None, 1.0, 2.0],), "real numbers"),
    (so3, "exp", (["1", "2", "3"],), "real numbers"),
    (so3, "exp", ([True, False, True],), "real numbers"),
    (so3, "exp", (np.array([1 + 2j, 0, 0]),), "real numbers"),
    (so3, "exp", ([np.nan, 0.0, 0.0],), "finite"),
    (so3, "exp", ([0.0, np.inf, 0.0],), "finite"),
    (so3, "exp", (np.ma.array([1.0, 2.0, 3.0], mask=[False, True, False]),), "masked"),
    (so3, "log", (np.full((3, 3), 0.5),), "orthonormal"),
    (so3, "log", (-np.eye(3),), "determinant"),
    (so3, "to_quaternion", (np.eye(3) * 1.1,), "orthonormal"),
    (so3, "vee", (np.eye(3),), "skew-symmetric"),
    (so3, "from_quaternion", ([1.0, 1.0, 0.0, 0.0],), "norm"),
    (so3, "right_jacobian_inverse", ([7.0, 0.0, 0.0],), "2\\*pi"),
    (so3, "left_jacobian_inverse", ([0.0, 0.0, -6.3],), "2\\*pi"),
    (se3, "exp", ([1.0, 2.0, 3.0],), "shape"),
    (se3, "exp", ([np.nan, 0, 0, 0, 0, 0],), "finite"),
    (se3, "log", (np.zeros((4, 4)),), "last row"),
    (se3, "log", (np.diag([1.0, 1.0, 1.0, 2.0]),), "last row"),
    (se3, "inverse", (np.diag([2.0, 1.0, 1.0, 1.0]),), "orthonormal"),
    (se3, "adjoint", (np.diag([1.0, 1.0, -1.0, 1.0]),), "determinant"),
    (se3, "right_jacobian_inverse", ([0, 0, 0, 7.0, 0, 0],), "2\\*pi"),
    (camera, "project", ("fisheye", PINHOLE, [[0.0, 0.0, 1.0]]), "unknown camera model"),
    (camera, "project", ("pinhole", [500.0, 500.0, 320.0], [[0.0, 0.0, 1.0]]), "expects shape"),
    (camera, "project", ("pinhole", [-500.0, 500.0, 320.0, 240.0], [[0, 0, 1.0]]), "positive"),
    (camera, "project", ("pinhole", PINHOLE, [[0.0, 0.0, -1.0]]), "in front"),
    (camera, "project", ("pinhole", PINHOLE, [[0.0, 0.0, 0.0]]), "in front"),
    (camera, "project_jacobians", ("pinhole", PINHOLE, [[1.0, 0.0, 0.0]]), "in front"),
    (camera, "unproject", ("pinhole", PINHOLE, [[1.0, 2.0, 3.0]]), "shape"),
]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("module", "function", "args", "message"),
    [
        pytest.param(m, f, b, msg, id=f"{m.__name__.rsplit('.', 1)[-1]}.{f}:{msg}")
        for m, f, b, msg in INVALID
    ],
)
def test_rejects_invalid_input(
    backend: str, module: ModuleType, function: str, args: tuple[object, ...], message: str
) -> None:
    """Every backend rejects bad input the same way: Python validates before dispatching."""
    check_rejects(module, function, args, message, backend)


def test_group_members_match_the_masks() -> None:
    """Each group's members, in order, as the per-group mask finds them; empty groups too."""
    group = np.array([2, 0, 2, 3, 0, 2], dtype=np.int64)
    members = arrays.group_members(group, 5)
    assert len(members) == 5
    for g, found in enumerate(members):
        np.testing.assert_array_equal(found, np.flatnonzero(group == g))
    empty = arrays.group_members(np.zeros(0, dtype=np.int64), 2)
    assert [m.size for m in empty] == [0, 0]
    with pytest.raises(InvalidInputError, match="outside"):
        arrays.group_members(np.array([0, 5], dtype=np.int64), 2)


_UNIT = np.full((8, 3), 1.0 / np.sqrt(3.0))
"""Eight unit bearings with positive z."""


@pytest.mark.parametrize(
    ("call", "message"),
    [
        (lambda: arrays.as_bearings(np.ones((2, 3))), "unit vectors"),
        (lambda: arrays.as_bearing_pairs(_UNIT, _UNIT[:7], 0), "equal"),
        (lambda: arrays.as_bearing_pairs(_UNIT, _UNIT, 9), "at least 9"),
        (lambda: arrays.as_bearing_pairs(-_UNIT, _UNIT, 0), "positive z"),
        (lambda: arrays.as_point_pairs(np.ones((4, 2)), np.ones((5, 2)), 0), "equal"),
        (lambda: arrays.as_point_pairs(np.ones((3, 2)), np.ones((3, 2)), 4), "at least 4"),
        (lambda: arrays.as_point_pairs(np.ones((3, 3)), np.ones((3, 3)), 0), "shape"),
        (lambda: arrays.as_matrix3(np.eye(4), "h"), "h"),
        (lambda: arrays.as_matrix3(np.stack([np.eye(3)] * 2), "h"), "one"),
    ],
)
def test_two_view_validators_reject_bad_input(call: object, message: str) -> None:
    """The shared correspondence checks reject shapes, sizes, non-unit and backward bearings."""
    assert callable(call)
    with pytest.raises(InvalidInputError, match=message):
        call()


def test_two_view_validators_return_copies() -> None:
    """Valid input comes back as C-contiguous float64 copies of the right shape."""
    b1, _ = arrays.as_bearing_pairs(_UNIT, _UNIT, 8)
    assert b1.shape == (8, 3)
    assert b1 is not _UNIT
    p1, _ = arrays.as_point_pairs([[0, 0], [1, 1]], [[1, 0], [0, 1]], 2)
    assert p1.dtype == np.float64
    assert p1.flags.c_contiguous
    assert arrays.as_matrix3(np.eye(3).tolist(), "m").shape == (3, 3)
