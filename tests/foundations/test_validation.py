"""Input validation across rung 0: every backend rejects bad input the same way.

Python validates before dispatching, so one table covers every module and backend.
"""

from __future__ import annotations

from types import ModuleType

import numpy as np
import pytest

from slamlab import backends
from slamlab.foundations import arrays, camera, se3, so3
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
