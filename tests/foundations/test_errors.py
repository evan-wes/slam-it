"""Tests for the shared error contract."""

from __future__ import annotations

import pytest

from slam_it_all.foundations import errors


@pytest.mark.parametrize(
    "error", [errors.InvalidInputError, errors.DegenerateError, errors.NotConvergedError]
)
def test_every_error_kind_is_a_slam_it_all_value_error(error: type[errors.SlamItAllError]) -> None:
    """Each error kind is a SlamItAllError, and so a ValueError callers can catch broadly."""
    assert issubclass(error, errors.SlamItAllError)
    assert issubclass(error, ValueError)
