"""Tests for the shared error contract."""

from __future__ import annotations

import pytest

from slam_it.foundations import errors


@pytest.mark.parametrize(
    "error", [errors.InvalidInputError, errors.DegenerateError, errors.NotConvergedError]
)
def test_every_error_kind_is_a_slam_it_value_error(error: type[errors.SlamItError]) -> None:
    """Each error kind is a SlamItError, and so a ValueError callers can catch broadly."""
    assert issubclass(error, errors.SlamItError)
    assert issubclass(error, ValueError)
