"""Tests for the shared error contract."""

from __future__ import annotations

import pytest

from slamlab.foundations import errors


@pytest.mark.parametrize(
    "error", [errors.InvalidInputError, errors.DegenerateError, errors.NotConvergedError]
)
def test_every_error_kind_is_a_slamlab_value_error(error: type[errors.SlamlabError]) -> None:
    """Each error kind is a SlamlabError, and so a ValueError callers can catch broadly."""
    assert issubclass(error, errors.SlamlabError)
    assert issubclass(error, ValueError)
