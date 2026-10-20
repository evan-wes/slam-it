"""The command line's paths reordered by ``tests/conftest.py``'s collection workaround."""

from __future__ import annotations

from types import SimpleNamespace
from typing import cast

import pytest

from tests.conftest import pytest_configure


def test_node_ids_are_collected_with_their_directory() -> None:
    """Node IDs sort by their file's path, each directory's together; the top levels keep order."""
    config = SimpleNamespace(
        args=["tests/edge/test_a.py::test_x", "tests/test_b.py", "tests/edge/test_c.py", "src"]
    )
    pytest_configure(cast(pytest.Config, config))
    assert config.args == [
        "tests/edge/test_a.py::test_x", "tests/edge/test_c.py", "tests/test_b.py", "src",
    ]  # fmt: skip
