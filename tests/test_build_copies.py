"""The copies the builds keep of one another's settings, held equal (B3's review panel, H15).

The worker's image cannot run ``scripts/setup_slam_it.sh``, and rung 0's crate, which allows the
``unsafe`` its GTSAM bridge needs, cannot inherit the workspace's lints, so each keeps a copy.
"""

from __future__ import annotations

import re
import shlex
import tomllib
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def _match(pattern: str, text: str) -> str:
    """Return the first group of a pattern's one match in a text."""
    found = re.search(pattern, text, re.MULTILINE | re.DOTALL)
    assert found is not None, pattern
    return found.group(1)


def test_the_worker_image_builds_gtsam_as_the_setup_does() -> None:
    """The image's GTSAM is the setup's version, with its options, but without the wrapper.

    The image's Python takes PyPI's wrapper of the same version, pinned by its hash.
    """
    setup = (REPO / "scripts" / "setup_slam_it.sh").read_text()
    image = (REPO / "docker" / "worker.Dockerfile").read_text().replace("\\\n", " ")
    options = shlex.split(_match(r"^GTSAM_OPTIONS=\((.*?)\)$", setup))
    configure = _match(r"cmake -S /tmp/gtsam -B /tmp/gtsam/build -G Ninja (.*?)&& cmake", image)
    imaged = shlex.split(configure)
    assert "-DGTSAM_BUILD_PYTHON=ON" in options
    wrapperless = [o for o in options if o != "-DGTSAM_BUILD_PYTHON=ON"]
    assert sorted(imaged) == sorted([*wrapperless, "-DGTSAM_BUILD_PYTHON=OFF"])
    version = _match(r'^GTSAM_VERSION="([^"]+)"$', setup)
    assert f"--branch {version} https://github.com/borglab/gtsam.git" in image
    lock = (REPO / "scripts" / "locks" / "worker-gtsam.lock").read_text()
    assert re.search(rf"^gtsam=={re.escape(version)} --hash=sha256:[0-9a-f]{{64}}$", lock, re.M)


def test_rung_0s_crate_keeps_the_workspaces_lints_but_for_unsafe_code() -> None:
    """``slam-it-foundations`` denies ``unsafe``, which the workspace forbids; the rest is equal."""
    workspace = tomllib.loads((REPO / "rust" / "Cargo.toml").read_text())["workspace"]["lints"]
    crate = tomllib.loads((REPO / "rust" / "slam-it-foundations" / "Cargo.toml").read_text())
    lints = crate["lints"]
    assert (workspace["rust"].pop("unsafe_code"), lints["rust"].pop("unsafe_code")) == (
        "forbid",
        "deny",
    )
    assert lints == workspace
