"""scripts/view_bundle.py: the page at ``/``, the bundle at ``/bundle/``, nothing outside them."""

from __future__ import annotations

import functools
import http.server
import importlib.util
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType

import pytest

REPO = Path(__file__).resolve().parents[1]
GOLDEN = REPO / "tests" / "fixtures" / "orchestration" / "bundle"


def _script() -> ModuleType:
    """Import scripts/view_bundle.py as a module."""
    spec = importlib.util.spec_from_file_location("view_bundle", REPO / "scripts/view_bundle.py")
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def served() -> Iterator[str]:
    """Serve the page and the golden bundle on a free port; yield the URL."""
    handler = functools.partial(_script().Handler, bundle=GOLDEN)
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def _get(url: str) -> tuple[bytes, str | None]:
    """Fetch a URL: its bytes and its Cache-Control header."""
    with urllib.request.urlopen(url, timeout=10) as response:
        return response.read(), response.headers.get("Cache-Control")


def test_the_page_and_the_bundle_are_served_from_their_folders(served: str) -> None:
    """``/`` is the page's folder, ``/bundle/`` the bundle's; the browser checks each again."""
    page, cache = _get(f"{served}/")
    assert page == (REPO / "viewer" / "index.html").read_bytes()
    assert cache == "no-cache"
    info, _ = _get(f"{served}/bundle/bundle.json")
    assert info == (GOLDEN / "bundle.json").read_bytes()
    spans, _ = _get(f"{served}/bundle/images/spans.npy")
    assert spans == (GOLDEN / "images" / "spans.npy").read_bytes()


def test_a_path_out_of_the_bundle_stays_in_it(served: str) -> None:
    """``..`` cannot climb out of the bundle: the path is resolved inside it, and not found."""
    with pytest.raises(urllib.error.HTTPError, match="404"):
        _get(f"{served}/bundle/../../../tests/conftest.py")
