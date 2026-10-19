"""Serve the run viewer and a bundle together, on this machine, and open the page.

The page (``viewer/``) is served at ``/`` and the bundle (written by
:mod:`slam_it.orchestration.bundles`) at ``/bundle/``, which is where the page looks by
default. Only this machine can reach it (``127.0.0.1``). Stop it with Ctrl-C.

Example::

    python scripts/view_bundle.py ../bundles/MH01-stereo
"""

from __future__ import annotations

import argparse
import functools
import http.server
import webbrowser
from pathlib import Path

VIEWER = Path(__file__).resolve().parent.parent / "viewer"
"""The page's folder."""


class Handler(http.server.SimpleHTTPRequestHandler):
    """Serves the page's files, and the bundle's under ``/bundle/``."""

    def __init__(self, *args: object, bundle: Path, **kwargs: object) -> None:
        """Serve the page's folder; remember the bundle's."""
        self.bundle = bundle
        super().__init__(*args, directory=str(VIEWER), **kwargs)  # type: ignore[arg-type]

    def end_headers(self) -> None:
        """Tell the browser to check each file again: the page and bundles change while used."""
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def translate_path(self, path: str) -> str:
        """Map ``/bundle/...`` into the bundle's folder, anything else into the page's."""
        if path.startswith("/bundle/"):
            mapped = Path(super().translate_path("/" + path.removeprefix("/bundle/")))
            return str(self.bundle / mapped.relative_to(VIEWER))
        return super().translate_path(path)


def main() -> None:
    """Serve until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("bundle", type=Path, help="the bundle's folder")
    parser.add_argument("--port", type=int, default=8800, help="the port (default: 8800)")
    parser.add_argument("--no-browser", action="store_true", help="do not open the page")
    args = parser.parse_args()
    bundle = args.bundle.resolve()
    if not (bundle / "bundle.json").is_file():
        parser.error(f"{bundle}: no bundle.json")
    handler = functools.partial(Handler, bundle=bundle)
    server = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    url = f"http://127.0.0.1:{server.server_port}/"
    print(f"serving {bundle} at {url} (Ctrl-C to stop)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
