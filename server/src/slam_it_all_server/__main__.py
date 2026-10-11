"""Run the server: ``python -m slam_it_all_server [--host HOST] [--port PORT]``.

Its settings come from the environment (:mod:`slam_it_all_server.config`); the port defaults to
``PORT``, which Railway sets, else 8000.
"""

from __future__ import annotations

import argparse
import os
import sys
from collections.abc import Sequence

import uvicorn

from slam_it_all_server.app import make_app
from slam_it_all_server.config import SettingsError, from_environment


def main(arguments: Sequence[str]) -> int:
    """Serve until stopped; 2 if a setting is missing or malformed.

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, or 2.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it_all_server")
    parser.add_argument("--host", default="127.0.0.1", help="the address to listen on")
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", "8000")))
    args = parser.parse_args(arguments)
    try:
        settings = from_environment()
    except SettingsError as error:
        print(f"slam_it_all_server: {error}", file=sys.stderr)
        return 2
    uvicorn.run(make_app(settings), host=args.host, port=args.port, server_header=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
