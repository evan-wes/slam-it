"""Run the server: ``python -m slam_it_all_server [--host HOST] [--port PORT]``.

Its settings come from the environment (:mod:`slam_it_all_server.config`); the port defaults to
``PORT``, which Railway sets, else 8000. The host ``::`` listens on IPv6 and IPv4 both
(:func:`listening`), as the image does.

uvicorn's access log is off: a request's path holds a result's token or a signed link. The
server logs its own lines instead, naming jobs by their id (:mod:`slam_it_all_server.app`).
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import sys
from collections.abc import Sequence

import uvicorn

from slam_it_all_server.app import make_app
from slam_it_all_server.config import Settings, SettingsError, from_environment

SHUTDOWN = 3
"""How long a stopping server waits for requests still open, in seconds: a worker's claim is
held for up to its wait, and a redeploy should not wait for it."""


def listening(host: str, port: int) -> socket.socket:
    """Return a socket bound to a host and port; ``::`` takes IPv6 and IPv4 both.

    uvicorn binds ``::`` to IPv6 alone. Railway reaches a service's public port on IPv4, and its
    private network, which the worker uses, is IPv6 alone in a project made before 16 October
    2025, so the image's server listens on both.

    Parameters
    ----------
    host : str
        The address: an IPv4 one, or an IPv6 one such as ``::``.
    port : int
        The port.

    Returns
    -------
    socket.socket
        The socket, bound, for uvicorn to listen on.
    """
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    bound = socket.socket(family, socket.SOCK_STREAM)
    bound.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if family == socket.AF_INET6:
        bound.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
    bound.bind((host, port))
    return bound


def server_config(settings: Settings) -> uvicorn.Config:
    """Return how uvicorn runs the app: no access log, no server header, a short shutdown.

    Parameters
    ----------
    settings : Settings
        The server's settings.

    Returns
    -------
    uvicorn.Config
        The configuration.
    """
    return uvicorn.Config(
        make_app(settings),
        server_header=False,
        access_log=False,
        timeout_graceful_shutdown=SHUTDOWN,
    )


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
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s server %(levelname)s %(message)s", stream=sys.stderr
    )
    uvicorn.Server(server_config(settings)).run(sockets=[listening(args.host, args.port)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
