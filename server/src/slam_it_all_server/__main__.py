"""Run the server: ``python -m slam_it_all_server [--host HOST] [--port PORT]``.

Its settings come from the environment (:mod:`slam_it_all_server.config`); the port defaults to
``PORT``, which Railway sets, else 8000. The host ``::`` listens on IPv6 and IPv4 both
(:func:`listening`), as the image does.

uvicorn's access log is off: a request's path holds a result's token or a signed link. The
server logs its own lines instead, naming jobs by their id (:mod:`slam_it_all_server.app`).

It refuses to run as root: the image's entrypoint (``docker/server-entrypoint.sh``) gives the
job list's folder to the server's own user, then runs it as that user.
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
from slam_it_all_server.jobs import SchemaError

SHUTDOWN = 3
"""How long a stopping server waits for requests still open, in seconds: a worker's claim is
held for up to its wait, and a redeploy should not wait for it."""


def listening(host: str, port: int) -> socket.socket:
    """Return a socket bound to a host and port; ``::`` takes IPv6 and IPv4 both.

    uvicorn binds ``::`` to IPv6 alone. The image's server takes IPv4, as before, and IPv6 too:
    Railway's private network, which the worker uses, is IPv6 alone in an environment made
    before 16 October 2025 (Railway's docs, "How private networking works").

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

    Raises
    ------
    slam_it_all_server.config.SettingsError
        If the viewer cannot be served.
    slam_it_all_server.jobs.SchemaError
        If the job list is not this server's.
    """
    return uvicorn.Config(
        make_app(settings),
        server_header=False,
        access_log=False,
        timeout_graceful_shutdown=SHUTDOWN,
    )


def main(arguments: Sequence[str]) -> int:
    """Serve until stopped; 2 if run as root, or a setting or the job list is wrong.

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
    if os.geteuid() == 0:
        print("slam_it_all_server: refusing to run as root", file=sys.stderr)
        return 2
    try:
        configured = server_config(from_environment())
    except (SettingsError, SchemaError) as error:
        print(f"slam_it_all_server: {error}", file=sys.stderr)
        return 2
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s server %(levelname)s %(message)s", stream=sys.stderr
    )
    uvicorn.Server(configured).run(sockets=[listening(args.host, args.port)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
