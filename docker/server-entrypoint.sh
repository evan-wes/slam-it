#!/bin/sh
# The upload server's entrypoint (docker/server.Dockerfile). Run as root, as Railway runs an
# image whose volume it mounts (RAILWAY_RUN_UID=0), it gives the job list's folder to the
# server's own user, then runs the server as that user; run as that user, it runs it as is.
set -eu
if [ "$(id -u)" = 0 ]; then
    folder="$(dirname "${SLAM_IT_ALL_DATABASE:?SLAM_IT_ALL_DATABASE is not set}")"
    mkdir -p "$folder"
    chown -R server:server "$folder"
    exec setpriv --reuid=server --regid=server --init-groups -- "$@"
fi
exec "$@"
