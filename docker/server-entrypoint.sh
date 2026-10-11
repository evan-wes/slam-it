#!/bin/sh
# The upload server's entrypoint (docker/server.Dockerfile). Run as root, as Railway runs an
# image whose volume it mounts (RAILWAY_RUN_UID=0), it gives the job list's folder to the
# server's own user, then runs the server as that user, unable to gain privileges again; run as
# that user, it runs it as is. The path must be absolute and its folder not / itself: it is
# chowned.
set -eu
if [ "$(id -u)" = 0 ]; then
    database="${SLAM_IT_ALL_DATABASE:?SLAM_IT_ALL_DATABASE is not set}"
    folder="$(realpath -m "$(dirname "$database")")"
    case "$database:$folder" in
        /*:/?*) ;;
        *)
            echo "server-entrypoint: SLAM_IT_ALL_DATABASE must be absolute, below /: $database" >&2
            exit 2
            ;;
    esac
    mkdir -p "$folder"
    chown -R server:server "$folder"
    exec setpriv --reuid=server --regid=server --init-groups --no-new-privs -- "$@"
fi
exec "$@"
