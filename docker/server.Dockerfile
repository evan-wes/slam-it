# The upload server's image (docs/design/2026-10-09-run-viewer.md, section 5.6): slim, holding
# slam_it_server and the pages it serves, but not slam_it, a compiler or a video decoder.
#
#   docker build -f docker/server.Dockerfile -t slam-it-server .
#
# Built from the repository's root. Its settings come from the environment (the variables in
# server/settings.template.env), and it listens on $PORT, else 8000, on IPv6 and IPv4 both.

# Python 3.11, as the env's lock; pinned by digest, so a rebuild is of the same base.
FROM python:3.11-slim-trixie@sha256:e88e9763f943ec1834f992a4b51e0f24500486803e8bc534e5767af9ea65f6ce

# The env's lock pins every Python package the server installs.
COPY scripts/locks/slam-it.lock /tmp/slam-it.lock
COPY server /opt/slam-it-server
RUN python -m pip install --no-cache-dir --constraint /tmp/slam-it.lock \
        /opt/slam-it-server \
    && rm -rf /tmp/slam-it.lock /opt/slam-it-server

# Only the files the server serves (slam_it_server.app.PAGES), and the showcase's page, which it
# fills in at startup (slam_it_server.app.showcase_page).
COPY viewer/index.html viewer/upload.html viewer/status.html viewer/app.css viewer/showcase.html \
     /opt/viewer/
COPY viewer/src /opt/viewer/src

ENV SLAM_IT_VIEWER=/opt/viewer PYTHONUNBUFFERED=1
RUN useradd --system --uid 10001 --no-create-home server
# Run as root (Railway runs an image whose volume it mounts as root), the entrypoint gives the job
# list's folder to the server's user and drops to it; run as that user, it just starts.
COPY docker/server-entrypoint.sh /usr/local/bin/slam-it-server-entrypoint
USER server
ENTRYPOINT ["/usr/local/bin/slam-it-server-entrypoint"]
CMD ["python", "-m", "slam_it_server", "--host", "::"]
