"""SLAM It!'s upload server: the job list, the guards, and the upload, status and viewer pages.

docs/design/2026-10-09-run-viewer.md, section 5.3. It never imports :mod:`slam_it`: the
process holding the bucket's keys never parses a video. Its modules, bottom to top:

- :mod:`~slam_it_server.config`: the settings, from the environment, and the caps;
- :mod:`~slam_it_server.jobs`: the job list, in SQLite;
- :mod:`~slam_it_server.storage`: an S3 bucket, or a folder on this machine, and its links;
- :mod:`~slam_it_server.app`: the routes and the guards;
- ``__main__``: ``python -m slam_it_server``, under uvicorn.
"""
