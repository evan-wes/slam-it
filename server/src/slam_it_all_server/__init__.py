"""SLAM It All!'s upload server: the job list, the guards, and the upload, status and viewer pages.

Docs/design/2026-10-09-run-viewer.md, section 5.3. It never imports :mod:`slam_it_all`: the
process holding the bucket's keys never parses a video. Its modules come with PR 11's server
commit.
"""
