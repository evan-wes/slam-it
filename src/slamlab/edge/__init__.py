"""Edge rung: sparse visual odometry for real-time use on edge devices (ORB-SLAM-lite).

Milestone 2's pipeline, in monocular, stereo and RGB-D modes, on rung 0's ORB front end:
tracking, keyframes and local bundle adjustment over a sliding window, as ORB-SLAM2 does them
without its covisibility graph, bag of words or second thread (design doc, "The edge rung is
ORB-SLAM-lite"; the plan is ``docs/design/2026-10-05-edge-vo.md``).

- :mod:`slamlab.edge.options`: every setting, ORB-SLAM's defaults.
- :mod:`slamlab.edge.frames`: a frame's features, bearings, depth and right columns, per mode.
- :mod:`slamlab.edge.map`: keyframes, their keypoints and the map points, a struct of arrays.

**Compute budget:** real time under ``laptop-cpu``, then ``jetson-class`` and
``pi-class``. CPU only; implemented in Python, C++ and Rust in lock-step. Must never import
PyTorch, a GPU stack or a depth model (enforced by import-linter). Builds only on
:mod:`slamlab.foundations`.
"""
