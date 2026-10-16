"""Edge rung: sparse visual odometry for real-time use on edge devices (ORB-SLAM-lite).

Milestone 2's pipeline, in monocular, stereo and RGB-D modes, on rung 0's ORB front end:
tracking, keyframes and local bundle adjustment over a sliding window, as ORB-SLAM2 does them
without its covisibility graph or second thread (design doc, "The edge rung is ORB-SLAM-lite";
the plan is ``docs/design/2026-10-05-edge-vo.md``); from milestone 2B's B1, a frame tracking
fails is relocalised against the whole map through a bag of words, as ORB-SLAM's is
(``docs/design/2026-10-14-online-slam.md``).

- :mod:`slam_it.edge.options`: every setting, ORB-SLAM's defaults.
- :mod:`slam_it.edge.frames`: a frame's features, bearings, depth and right columns, per mode.
- :mod:`slam_it.edge.map`: keyframes, their keypoints and the map points, a struct of arrays.
- :mod:`slam_it.edge.database`: the keyframe database, each keyframe's words and the inverted
  index from each word to its keyframes (ORB-SLAM's ``KeyFrameDatabase``).
- :mod:`slam_it.edge.residuals`: each level's variance and the chi-square outlier test.
- :mod:`slam_it.edge.adjust`: bundle adjustment over keyframes and the points they observe.
- :mod:`slam_it.edge.initialization`: starting the map, from depth or from two views.
- :mod:`slam_it.edge.tracking`: placing each frame against the map, and the fallback.
- :mod:`slam_it.edge.relocalization`: placing a frame the fallback fails in the whole map,
  through the database's candidates (ORB-SLAM's ``Tracking::Relocalization``).
- :mod:`slam_it.edge.mapping`: growing and cleaning the map around each new keyframe.
- :mod:`slam_it.edge.pipeline`: the per-frame loop over them, holding the map.

**Compute budget:** real time under ``laptop-cpu``, then ``jetson-class`` and
``pi-class``. CPU only; implemented in Python, C++ and Rust in lock-step. Must never import
PyTorch, a GPU stack or a depth model (enforced by import-linter). Builds only on
:mod:`slam_it.foundations`.
"""
