"""Edge rung: sparse visual odometry for real-time use on edge devices.

Features, tracking, keyframes and local bundle adjustment, ORB-SLAM-lite, with an
optional RGB-D mode taking predicted depth (design doc, milestones 2 and 7).

**Compute budget:** real time under ``laptop-cpu``, then ``jetson-class`` and
``pi-class``. CPU only; implemented in Python, C++ and Rust. Must never import
PyTorch, a GPU stack or a depth model (enforced by import-linter). Builds only on
:mod:`slamlab.foundations`.

Stub: no code yet.
"""
