"""Rung 0, foundations: the geometry every other rung builds on.

Camera models, SO(3)/SE(3)/Sim(3), two-view geometry, RANSAC, triangulation, PnP,
and the bundle-adjustment solver (design doc, milestone 1).

**Compute budget:** CPU only, every budget down to ``pi-class``. Implemented in
Python, C++ and Rust. Must never import PyTorch, a GPU stack or a depth model
(enforced by import-linter).
"""
