"""Rung 0, foundations: the geometry every other rung builds on.

Camera models, SO(3)/SE(3)/Sim(3), two-view geometry, RANSAC, triangulation, PnP,
and the bundle-adjustment solver (design doc, milestone 1); the front end, fixed-point
images, ORB features, Hamming matching and rectified stereo (milestone 2, PR 7); and place
recognition's vocabulary, a tree of binary words over those features, with matching by word
(milestone 2B's B1). The front end and the vocabulary are here rather than in the edge rung so
that the classic rung matches and recognises with the same features.

**Compute budget:** CPU only, every budget down to ``pi-class``. Implemented in
Python, C++ and Rust. Must never import PyTorch, a GPU stack or a depth model
(enforced by import-linter).
"""
