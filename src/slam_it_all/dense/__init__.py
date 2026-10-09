"""Dense rung: dense bundle adjustment with a monocular depth prior and motion masks.

The back end of the Shape-of-Motion recipe; also refines reifyscene's cameras
(design doc, milestones 3 and 5).

**Compute budget:** GPU. Implemented in Python. Depth models are inputs, passed in
as arrays through a provider interface, never imported here. Independent of the edge
and classic branches; builds only on :mod:`slam_it_all.foundations`.

Stub: no code yet.
"""
