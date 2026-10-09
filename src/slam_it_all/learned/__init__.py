"""Learned rungs: DROID-style dense SLAM, then MegaSaM-style for dynamic scenes.

Correlation volumes, a recurrent flow update and a dense-BA layer, first with
DROID's published weights, then trained by us (design doc, milestones 5, 8, 9).

**Compute budget:** GPU (``desktop-gpu``), PyTorch. Builds on :mod:`slam_it_all.dense`
and :mod:`slam_it_all.foundations`. Depth models are inputs through a provider
interface, never imported here.

Stub: no code yet.
"""
