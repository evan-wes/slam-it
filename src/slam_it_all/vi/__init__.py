"""Visual-inertial rung: IMU preintegration and tight visual-inertial optimisation.

Builds on the edge rung's sparse VO (design doc, milestone 6).

**Compute budget:** edge budgets. CPU only; implemented in Python and C++. Must never
import PyTorch, a GPU stack or a depth model (enforced by import-linter). May build
on :mod:`slam_it_all.edge` and :mod:`slam_it_all.foundations` only.

Stub: no code yet.
"""
