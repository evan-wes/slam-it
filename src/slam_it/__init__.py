"""SLAM It! -- a learning ladder for visual SLAM, in Python, C++ and Rust.

One package per rung of the ladder (see ``docs/design/``):

- :mod:`slam_it.foundations` -- rung 0: camera models, Lie groups, two-view
  geometry, the bundle-adjustment solver; shared by every other rung.
- :mod:`slam_it.edge`, :mod:`slam_it.vi`, :mod:`slam_it.classic`,
  :mod:`slam_it.dense`, :mod:`slam_it.learned` -- the rungs above it.
- :mod:`slam_it.eval`, :mod:`slam_it.orchestration` -- metrics and pipelines.

Every algorithm has a Python reference implementation; C++ and Rust implement a
subset, reached through each algorithm's ``backend=`` argument
(see :mod:`slam_it.backends`).
"""

from importlib.metadata import version

__version__: str = version("slam-it")

__all__ = ["__version__"]
