"""SLAM It All! A learning ladder for visual SLAM, in Python, C++ and Rust.

One package per rung of the ladder (see ``docs/design/``):

- :mod:`slam_it_all.foundations` -- rung 0: camera models, Lie groups, two-view
  geometry, the bundle-adjustment solver; shared by every other rung.
- :mod:`slam_it_all.edge`, :mod:`slam_it_all.vi`, :mod:`slam_it_all.classic`,
  :mod:`slam_it_all.dense`, :mod:`slam_it_all.learned` -- the rungs above it.
- :mod:`slam_it_all.eval`, :mod:`slam_it_all.orchestration` -- metrics and pipelines.

Every algorithm has a Python reference implementation; C++ and Rust implement a
subset, reached through each algorithm's ``backend=`` argument
(see :mod:`slam_it_all.backends`).
"""

from importlib.metadata import version

__version__: str = version("slam-it-all")

__all__ = ["__version__"]
