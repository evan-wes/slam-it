"""slamlab: a learning ladder for visual SLAM, in Python, C++ and Rust.

One package per rung of the ladder (see ``docs/design/``):

- :mod:`slamlab.foundations` -- rung 0: camera models, Lie groups, two-view
  geometry, the bundle-adjustment solver; shared by every other rung.
- :mod:`slamlab.edge`, :mod:`slamlab.vi`, :mod:`slamlab.classic`,
  :mod:`slamlab.dense`, :mod:`slamlab.learned` -- the rungs above it.
- :mod:`slamlab.eval`, :mod:`slamlab.orchestration` -- metrics and pipelines.

Every algorithm has a Python reference implementation; C++ and Rust implement a
subset, reached through each algorithm's ``backend=`` argument
(see :mod:`slamlab.backends`).
"""

from importlib.metadata import version

__version__: str = version("slamlab")

__all__ = ["__version__"]
