"""slamlab: a learning ladder for visual SLAM, in Python, C++ and Rust.

Every algorithm has a Python reference implementation in this package. C++
(``slamlab._cpp``) and Rust (``slamlab_rs``) implementations of a subset are
reached through :mod:`slamlab.backends`.
"""

from importlib.metadata import version

from slamlab import backends

__version__: str = version("slamlab")

__all__ = ["__version__", "backends"]
