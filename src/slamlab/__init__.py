"""slamlab: a learning ladder for visual SLAM, in Python, C++ and Rust."""

from importlib.metadata import version

from slamlab import backends

__version__ = version("slamlab")

__all__ = ["__version__", "backends"]
