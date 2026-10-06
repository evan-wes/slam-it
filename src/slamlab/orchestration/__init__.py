"""Orchestration: end-to-end pipelines that run a rung on a sequence.

The top layer: may use every rung and :mod:`slamlab.eval`; nothing imports it.

- :mod:`slamlab.orchestration.vo`: a sequence through the edge pipeline, in Python or a native
  program (C++ or Rust), its run folder written.
"""
