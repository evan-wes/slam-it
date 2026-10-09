"""Orchestration: end-to-end pipelines that run a rung on a sequence.

The top layer: may use every rung and :mod:`slam_it_all.eval`; nothing imports it.

- :mod:`slam_it_all.orchestration.vo`: a sequence through the edge pipeline, in Python in-process
  or as any backend's program (Python's, C++'s or Rust's), its run folder written.
"""
