"""Orchestration: end-to-end pipelines that run a rung on a sequence, and their runs baked.

The top layer: may use every rung and :mod:`slam_it_all.eval`; nothing imports it.

- :mod:`slam_it_all.orchestration.vo`: a sequence through the edge pipeline, in Python in-process
  or as any backend's program (Python's, C++'s or Rust's), its run folder written.
- :mod:`slam_it_all.orchestration.bundles`: one sequence's runs, baked with its images into a
  bundle for the run viewer (``viewer/``). Above ``vo``, never below it, so the program the
  budget runs time never carries the bake.
"""
