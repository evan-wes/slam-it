"""Evaluation: readers, references and metrics for judging slamlab against the outside world.

- :mod:`slamlab.eval.readers`: dataset and reconstruction formats (BAL, COLMAP text), read
  into this package's conventions.
- :mod:`slamlab.eval.tum`: the TUM RGB-D benchmark's colour frames, ground-truth poses and
  calibrations. EuRoC joins it at milestone 2.
- :mod:`slamlab.eval.colmap_reference`: COLMAP's Ceres bundle adjuster, as the reference
  :mod:`slamlab.foundations.ba` is compared against (needs the ``colmap`` extra).
- :mod:`slamlab.eval.reprojection`: reprojection error, to compare solvers on one cost.

Still to come (design doc, "Metrics" and "Compute budgets"): ATE (after SE(3) or Sim(3)
alignment, as the rung warrants) and RPE, depth error, and the budget harness (per-budget
frame rate, latency and peak memory).

May use every rung; no rung may import it.
"""
