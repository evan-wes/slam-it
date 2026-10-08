"""Evaluation: readers, references and metrics for judging slamlab against the outside world.

- :mod:`slamlab.eval.readers`: dataset and reconstruction formats (BAL, COLMAP text), read
  into this package's conventions.
- :mod:`slamlab.eval.tum`: the TUM RGB-D benchmark's colour frames, depth maps, ground-truth
  poses and calibrations.
- :mod:`slamlab.eval.euroc`: the EuRoC MAV datasets' stereo frames, ground-truth poses and
  calibrations.
- :mod:`slamlab.eval.images`: reading image files (the one module that imports Pillow; C++'s
  ``slamlab_eval`` library and Rust's ``slamlab-eval`` crate read PNGs natively and the same
  way).
- :mod:`slamlab.eval.metrics`: the absolute trajectory error (after SE(3) or Sim(3)
  alignment) and the relative pose error, as the TUM benchmark defines them, and associating
  timestamps.
- :mod:`slamlab.eval.runs`: the run format every backend's pipeline writes, and scoring a run.
- :mod:`slamlab.eval.timestamps`: checking timestamps and finding the nearest, for the readers
  and metrics.
- :mod:`slamlab.eval.colmap_reference`: COLMAP's Ceres bundle adjuster, as the reference
  :mod:`slamlab.foundations.ba` is compared against (needs the ``colmap`` extra).
- :mod:`slamlab.eval.reprojection`: reprojection error, to compare solvers on one cost.
- :mod:`slamlab.eval.budgets`: compute budgets: a program's peak memory, and (PR 9) running
  it within a budget and judging its frames against their deadline.

Still to come (design doc, "Metrics"): depth error.

May use every rung; no rung may import it.
"""
