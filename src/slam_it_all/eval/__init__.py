"""Evaluation: readers, references and metrics for judging SLAM It All! against the outside world.

- :mod:`slam_it_all.eval.readers`: dataset and reconstruction formats (BAL, COLMAP text), read
  into this package's conventions.
- :mod:`slam_it_all.eval.tum`: the TUM RGB-D benchmark's colour frames, depth maps, ground-truth
  poses and calibrations.
- :mod:`slam_it_all.eval.euroc`: the EuRoC MAV datasets' stereo frames, ground-truth poses and
  calibrations.
- :mod:`slam_it_all.eval.images`: reading and writing image files, and encoding the run viewer's
  JPEGs (the one module that imports Pillow; C++'s ``slam_it_all_eval`` library and Rust's
  ``slam-it-all-eval`` crate read PNGs natively and the same way).
- :mod:`slam_it_all.eval.video`: importing a phone's recording, a video or a Stray Scanner zip,
  into a TUM-format folder (the one module that imports PyAV, an optional extra).
- :mod:`slam_it_all.eval.lenses`: a phone video's intrinsics, from the best source that has
  them (a Gyroflow lens profile, the 35mm-equivalent focal length, a nominal field of view),
  and undistorting a fisheye lens to pinhole.
- :mod:`slam_it_all.eval.metrics`: the absolute trajectory error (after SE(3) or Sim(3)
  alignment) and the relative pose error, as the TUM benchmark defines them, and associating
  timestamps.
- :mod:`slam_it_all.eval.runs`: the run format every backend's pipeline writes, a sequence's
  ground truth, and scoring a run.
- :mod:`slam_it_all.eval.timestamps`: checking timestamps and finding the nearest, for the readers
  and metrics.
- :mod:`slam_it_all.eval.colmap_reference`: COLMAP's Ceres bundle adjuster, as the reference
  :mod:`slam_it_all.foundations.ba` is compared against (needs the ``colmap`` extra).
- :mod:`slam_it_all.eval.reprojection`: reprojection error, to compare solvers on one cost.
- :mod:`slam_it_all.eval.budgets`: compute budgets: running a program within one, judging its
  frames against their deadline, and what a program records of its own process (its peak
  memory, the CPUs it could run on, its memory cap).

Still to come (design doc, "Metrics"): depth error.

May use every rung; no rung may import it.
"""
