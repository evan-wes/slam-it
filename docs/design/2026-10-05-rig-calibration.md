# Rig calibration (a side track, Python and C++)

A side track beside milestone 2: calibrating a multi-sensor rig, an RGB-D camera, a 2D lidar
and a rangefinder, against each other, using the rig's own motion. A learning project in what
each sensor and each motion makes observable. Decided with evan-wes on 2026-10-05: Python
and C++, no Rust; the bag reader hand-written, no new dependency; the data and its config
outside the repo; run in its own worktree beside PR 8. The ladder's design doc
([2026-10-02-slamlab-ladder.md](2026-10-02-slamlab-ladder.md)) records where it sits; this
note is the track's plan.

**The data is private, and nothing in the repo describes it.** A recording and its config
(which topic is which sensor, and a description of the capture) live together outside the
repo; the repo holds the config's template, and every fixture is synthetic.

## 1. The method

The unknowns: the lidar's pose in the camera frame, `T_cam_lidar` (6); the rangefinder's
lever arm along its beam (1; the beam taken along the motion's axis, or its direction too,
2 more); the lidar's and the rangefinder's time offsets to the camera (2).

1. **The camera's trajectory:** rung 0's frame-to-frame PnP on depth to start; PR 8's RGB-D
   VO once it lands, through the export (section 2).
2. **The motion's axis and the rangefinder.** This needs motion along the rangefinder's
   beam. While the rig translates along it, the camera's displacement and the rangefinder's
   range change agree along that axis. Fitting them gives the axis in the camera frame and
   the time offset, and checks the depth scale. Where the depth sees the surface the
   rangefinder measures, that plane gives the camera's distance to it, so the rangefinder's
   lever arm.
3. **Stacking the scans:** each beam placed by its own time (a `LaserScan` is stamped at its
   first beam) and the trajectory (motion correction), so the scans taken while the rig
   moves build a 3D lidar cloud.
4. **Registration:** point-to-plane residuals of the lidar's points against the RGB-D map's
   surfaces, over `T_cam_lidar` and the lidar's time offset, with a robust loss, on
   `foundations.nls`.
5. **Validation, without ground truth:**
   - disjoint parts of the motion calibrate independently and should agree;
   - the lidar's points, overlaid on the depth images, land on surfaces;
   - the solve's covariance says which directions the data pin down (the observability
     report): scans of surfaces that do not change across the scan plane leave the lidar's
     offset across it free; motion across the plane, past edges, fixes it.

## 2. The pieces

### Reading the rig (Python, `slamlab.eval`)

- **`eval.rosbag`**, ROS 1 bag format 2.0, hand-written: the index (connections and chunk
  info), chunks uncompressed or bz2 (lz4 refused, by name), and `sensor_msgs` decoded into
  arrays: `Image` (`rgb8`, `mono8`, `16UC1`), `CompressedImage` (JPEG, through
  `eval.images`), `CameraInfo` (`plumb_bob` as `radtan` intrinsics), `LaserScan` (angles,
  ranges, per-beam times, validity), `Range`. Messages are read by topic and time window, a
  chunk at a time.
- **The rig config** (TOML, read with the standard library's `tomllib`): which topic is which
  sensor, outside the repo beside its recording; the repo holds its template.
- **The export:** the RGB-D stream to a TUM-format folder (`rgb/`, `depth/`, `rgb.txt`,
  `depth.txt`, and a `camera.txt` with the model, intrinsics, image size and depth scale),
  which PR 8's readers accept in all three languages (see
  [the edge VO design](2026-10-05-edge-vo.md)), so every backend's VO runs on the rig.

### Calibrating (`slamlab.calib`; C++ library `slamlab_calib`)

- **scans:** beams to points in the sensor frame, motion-corrected by per-beam time against
  a trajectory;
- **a voxel grid** for neighbour search, hand-written in both languages: Eigen has none, and
  SciPy's KD-tree would choose neighbours differently from C++;
- **surfaces:** normals and the fused RGB-D map;
- **the problems:** the motion's axis and the rangefinder; point-to-plane registration with a
  time offset; each on its language's `foundations.nls`;
- **the observability report**, from the solve's covariance.

**Placement:** an independent sibling of `edge` (`dense | edge | classic | calib`), taking
poses and points as arrays: it never imports `eval` or `edge`, and the study script
(`scripts/calibrate_rig.py`) joins them. Its generic parts move into rung 0, in all three
languages, only when a second rung needs them.

**Parity**, as rung 0's, Python against C++: integer decisions (voxel membership, neighbour
sets, inliers) exact, estimates to a tolerance.

## 3. Running beside PR 8

- **A second session** works the track in its own worktree,
  `../slamlab-calib`, its branches off `develop`, with its own conda
  env, `slamlab-calib`: an editable install points at one source tree, so two worktrees
  cannot share an env.
- **Offline CI's checkout is shared:** C1 adds a lock so that two runs wait for each other;
  until then, one run at a time.
- **The review panel** runs on the worktree, its `REPO` set to the worktree's path.
- **The PRs do not depend on PR 8's.** They meet in `VERSION`, `CHANGELOG.md`, the layer
  contract and the design doc, and whichever lands second merges `develop` first. C2's
  switch to the VO waits for PR 8 to merge: a script change, not a code dependency.

## 4. PRs

1. **C1, reading the rig** (Python): `eval.rosbag`, the rig config, the export; a synthetic
   bag fixture, written by a test generator; the offline-CI lock. PATCH.
2. **C2, calibrating the rig** (Python and C++): `slamlab.calib`, `scripts/calibrate_rig.py`,
   the note `docs/notes/rig-calibration.md` with the method and its results on synthetic
   rigs (results on a recording stay beside it, outside the repo). PATCH. If it grows too
   large, it splits into the geometry (C2) and the problems with the results (C3).

The panel runs once on each finished PR; offline CI gates each merge.
