# A dashcam's recordings: drives, mask, overlay, calibration, import

Design note for `slam_it.eval.dashcam` and `slam_it.eval.gnss` (2026-10-17): a front and rear
dashcam's card turned into sequences a run can take, a GPS track to score them against, and
the two cameras calibrated as a rig. Built for the Terunsoul D016, the one profile built in
(`dashcam.d016`); another dashcam is another profile beside it.

Nothing about the owner's car or drives is in the repo: the card's copy, the masks, the
lenses, the rig and every result live with the recordings, named by a config whose template
is `docs/dashcam.template.toml`. The tests are synthetic.

## The D016's quirks

What the recordings showed, which the code is shaped by:

- **Both cameras at 4K overrun the card**, so both record at 2560x1440. The overlay's box is
  known for both sizes, 3840x2160 and 2560x1440 (`d016.OVERLAY_BOXES`); a frame of another
  size is refused rather than its box guessed.
- **Each camera records short segments** (`video/F/YYYYMMDD_HHMMSSF.ts`, `video/R/...R.ts`),
  MPEG-TS, named by their start on the camera's clock; a drive is many of them, front and rear
  starting within a few seconds of each other.
- **The data stream's coordinates are scrambled.** Each segment carries a 160-byte GPS record
  about each second (`d016.decode_record`): its UTC time, fix flag, speed (knots) and course
  read sensibly, but its float64 latitude and longitude move where the true fix does not. So
  the fix comes from the overlay, which prints it, and the records are a second source of
  speed and course (`dashcam.stream`).
- **The overlay clocks differ between channels.** Each camera burns in its own clock, the two
  not in step, and neither is its segments' names' clock: each camera's lag is measured from
  its own overlay, and the rear's offset from the front's from the two cameras' motion.
- **The rear camera has a different lens** from the front's, so each is calibrated alone.

## The steps

`python -m slam_it.eval.dashcam <step> CONFIG`, each writing into the config's work folder, a
folder a drive named by its id (its first front segment's start):

1. **`drives`**: a card's segments grouped into drives, a new one wherever the front camera's
   starts leave a gap of more than 6 minutes, each with the rear segments that start within it.
2. **`mask`**: a camera's static mask, from minutes of driving. An edge (a 3x3 blur's
   gradient) that sits on a pixel in more than 40% of frames sampled a second apart is the
   car's or the overlay's, whatever the light; specks are dropped, the rest closed and grown,
   and the config's drawn parts added (a band across the bottom, a thin line too soft to
   measure). numpy only: the library imports neither OpenCV nor SciPy. A preview, the mask
   darkened over a frame, is written to judge it by eye.
3. **`overlay`**: each camera's overlay read with Tesseract, five reads a second
   (`dashcam.overlay`):
   - the overlay's white (bright in every channel, nearly grey) ANDed over three frames,
     since the text holds still within its second while the scene moves, then read with the
     overlay's characters only;
   - next to the clock's tick the AND mixes two seconds' digits and reads the strokes they
     share, often the old second's, which would time the tick a frame late: a frame whose
     clock reads other than a neighbour's is read again, alone;
   - the lag: a read of second `N` at video time `t` puts the lag in
     `[N - start - t, N - start - t + 1)`, and the lag is the middle of the span most reads
     share. A median of the reads' own values would be half a second early, since the clock
     shows the second that has begun; the shared span narrows, over a drive, to the time
     between reads. A read far from it is a misread;
   - each second's fix is the one most of its reads agree on (range checked, dated as its
     segment).

   `eval.gnss` turns the fixes into a track: metres east and north, a constant-velocity Kalman
   filter whose fixes are refused past a χ² bound of 13.82 (2 degrees of freedom, 99.9%),
   started from a Theil-Sen line through the first five fixes, letting its state go and taking
   the third of three fixes in a row it would refuse (it, not they, has lost the car), then a
   Rauch-Tung-Striebel smoother.
4. **`calibrate`** (`dashcam.calibrate`): each camera alone by COLMAP (pycolmap): frames 4 a
   second at 1600 wide, named by their time on the camera's clock; SIFT with the static mask;
   sequential matching only, each frame to its next 10 (a garage's levels look alike, and
   matching by appearance across them folds a spiral onto itself); incremental SfM with the
   `OPENCV_FISHEYE` lens self-calibrated. Then the rig, from each camera's own motion, as
   hand-eye calibration does it:
   - **the clocks**: the rear's offset is where the two cameras' turns over 1 s correlate best,
     searched ±3 s and refined to 10 ms;
   - **the rotation**: each pair of 1 s motions `A` (front) and `B` (rear) obeys `A X = X B`, so
     the turning pairs' rotation axes obey `a = R_X b`. A drive turns about the vertical almost
     only, which leaves the rotation about the vertical open; the straight pairs' directions of
     travel (the same displacement, in each camera's frame) close it, as camera-to-odometry
     calibration closes it from the translations (Guo et al., ICRA 2012). Kabsch fits both;
   - **the translation and the rear map's scale**: `(R_A - I) t - s R_X t_B = -f t_A` is linear,
     the front map's `f` from the GPS track's speed (the median over windows above 2 m/s), with
     a Gaussian prior on `t` at the measured baseline (σ 0.3, 0.3, 0.15 m), since a level drive
     leaves `t`'s vertical part weakly seen. Linear and Gaussian, so its maximum a posteriori
     estimate and covariance are closed form.

   `calibration.json` reports each lens at its video's size (and whether `fx = fy` fits, to
   1%), the clock offset and its correlation, the rotation with its residuals, the
   translation with its standard deviations, and the scales; the config's lens tables take
   the lenses from it, by hand, after a look.
5. **`import`**: one camera's share of a drive, or a window of it, as one TUM-format sequence
   (`dashcam.ingest`, through `eval.video.import_segments`): decoded straight through, each
   frame timed by its segment's start plus the camera's lag plus its time in the segment, so
   the run and the GPS track share a clock and a gap between segments stays a gap; with the
   config's lens and mask. No upload cap, only limits (20,000 frames by default), which the
   config or `--max-frames` widen.

`eval.gnss.score` then scores a run's maps against the track in each map's plane of motion: its
positions in their two leading principal axes, a 2D similarity aligned to the track (both
mirror images tried), the errors in metres (RMSE, median, 90th percentile).

## What is not done yet

- **A rig bundle adjustment.** GTSAM arrives with milestone 2B's B3, not yet on `develop`; the
  translation and scale are the closed-form linear MAP above, the rotation Kabsch's, and the
  lenses COLMAP's per camera. Refining all of them together, with SLAM It!'s own maps, is the
  next step once GTSAM is there.
- **The receiver's latency.** A fix is taken to be where the car was at its second's start; the
  receiver's own delay, if any, is not measured.
- **A real drive.** Every test is synthetic: segments PyAV writes with a burned-in overlay and a
  static hood, Tesseract reading a test font, and rig trajectories whose clocks, rotation,
  translation and scales are known. The first real drive is where the thresholds (the mask's
  share, the overlay's white, the minimum turn) are checked.
