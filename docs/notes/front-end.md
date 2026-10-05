# The front end: images, ORB, matching and stereo

Design note for `slamlab.foundations.image`, `orb`, `matching` and `stereo` (the Python
references, mirrored in the C++ and Rust cores), and `slamlab.eval.images`, `tum`, `euroc`,
`metrics` and `runs`. Milestone 1 built the geometry; this is what turns pixels into the
correspondences it consumes, and what scores the trajectories milestone 2's pipeline will
produce.

## The problem

A visual odometry pipeline needs, for every frame, a few hundred to a thousand keypoints that
are spread over the image, repeatable across viewpoints and cheap to match; for a stereo pair,
their depths too. ORB-SLAM's front end is the reference design for a CPU budget: FAST corners
on an image pyramid, spread by a grid and a quadtree, each oriented and described by a rotated
binary test pattern (rBRIEF), matched by Hamming distance with a handful of filters.

What makes this PR different from milestone 1 is a requirement the geometry did not have:
**the three languages must find the same features.** A tracker that takes a different
decision in C++ than in Python cannot be compared step by step, and a front end is all
decisions: a corner exists or not, a descriptor bit is set or not.

## Exact by construction

Every image operation is integer arithmetic, so the three languages produce bit-identical
images, and so identical corners and descriptors.

| Operation | Definition |
|---|---|
| Gray | BT.601 in 15-bit integer weights, rounded half up: OpenCV's `cvtColor`, bit for bit |
| Blur | separable integer kernel summing to `2^b`; the two passes summed exactly, divided once |
| Resize | bilinear with aligned centres; the source coordinate an exact rational, its fraction rounded to 11 bits |
| Pyramid | level sizes `round(n / s^l)` with `s^l` by repeated multiplication; each level resized from the previous |
| Remap | coordinates rounded to a 32nd of a pixel; neighbours outside the image read as 0 |

Floating point could not promise this: a fused multiply-add rounds once where separate
operations round twice, and compilers fuse by default on ARM. Where floats remain, each is a
fixed sequence of IEEE operations on exact inputs (a level position's full-image coordinate,
`((2x + 1) n_0 - n_l) / (2 n_l)`; the stereo fit's sub-pixel offset), which every language
rounds alike; the C++ cores are compiled with `-ffp-contract=off`, so no compiler fuses them.

## ORB

`orb.detect` and `orb.extract` follow ORB-SLAM's `ORBextractor`:

- **FAST-9/16 scores**, the largest threshold at which a pixel is a corner, which reproduce
  OpenCV's `FastFeatureDetector` exactly (corner sets, non-maximum suppression and scores).
- **Cells of about 30 pixels**, each trying threshold 20 and falling back to 7, so
  low-texture regions still contribute; keypoints stay 19 pixels inside their level, so the
  rotated descriptor pattern (reach 18.4 pixels) never leaves it.
- **A quadtree** quarters the crowded regions until there are as many nodes as the level's
  share of the budget, and keeps each node's strongest corner.
- **Orientation** from the intensity centroid over a radius-15 disc, quantised to 30 bins of
  12 degrees (as the ORB paper discretises it), by fixed-point directions: each bin is the
  nearest to the centroid's angle.
- **rBRIEF**: OpenCV's learned 256-pair pattern (BSD, see `THIRD_PARTY_NOTICES`), rotated to
  the bin, on the level blurred by a 7-tap integer Gaussian. Given our keypoints and angles,
  OpenCV's ORB computes descriptors that differ from ours in 0.15 bits of 256 on average (its
  blur and rotation round in float): the pattern's orientation, sign and bit order are its.

Three deliberate departures from ORB-SLAM, each for determinism: non-maximum suppression runs
over the whole level rather than inside each cell (ORB-SLAM's per-cell FAST can keep two
adjacent corners across a cell edge); quadtree ties break by raster order rather than by
memory address; the orientation is a bin rather than a float angle.

A fourth, for accuracy: a keypoint's full-image `xy` is its level pixel carried exactly
through the pyramid's resizes, `((2x + 1) n_0 - n_l) / (2 n_l)` per axis
(`image.full_image_coordinate`). ORB-SLAM and OpenCV scale by `s^l`, which drops the resize's
half-pixel term and uses `s^l` for `n_0 / n_l`: a systematic offset of up to about 2 pixels on
a coarse level, in every observation a tracker then consumes. The features also record the
pyramid they span (`scale_factor` and `levels`), so stereo matching never has to be told it.

## Matching

`matching.match` is brute-force Hamming matching with ORB-SLAM's filters: a distance cap (50),
Lowe's ratio test (0.8), one match per second-set descriptor, and the rotation-consistency
histogram, which keeps only matches whose orientation change falls in the three most common
bins. With 12-degree orientation bins, a match's histogram bin is exactly
`(angle1 - angle2) mod 30`.

## Stereo

`stereo.rectify` is Bouguet's algorithm, as OpenCV's `stereoRectify`: each camera turns half
the relative rotation, then both turn the baseline onto the x axis. The rotations equal
OpenCV's to 1e-15. The rectified camera is one pinhole: the smallest input focal length, the
principal point at the image centre. `rectification_maps` maps each rectified pixel back
through each camera's own model, distortion included, for `image.remap`.

`stereo.rectify` refuses a pair whose right camera lies to the left of the left one (its
disparities would be negative, and matching would find only false matches); swap the cameras.

`stereo.match` follows ORB-SLAM's `ComputeStereoMatches`: candidates within a level and a row
band, the nearest descriptor, then a search along the row on the left keypoint's level and a
sub-pixel fit through the best three sums; matches whose best sum exceeds 2.1 times the median
are dropped. It checks every keypoint against the pyramid its features record: inside its
level, with `xy` exactly its level pixel's full-image coordinate. The departures (the module
docstring lists them all):

- **Zero-mean SAD and the V fit.** ORB-SLAM2 subtracts each patch's centre pixel before
  summing absolute differences, then fits a parabola. The centre is a FAST corner, the
  steepest point of its patch, so subtracting it shifts the whole patch by an amount that
  changes with the sub-pixel shift. On pairs whose two views are both area-sampled from one
  texture, that over-estimated disparity by a median 0.07 to 0.27 pixels (2.7% of depth at 10
  pixels), a systematic bias. ORB-SLAM3 drops the subtraction. slamlab sums `|n (L - R) - (sum L - sum R)|`, zero-mean SAD in integers (still
  invariant to a brightness offset between the cameras), and fits a V (equiangular; Shimizu
  and Okutomi, IJCV 2005), which suits absolute differences better than a parabola: the
  median error is under 0.002 pixels, and under 0.06 on every level.
- **Exact coordinates**, as for keypoints: the right keypoint's column is carried to the left
  keypoint's level as the nearest pixel to its centre, in integers, and the refined column
  back to the full image by the exact map.
- **Defined edge cases.** A non-positive disparity is dropped (ORB-SLAM clamps it to 0.01, a
  point near infinity); the outlier filter keeps matches equal to its limit, and every match
  when the median sum is 0 (ORB-SLAM drops them all); the search's bounds are checked as
  `u0 - L - w` (ORB-SLAM's `u0 + L - w` is a slip). The first smallest sum always has a
  larger left neighbour, so the fit's denominator is positive and its offset within half a
  pixel; ORB-SLAM's strict comparison guarantees the same, so neither needs a flat-valley
  check.

## Reading datasets

Images reach the algorithms as arrays; files are read in `slamlab.eval.images` (Pillow, the
one module allowed to import it), C++'s `slamlab_eval` (the vendored stb_image, its symbols
private to the library) and Rust's `slamlab-eval` (the `png` crate), each as `read_gray`,
`read_rgb` and `read_depth`, held to seven shared PNG fixtures. They refuse the same files,
in different kinds: the native readers report every failure, a missing file included, as
`InvalidInput` naming the file (the cores have no I/O kind), where Python raises
`FileNotFoundError` for a missing one; and stb_image skips PNG checksums, so a file damaged
only in its CRC decodes in C++ and is refused by the other two. `tum.read_depth` gives a
depth map in metres (NaN where nothing was measured); `euroc.read_sequence` reads the ASL
format's stereo frames, calibrations and ground truth, refusing non-finite values and
repeated timestamps. Both readers pair a frame with its nearest ground-truth sample through
`eval.timing`, the one nearest-time search the eval modules share.

## Scoring

`metrics.absolute_error` is the ATE after a Sim(3) (monocular) or SE(3) (stereo, RGB-D)
alignment, `metrics.relative_error` the RPE; both follow the TUM benchmark. The RPE pairs
poses by time, as TUM's `evaluate_rpe.py` does: each pose with the one nearest `delta` seconds
later, if one is within a tolerance, so a tracking gap is skipped rather than counted as one
step (paired by index, a 60-frame gap read as 60 frames' drift in one). `metrics.associate`
pairs an estimate's timestamps with the ground truth's as TUM's `associate.py` does, closest
first, each used once.

A run is a folder (`runs.write_run`): `run.json`, a TUM-format `trajectory.txt`, keyframes
and points as `.npy`, and per-frame diagnostics as JSON lines, written by every backend's
pipeline and scored by `runs.evaluate` (the ATE, and the RPE over one frame period).
`write_run` checks everything before writing and replaces each file whole; `read_run` refuses
a malformed file with `InvalidInputError`, naming it.

## Results

| Check | Result |
|---|---|
| Parity | images, FAST scores, keypoints, descriptors, matches and stereo matches bit-identical in Python, C++ and Rust on TUM and EuRoC frames and every fixture |
| OpenCV | gray exact; FAST exact; resize and blur within one intensity; descriptors 0.15 bits apart; rectifying rotations to 1e-15 |
| Two-view demo (TUM fr1/xyz, frames 0 and 30) | ORB: 237 matches, rotation 0.88 deg, translation direction 2.27 deg; SIFT before: 434, 0.63 and 2.32 |
| Absolute-pose survey (fr1/xyz) | ORB: 32 placed, median 0.77 deg and 0.98 cm; SIFT before: 31, 0.76 deg and 1.11 cm |
| EuRoC V1_01, MH_01 | 563 and 529 stereo matches on a rectified frame, at 1.2-2.8 m and 2.6-8.6 m (5th to 95th percentile depth) |
| Extraction time (640x480, 1000 features) | C++ 24 ms, Rust 35 ms, Python 160 ms |

The extraction times are first versions: making them fit the `laptop-cpu` budget is PR 9's
work, with the profiler.
