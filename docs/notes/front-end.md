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
operations round twice, and compilers fuse by default on ARM. Where floats remain, each is one
IEEE operation on exact inputs (a level coordinate times a power of the scale factor; the
stereo parabola's vertex), which every language rounds alike.

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

`stereo.match` follows ORB-SLAM's `ComputeStereoMatches`: candidates within a level and a row
band, the nearest descriptor, then a sum-of-absolute-differences search along the row on the
left keypoint's level, and a parabola through the best three sums for the sub-pixel disparity;
matches whose best sum exceeds 2.1 times the median are dropped. On a synthetic pair shifted
by 9.6 pixels, the median disparity is within a third of a pixel: the parabola is pulled
toward whole pixels ("pixel locking"), a known bias of this refinement that ORB-SLAM shares.
Where ORB-SLAM's code is loose (a flat SAD valley divides by zero and the NaN passes its
range check), the cases are defined: a flat valley is rejected.

## Reading datasets

Images reach the algorithms as arrays; files are read in `slamlab.eval.images` (Pillow, the
one module allowed to import it), C++'s `slamlab_eval` (the vendored stb_image) and Rust's
`slamlab-eval` (the `png` crate), held to seven shared PNG fixtures. `tum.read_depth` gives a
depth map in metres (NaN where nothing was measured); `euroc.read_sequence` reads the ASL
format's stereo frames, calibrations and ground truth.

## Scoring

`metrics.absolute_error` is the ATE after a Sim(3) (monocular) or SE(3) (stereo, RGB-D)
alignment, `metrics.relative_error` the RPE; both follow the TUM benchmark. A run is a folder
(`runs.write_run`): `run.json`, a TUM-format `trajectory.txt`, keyframes and points as `.npy`,
and per-frame diagnostics as JSON lines, written by every backend's pipeline and scored by
`runs.evaluate`.

## Results

| Check | Result |
|---|---|
| Parity | images, FAST scores, keypoints, descriptors, matches and stereo matches bit-identical in Python, C++ and Rust on TUM and EuRoC frames and every fixture |
| OpenCV | gray exact; FAST exact; resize and blur within one intensity; descriptors 0.15 bits apart; rectifying rotations to 1e-15 |
| Two-view demo (TUM fr1/xyz, frames 0 and 30) | ORB: 237 matches, rotation 0.92 deg, translation direction 2.17 deg; SIFT before: 434, 0.63 and 2.32 |
| Absolute-pose survey (fr1/xyz) | ORB: 32 placed, median 0.73 deg and 1.07 cm; SIFT before: 31, 0.76 deg and 1.11 cm |
| EuRoC V1_01, MH_01 | 559 and 529 stereo matches on a rectified frame, at 1.2-3.5 m and 2.6-9.1 m |
| Extraction time (640x480, 1000 features) | C++ 24 ms, Rust 35 ms, Python 160 ms |

The extraction times are first versions: making them fit the `laptop-cpu` budget is PR 9's
work, with the profiler.
