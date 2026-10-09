# The edge VO pipeline: ORB-SLAM-lite in three languages

Design note for `slam_it_all.edge` (`frames`, `map`, `residuals`, `adjust`, `initialization`,
`tracking`, `mapping`, `pipeline`), `slam_it_all.orchestration.vo` and the native programs
`slam_it_all_vo`. The plan is [the edge VO design](../design/2026-10-05-edge-vo.md); this note is
what was built, the decisions taken on the way, and how it does on TUM and EuRoC.

## The problem

Visual odometry estimates each frame's camera pose, and a sparse map of 3D points to estimate
it against, as the frames arrive. ORB-SLAM is the reference design on a CPU budget: ORB
features tracked against a local map, new points triangulated around each keyframe, and a
local bundle adjustment keeping both consistent. Its full system adds a bag of words for
relocalisation and loop closure, a covisibility graph and three threads.

The edge rung builds the part that runs per frame, in monocular, stereo and RGB-D, and holds
it to the same rule as rung 0: **the three languages take the same decisions.** Every stage
is a function of explicit state, a Python reference with C++ and Rust ports checked against
it on the same inputs; the pipeline is the loop over the stages; and a native program runs
that loop over a whole sequence without Python.

**Lite**, against ORB-SLAM:

- a **sliding window of the last N keyframes** (N = 10) stands in for the covisibility graph
  wherever ORB-SLAM takes a keyframe's neighbours, and the newest keyframe for the reference
  keyframe;
- **no bag of words**: no relocalisation against the whole map, no loop closure (the classic
  rung's, milestone 4). When the motion model fails, PnP against the local map recovers the
  frame if it can; after `lost_frames` failures in a row the track is lost and the run ends;
- **one thread**: mapping runs after tracking on each keyframe, so a run is deterministic and
  every backend can be compared with every other frame by frame.

## The state

Every mode ends in the same `Frame` (`edge.frames`): ORB features, each keypoint's bearing,
and for stereo and RGB-D its depth and right column `u_r = u - f b / z`. Stereo matches the
rectified pair along rows; RGB-D reads the depth map and gives the keypoint a **virtual**
right column from a virtual baseline (ORB-SLAM's `Camera.bf`, 40 pixel-metres for TUM), so
everything after the frame treats a depth measurement as a stereo one. Keypoints stay in the
distorted image: bearings come from the camera model, projections go through it.

The `Map` (`edge.map`) is three tables of arrays, row for row: keyframes (id, time, pose
`T_cam_world`, where their keypoints start), keypoints (what tracking and mapping read of a
keyframe's features, and the id of the point each observes) and points (id, position, the
distinctive descriptor, the mean viewing direction and the distances over which its scale is
valid, visible and found counts: ORB-SLAM's `MapPoint`). Ids are given once, in increasing
order, so rows are found by binary search; every operation returns a new map. A struct of
arrays crosses the bindings as a dict of arrays, as rung 0's types do, so each stage function
has a compiled twin that the tests call on the same state.

## A frame, step by step

1. **Initialise**, until there is a map. Stereo and RGB-D: the first frame with more than
   500 keypoints, at least 100 of them with depth, is the first keyframe, a point at each
   keypoint with depth, metric from the start. Lite: ORB-SLAM needs no depths, since it
   resets a map lost within its first five keyframes; a lite run would end there instead.
   Monocular: a reference frame and a later one are matched, each reference keypoint
   searched where it last matched (ORB-SLAM's `vbPrevMatched`), and initialised by rung 0's
   two-view (`twoview.initialize`: E or H by GRIC). The chosen motion's inliers in front of
   both views with more than 0.36° of parallax become points (`CheckRT`), a 20-iteration BA
   refines them, at least 100 must remain, and the map is scaled to a median depth of 1. Too
   few matches make the current frame the reference; too few features drop it. Monocular
   frames are extracted with twice the features until then.
2. **Track the last frame** (`TrackWithMotionModel`): predict by constant velocity, project
   the last frame's map points, and match each within 15 pixels (7 for stereo) times its
   level's scale, twice as wide if fewer than 20 match. Refine the pose: four rounds of rung
   0's `refine_pose`, each residual weighted by its level's information (`1/s^(2l)`, ORB-SLAM's
   `invSigma2`), each match classified afresh by the chi-square test after each round. Lite:
   a Cauchy loss in every round and the left image only, where ORB-SLAM drops its Huber loss
   after the third round and gives a match with depth a stereo edge.
3. **Track the local map** (`TrackLocalMap`): the window's points that are in view (in front,
   in the image, within 0.8 to 1.2 of their scale range, within 60 degrees of their viewing
   direction) are matched at their predicted level, and the pose refined again. Fewer than 30
   inliers is a failed frame.
4. **Recover** when the motion model fails: the frame's descriptors matched against the local
   map, then PnP RANSAC (rung 0). The recovery frame and the next search the local map with a
   window of 5 in place of 1 (3 for RGB-D), and for `max_frames` frames after it a frame needs
   50 inliers (ORB-SLAM's `th` and rules after a relocalisation).
5. **Decide on a keyframe** (`NeedNewKeyFrame`): the frame tracks fewer than 90% (75% with
   depth, 40% while the map has one keyframe) of the points the newest keyframe shares with
   others, or, with depth, fewer than 100 close points while more than 70 close keypoints go
   untracked; and it has more than 15 inliers. With mapping never busy, ORB-SLAM's
   frame-count conditions always hold, so these decide.
6. **Map** around a new keyframe (`LocalMapping`, in order): insert it, with points from its
   close depth for stereo and RGB-D; cull the recent points found in under 25% of the frames
   that should have seen them; triangulate its free keypoints against the window's other
   keyframes; fuse duplicates; once the map holds three keyframes, run local BA over the
   window with the first keyframe fixed (ORB-SLAM's `KeyFramesInMap() > 2`); and cull
   keyframes 90% of whose points three others see at the same or a finer scale.

The **chi-square test** is one function for every stage: a keypoint on pyramid level `l` is
located to `s^l` pixels, so an observation is an outlier when its squared reprojection error
over `s^(2l)` exceeds 5.991 (2 dof), or 7.815 (3 dof) with a right column.

**Local BA** (`edge.adjust`) puts the window's keyframes, their points and every other
keyframe observing those points (held fixed) into rung 0's bundle adjustment: each
observation weighted by its level's information, an observation with depth adding an
inverse-depth prior, a Huber loss. It solves, drops the observations failing the chi-square
test, solves again, forgets the observations still failing, and refreshes the points.

Its depth model is not ORB-SLAM's stereo edge. The prior has no robust loss, where ORB-SLAM's
three-row edge has Huber at √7.815; the Huber loss stays on in the second solve, where ORB-SLAM
drops it; and the prior's information `(f b)²/σ²` treats the disparity as independent of the
column with variance σ², where ORB-SLAM's isotropic edge on (u, v, u_r) implies var(d) = 2σ²
and cov(u, d) = σ². A Huber loss on the prior did not help on the room with a tenth of its
depths read 2.5 times too far (the largest keyframe-centre error 58 mm with it, 29 mm without;
17 mm either way on clean depth), so the prior stays as it is.

## Decisions taken while building it

- **A fixed keyframe keeps its pose bit for bit.** Only the free keyframes' poses are written
  back from BA, so the map's first keyframe stays exactly at the origin.
- **A point with fewer than two inlier views restarts BA's second pass from where it
  began.** With weak depth, a point's outlier views can drag it in the first pass until every
  view fails, erasing a good point. Rung 0's BA leaves a point seen once out of the solve, and
  ORB-SLAM's second pass would re-solve it from its one stereo edge; either way it should not
  keep where its outliers pulled it.
- **Outliers are forgotten before the points are refreshed** (ORB-SLAM's order), so a point's
  descriptor and scale range come from its inlier views only.
- **The prediction's rotation is projected back onto SO(3).** Pose refinement keeps its
  starting pose's drift from orthonormality, and the velocity, inverted by transposing, fed it
  into the next prediction: it grew about 2.4 times a frame until the pose check refused the
  26th frame of the synthetic room.
- **The search levels follow the camera's motion along its axis.** With depth, a frame that
  moved forward by more than the baseline searches the last frame's level and coarser ones,
  one that moved back that level and finer ones (ORB-SLAM's `tlc` test). A draft had the
  test inverted; a scene whose keypoints sit on chosen levels now checks it in every language.
- **The velocity spans consecutive tracked frames only**: a failed frame clears it, so the
  next frame is predicted with no motion. A draft kept the last velocity through failures.
- **No ratio test for triangulation**: ORB-SLAM's `SearchForTriangulation` has none, so the
  setting was removed. Without a bag of words, every pair of free keypoints within the
  stricter descriptor distance and the epipolar band is a candidate, the nearest kept.
- **A homography of rank below 3 is refused** by H-RANSAC itself (its smallest singular value
  below 1e-4 of its largest, in the normalised frame the DLT fits in), so two-view no longer
  needs its own rank check.
- **Guided matching searches a feature grid** in C++ and Rust (64 x 48 cells, ORB-SLAM's
  `GetFeaturesInArea`), and finds exactly what Python's brute force finds.
- **The native maps index each point's observations.** Written first as Python's map is, the
  C++ and Rust maps found a point's observations by scanning `kp_point`, every keypoint of
  every keyframe, and the keyframe decision, fusion's merges, point refreshes and BA's problem
  each scanned it again. The cost of a frame grew with the whole map: by the end of MH_01
  stereo (539 keyframes) a keyframe took 2.6 s at the median, and the run averaged 0.49 s a
  frame. The native maps now keep `pt_observations`, each point's observing keypoint rows,
  updated by the operations that change an observation, as ORB-SLAM's `MapPoint` keeps its
  observations. The rows stay in ascending order, so every sum runs in the order it did, and
  every run on TUM and EuRoC is the same to the bit. MH_01 stereo now averages 21 ms a frame,
  its keyframes 123 ms at the median. Its keyframes still take longer as it goes, from 57 ms in
  its first quarter to 160 ms in its last; that remainder is not yet profiled. Python's
  reference still scans: it runs short sequences, and stays the plain statement of the rules.
- **Pose refinement weighs each match by its level.** Rung 0's `refine_pose` takes a weight
  per correspondence, whitening its residual and Jacobian by √w, and tracking passes
  `1/s^(2l)`: the noise model its chi-square test already used. Unweighted, a keypoint on the
  coarsest level (σ = 3.6 pixels) pulled as hard as one on the finest; in a Monte Carlo of
  pose-only refinement, the camera centre's error was 24% larger.
- **Monocular initialisation follows ORB-SLAM2.** The map keeps every inlier in front of both
  views with more than 0.36° of parallax (`cos < 0.99998`), not only the points two-view
  triangulates for its own decision (more than 1°), and needs 100 of them after its bundle
  adjustment (ORB-SLAM3 relaxes it to 50); each reference keypoint is searched where it last
  matched. Keeping only two-view's points refused every early start on MH_01: two-view
  initialised from frame 6, but no map kept 100 points until frame 1393 (an earlier version
  of this note blamed two-view). It now starts at frame 140. Lite, and not binding on MH_01
  (none of its candidate starts has a kept point beyond 2 pixels, and its initial BA forgets
  none): the kept points have no reprojection test, where `CheckRT` drops those beyond 2σ,
  and the 100 are counted after the initial BA forgets its outliers, where ORB-SLAM2's
  forgets none.
- **Every threshold is a setting.** ORB-SLAM's constants (the windows, the ratio tests, the
  distance margins, the observation counts, the iterations, the descriptor distances
  `TH_LOW` and `TH_HIGH`, and the stereo matcher's) are `Options` fields, 76 in all, so each
  is recorded in a run's `run.json`, settable from the command line, and defined once per
  language; each refuses a bad value by its name, in every language. The virtual
  baseline is the sensor's: `camera.txt`'s optional `bf`, 40 for TUM's freiburg cameras from
  the readers' table; an RGB-D run without one is refused.

## Three languages, one run

Each stage's parity test runs every backend on states built from a synthetic room (a textured
box room ray-cast along a known path, `tests/edge/room.py`): matches, levels, culled rows and
the keyframe decision exactly; positions and poses to 1e-12, or 1e-9 through a bundle
adjustment.

The whole run is held the same way. The room, written as a TUM-format folder and as a EuRoC
sequence, goes through `slam_it_all.orchestration.vo.run` with every backend in every mode the
dataset records: Python in-process, C++ and Rust as their native programs, which read the
folder with their own readers and write the run folder with their own writers. The native
runs track the same frames with the same matches and make the same keyframes as Python's,
and place every pose and point within 1e-9 of Python's; measured, within 6e-15 m.

A second room drops ten frames, narrows the motion model's window, holds a blank frame (no
texture, no depth) before the gap, and lowers `max_frames` below the keyframes the map
holds. The blank frame fails, and the next is predicted with no motion; across the gap the
motion model fails and the fallback recovers, twice, since the recovery's velocity spans the
gap; just after it the keyframe hold decides. The backends agree across it, and again with
every setting the review's fixes added moved off its default. Both rooms' runs are also held
to a committed golden record (`tests/fixtures/edge/room_runs.json`): the decisions, frame
times, keyframes and point count exactly, the poses to 1e-9, with the rooms' inputs checked
first. Live parity
with Python cannot catch a rule changed in all three languages at once; the record can, until
it is deliberately regenerated (`python -m tests.fixtures.edge.generate_room_runs`).

## Results

Each sequence ran through each native program with ORB-SLAM2's settings, never tuned per
sequence: 1000 ORB features (1200 for EuRoC stereo), 8 levels, scale 1.2, and for EuRoC
`EuRoC.yaml`'s `ThDepth` of 35 baselines and `Camera.fps` of 20 (`close_depth` and
`max_frames`); on one desktop (Intel Core Ultra 9 285). The ATE is the RMSE, SE(3)-aligned
with depth and Sim(3)-aligned for monocular, twice: over every tracked frame, at its pose when
it was tracked, and over the map's keyframes at the end of the run, after their last local BA
(ORB-SLAM's `KeyFrameTrajectory.txt`). The RPE is over one frame period (30 Hz for TUM, 20 Hz
for EuRoC), at the alignment's scale. The time is the pipeline's per frame, tracking and
mapping, frame building aside.

**Whole sequences.** C++'s numbers. A whole run is sensitive to where it starts (below), so
each ATE is the median of five runs, with their range: one from the sequence's first frame,
and four started one to four frames after the frame that run initialised from (for monocular,
the first of its initialising pair). A run that loses the track counts as a failure, worse
than any that finishes, as ORB-SLAM's tables mark one; the range covers the runs that finish.
The other columns are the first run's. Rust's first runs take the same decisions as C++'s on
every frame, and their poses differ by at most 8e-12.

| Sequence | Mode | Frames tracked | Keyframes | ATE, every frame | ATE, keyframes | RPE | Time, C++ / Rust | Published ATE |
|---|---|---|---|---|---|---|---|---|
| fr1/xyz | RGB-D | 798 of 798 | 27 | 1.09 cm (1.06–1.15) | 1.38 cm (1.33–1.40) | 0.64 cm, 0.40° | 2.7 / 2.2 ms | — |
| fr1/desk | RGB-D | 596 of 596 | 45 | 3.13 cm (2.56–3.43) | 2.79 cm (2.16–3.00) | 1.03 cm, 0.62° | 2.9 / 2.3 ms | 1.6 cm (ORB-SLAM2) |
| fr3/long_office | RGB-D | 2488 of 2488 | 70 | 5.36 cm (2.12–9.03) | 3.72 cm (1.56–7.96) | 0.86 cm, 0.34° | 2.2 / 1.7 ms | 1.0 cm (ORB-SLAM2) |
| fr1/xyz | monocular | 793 of 798 | 41 | 1.10 cm (1.05–5.73) | 1.30 cm (1.06–5.86) | 0.63 cm, 0.41° | 5.3 / 4.1 ms | 0.90 cm (ORB-SLAM) |
| fr1/desk | monocular | 551 of 613 | 72 | lost in 3 of 5 (finished: 2.04, 2.68 cm) | lost in 3 of 5 (finished: 2.34, 2.47 cm) | 1.18 cm, 0.68° | 8.2 / 6.4 ms | 1.69 cm (ORB-SLAM) |
| fr3/long_office | monocular | 2564 of 2585 | 289 | 9.85 cm (4.96–19.15) | 7.90 cm (4.74–14.15) | 0.54 cm, 0.26° | 29.5 / 25.2 ms | 3.45 cm (ORB-SLAM) |
| MH_01 | stereo | 3681 of 3682 | 602 | 5.44 cm (4.11–6.24) | 5.38 cm (4.13–6.34) | 0.27 cm, 0.04° | 25.3 / 22.3 ms | 3.5 cm (ORB-SLAM2), 2.9 cm (ORB-SLAM3) |
| V1_01 | stereo | 2912 of 2912 | 122 | 5.25 cm (4.97–5.63) | 4.32 cm (4.21–4.70) | 0.54 cm, 0.11° | 2.8 / 2.0 ms | 3.5 cm (ORB-SLAM2, ORB-SLAM3) |
| MH_01 | monocular | 3542 of 3682 | 187 | 11.20 cm (5.90–11.20; lost in 2 of 5) | 9.96 cm (6.31–9.96; lost in 2 of 5) | 0.80 cm, 0.09° | 4.7 / 3.7 ms | 1.6 cm (ORB-SLAM3), 4.6 cm (DSO) |
| V1_01 | monocular | 2799 of 2912 | 89 | 6.67 cm (4.35–6.80; lost in 1 of 5) | 5.70 cm (3.68–6.84; lost in 1 of 5) | 1.76 cm, 0.31° | 2.0 / 1.5 ms | 3.3 cm (ORB-SLAM3), 8.9 cm (DSO) |

No first run loses the track, nor any RGB-D or stereo run. Six of the twenty monocular runs
do: three of five on fr1/desk, two on MH_01 and one on V1_01. Each loses it deep into the run,
with 44 to 128 keyframes mapped, when its inliers fall below 30, ORB-SLAM's threshold for a
lost track. ORB-SLAM relocalises from there; this pipeline has no relocalisation yet
(milestone 4), so the run ends. The first monocular runs start at the first pair of frames
that initialises: frame 5 on fr1/xyz, 21 on fr3/long_office, 62 on fr1/desk, 140 on MH_01 and
113 on V1_01; their frame counts include the images RGB-D skips for want of a depth map (17
on fr1/desk, 97 on fr3/long_office). Rust's pipeline is a fifth faster than C++'s, but its
frame building is slower, and whole runs, reading the images and building the frames
included, take 29 to 95 ms a frame in C++ and 37 to 132 ms in Rust: building the frames, not
the pipeline, sets the frame rate. PR 9 brought building and tracking a frame to 9 to 26 ms
(medians) in both languages, within a laptop-CPU budget, no frame late on any sequence
([the real-time note](real-time.md)).

**How much one run says.** A run is deterministic, but a long one is sensitive to small
perturbations. Started one to four frames later, with the same rules, MH_01 stereo's ATE
ranges from 4.1 to 6.2 cm, fr3/long_office RGB-D's from 2.1 to 9.0 cm, and fr3/long_office
monocular's from 5.0 to 19.2 cm. The code before the review's fixes spreads the same way:
over ten starts on MH_01 stereo its median is 5.17 cm, against 5.14 cm after them, and the
3.16 cm its first run gave was its best start; on fr1/desk RGB-D its median over five is
2.95 cm against 3.13 cm, within a spread of a centimetre, and its first run's 2.36 cm was
again its best. Changing one rule moves one run about as much: local BA from the second
keyframe, not the third, gives 4.7 cm on MH_01 stereo and 2.6 cm on fr1/desk RGB-D. A
difference of a centimetre between two single runs of these sequences says little about
either rule. The level weights in pose refinement are a case in point: the 24% they gain in
the Monte Carlo does not show over a whole sequence (over the same five starts, 5.25 cm
without them and 5.44 cm with them on MH_01 stereo, 3.14 and 3.13 cm on fr1/desk RGB-D), so
they stand as the right noise model, not as a measured gain. ORB-SLAM's papers report the
median of five or ten runs for the same reason (theirs vary with thread timing).

**The first 300 frames, in three languages.** Python takes 0.05 to 3.1 s a frame, so it runs
the first 300 frames of each sequence. Every backend takes the same decisions on every frame
(the frames tracked, their inlier counts, the keyframes), and every pose entry agrees with
Python's to 8e-12.

| Sequence | Mode | Frames tracked | Keyframes | ATE | Largest pose difference from Python, C++ / Rust | Time, Python / C++ / Rust |
|---|---|---|---|---|---|---|
| fr1/xyz | RGB-D | 300 of 300 | 15 | 1.19 cm | 2e-15 / 2e-15 | 211 / 2.6 / 2.1 ms |
| fr1/desk | RGB-D | 283 of 283 | 25 | 2.36 cm | 8e-15 / 1e-14 | 274 / 3.2 / 2.6 ms |
| fr3/long_office | RGB-D | 289 of 289 | 10 | 1.10 cm | 6e-15 / 6e-15 | 123 / 2.1 / 1.6 ms |
| fr1/xyz | monocular | 295 of 300 | 35 | 1.16 cm | 4e-13 / 4e-13 | 827 / 10.7 / 8.1 ms |
| fr1/desk | monocular | 238 of 300 | 54 | 1.81 cm | 1e-14 / 5e-14 | 1107 / 13.6 / 11.1 ms |
| fr3/long_office | monocular | 279 of 300 | 30 | 1.05 cm | 9e-15 / 5e-15 | 980 / 11.7 / 9.9 ms |
| MH_01 | stereo | 300 of 300 | 73 | 0.50 cm | 1e-15 / 2e-15 | 3057 / 22.7 / 17.4 ms |
| V1_01 | stereo | 300 of 300 | 9 | 1.42 cm | 1e-12 / 8e-12 | 45 / 1.3 / 0.9 ms |
| MH_01 | monocular | 160 of 300 | 17 | 0.37 cm | 5e-15 / 2e-15 | 189 / 3.9 / 3.3 ms |
| V1_01 | monocular | 187 of 300 | 23 | 1.16 cm | 1e-14 / 1e-14 | 446 / 7.0 / 6.2 ms |

The RGB-D runs have fewer than 300 frames where frames without a depth map were skipped.

**Against ORB-SLAM.** The published numbers come from complete SLAM systems with loop closure.
ORB-SLAM's TUM monocular numbers (Mur-Artal et al., T-RO 2015, Table III) and ORB-SLAM2's
(Mur-Artal and Tardós, T-RO 2017, Tables II and III) are medians of five runs, ORB-SLAM's over
keyframes. ORB-SLAM3's EuRoC numbers (Campos et al., T-RO 2021, Table II) are medians of ten
runs over every frame; the DSO row beside them is DSO's authors' figure, as ORB-SLAM3 reprints
it. Ours are odometry, the medians of five deterministic runs, with no relocalisation to
recover a run that loses the track. Loop closure removes accumulated drift, so the gap widens
with a sequence's length and its revisits (fr3/long_office returns to its start; the EuRoC
sequences revisit their rooms).

- **Short TUM sequences:** monocular over keyframes has about one and a half times ORB-SLAM's
  error on fr1/xyz (1.30 against 0.90 cm). On fr1/desk three of five monocular runs lose the
  track; the two that finish have 2.34 and 2.47 cm over keyframes, against ORB-SLAM's
  1.69 cm. RGB-D on fr1/desk has about twice ORB-SLAM2's error (3.1 cm, 2.8 over keyframes,
  against 1.6 cm).
- **Where a loop closes**, the pipeline falls behind, and its runs spread widest: on
  fr3/long_office RGB-D, 3.7 cm over keyframes (1.6 to 8.0) against ORB-SLAM2's 1.0 cm, and
  7.9 cm monocular (4.7 to 14.2) against ORB-SLAM's 3.45 cm.
- **EuRoC stereo:** 5.4 and 5.3 cm against ORB-SLAM2's 3.5 cm and ORB-SLAM3's 2.9 and
  3.5 cm.
- **EuRoC monocular:** the first runs track 96% of each sequence's frames, all but those
  before initialisation; two of five runs lose the track on MH_01, one on V1_01. The error is
  11.2 cm on MH_01, against ORB-SLAM3's 1.6 cm and DSO's 4.6 cm, and 6.7 cm on V1_01, against
  3.3 and 8.9 cm. MH_01's 11.2 cm, its first run's, is mostly drift between the sequence's
  first minute and the rest. Each aligned on its own, the first minute has 1.45 cm and the
  frames after it 3.77 cm (3.72 cm in a version that tracked only those), but one similarity
  cannot fit both across the nearly stationary stretch between them (20 to 40 s), where the
  scale drifted. ORB-SLAM3's loop closure and map merging absorb such drift.

## What it does not do yet

- **Relocalisation and loop closure** need a place-recognition index (the bag of words): the
  classic rung (milestone 4). Without relocalisation, a run that loses the track ends: six of
  the twenty monocular runs above.
- **The timings above** are an unconstrained desktop's, from before PR 9, which runs these
  programs within a laptop-CPU budget ([the real-time note](real-time.md)); the edge budgets
  (milestone 7) add smaller ones.
- **Monocular scale** drifts without loop closure; the Sim(3) alignment hides a single scale
  error, not its drift along the run.
- **Re-anchoring after local BA:** a frame keeps its pose from when it was tracked, so the
  every-frame ATE scores tracking-time poses; ORB-SLAM re-expresses each frame relative to
  its BA-refined reference keyframe (the ladder's open items).
