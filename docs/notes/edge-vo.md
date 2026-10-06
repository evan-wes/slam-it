# The edge VO pipeline: ORB-SLAM-lite in three languages

Design note for `slamlab.edge` (`frames`, `map`, `residuals`, `adjust`, `initialization`,
`tracking`, `mapping`, `pipeline`), `slamlab.orchestration.vo` and the native programs
`slamlab_vo`. The plan is [the edge VO design](../design/2026-10-05-edge-vo.md); this note is
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

1. **Initialise**, until there is a map. Stereo and RGB-D: the first frame with enough
   keypoints that have depth is the first keyframe, a point at each, metric from the start.
   Monocular: a reference frame and a later one are matched in a window and initialised by
   rung 0's two-view (`twoview.initialize`: E or H by GRIC), refined by a 20-iteration BA and
   scaled to a median depth of 1. Too few matches make the current frame the reference; too
   few features drop it. Monocular frames are extracted with twice the features until then.
2. **Track the last frame** (`TrackWithMotionModel`): predict by constant velocity, project
   the last frame's map points, and match each within 15 pixels (7 for stereo) times its
   level's scale, twice as wide if fewer than 20 match. Refine the pose: four rounds of rung
   0's `refine_pose`, each match classified afresh by the chi-square test after each round.
3. **Track the local map** (`TrackLocalMap`): the window's points that are in view (in front,
   in the image, within 0.8 to 1.2 of their scale range, within 60 degrees of their viewing
   direction) are matched at their predicted level, and the pose refined again. Fewer than 30
   inliers is a failed frame.
4. **Recover** when the motion model fails: the frame's descriptors matched against the local
   map, then PnP RANSAC (rung 0). The recovery frame and the next search the local map five
   times wider, and for `max_frames` frames after it a frame needs 50 inliers (ORB-SLAM's
   rules after a relocalisation).
5. **Decide on a keyframe** (`NeedNewKeyFrame`): the frame tracks fewer than 90% (75% with
   depth, 40% while the map has one keyframe) of the points the newest keyframe shares with
   others, or, with depth, fewer than 100 close points while more than 70 close keypoints go
   untracked; and it has more than 15 inliers. With mapping never busy, ORB-SLAM's
   frame-count conditions always hold, so these decide.
6. **Map** around a new keyframe (`LocalMapping`, in order): insert it, with points from its
   close depth for stereo and RGB-D; cull the recent points found in under 25% of the frames
   that should have seen them; triangulate its free keypoints against the window's other
   keyframes; fuse duplicates; run local BA over the window with the first keyframe fixed;
   and cull keyframes 90% of whose points three others see at the same or a finer scale.

The **chi-square test** is one function for every stage: a keypoint on pyramid level `l` is
located to `s^l` pixels, so an observation is an outlier when its squared reprojection error
over `s^(2l)` exceeds 5.991 (2 dof), or 7.815 (3 dof) with a right column.

**Local BA** (`edge.adjust`) puts the window's keyframes, their points and every other
keyframe observing those points (held fixed) into rung 0's bundle adjustment: each
observation weighted by its level's information, an observation with depth adding an
inverse-depth prior, a Huber loss. It solves, drops the observations failing the chi-square
test, solves again, forgets the observations still failing, and refreshes the points.

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
- **The velocity spans consecutive tracked frames only**: after a failed frame the motion
  model starts from no motion.
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

## Three languages, one run

Each stage's parity test runs every backend on states built from a synthetic room (a textured
box room ray-cast along a known path, `tests/edge/room.py`): matches, levels, culled rows and
the keyframe decision exactly; positions and poses to 1e-12, or 1e-9 through a bundle
adjustment.

The whole run is held the same way. The room, written as a TUM-format folder and as a EuRoC
sequence, goes through `slamlab.orchestration.vo.run` with every backend in every mode the
dataset records: Python in-process, C++ and Rust as their native programs, which read the
folder with their own readers and write the run folder with their own writers. The native
runs track the same frames with the same matches and make the same keyframes as Python's,
and place every pose and point within 1e-9 of Python's; measured, within 6e-15 m.

## Results

Each sequence ran once through each native program with ORB-SLAM's settings (1000 ORB
features, 1200 for EuRoC stereo as in ORB-SLAM2's EuRoC stereo settings; 8 levels, scale 1.2),
never tuned per sequence, on one desktop (Intel Core Ultra 9 285). The ATE is the RMSE over
every tracked frame, SE(3)-aligned with depth and Sim(3)-aligned for monocular; the RPE is
over one frame period (30 Hz for TUM, 20 Hz for EuRoC), at the alignment's scale. The time is
the pipeline's per frame, tracking and mapping, frame building aside.

**Whole sequences.** C++'s numbers. Rust's runs take the same decisions on every frame, and
their poses differ from C++'s by at most 6e-12 (5e-8 on fr3/long_office monocular, after 2585
frames of bundle adjustment).

| Sequence | Mode | Frames tracked | Keyframes | ATE | RPE | Time, C++ / Rust | Published ATE |
|---|---|---|---|---|---|---|---|
| fr1/xyz | RGB-D | 798 of 798 | 30 | 1.08 cm | 0.62 cm, 0.41° | 3.0 / 2.5 ms | — |
| fr1/desk | RGB-D | 596 of 596 | 41 | 2.36 cm | 0.99 cm, 0.60° | 2.7 / 2.2 ms | 1.6 cm (ORB-SLAM2) |
| fr3/long_office | RGB-D | 2488 of 2488 | 58 | 7.04 cm | 0.77 cm, 0.33° | 1.7 / 1.3 ms | 1.0 cm (ORB-SLAM2) |
| fr1/xyz | monocular | 772 of 798 | 46 | 1.21 cm | 0.65 cm, 0.42° | 6.5 / 5.0 ms | 0.90 cm (ORB-SLAM) |
| fr1/desk | monocular | 342 of 409, lost | 49 | 2.66 cm | 1.32 cm, 0.75° | 7.7 / 5.9 ms | 1.69 cm (ORB-SLAM) |
| fr3/long_office | monocular | 2563 of 2585 | 177 | 16.99 cm | 0.69 cm, 0.30° | 9.2 / 7.7 ms | 3.45 cm (ORB-SLAM) |
| MH_01 | stereo | 3679 of 3682 | 539 | 3.16 cm | 0.29 cm, 0.04° | 21.1 / 18.0 ms | 3.5 cm (ORB-SLAM2), 2.9 cm (ORB-SLAM3) |
| V1_01 | stereo | 2912 of 2912 | 104 | 6.02 cm | 0.62 cm, 0.12° | 2.3 / 1.6 ms | 3.5 cm (ORB-SLAM2, ORB-SLAM3) |
| MH_01 | monocular | 2288 of 3682 | 111 | 3.72 cm | 1.13 cm, 0.09° | 3.8 / 3.5 ms | 1.6 cm (ORB-SLAM3), 4.6 cm (DSO) |
| V1_01 | monocular | 2761 of 2912 | 150 | 6.03 cm | 1.04 cm, 0.20° | 4.7 / 3.7 ms | 3.3 cm (ORB-SLAM3), 8.9 cm (DSO) |

fr1/desk monocular loses the track at frame 409 of 596, and the run ends there (there is no
relocalisation). MH_01 monocular initialises at frame 1393: no earlier pair of frames passed
two-view initialisation. Rust's pipeline is a fifth faster than C++'s, but its frame building
is slower, and whole runs, reading the images and building the frames included, take 31 to
92 ms a frame in C++ and 38 to 125 ms in Rust: building the frames, not the pipeline, sets
the frame rate.

**The first 300 frames, in three languages.** Python takes 0.05 to 1.2 s a frame, so it runs
the first 300 frames of each sequence. Every backend takes the same decisions on every frame
(the frames tracked, their inlier counts, the keyframes), and every pose entry agrees with
Python's to 6e-12 (6e-10 on fr3/long_office monocular).

| Sequence | Mode | Frames tracked | Keyframes | ATE | Largest pose difference from Python, C++ / Rust | Time, Python / C++ / Rust |
|---|---|---|---|---|---|---|
| fr1/xyz | RGB-D | 300 of 300 | 15 | 1.20 cm | 2e-15 / 2e-15 | 219 / 2.8 / 2.1 ms |
| fr1/desk | RGB-D | 283 of 283 | 23 | 1.95 cm | 1e-14 / 8e-15 | 247 / 3.2 / 2.4 ms |
| fr3/long_office | RGB-D | 289 of 289 | 10 | 0.94 cm | 1e-14 / 7e-15 | 120 / 2.2 / 1.7 ms |
| fr1/xyz | monocular | 274 of 300 | 35 | 1.53 cm | 3e-15 / 6e-15 | 902 / 11.9 / 9.4 ms |
| fr1/desk | monocular | 238 of 300 | 42 | 2.53 cm | 6e-14 / 6e-14 | 604 / 9.6 / 7.3 ms |
| fr3/long_office | monocular | 278 of 300 | 30 | 1.26 cm | 6e-10 / 6e-10 | 1184 / 14.6 / 11.9 ms |
| MH_01 | stereo | 299 of 300 | 42 | 0.48 cm | 2e-15 / 2e-15 | 657 / 8.5 / 5.7 ms |
| V1_01 | stereo | 300 of 300 | 9 | 1.36 cm | 2e-12 / 6e-12 | 46 / 1.4 / 0.9 ms |
| V1_01 | monocular | 149 of 300 | 28 | 1.23 cm | 6e-15 / 9e-15 | 948 / 12.3 / 10.8 ms |

MH_01 monocular is left out: no backend initialises in its first 300 frames. The RGB-D runs
have fewer than 300 frames where frames without a depth map were skipped.

**Against ORB-SLAM.** The published numbers come from complete SLAM systems with loop
closure, each the median of five runs, the monocular TUM ones over keyframes only; ours are
odometry, one deterministic run, every frame. Loop closure removes accumulated drift, so the
gap widens with a sequence's length and its revisits (fr3/long_office returns to its start;
the EuRoC sequences revisit their rooms). With depth on short sequences the edge pipeline is
within a centimetre of ORB-SLAM2 (fr1/desk, 2.4 cm against 1.6 cm), and on MH_01 stereo it
matches it (3.2 cm against 3.5 cm); where a loop closes it falls far behind (fr3/long_office,
seven times ORB-SLAM2's error with depth, five times ORB-SLAM's monocular). Monocular on
EuRoC, its error over the frames it tracks is below DSO's, the other odometry in the table
(3.7 and 6.0 cm against 4.6 and 8.9 cm), and about twice ORB-SLAM3's.

## What it does not do yet

- **Relocalisation and loop closure** need a place-recognition index (the bag of words): the
  classic rung (milestone 4).
- **The edge budgets** (milestone 7) run these programs under a simulated compute budget; the
  timings above are an unconstrained desktop's.
- **Monocular scale** drifts without loop closure; the Sim(3) alignment hides a single scale
  error, not its drift along the run.
