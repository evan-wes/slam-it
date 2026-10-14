# The test set: the VO on phones, what a lens, a size and a confidence change, and a new map

Results note for milestone 2A (`docs/design/2026-10-12-phone-robust-vo.md`):
- A1's (section 4): the test set's 77 runs, what a guessed lens costs, whether importing ADVIO's
  video at 720 pixels pays, and LiDAR depth at medium confidence against high;
- A2's (section 5): the same runs with a new map after each lost track
  ([A2](#a2-a-new-map-after-each-lost-track)).

A1's imports and the measure are the modules and the script the design names
(`slam_it.eval.zju`, `advio`, `arkitscenes`, `lidar`; `scripts/measure_testset.py`). The review
panel and a security pass read them
([report](../reviews/2026-10-13-feature-2a-a1-measure-and-import.md),
[security](../reviews/2026-10-13-feature-2a-a1-measure-and-import-security.md)). Every figure
here is from public datasets. The owner's recordings are a check on the demo, and their results
stay outside the repository.

## The problem

The demo's first phone clips lost the track within seconds (the design note's section 1), and
one recording cannot say why, or what would help. A1 measures the pipeline on public hard
handheld and phone sequences with ground truth or a reference, so that A2, a new map after a
lost track, is built against numbers, and so that three questions about importing phone data
are answered by measurement rather than guessed:
- what a guessed lens costs;
- whether a larger frame keeps the track;
- whether LiDAR depth of medium confidence helps.

## The test set

| Dataset | Runs | Frames as run | Camera | Reference |
|---|---|---|---|---|
| **TUM RGB-D**, nine hard sequences: `fr1_360`, `fr1_rpy`, `fr2_rpy`, `fr2_360_hemisphere`, `fr2_360_kidnap`, `fr1_floor`, and three of `fr3`'s structure/texture set | 18: each monocular and RGB-D | 640x480, 30 Hz | Each sensor's ROS default: radtan for `freiburg1` and `2`, undistorted for `3` | Motion capture |
| **ZJU-SenseTime**: A0-A7 (Xiaomi Mi 8), B0-B7 (iPhone X) | 16, monocular | 640x480 greyscale, 30 Hz | Each phone's calibrated pinhole | VICON, moved to the camera |
| **ADVIO**: all 23 sequences | 23, monocular | 480x854: its 720x1280 portrait video at a 480-pixel short side, every other frame of 60 fps | Its capture batch's Kalibr calibration, radtan, the frames kept distorted | From the IMU and known fixation points |
| **ARKitScenes**: ten raw captures, the first ten of its Validation fold with a laser-scanned visit, one per visit | 20: each monocular and RGB-D, LiDAR depth of high confidence | 640x480, 30 Hz | A pinhole of the frames' median intrinsics | ARKit's own poses, interpolated at each frame's time: a reference, not truth |

**The measure,** per run:
- the share of the sequence's frames tracked;
- when tracking began and ended, and whether the track was lost;
- the ATE, the RMSE over the tracked frames matched to the reference within 20 ms, aligned
  Sim(3) for monocular and SE(3) for RGB-D;
- the 95th-percentile latency of building and tracking a frame, against the frame period.

A1's runs end at the frame that loses the track, as the pipeline did before A2. So a lost run's
share there is what it tracked before its first loss, and the frames after are never run. An ATE
covers a run's own tracked stretch, so runs of different lengths do not compare by it.

## Conditions

- **The code:** commit 26f9d6d on `feature/2a-a1-measure-and-import`; the C++ backend, a
  release build; ORB-SLAM's settings, with the features for the frame size
  (`vo.options_for`): 1,000 at 640x480, 1,334 at 480x854, 2,000 at 720x1280.
- **Seeds:** the edge VO is deterministic for a RANSAC seed. A rerun of zju-B4 at the measure's
  seed, 0, wrote a byte-identical trajectory. Every batch was also run at seeds 1 and 2, by
  setting `Options.seed` around the measure (a local wrapper; the measure itself has no option
  for it), so that each change is read against what the seed alone moves.
- **Latency** is from the seed-0 runs, made one at a time but not pinned to the `laptop-cpu`
  budget's cores, on an Intel Core Ultra 9 285 (24 cores) that other work shared. The runs at
  seeds 1 and 2 ran four at a time, and their latency is not used.
- **Measured** 2026-10-13.

## Results (A1)

Seed 0, per dataset and mode: the median share tracked, the runs lost, the runs that never
started a map, the median time tracking began, the median share of a lost run never run, the
median ATE, the range of the runs' p95 latency, and the frames past the period.

| Dataset | Mode | Runs | Tracked | Lost | Never started | Began | Unrun after the loss | ATE | p95 | Late |
|---|---|---|---|---|---|---|---|---|---|---|
| TUM | monocular | 9 | 7% | 6 | 2 | 0.8 s | 84% | 6.4 cm | 5-14 ms of 32 | 0 |
| TUM | RGB-D | 9 | 71% | 3 | 1 | 0.0 s | 69% | 4.1 cm | 6-11 ms of 32 | 0 |
| ZJU | monocular | 16 | 37% | 12 | 0 | 5.9 s | 58% | 1.4 cm | 10-13 ms of 33 | 0 |
| ADVIO | monocular | 23 | 9% | 22 | 0 | 2.9 s | 88% | 79.1 cm | 11-22 ms of 33 | 0 |
| ARKitScenes | monocular | 10 | 16% | 10 | 0 | 1.0 s | 79% | 6.4 cm | 9-11 ms of 33 | 0 |
| ARKitScenes | RGB-D | 10 | 32% | 8 | 0 | 0.0 s | 78% | 6.2 cm | 8-10 ms of 33 | 0 |

- **Most of the test set is after a lost track.** 61 of the 77 runs lose it. Of the set's
  246,529 frames, 159,421 (65%) come after a loss and are never run, and a lost run leaves a
  median 85% of its sequence. This is A2's case: a new map would run them.
- **A phone's own video is the hardest.** ADVIO's runs track a median 9% of a sequence, and 22
  of 23 are lost, 12 of those 22 within 15 s of the start. While tracked, the error is a median
  of 79 cm, against ZJU's 1.4 cm. The likeliest cause is scale drift: over minutes of walking
  through malls and stations (advio-21 tracks for 306 s), a monocular map's scale has nothing to
  correct it. ADVIO's reference, from the IMU and fixation points, is looser than motion
  capture, too.
- **ZJU's phones track longer** (a median of 37%, 81% on the Xiaomi's A sequences), and while
  tracked, to a median 1.4 cm. Its B sequences are its robustness set. B5, B6 and B7 go black
  for 1 to 3 s at a time from 30 s on, and each of those runs tracks its last frame before the
  first black one, 30.0 s in: the case for A2's new map, and for milestone 2B's
  relocalisation.
- **LiDAR depth doubles ARKitScenes' share** (a median of 32% RGB-D against 16% monocular). An
  RGB-D run starts its map at the first frame, where a monocular one waits a median of 1.0 s for
  parallax. Two RGB-D runs (42446100, 42446137) track every frame. Eight are still lost, five
  of them by 17 s.
- **TUM's hard sequences:** RGB-D tracks a median 71%: fr1_360 and fr1_rpy whole, and fr2_rpy
  all but 3 of its 3,290 frames. Monocular tracks a median 7%: turning on the spot gives a
  single camera no parallax to start or keep a map. fr3's two texture-less sequences never
  start a monocular map, and the one without structure starts no RGB-D map either: its frames
  give too few keypoints, where an RGB-D map needs more than 500 to start.
- **Real time holds at a 480-pixel short side.** No frame of any run is late. ADVIO's 1,334
  features at 480x854 take a p95 of 11 to 22 ms of the 33 ms period.

**The seed alone** moves a run's share by more than 10 points on 5 of the 77 runs, and by more
than 30 on 3, all monocular:
- advio-05: 1%, 1% and 77% at seeds 0, 1 and 2;
- advio-10: 91%, 17% and 91%;
- zju-A1: 37%, 19% and 2%.

18 of the 19 RGB-D runs write the same trajectory at all three seeds: an RGB-D map starts from
depth, without the two-view RANSAC a monocular one starts from. The other, fr1_floor, moved, but
its share did not. Of the 58 monocular runs, 24 write the same trajectory at every seed. The ATE
moves with the seed too: fr1_floor monocular tracks 669 frames at every seed, at 17.3, 2.1 and
2.1 cm.
Every run, seed by seed, is in [Every run](#every-run).

## A2: a new map after each lost track

The same 77 runs with A2's pipeline (the design note's section 5): a lost track closes its map,
kept or, with 10 keyframes or fewer, reset, and the next frame starts a new one, so every run
goes on to its sequence's end. Each map is scored on its own alignment, and the ATE pooled over
a run's maps.

**Conditions:** commit 6e609fa on `feature/2a-a2-new-map`, otherwise as A1's: the C++ backend,
a release build, the features for the frame size, seeds 0, 1 and 2 through the same wrapper.
Seed 0's runs were made one at a time, and its latency is the one given; seeds 1 and 2 ran two
at a time. Measured 2026-10-13 and 14.

Seed 0, per dataset and mode: the median share tracked in A1 and in A2, the runs that lost the
track, the losses, the maps kept and reset, the frames the resets dropped, a kept map's median
span, the median ATE (A1's over its one map, A2's pooled over a run's), the range of the
runs' p95 latency, and the frames past the period.

| Dataset | Mode | Runs | Tracked, A1 | Tracked, A2 | Lost | Losses | Maps kept | Maps reset | Frames reset | Map span | ATE, A1 | ATE, A2 | p95 | Late |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| TUM | monocular | 9 | 7% | 74% | 6 | 22 | 22 | 6 | 76 | 4.5 s | 6.4 cm | 6.2 cm | 5-12 ms of 32 | 0 |
| TUM | RGB-D | 9 | 71% | 97% | 3 | 6 | 12 | 2 | 103 | 24.6 s | 4.1 cm | 4.1 cm | 6-11 ms of 32 | 0 |
| ZJU | monocular | 16 | 37% | 85% | 12 | 64 | 71 | 8 | 84 | 6.8 s | 1.4 cm | 2.7 cm | 9-13 ms of 33 | 0 |
| ADVIO | monocular | 23 | 9% | 90% | 22 | 195 | 177 | 35 | 664 | 10.1 s | 79.1 cm | 208.3 cm | 11-18 ms of 33 | 105 |
| ARKitScenes | monocular | 10 | 16% | 90% | 10 | 38 | 46 | 2 | 176 | 10.8 s | 6.4 cm | 12.8 cm | 8-10 ms of 33 | 0 |
| ARKitScenes | RGB-D | 10 | 32% | 98% | 8 | 23 | 31 | 2 | 101 | 14.8 s | 6.2 cm | 7.2 cm | 8-10 ms of 33 | 0 |

- **The set is now run, and most of it tracked.** Every run goes on to its sequence's end (eight
  RGB-D runs skip 251 colour frames between them that have no depth map to pair). Of the set's
  246,529 frames, 214,583 (87%) are tracked, against A1's 77,603 (31%). A run's median share is
  90%, against 20%: the 61 runs that lose the track all track more, and the 16 that never lose
  it, 3 of them never starting a map, are unchanged.
- **Up to its first loss, each run is A1's, frame by frame.** Every frame A1 ran, 87,108 of
  them, is decided alike: each frame's diagnostics, timings aside, are the same. The runs lose
  the track on the same frames, and the 61 lost runs are the same 61. On three ADVIO runs, 01,
  05 and 08, the first map was lost with 10 keyframes or fewer and is reset, so the 75, 61 and
  22 frames A1 tracked there are dropped.
- **A lost run loses the track again and again:** 348 losses over 61 runs, a median of 4 a run,
  8 on ADVIO. 293 of the closed maps are kept and 55 reset, and the resets drop 1,204
  tracked frames, 0.5% of the set, 664 of them on ADVIO.
- **ZJU's black frames are bridged.** B5, B6 and B7, whose A1 runs end at their first black
  frame, 30.0 s in, track again 1.2 to 5.1 s later in a new map, and again after each later
  loss: 92%, 72% and 71% of the sequence.
- **The maps are short, and separate.** A kept map spans a median of 10.1 s on ADVIO, 6.8 s on
  ZJU and 4.5 s on TUM monocular, and an ADVIO run's largest map holds a median 33% of the frames
  it tracks. Each map has its own frame and, monocular, its own scale, so a run's maps are
  pieces of a trajectory, not one, until milestone 2B joins them.
- **The ATE does not compare with A1's.** A2's covers the frames after the first loss, which A1
  never ran, each map aligned on its own. It is higher on ZJU (2.7 against 1.4 cm), ARKitScenes
  monocular (12.8 against 6.4 cm) and ADVIO (208 against 79 cm). On the runs with two maps or
  more kept, the later maps' pooled error is higher than the first kept map's, by median, on four
  of the six groups. The frames A1 tracked score as before: a run's first map is A1's run, but
  for the three reset.
- **Real time holds except while a monocular map starts.** The 105 late frames are all ADVIO's,
  on 7 runs, and every one is a two-view start that failed: 95 for too little parallax, 10 for
  too few points, the slowest 0.43 s. A1 started one map a run; A2 starts one after each loss,
  in the stretch that lost the track. The likeliest cause, not yet checked, is the two-view
  RANSAC's sample limit: 10,000 per model, where ORB-SLAM's initialiser draws 200.

**Why the track is lost.** At each loss, the check that failed on the first frame of its
losing streak (the `lost_frames`, 5, failures in a row), from the diagnostics the runs write:

| Dataset and mode | Losses | Placed, too few local-map inliers | Not placed: too few matches for PnP | Not placed: PnP found no pose |
|---|---|---|---|---|
| TUM monocular | 22 | 15 (68%) | 3 (14%) | 4 (18%) |
| TUM RGB-D | 6 | 2 (33%) | 4 (67%) | 0 |
| ZJU monocular | 64 | 13 (20%) | 46 (72%) | 5 (8%) |
| ADVIO monocular | 195 | 110 (56%) | 32 (16%) | 53 (27%) |
| ARKitScenes monocular | 38 | 29 (76%) | 8 (21%) | 1 (3%) |
| ARKitScenes RGB-D | 23 | 15 (65%) | 8 (35%) | 0 |
| All | 348 | 184 (53%) | 101 (29%) | 63 (18%) |

- **Placed, too few local-map inliers:** the motion model or PnP placed the frame, but tracking
  the local map kept fewer than 30 inliers (50 just after a recovery). The frame before kept a
  median of 35, just over the threshold, with a median of 134 local-map points in view: the
  track thins out before it is lost, and only 13 of the 184 kept 100 or more the frame before.
- **Not placed:** the motion model failed, and then PnP against the local map either had fewer
  than 30 matches to try or found no pose with 30 inliers. ZJU's losses are mostly the first
  (46 of 64), its black frames among them: each of B5's four begins on a frame that matched
  nothing. Of all 101 with too few matches, the frame before kept a median of 52 inliers, and 22
  kept 100 or more: a sudden loss.
- **ADVIO and ARKitScenes lose mostly by the local map** (110 of 195 and 44 of 61): where a
  larger local map, as ORB-SLAM's covisibility graph gives (milestone 2B), may help.

**The seed alone** moves a run's share by more than 10 points on 3 of the 77 runs, by 13 at
most (advio-01: 73%, 86% and 80% at seeds 0, 1 and 2), where A1's moved on 5, one by 76. A1's
three most moved hardly move now:
- advio-05: 90%, 90% and 91% (A1: 1%, 1% and 77%);
- advio-10: 92%, 91% and 92% (91%, 17% and 91%);
- zju-A1: 76%, 77% and 77% (37%, 19% and 2%).

In A1 a seed decided where a run's one map was lost, and with it the rest of the sequence; in A2
a loss starts another map. Over the set, 87.0%, 87.3% and 87.5% of the frames are tracked at
the three seeds, with 348, 356 and 347 losses. Every run is in
[Every run, A2](#every-run-a2).

## What a guessed lens costs

Each run was repeated with the nominal guess in place of its camera: a pinhole of the typical
phone's 72.37° diagonal field of view (`lenses.nominal`), centred, without distortion. That is
546.8 pixels at 640x480 and 669.6 at 480x854. An RGB-D run keeps its virtual baseline, so its
near and far points split as before.

The share tracked compares poorly, since where a track is lost turns on one moment. So the
accuracy is compared on the frames both runs tracked, each run aligned on its own. Per
sequence, the nominal run's ATE over those frames is divided by the exact run's, and the median
taken over the three seeds. The table gives each camera's median over its sequences, and the
sequences where the guess was worse.

| Camera | Guess's focal length | Distortion dropped | Monocular: ATE ratio, worse on | RGB-D: ATE ratio, worse on |
|---|---|---|---|---|
| TUM `freiburg3` | +2.1% | none to drop | 0.60, 0 of 1 | 0.85, 0 of 2 |
| ARKitScenes | +2.3 to +3.0% | none to drop | 1.11, 6 of 10 | 1.05, 5 of 10 |
| ZJU B (iPhone X) | +4.1% | none to drop | 0.99, 4 of 8 | |
| TUM `freiburg2` | +5.0% | yes | 1.02, 1 of 2 | 1.77, 3 of 3 |
| TUM `freiburg1` | +5.7% | yes | 1.37, 3 of 3 | 2.67, 3 of 3 |
| ADVIO | -7.2 to -6.7% | yes | 1.00, 11 of 23 | |
| ZJU A (Xiaomi Mi 8) | +10.9% | none to drop | 1.70, 7 of 8 | |

(A sequence with no frames tracked by both runs, as fr2_rpy monocular or a run that never
started, has no ratio: 73 of the 77 have one.)

- **The guess costs accuracy, more the further it is off.** Within a few per cent, as on
  ARKitScenes, `freiburg3` and the iPhone X, the error moves by no consistent amount. The
  Xiaomi's 10.9% makes it a median 1.7 times as large, larger on 7 of its 8 sequences.
- **RGB-D pays more than monocular on the same sensor:** 2.67 against 1.37 times on
  `freiburg1`, 1.77 against 1.02 on `freiburg2`. With depth measured, a wrong focal length
  scales a point's sideways offset (x = (u - cx) z / fx) but not its depth, so the map is
  stretched in a way no rigid alignment removes.
- **ADVIO shows no cost** from a 7% error and its distortion dropped: worse on 11 of 23, a
  median ratio of 1.00. Errors of tens of centimetres to metres over its tracked stretches
  probably hide the lens's share.
- **The guess does not cost the track.** The share moved by more than 10 points on 14 of the 77
  runs, against 5 with the seed alone: longer on 10, shorter on 4. The runs that went on are not
  better runs. fr2_360_hemisphere RGB-D tracks the whole sequence with the guess, against 22%
  exact, at 1.77 times the error on the frames both tracked. Over all 73, the ratio's median is
  1.08, worse on 43.

The design note's section 1 saw the same on one recording, with a larger error. A video's
intrinsics are worth finding, and `eval.lenses`' profiles and Stray Scanner's camera matrix find
them. A1 changes nothing here.

## ADVIO at 720 pixels against 480

Four sequences (02, 11, 13 and 20, from three of the four capture batches) imported at a
720-pixel short side, 720x1280, and run with 2,000 features. The share is the median over the
three seeds; the ATE ratio is on the frames both tracked, as above, and the latency is seed 0's,
each run alone.

| Sequence | Tracked, 480 / 720 | ATE ratio, 720 / 480 | p95, 480 / 720 | Late at 720 |
|---|---|---|---|---|
| advio-02 | 30% / 23% | 1.67 | 14.1 / 25.3 ms | 0 |
| advio-11 | 5% / 4% | 0.54 | 13.8 / 27.7 ms | 0 |
| advio-13 | 31% / 6% | 0.97 | 12.8 / 24.1 ms | 1 of 413 |
| advio-20 | 9% / 9% | 0.88 | 19.8 / 36.7 ms | 441 of 892 |

- **720 does not keep the track.** It is shorter on 13 and 02, and the same on 11 and 20.
- **Its accuracy is mixed:** lower error on three sequences, by a median ratio of 0.93 over
  all four, and 1.67 times on 02.
- **It costs real time:** a frame's p95 is 1.8 to 2.0 times as long, and advio-20 misses the
  period on half its frames. At 480 no frame is late.

A 480-pixel short side stays the import's. Below it, the design note's section 1 lost the track
sooner; above it, no track is gained. Stray Scanner's video is not in the public set, so
its 720 is not measured here. A private config can list a recording, its results kept outside
the repository.

## LiDAR depth of medium confidence against high

ARKitScenes' ten captures in RGB-D, imported again keeping depth of medium confidence and up
(`--min-confidence 1`). Sampled every 30th frame, high confidence leaves a median of 89.5% of a
frame's pixels with depth (82.0 to 93.5% by capture), and medium 95.5% (93.4 to 98.8%).

- **The share tracked does not move:** within 8 frames on every capture, at every seed.
- **The error does not consistently drop.** On the frames both tracked, the ratio is 1.03 by
  median (0.74 to 1.65), worse on 5 of the 10.

High confidence stays the default. Indoors, at an iPad's range, it already keeps nine pixels in
ten, and what medium adds does not improve the runs.

## The references

- **ARKitScenes' poses** are ARKit's own visual-inertial estimate, so an ATE against them is
  agreement with ARKit, not accuracy. They come at 10 Hz, between the 30 Hz frames, and the
  import interpolates them at each frame's time (`arkitscenes.at_frames`): linear for the
  position, along the shortest arc for the rotation, across spans of at most 0.15 s. Leaving
  out each of the ten captures' 10,906 poses with neighbours at most 0.25 s apart, and
  interpolating it from those neighbours, 0.2 s apart, misses by a median of 1.6 mm and 0.28°
  (95th percentile 4.4 mm and 0.81°). A frame's span is half that. ARKit's poses begin 0.6 to
  0.8 s after a capture's first frame, so its first 18 to 25 frames have none. 42444946 also
  has a gap of 1.17 s from 6.0 s, inside its RGB-D run's tracked stretch: that run scores 196
  of its 250 tracked frames.
- **ARKit's intrinsics** change from frame to frame with the lens's focus. The import keeps
  their median. On 42444946 and 42446016, 95.7% and 98.1% of frames are within 1% of it.
- **ADVIO's calibrations** are the published Kalibr results, one per capture batch, and the
  import keeps them as published. Batch iphone-01 (sequences 18 and 19) is suspect: its `cy`
  differs from its README's, and its `p2` moves an edge pixel about 50 pixels at 720x1280. The
  panel's algorithm review took ORB matches 15 frames apart, undistorted them with each
  variant, and compared the two-view rotations with the reference (its L11). That check favours
  `p2 = 0`: a median error of 0.45° against 0.63° published on advio-18, and 0.43° against 0.47°
  on advio-19. It cannot judge `cy`, to which a two-view rotation is nearly blind. advio-18's
  nominal runs, without distortion, score 0.30 to 0.39 times the exact ones' error on the same
  frames at the three seeds, which fits, but those runs change the focal length too. A
  self-calibration would settle it.

## What it does not do yet

- **Say why a frame fails.** A2 tables the check that failed at each loss. What in the images
  made it fail, whether fast turns, blur or the rolling shutter, is not read from them.
- **Join a run's maps.** Each map is in its own frame and, monocular, at its own scale.
  Relocalising against a lost map, and joining maps where a place is recognised, are milestone
  2B's.
- **Start a monocular map in real time.** A failed two-view start takes up to 0.43 s on ADVIO
  (A2's results). Whether ORB-SLAM's 200 samples a model would bound it is not measured.
- **Run Python and Rust on the test set.** Only C++ ran. The three agree frame by frame on the
  synthetic rooms' whole runs (`docs/notes/edge-vo.md`), and since A2 across the lost room's new
  map.
- **Pin the latency.** The p95 figures are from one unpinned run each on a shared machine:
  they show the margin, not a budget's verdict.
- **Measure Stray Scanner at 720,** which needs a recording outside the public set.

## What was learned

- **One run of a change can mislead.** A track's length turns on one moment, so a lens, a
  size or a seed can move it by most of a sequence: advio-05 tracks 1% or 77% by its seed
  alone. Accuracy compared on the frames both runs tracked, over several seeds, shows what the
  lengths hide.
- **The lens matters in proportion to its error, and most with depth.** Small errors are lost
  in the run's own; a 10% error is not, and an RGB-D map absorbs one least.
- **More pixels do not keep a phone's track.** ADVIO at 720 tracks no longer and takes nearly
  twice the time a frame. On ADVIO, resolution is not what loses the track.
- **Most of the set's frames come after a loss.** 65% are never run. A2's new map addresses
  more of the test set than any import could.
- **A new map runs them, and changes nothing before.** With A2 the set's tracked share goes
  from 31% to 87% of its frames, and every frame A1 ran is decided as A1 decided it.
- **It recovers frames, not a trajectory.** A kept map spans a median of 4.5 to 24.6 s by
  dataset and mode, each in its own frame: a run is pieces until milestone 2B joins them.
- **A track mostly thins out before it is lost.** Over half the losses are frames the motion
  model or PnP placed whose local map no longer held 30 inliers, after a frame that kept a
  median of 35.

## Every run

Seed 0, exact intrinsics, the share at seeds 1 and 2 beside it, and the p95 latency against
the frame period. ARKitScenes' captures are named by their video id; `fr` is TUM's
`freiburg`.

| Sequence | Mode | Frames | Tracked | From-to | Lost | ATE (matched) | Seeds 1, 2 | p95 / period |
|---|---|---|---|---|---|---|---|---|
| fr1_360 | mono | 756 | 26 (3%) | 0.1-0.9 s | yes | 1.9 cm (26) | 3%, 3% | 13 / 32 ms |
| fr1_360 | RGB-D | 756 | 756 (100%) | 0.0-25.2 s | no | 27.1 cm (756) | 100%, 100% | 10 / 32 ms |
| fr1_rpy | mono | 723 | 47 (7%) | 0.8-2.4 s | yes | 1.4 cm (47) | 6%, 6% | 12 / 32 ms |
| fr1_rpy | RGB-D | 723 | 723 (100%) | 0.0-24.1 s | no | 3.4 cm (720) | 100%, 100% | 10 / 32 ms |
| fr2_rpy | mono | 3290 | 2687 (82%) | 20.1-109.7 s | no | 0.8 cm (2687) | 82%, 82% | 11 / 32 ms |
| fr2_rpy | RGB-D | 3290 | 3287 (100%) | 0.0-109.7 s | no | 0.5 cm (3287) | 100%, 100% | 10 / 32 ms |
| fr2_360_hemisphere | mono | 2729 | 110 (4%) | 0.9-4.6 s | yes | 6.7 cm (110) | 4%, 4% | 11 / 32 ms |
| fr2_360_hemisphere | RGB-D | 2729 | 599 (22%) | 0.0-20.0 s | yes | 10.8 cm (599) | 22%, 22% | 10 / 32 ms |
| fr2_360_kidnap | mono | 1431 | 280 (20%) | 0.8-10.1 s | yes | 10.3 cm (280) | 20%, 28% | 14 / 32 ms |
| fr2_360_kidnap | RGB-D | 1431 | 435 (30%) | 0.0-14.5 s | yes | 8.5 cm (435) | 30%, 30% | 10 / 32 ms |
| fr1_floor | mono | 1242 | 669 (54%) | 0.8-23.1 s | yes | 17.3 cm (669) | 54%, 54% | 10 / 32 ms |
| fr1_floor | RGB-D | 1242 | 880 (71%) | 0.0-31.3 s | yes | 2.2 cm (879) | 71%, 71% | 11 / 32 ms |
| fr3_nostructure_texture_near_withloop | mono | 1682 | 1555 (92%) | 1.5-53.8 s | yes | 6.4 cm (1555) | 97%, 97% | 9 / 32 ms |
| fr3_nostructure_texture_near_withloop | RGB-D | 1682 | 1639 (97%) | 0.0-56.6 s | no | 4.8 cm (1636) | 97%, 97% | 10 / 32 ms |
| fr3_structure_notexture_far | mono | 814 | 0 (0%) | - | no | - | 0%, 0% | 5 / 32 ms |
| fr3_structure_notexture_far | RGB-D | 814 | 251 (31%) | 18.9-27.4 s | no | 0.9 cm (251) | 31%, 31% | 6 / 32 ms |
| fr3_nostructure_notexture_far | mono | 474 | 0 (0%) | - | no | - | 0%, 0% | 6 / 32 ms |
| fr3_nostructure_notexture_far | RGB-D | 474 | 0 (0%) | - | no | - | 0%, 0% | 6 / 32 ms |
| advio-01 | mono | 7795 | 75 (1%) | 4.7-7.2 s | yes | 3.3 cm (75) | 1%, 1% | 17 / 33 ms |
| advio-02 | mono | 5964 | 46 (1%) | 6.9-8.4 s | yes | 10.3 cm (46) | 30%, 30% | 14 / 33 ms |
| advio-03 | mono | 4423 | 93 (2%) | 2.6-5.6 s | yes | 32.5 cm (93) | 2%, 2% | 14 / 33 ms |
| advio-04 | mono | 3512 | 351 (10%) | 2.7-14.4 s | yes | 100.4 cm (351) | 10%, 15% | 14 / 33 ms |
| advio-05 | mono | 4835 | 61 (1%) | 7.4-9.4 s | yes | 8.0 cm (61) | 1%, 77% | 12 / 33 ms |
| advio-06 | mono | 8268 | 1415 (17%) | 7.0-54.1 s | yes | 134.7 cm (1415) | 17%, 17% | 13 / 33 ms |
| advio-07 | mono | 2859 | 148 (5%) | 10.0-14.9 s | yes | 6.4 cm (148) | 7%, 7% | 15 / 33 ms |
| advio-08 | mono | 3206 | 22 (1%) | 10.9-11.6 s | yes | 1.3 cm (22) | 1%, 1% | 17 / 33 ms |
| advio-09 | mono | 2891 | 168 (6%) | 4.1-9.9 s | yes | 36.6 cm (168) | 6%, 6% | 22 / 33 ms |
| advio-10 | mono | 3686 | 3347 (91%) | 3.0-114.5 s | yes | 768.2 cm (3347) | 17%, 91% | 12 / 33 ms |
| advio-11 | mono | 6198 | 338 (5%) | 2.9-14.2 s | yes | 232.1 cm (338) | 8%, 4% | 14 / 33 ms |
| advio-12 | mono | 3441 | 353 (10%) | 4.0-15.7 s | yes | 200.7 cm (353) | 10%, 10% | 18 / 33 ms |
| advio-13 | mono | 4550 | 1421 (31%) | 3.7-51.0 s | yes | 79.1 cm (1421) | 31%, 31% | 13 / 33 ms |
| advio-14 | mono | 3373 | 301 (9%) | 2.2-12.3 s | yes | 23.3 cm (301) | 9%, 9% | 11 / 33 ms |
| advio-15 | mono | 1553 | 1467 (94%) | 2.9-51.7 s | no | 81.9 cm (1467) | 95%, 95% | 16 / 33 ms |
| advio-16 | mono | 4389 | 94 (2%) | 1.1-4.2 s | yes | 8.6 cm (94) | 2%, 2% | 18 / 33 ms |
| advio-17 | mono | 4724 | 594 (13%) | 1.1-20.9 s | yes | 32.3 cm (594) | 13%, 13% | 11 / 33 ms |
| advio-18 | mono | 5913 | 665 (11%) | 3.1-25.3 s | yes | 120.7 cm (665) | 11%, 11% | 12 / 33 ms |
| advio-19 | mono | 4262 | 1098 (26%) | 2.5-39.1 s | yes | 107.5 cm (1098) | 26%, 26% | 12 / 33 ms |
| advio-20 | mono | 9076 | 738 (8%) | 1.6-26.3 s | yes | 427.5 cm (738) | 9%, 9% | 20 / 33 ms |
| advio-21 | mono | 9520 | 9158 (96%) | 1.2-306.2 s | yes | 1747.3 cm (9158) | 96%, 96% | 15 / 33 ms |
| advio-22 | mono | 11572 | 2558 (22%) | 1.2-86.4 s | yes | 185.4 cm (2558) | 22%, 22% | 13 / 33 ms |
| advio-23 | mono | 6169 | 296 (5%) | 0.9-10.8 s | yes | 21.3 cm (296) | 5%, 5% | 14 / 33 ms |
| 42444946 | mono | 2822 | 173 (6%) | 0.4-6.1 s | yes | 24.2 cm (164) | 6%, 6% | 11 / 33 ms |
| 42444946 | RGB-D | 2822 | 250 (9%) | 0.0-8.3 s | yes | 49.3 cm (196) | 9%, 9% | 10 / 33 ms |
| 42444966 | mono | 3386 | 209 (6%) | 0.7-7.6 s | yes | 6.5 cm (207) | 6%, 6% | 11 / 33 ms |
| 42444966 | RGB-D | 3386 | 230 (7%) | 0.0-7.6 s | yes | 3.8 cm (207) | 7%, 7% | 9 / 33 ms |
| 42445021 | mono | 1449 | 436 (30%) | 1.0-15.5 s | yes | 18.8 cm (436) | 30%, 29% | 11 / 33 ms |
| 42445021 | RGB-D | 1449 | 509 (35%) | 0.0-16.9 s | yes | 21.1 cm (490) | 35%, 35% | 8 / 33 ms |
| 42445028 | mono | 1992 | 243 (12%) | 1.8-9.8 s | yes | 2.0 cm (243) | 12%, 12% | 10 / 33 ms |
| 42445028 | RGB-D | 1992 | 297 (15%) | 0.0-9.9 s | yes | 3.9 cm (272) | 15%, 15% | 9 / 33 ms |
| 42445429 | mono | 2527 | 513 (20%) | 7.3-24.4 s | yes | 6.2 cm (513) | 24%, 6% | 10 / 33 ms |
| 42445429 | RGB-D | 2527 | 739 (29%) | 0.0-24.6 s | yes | 5.3 cm (720) | 29%, 29% | 10 / 33 ms |
| 42445966 | mono | 5110 | 420 (8%) | 1.1-15.1 s | yes | 1.9 cm (420) | 8%, 8% | 11 / 33 ms |
| 42445966 | RGB-D | 5110 | 446 (9%) | 0.0-14.9 s | yes | 2.5 cm (428) | 9%, 9% | 9 / 33 ms |
| 42445984 | mono | 2939 | 1419 (48%) | 0.7-48.0 s | yes | 13.7 cm (1419) | 48%, 44% | 10 / 33 ms |
| 42445984 | RGB-D | 2939 | 2731 (93%) | 0.0-91.0 s | yes | 44.9 cm (2712) | 93%, 93% | 10 / 33 ms |
| 42446016 | mono | 6820 | 1954 (29%) | 2.2-67.3 s | yes | 4.8 cm (1954) | 29%, 28% | 9 / 33 ms |
| 42446016 | RGB-D | 6820 | 5476 (80%) | 0.0-183.2 s | yes | 3.5 cm (5457) | 80%, 80% | 10 / 33 ms |
| 42446100 | mono | 4025 | 1036 (26%) | 2.3-36.8 s | yes | 12.9 cm (1036) | 26%, 26% | 11 / 33 ms |
| 42446100 | RGB-D | 4025 | 4025 (100%) | 0.0-134.1 s | no | 7.2 cm (4004) | 100%, 100% | 9 / 33 ms |
| 42446137 | mono | 1936 | 115 (6%) | 0.4-4.2 s | yes | 2.3 cm (109) | 6%, 6% | 9 / 33 ms |
| 42446137 | RGB-D | 1936 | 1936 (100%) | 0.0-64.5 s | no | 7.2 cm (1918) | 100%, 100% | 8 / 33 ms |
| zju-A0 | mono | 1742 | 589 (34%) | 9.2-28.8 s | yes | 5.3 cm (589) | 34%, 34% | 10 / 33 ms |
| zju-A1 | mono | 3555 | 1309 (37%) | 21.5-65.1 s | yes | 37.3 cm (1309) | 19%, 2% | 10 / 33 ms |
| zju-A2 | mono | 2175 | 1620 (74%) | 7.7-61.7 s | yes | 8.5 cm (1620) | 74%, 74% | 11 / 33 ms |
| zju-A3 | mono | 1335 | 1171 (88%) | 5.4-44.5 s | no | 1.6 cm (1171) | 88%, 88% | 12 / 33 ms |
| zju-A4 | mono | 2895 | 2608 (90%) | 9.6-96.5 s | no | 2.1 cm (2608) | 90%, 90% | 11 / 33 ms |
| zju-A5 | mono | 2275 | 2142 (94%) | 4.4-75.8 s | no | 3.4 cm (2142) | 94%, 94% | 11 / 33 ms |
| zju-A6 | mono | 2143 | 1200 (56%) | 11.8-51.8 s | yes | 1.0 cm (1200) | 56%, 56% | 10 / 33 ms |
| zju-A7 | mono | 2194 | 1959 (89%) | 7.8-73.1 s | no | 0.5 cm (1959) | 89%, 89% | 11 / 33 ms |
| zju-B0 | mono | 1188 | 125 (11%) | 1.1-5.3 s | yes | 15.2 cm (125) | 11%, 11% | 12 / 33 ms |
| zju-B1 | mono | 1705 | 204 (12%) | 1.3-8.1 s | yes | 0.9 cm (204) | 12%, 12% | 12 / 33 ms |
| zju-B2 | mono | 1897 | 176 (9%) | 1.3-7.1 s | yes | 0.9 cm (176) | 9%, 9% | 12 / 33 ms |
| zju-B3 | mono | 1426 | 265 (19%) | 0.3-9.1 s | yes | 1.3 cm (265) | 19%, 18% | 10 / 33 ms |
| zju-B4 | mono | 1368 | 132 (10%) | 0.4-4.8 s | yes | 1.1 cm (132) | 10%, 10% | 12 / 33 ms |
| zju-B5 | mono | 2144 | 881 (41%) | 0.7-30.0 s | yes | 0.7 cm (881) | 41%, 41% | 10 / 33 ms |
| zju-B6 | mono | 2134 | 602 (28%) | 10.0-30.0 s | yes | 1.5 cm (602) | 28%, 28% | 13 / 33 ms |
| zju-B7 | mono | 1880 | 712 (38%) | 6.3-30.0 s | yes | 1.0 cm (712) | 38%, 38% | 13 / 33 ms |

## Every run, A2

Seed 0, as above: the frames tracked across all maps, A1's share beside them, the maps kept
(and reset), the losses, the ATE pooled over the maps, the share at seeds 1 and 2, and the p95
latency against the frame period.

| Sequence | Mode | Frames | Tracked | A1 | Maps kept (reset) | Losses | ATE (matched) | Seeds 1, 2 | p95 / period |
|---|---|---|---|---|---|---|---|---|---|
| fr1_360 | mono | 756 | 557 (74%) | 3% | 5 (4) | 8 | 5.5 cm (557) | 67%, 78% | 10 / 32 ms |
| fr1_360 | RGB-D | 756 | 756 (100%) | 100% | 1 (0) | 0 | 27.1 cm (756) | 100%, 100% | 10 / 32 ms |
| fr1_rpy | mono | 723 | 587 (81%) | 7% | 2 (0) | 1 | 2.5 cm (584) | 81%, 81% | 11 / 32 ms |
| fr1_rpy | RGB-D | 723 | 723 (100%) | 100% | 1 (0) | 0 | 3.4 cm (720) | 100%, 100% | 10 / 32 ms |
| fr2_rpy | mono | 3290 | 2687 (82%) | 82% | 1 (0) | 0 | 0.8 cm (2687) | 82%, 82% | 11 / 32 ms |
| fr2_rpy | RGB-D | 3290 | 3287 (100%) | 100% | 1 (0) | 0 | 0.5 cm (3287) | 100%, 100% | 10 / 32 ms |
| fr2_360_hemisphere | mono | 2729 | 1023 (37%) | 4% | 6 (1) | 7 | 10.9 cm (1023) | 32%, 37% | 12 / 32 ms |
| fr2_360_hemisphere | RGB-D | 2729 | 2722 (100%) | 22% | 2 (0) | 1 | 20.8 cm (2699) | 100%, 100% | 9 / 32 ms |
| fr2_360_kidnap | mono | 1431 | 1069 (75%) | 20% | 2 (1) | 2 | 8.7 cm (1069) | 80%, 83% | 12 / 32 ms |
| fr2_360_kidnap | RGB-D | 1431 | 1302 (91%) | 30% | 2 (1) | 2 | 13.2 cm (1302) | 91%, 91% | 9 / 32 ms |
| fr1_floor | mono | 1242 | 816 (66%) | 54% | 4 (0) | 3 | 15.7 cm (816) | 66%, 66% | 11 / 32 ms |
| fr1_floor | RGB-D | 1242 | 1118 (90%) | 71% | 3 (1) | 3 | 2.0 cm (1117) | 90%, 90% | 11 / 32 ms |
| fr3_nostructure_texture_near_withloop | mono | 1682 | 1630 (97%) | 92% | 2 (0) | 1 | 6.2 cm (1630) | 97%, 97% | 9 / 32 ms |
| fr3_nostructure_texture_near_withloop | RGB-D | 1682 | 1639 (97%) | 97% | 1 (0) | 0 | 4.8 cm (1636) | 97%, 97% | 10 / 32 ms |
| fr3_structure_notexture_far | mono | 814 | 0 (0%) | 0% | 0 (0) | 0 | - | 0%, 0% | 5 / 32 ms |
| fr3_structure_notexture_far | RGB-D | 814 | 251 (31%) | 31% | 1 (0) | 0 | 0.9 cm (251) | 31%, 31% | 6 / 32 ms |
| fr3_nostructure_notexture_far | mono | 474 | 0 (0%) | 0% | 0 (0) | 0 | - | 0%, 0% | 5 / 32 ms |
| fr3_nostructure_notexture_far | RGB-D | 474 | 0 (0%) | 0% | 0 (0) | 0 | - | 0%, 0% | 6 / 32 ms |
| advio-01 | mono | 7795 | 5670 (73%) | 1% | 18 (6) | 23 | 213.6 cm (5670) | 86%, 80% | 15 / 33 ms |
| advio-02 | mono | 5964 | 5478 (92%) | 1% | 8 (0) | 7 | 259.1 cm (5478) | 93%, 92% | 13 / 33 ms |
| advio-03 | mono | 4423 | 4161 (94%) | 2% | 11 (1) | 12 | 151.1 cm (4161) | 92%, 92% | 13 / 33 ms |
| advio-04 | mono | 3512 | 2998 (85%) | 10% | 14 (2) | 16 | 107.4 cm (2998) | 85%, 88% | 14 / 33 ms |
| advio-05 | mono | 4835 | 4349 (90%) | 1% | 4 (1) | 4 | 152.8 cm (4349) | 90%, 91% | 14 / 33 ms |
| advio-06 | mono | 8268 | 7501 (91%) | 17% | 9 (2) | 11 | 226.2 cm (7501) | 92%, 94% | 13 / 33 ms |
| advio-07 | mono | 2859 | 1804 (63%) | 5% | 9 (2) | 10 | 20.4 cm (1804) | 53%, 64% | 15 / 33 ms |
| advio-08 | mono | 3206 | 2513 (78%) | 1% | 9 (4) | 12 | 127.8 cm (2513) | 80%, 79% | 16 / 33 ms |
| advio-09 | mono | 2891 | 2527 (87%) | 6% | 3 (1) | 3 | 94.6 cm (2527) | 90%, 90% | 16 / 33 ms |
| advio-10 | mono | 3686 | 3404 (92%) | 91% | 2 (0) | 2 | 761.7 cm (3404) | 91%, 92% | 15 / 33 ms |
| advio-11 | mono | 6198 | 5207 (84%) | 5% | 6 (2) | 7 | 134.9 cm (5207) | 86%, 82% | 13 / 33 ms |
| advio-12 | mono | 3441 | 3165 (92%) | 10% | 5 (0) | 4 | 208.3 cm (3165) | 90%, 92% | 11 / 33 ms |
| advio-13 | mono | 4550 | 4065 (89%) | 31% | 8 (1) | 8 | 307.9 cm (4065) | 89%, 93% | 14 / 33 ms |
| advio-14 | mono | 3373 | 1775 (53%) | 9% | 8 (1) | 8 | 53.5 cm (1775) | 52%, 50% | 12 / 33 ms |
| advio-15 | mono | 1553 | 1467 (94%) | 94% | 1 (0) | 0 | 81.9 cm (1467) | 95%, 95% | 15 / 33 ms |
| advio-16 | mono | 4389 | 3502 (80%) | 2% | 12 (1) | 13 | 530.8 cm (3502) | 80%, 80% | 16 / 33 ms |
| advio-17 | mono | 4724 | 4240 (90%) | 13% | 10 (2) | 11 | 236.7 cm (4240) | 89%, 91% | 13 / 33 ms |
| advio-18 | mono | 5913 | 4311 (73%) | 11% | 10 (6) | 15 | 171.8 cm (4311) | 76%, 76% | 13 / 33 ms |
| advio-19 | mono | 4262 | 3930 (92%) | 26% | 7 (0) | 6 | 111.5 cm (3930) | 91%, 89% | 14 / 33 ms |
| advio-20 | mono | 9076 | 8773 (97%) | 8% | 3 (0) | 3 | 795.2 cm (8773) | 98%, 97% | 18 / 33 ms |
| advio-21 | mono | 9520 | 9338 (98%) | 96% | 3 (2) | 4 | 1730.4 cm (9338) | 98%, 98% | 15 / 33 ms |
| advio-22 | mono | 11572 | 11440 (99%) | 22% | 8 (0) | 7 | 497.0 cm (11440) | 99%, 99% | 13 / 33 ms |
| advio-23 | mono | 6169 | 6009 (97%) | 5% | 9 (1) | 9 | 270.1 cm (6009) | 97%, 99% | 18 / 33 ms |
| 42444946 | mono | 2822 | 1473 (52%) | 6% | 6 (0) | 5 | 31.8 cm (1464) | 54%, 49% | 9 / 33 ms |
| 42444946 | RGB-D | 2822 | 2059 (73%) | 9% | 8 (1) | 8 | 16.1 cm (2005) | 73%, 73% | 9 / 33 ms |
| 42444966 | mono | 3386 | 3217 (95%) | 6% | 6 (0) | 5 | 10.5 cm (3212) | 96%, 95% | 9 / 33 ms |
| 42444966 | RGB-D | 3386 | 3311 (98%) | 7% | 3 (0) | 2 | 3.1 cm (3285) | 98%, 98% | 9 / 33 ms |
| 42445021 | mono | 1449 | 1163 (80%) | 30% | 4 (0) | 3 | 12.0 cm (1161) | 82%, 79% | 9 / 33 ms |
| 42445021 | RGB-D | 1449 | 1311 (90%) | 35% | 3 (0) | 2 | 13.3 cm (1290) | 90%, 90% | 8 / 33 ms |
| 42445028 | mono | 1992 | 1537 (77%) | 12% | 2 (0) | 1 | 30.6 cm (1535) | 77%, 71% | 8 / 33 ms |
| 42445028 | RGB-D | 1992 | 1920 (96%) | 15% | 3 (0) | 2 | 7.7 cm (1893) | 96%, 96% | 9 / 33 ms |
| 42445429 | mono | 2527 | 1519 (60%) | 20% | 4 (2) | 5 | 6.4 cm (1516) | 56%, 64% | 10 / 33 ms |
| 42445429 | RGB-D | 2527 | 2272 (90%) | 29% | 4 (0) | 3 | 4.4 cm (2250) | 90%, 90% | 10 / 33 ms |
| 42445966 | mono | 5110 | 4765 (93%) | 8% | 8 (0) | 7 | 10.4 cm (4764) | 93%, 93% | 10 / 33 ms |
| 42445966 | RGB-D | 5110 | 5018 (98%) | 9% | 4 (0) | 3 | 4.3 cm (4999) | 98%, 98% | 9 / 33 ms |
| 42445984 | mono | 2939 | 2683 (91%) | 48% | 5 (0) | 4 | 15.5 cm (2683) | 92%, 91% | 10 / 33 ms |
| 42445984 | RGB-D | 2939 | 2906 (99%) | 93% | 2 (0) | 1 | 43.5 cm (2887) | 99%, 99% | 10 / 33 ms |
| 42446016 | mono | 6820 | 6741 (99%) | 29% | 2 (0) | 1 | 77.1 cm (6741) | 94%, 96% | 10 / 33 ms |
| 42446016 | RGB-D | 6820 | 6675 (98%) | 80% | 2 (1) | 2 | 3.4 cm (6656) | 98%, 98% | 10 / 33 ms |
| 42446100 | mono | 4025 | 3943 (98%) | 26% | 2 (0) | 1 | 13.6 cm (3941) | 94%, 94% | 9 / 33 ms |
| 42446100 | RGB-D | 4025 | 4025 (100%) | 100% | 1 (0) | 0 | 7.2 cm (4004) | 100%, 100% | 9 / 33 ms |
| 42446137 | mono | 1936 | 1718 (89%) | 6% | 7 (0) | 6 | 8.3 cm (1712) | 91%, 97% | 10 / 33 ms |
| 42446137 | RGB-D | 1936 | 1936 (100%) | 100% | 1 (0) | 0 | 7.2 cm (1918) | 100%, 100% | 8 / 33 ms |
| zju-A0 | mono | 1742 | 1438 (83%) | 34% | 4 (0) | 3 | 6.9 cm (1438) | 83%, 83% | 10 / 33 ms |
| zju-A1 | mono | 3555 | 2707 (76%) | 37% | 5 (0) | 4 | 27.0 cm (2707) | 77%, 77% | 10 / 33 ms |
| zju-A2 | mono | 2175 | 1912 (88%) | 74% | 2 (0) | 1 | 7.9 cm (1912) | 88%, 88% | 11 / 33 ms |
| zju-A3 | mono | 1335 | 1171 (88%) | 88% | 1 (0) | 0 | 1.6 cm (1171) | 88%, 88% | 12 / 33 ms |
| zju-A4 | mono | 2895 | 2608 (90%) | 90% | 1 (0) | 0 | 2.1 cm (2608) | 90%, 90% | 11 / 33 ms |
| zju-A5 | mono | 2275 | 2142 (94%) | 94% | 1 (0) | 0 | 3.4 cm (2142) | 94%, 94% | 11 / 33 ms |
| zju-A6 | mono | 2143 | 1780 (83%) | 56% | 2 (0) | 1 | 0.9 cm (1780) | 83%, 83% | 10 / 33 ms |
| zju-A7 | mono | 2194 | 1959 (89%) | 89% | 1 (0) | 0 | 0.5 cm (1959) | 89%, 89% | 11 / 33 ms |
| zju-B0 | mono | 1188 | 979 (82%) | 11% | 7 (0) | 6 | 5.5 cm (979) | 82%, 82% | 11 / 33 ms |
| zju-B1 | mono | 1705 | 1506 (88%) | 12% | 4 (0) | 4 | 15.0 cm (1506) | 88%, 88% | 11 / 33 ms |
| zju-B2 | mono | 1897 | 1430 (75%) | 9% | 8 (1) | 8 | 1.0 cm (1430) | 75%, 76% | 10 / 33 ms |
| zju-B3 | mono | 1426 | 1357 (95%) | 19% | 4 (0) | 3 | 8.0 cm (1357) | 94%, 94% | 9 / 33 ms |
| zju-B4 | mono | 1368 | 762 (56%) | 10% | 17 (7) | 23 | 7.0 cm (762) | 57%, 58% | 11 / 33 ms |
| zju-B5 | mono | 2144 | 1979 (92%) | 41% | 5 (0) | 4 | 0.7 cm (1979) | 92%, 92% | 10 / 33 ms |
| zju-B6 | mono | 2134 | 1540 (72%) | 28% | 5 (0) | 4 | 1.3 cm (1540) | 72%, 72% | 12 / 33 ms |
| zju-B7 | mono | 1880 | 1327 (71%) | 38% | 4 (0) | 3 | 1.0 cm (1327) | 71%, 71% | 13 / 33 ms |
