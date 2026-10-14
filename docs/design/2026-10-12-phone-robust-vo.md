# VO that survives a phone (milestone 2A, PRs A1 and A2)

Milestone 2A, the first of the two milestones the ladder places before milestone 3
([Milestones 2A and 2B](2026-10-02-slam-it-ladder.md#milestones)):
- **A1** measures the pipeline on a public test set of hard handheld and phone sequences, and
  imports phone data better: Stray Scanner's LiDAR depth as RGB-D, feature settings scaled to
  the frame size, and warnings where an upload will track poorly.
- **A2** lets a run go on after a lost track, in a new map, as ORB-SLAM3 does, so the rest of a
  clip is processed.

Decided with evan-wes on 2026-10-12:
- **Order:** A1, then A2, on A1's measurements.
- **The test set is public,** with ground truth or a reference; the owner's recordings are a
  demo's check, not the yardstick.
- **Depth:** ARKit's high-confidence depth by default; medium measured beside it.
- **Features:** scaled with the image's area against ORB-SLAM's 1,000 at 640x480, within 1,000
  to 2,000 (decided 2026-10-13): ORB-SLAM's settings never ask for fewer than 1,000, and a
  stereo or RGB-D frame needs more than 500 keypoints to start the map, which a floor of 500
  would never give.
- **The worker** runs a Stray Scanner zip that has depth as RGB-D, and monocular otherwise.
- **Upload warnings** never refuse an upload.

Decided with evan-wes on 2026-10-13, for A2:
- **A lost map** is kept, or reset if it has 10 keyframes or fewer, as ORB-SLAM3 resets its
  active map: its keyframes and points removed, its frames' poses dropped. The 10 is a setting,
  `reset_keyframes`.
- **The depth start** is ORB-SLAM's alone: a frame with more than 500 keypoints starts a map,
  however few have depth. `min_init_depth_points`, which stood in for the reset, goes (open
  item L13).
- **Scoring:** each kept map is aligned on its own; the measure reports each map's ATE and one
  over all their frames, beside the share of the sequence tracked across all maps.
- **The losses:** the note records which check failed at each loss, from the diagnostics the
  runs already write.

## 1. The problem, as measured

The first two phone clips through the demo, both from an iPhone 14 Pro, lost the track early:
- **Recorded in the browser** ("Take Video" on the upload page): the frames arrived at 360x480,
  with no lens metadata, so the intrinsics were the nominal guess; tracked for 2 s.
- **Recorded with Stray Scanner** (640x480 after import, its exact camera matrix): tracked for
  13 s of 27 s, then lost while panning. While the camera turned on the spot, the map points in
  view fell from about a thousand to under a hundred: a single camera makes new points only from
  sideways motion, and each new keyframe added a few dozen at most.

Rerunning the Stray recording with one thing changed at a time (the same frames each time):
with the nominal focal length, 23% too long for this camera, tracking lasted as long but the
path's ATE against Stray's poses was about 12 times larger; at three quarters of the resolution
(480x360), tracking lasted under a quarter as long. Resolution keeps the track; intrinsics keep
it accurate. These are one recording's results, which A1's test set replaces.

## 2. How others do it

- **ORB-SLAM's settings follow the camera:** its configurations ask for 1,000 ORB features at
  640x480 (TUM) and 2,000 at 1241x376 (KITTI), with the same FAST thresholds and pyramid; only
  the count changes with the image.
- **RGB-D from a phone:** ARKit gives 256x192 LiDAR depth with a per-pixel confidence (low,
  medium, high). ORB-SLAM's RGB-D mode takes depth at a frame's keypoints as a measurement, so
  each keyframe makes points from depth rather than from parallax: a turning camera keeps its
  map.
- **A lost track:** ORB-SLAM2 relocalises against its map with the bag of words, and resets a
  map lost within its first 5 keyframes; ORB-SLAM3 stores the lost map in its atlas and starts a
  new one (resetting one of fewer than 10 keyframes), joining the two later if a place is
  recognised. Relocalisation and joining are milestone 2B's; the new map is A2's.
- **Measuring robustness:** ZJU-SenseTime's benchmark (Li et al., 2019) scores tracking
  robustness and relocalisation time beside accuracy; ORB-SLAM3's paper reports the sequences a
  system loses. A1 reports the share of each sequence tracked beside its ATE.

## 3. The test set (A1)

Public data with ground truth, kept outside the repo in `../datasets/`,
used for evaluation and never redistributed. Each reader writes the TUM-format folder the
pipeline already reads, as the Stray Scanner import does, so the pipeline itself needs nothing
new.

| Dataset | What it tests | Ground truth | Size, licence |
|---|---|---|---|
| **TUM RGB-D**, its hard sequences: `fr1_360`, `fr1_rpy`, `fr2_rpy`, `fr2_360_hemisphere`, `fr2_360_kidnap`, `fr1_floor`, and a few of `fr3`'s structure/texture set | Turning on the spot, a full turn, the camera covered (a kidnap), fast sweeps, low texture; RGB-D and monocular | Motion capture | A few GB; the reader exists |
| **ZJU-SenseTime** (Li et al., 2019): A0-A7 (Xiaomi Mi 8), B0-B7 (iPhone X) | Handheld phones: walking and looking around, texture-less areas, rapid rotation and translation, shaking, a person walking through, the camera covered, black frames; monocular | VICON, 400 Hz | 5.4 GB, 16 zips; B0, checked: greyscale 640x480 at 30 Hz, a calibrated pinhole; no licence stated; downloaded over HTTP (its HTTPS is broken) |
| **ADVIO** (Cortés et al., ECCV 2018): 23 sequences | A real iPhone's video, 1280x720 portrait at 60 fps, walking in malls, a metro station, offices and outdoors; monocular | From the IMU and known fixation points, looser than motion capture | 5.1 GB; CC BY-NC 4.0 |
| **ARKitScenes** (Baruch et al., 2021), a subset of its raw split | The LiDAR depth path: the wide camera at 640x480 and 30 fps (`vga_wide`), depth and confidence at 256x192 and 60 fps, paired by time | ARKit's own poses (`lowres_wide.traj`): a reference, not independent truth | A budget of 20 GB, captures chosen in A1 with its download script; CC BY-NC-SA 4.0 |

**The measure,** per sequence, backend and mode: the share of its frames tracked; when tracking
began; when the track was lost, if it was; the ATE over the tracked frames (Sim(3) for
monocular), against ground truth or, for ARKitScenes, against ARKit's poses, the report naming
which; the 95th-percentile latency against the frame period. A command runs the set from a
config that lists its sequences (a template in the repo); a private recording can be listed the
same way, its results kept outside the repo. Results on the public set go in a note.

## 4. A1: measured, and imported better (Python; 0.3.7.0 -> 0.3.8.0)

1. **The readers:** ZJU-SenseTime's (its camera `sensor.yaml`, `data.csv`, and ground truth in
   the body frame, moved to the camera by the camera's extrinsic), ADVIO's (its video through
   the video import, with its published calibration, to be checked), and ARKitScenes' (colour,
   depth and confidence paired by time, ARKit's poses as the reference).
2. **Depth as RGB-D,** from ARKitScenes and from a Stray Scanner zip (its `depth/` and
   `confidence/`): depth kept where confidence is high (medium as an option, to measure),
   brought to the colour frame's size by nearest neighbour (interpolating would invent depth
   across edges), zero where kept out.
3. **Features by frame size:** `n_features` from the frame's area, bounded, computed where a run
   is set up and passed to every backend as the setting it already is; no core changes.
4. **Two measurements, no code:** what a guessed lens costs (each sequence with exact intrinsics
   rerun with the nominal guess), and whether importing above a 480-pixel short side pays for
   its time (ADVIO's 720-pixel video, and Stray Scanner's).
5. **The worker:** a Stray Scanner zip with depth runs as RGB-D; the summary names the mode.
6. **Warnings, never refusals:** the upload page advises recording with the Camera app rather
   than in the browser; the status page warns when the frames arrived under 480 pixels on their
   short side, and why that may be.
7. **The test set's results,** in a note (`docs/notes/test-set.md`).

## 5. A2: a new map after a lost track (all three languages; 0.3.8.0 -> 0.3.9.0)

1. **The pipeline:** a lost track closes its map, kept or reset (above), and the next frame
   starts a new one, as ORB-SLAM3's `CreateMapInAtlas` does. Maps are numbered from 0 in the
   order they start. Each map numbers its own keyframes and points from 0, so its first
   keyframe, which local BA holds fixed and culling keeps, is always keyframe 0; the run's ids
   continue across maps, each map's offset by those before it, as ORB-SLAM3's one counter gives
   them. The map, mapping and BA code is unchanged. The run ends only at the sequence's end.
2. **The run format, version 4:** each frame's diagnostic names its map; `pose_maps.npy`,
   `keyframe_maps.npy` and `point_maps.npy` give each pose's, keyframe's and point's map; a
   reset map's keyframes and points are removed as `reset`. Scores per map (each map its own
   alignment) and the share of the sequence tracked across all: `runs.evaluate` aligns each map
   by its ATE alone and gives its RPE where it has pairs (closing A1's deferred A10), and
   `runs.tracking` gives a segment per map with the losses and the resets.
3. **The bake and the viewer,** the bundle's version 2: each map moved onto the truth by its own
   alignment; a backend's frames split by map on the timeline, a reset map's drawn broken; the
   scene shows the map of the frame shown, or the last before it, the truth only while that
   map is placed on it.
4. **The worker's summary and the status page:** the segments tracked, the first 20 of them
   so that a summary fits the server's 4,096 bytes, for example "Tracked 2.9–15.9 s and
   17.0–27.1 s of your 27.1 s clip.", the losses and the maps reset.
5. **Parity:** the lost room, the room with `lost_frames` frames blanked from frames 12 and 31,
   run with its lost maps reset and with them kept, holds the three languages to one another
   and to the golden record across new maps; a stereo or RGB-D run reaches a third map. (The
   gap room, as built, never loses the track.)
6. **The results** of the test set rerun at seeds 0, 1 and 2, in `docs/notes/test-set.md`.

## 6. Plan

- **A1** (0.3.7.0 -> 0.3.8.0): the datasets downloaded (sizes checked first); the readers and
  their tests on small fixtures; depth as RGB-D; features by frame size; the measure and its
  command; the worker's mode; the warnings; the measurements and the note. The review panel on
  the finished slice; offline CI before the merge.
- **A2** (0.3.8.0 -> 0.3.9.0): the pipeline's new map in Python, C++ and Rust; run format v4;
  the bake, the viewer and the status page; the test set rerun. The review panel; offline CI.
- **Not in 2A:** relocalisation, the covisibility graph, loop closure and joining maps (2B); the
  IMU (milestone 6), although ZJU-SenseTime and ADVIO carry it.
