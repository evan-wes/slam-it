# The run viewer: what a run records, the bake, and the page

Design note for PR 10: the run format's version 3 (`slam_it_all.eval.runs`), the per-frame
solver summaries, point and keyframe lifetimes and feature records, in all three languages; the
bake (`slam_it_all.orchestration.bundles`); and the page (`viewer/`). The plan was
`docs/design/2026-10-09-run-viewer.md`, sections 1 to 4; the forks it left open were decided
with the owner on 2026-10-10. This note is what was built and how it was checked.

## The problem

A run wrote what a pipeline decided and where it put things, but not why: the trajectory, the
final keyframes and map, and per frame its state, tracked points and step times. Seeing a run
meant reading `diagnostics.jsonl` beside a rerun recording of the final map. The viewer shows a
run frame by frame: the image as the pipeline saw it with each keypoint's fate, the map as it
stood after that frame, and how every solver went, for each backend side by side. It needs
three things a run did not record, a way to package a run with its images for a browser, and
the page.

## What a run records

**How each stage went** (always). Each frame's `diagnostics.jsonl` line gains `placed_by`,
`init`, `motion`, `pnp`, `local_map` and `mapping`, each an object or null where the stage did
not run (`slam_it_all.edge.pipeline.FrameSummary`). A bundle adjustment records each solve's
initial and final cost, iterations and why it stopped, in rung 0's names. Each stage returns its
summary with its result (decided 2026-10-10), so the pipeline only gathers them; the C++ and
Rust writers write nested JSON, C++ keeping it as text checked when written, since a copyable
recursive value recurses (`misc-no-recursion`).

**Lifetimes** (always; run format version 3). The map keeps what it removes aside, with why
(`Removal`: culled, fused, or erased with its views), and the pipeline dates each removal by its
frame. A run's `points.npy`, `keyframes.npy` and `keyframe_poses.npy` now hold every point and
keyframe it made, by id, where each last was, and `point_lives.npy` and `keyframe_lives.npy`
their lives: the frame that made it, the frame that removed it or -1, and why or -1. The final
map is the rows still alive (`Run.final_points()`, `Run.final_keyframes()`), which the
evaluation and the rerun recording read.

**Feature records** (opt-in: `--record-features`, `vo.run(..., record_features=True)`). Each
frame's keypoints, with their pixel, pyramid level, fate and the point concerned, streamed to
`feature_pixels.npy` (N, 2) float32 and `feature_records.npy` (N, 4) int32 as the run goes (two
plain arrays, decided 2026-10-10). Each header is rewritten after every frame, numpy's padding
keeping it one length, so the files read whole mid-run; `run.json` says whether the run
recorded them, so files left in a reused folder are never taken for its own. It travels with
the request, as `first` and `count` do, not in `Options`: it changes no decision.

A keypoint's **fate** is the first that applies, in the order made, tracked, rejected (decided
2026-10-10):

- **made**: a point was made of it on this frame, by initialisation or by mapping the frame as
  a keyframe, from its depth or triangulated. Mapping's points are taken before fusion, which
  may replace a new point at once.
- **tracked**: an inlier of the frame's final pose.
- **rejected**: the frame's last tracking stage matched it (the local map's search, or, on a
  frame never placed, the motion model's or the PnP fallback's), and it is not tracked in the
  end: refinement dropped it, or the frame was not tracked.
- **unmatched**: none of these. An earlier stage may have matched it: a keypoint the motion
  model matched and its refinement dropped is rejected only if the local map's search matches
  it again (a search ORB-SLAM2 would skip; deferred, the review panel's L7).

The point is the one the fate concerns: the new one, the tracked one, or the one it was matched
to and rejected from. A keypoint tracked to a point that mapping then culls can have a new point
made of it on the same keyframe, so a keyframe's tracked keypoints can be fewer than its
inliers; in the two rooms' eight runs, eight keyframes are.

**Repeatable in C++.** Checking that nothing moved found that C++ runs were not repeatable on
this desktop: Eigen blocks a large product by the CPU's cache sizes, read from the core that
runs the first one, and the Core Ultra 9 285's performance and efficiency cores report different
ones, so a run's last bits depended on where it started. Every rung now compiles with
`EIGEN_NO_CPUID`, so Eigen blocks by its own defaults on every core, and runs repeat bit for bit
on either core type (fixed first, inside PR 10, decided 2026-10-10; at compile time since the
review panel). Rust's runs repeat too: its products block by cache sizes read once per run for
the whole machine.

## The bake

`python -m slam_it_all.orchestration.bundles RUN_DIR... --sequence PATH --out BUNDLE` bakes one
sequence's runs in one mode, one per backend, into a bundle, a folder any plain file server can
serve (its layout is the module's docstring). It lives in orchestration, not eval (decided
2026-10-10): it takes each frame's image from the run's own source, so the image is the one the
pipeline saw (a stereo pair's left image rectified) and the keypoints sit where they were found.

- **Frames.** The bundle's frames are every frame a run ran, matched by time. Every life,
  feature record and pose is renumbered to them, so Python's 300 frames and the natives' whole
  run line up.
- **Alignment.** With ground truth, each run is moved onto it by its own ATE alignment (Sim(3)
  for monocular, SE(3) otherwise), points and keyframes with it, so the backends overlay; its
  header names the alignment. A run the alignment cannot place (fewer than three frames
  tracked, as a monocular run that never initialised, or its centres on a line) stays in its
  own frame, unscored, rather than failing the bundle, and the page says so.
- **Tables.** `bundle.json` carries the run format's fates and step names it was baked with,
  and the page refuses a bundle whose tables are not its own, so the two cannot drift apart.
- **Writing.** The bundle is written into a hidden folder beside its own and moved onto it
  once whole: a bake that fails leaves nothing, and baking again needs no clearing up.
- **Arrays** are `.npy` files, little-endian float32 and int32 (decided 2026-10-10): the page
  reads each straight into a typed array after its 128-byte header, and numpy reads a bundle
  directly. Each backend's records are JSON columns.
- **Images** are JPEG at quality 75, in chunks of 100 frames with each frame's byte span, so
  scrubbing fetches a few files: 34 to 51 KB a frame of EuRoC's grey ones and 35 to 37 KB of
  TUM's colour ones, where quality 90 takes 65% more on MH_01 stereo and 77% more on fr1/desk.

## The page

Plain JavaScript modules with no build step, three.js 0.186.0 from jsDelivr through an import
map, reifyscene's tokens and panels. `scripts/view_bundle.py BUNDLE` serves it with a bundle
on this machine.

- **The header**: the sequence and mode, the backend toggle, and the run's numbers (ATE, RPE,
  frames tracked, keyframes and points, latency's median and 95th percentile against the
  deadline, peak memory), each explained on hover; whether the backends' decisions agree.
- **The image**: the frame with its keypoints in their fate's colour, and each fate's count.
- **The map**: the trajectory tracked so far in its backend's colour, the others' faded beside
  it, the truth, the keyframes and points alive at the frame, and the camera, followed or
  orbited. Every point and keyframe sits in one buffer with its life, and a shader hides each
  outside it, so scrubbing changes one number.
- **The solvers**: tracked points and the local map's matches over the run, every backend's
  latency against the deadline, the frames where the backends decided differently ticked; the
  backends' step times at the frame side by side; and the frame's stage summaries, each field
  explained on hover.

The modules that read a bundle and work out what a frame shows never touch the page, so
`node --test` checks them, on the golden bundle (`tests/fixtures/orchestration/bundle/`, the
room's first frames from all three backends), against what its files say of each other rather
than fixed numbers; playback is a module of its own (`playback.js`) for the same reason. Layer
1 also runs `node --check`, `tsc --checkJs` over the JSDoc types twice (the page, and the logic
modules without the DOM, so the split holds), and prettier; TypeScript, three.js's types and
prettier are pinned in `viewer/package-lock.json` and installed by `npm ci --prefix viewer`
(decided 2026-10-10).

## Checks

- **No decision moves.** The rename's twelve comparison runs (C++ and Rust whole runs on five
  sequence-modes, Python's first 300 frames of two) were run after each commit and compared with
  the commit before: the trajectories, every existing diagnostics field and, from version 3 on,
  the final keyframes and map match byte for byte. The C++ fix is the one exception, as it was
  meant to be: it moved C++'s last bits once, and no decision. With features recorded, nothing
  else changes.
- **The languages agree on the new records**, exactly: on the room in layer 1, every
  backend's lives and feature records equal Python's; on the datasets, C++'s and Rust's
  lives and feature files are byte-identical on all five sequence-modes, and Python's 300 frames
  of MH_01 stereo record what their first 300 do.
- **The bake** holds the golden bundle's layout; each run's aligned centres sit on the truth by
  its ATE, to float32's precision, on the room and on MH_01.

## The edge note's bundles

The edge note's ten sequence-modes were run again with their features recorded (C++ and Rust
whole, Python's first 300 frames, ORB-SLAM2's settings as in the edge note's results) and each
baked into a bundle: 2.1 GB in all, kept outside the repo as runs are
(`../bundles/edge-note-2026-10-10/`). The features are one whole run's (C++'s and Rust's are the
same size) and Python's; the bake's time is on this desktop (Intel Core Ultra 9 285).

| Sequence | Mode | Frames | Bundle | Images | Features, whole run / Python | Bake |
|---|---|---|---|---|---|---|
| fr1/xyz | RGB-D | 798 | 77 MB | 29 MB | 19 / 7 MB | 4 s |
| fr1/desk | RGB-D | 596 | 60 MB | 22 MB | 14 / 7 MB | 3 s |
| fr3/long_office | RGB-D | 2,488 | 218 MB | 87 MB | 60 / 7 MB | 12 s |
| fr1/xyz | monocular | 798 | 77 MB | 29 MB | 19 / 7 MB | 4 s |
| fr1/desk | monocular | 613 | 65 MB | 22 MB | 16 / 9 MB | 3 s |
| fr3/long_office | monocular | 2,585 | 235 MB | 90 MB | 63 / 8 MB | 12 s |
| MH_01 | stereo | 3,682 | 396 MB | 163 MB | 107 / 9 MB | 49 s |
| V1_01 | stereo | 2,912 | 277 MB | 98 MB | 83 / 9 MB | 37 s |
| MH_01 | monocular | 3,682 | 388 MB | 189 MB | 92 / 11 MB | 8 s |
| V1_01 | monocular | 2,912 | 270 MB | 112 MB | 73 / 10 MB | 6 s |

Feature records are most of a bundle: 24 bytes a keypoint, 24 to 29 KB a frame for a whole run,
against 34 to 51 KB for the frame's JPEG. A monocular run finds twice its features until it
initialises, as ORB-SLAM does, so Python's first 300 frames of EuRoC monocular, which
initialises late (frame 140 on MH_01, 113 on V1_01), take a third more a frame than the whole
run. Every bundle opens in the page (headless Chromium) without an error, says its backends'
decisions agree, and scrubs to its last frame. MH_01 stereo, the largest, shows its first frame
0.6 s after the page opens, served from this machine, and its last 0.6 s after the scrubber
reaches it.

**Still real time with features recorded.** The real-time note's whole runs, at this branch's
head, each native program within `laptop-cpu`, once without feature records and once with: no
frame is late in any of the forty. Recording features moves a run's median latency by -0.5 to
+0.1 ms, and its 95th percentile and maximum by under 0.8 ms either way, which is run-to-run
noise; without them, every median is PR 9's to within 0.5 ms. MH_01 stereo in C++ stays the
tightest, its 95th percentile at 61% of the 50 ms deadline. Every run takes PR 9's decisions on
every frame and ends with its keyframes: Rust's trajectories match byte for byte, C++'s within
8e-11 m, since its products now block by fixed sizes.

**Memory.** Peak memory grows with the lifetimes, not with the features: the map keeps what it
removes, so a run holds everything it made, not only its final map. On these runs that is up to
26 MiB more than PR 9's: fr3/long_office monocular from 63 to 88 MiB in C++, and MH_01 stereo
from 133 to 159 MiB. Recording features moves it by under 6 MiB either way.

## What it does not do yet

- A phone video (PR 11): the upload, the import and the intrinsics' source in the header.
- Map snapshots after each bundle adjustment: a point sits where adjustment finally put it.
- A live camera.
