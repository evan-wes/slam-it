# The run viewer and your own video (milestone 2, PRs 10 and 11)

Milestone 2's last two slices:
- **PR 10** builds the run viewer that the ladder's
  [Public presentation](2026-10-02-slam-it-ladder.md#public-presentation) section promises.
  It is a page that replays a recorded run frame by frame, in each of the three languages.
- **PR 11** lets anyone with an invite code upload a phone video, or a Stray Scanner recording.
  It is run through the pipeline and the result opens in the same viewer. It is packaged to run
  locally, then on Railway: first at a staging address, and at `slam-it.timedilated.com`
  once the owner calls go-live.

Decided with evan-wes on 2026-10-08:
- **Recording:** in full, with per-frame features opt-in (run format v3).
- **Shape:** a static viewer that reads baked bundles, plus a thin upload server built as a job
  list and a worker.
- **Map replay:** final positions. Each point is shown from the frame that made it to the frame
  that removed it.
- **Intrinsics:** a phone's come from the best source available, and the frames are undistorted
  on import.
- **Publishing:** nothing goes public before the owner's go-live call.

Section 7 records the decisions of 2026-10-09.

## 1. What the viewer shows

**How others do it:**
- **ORB-SLAM's viewer** (Pangolin) draws the current frame with its tracked features. Beside it,
  it draws the map, the keyframe frustums and the trajectory. All of this is live: nothing is
  kept, so nothing can be replayed.
- **rerun** replays a recording. Ours (`scripts/run_vo.py --rerun`) holds only the trajectory,
  the keyframes and the final map.

Our viewer is ORB-SLAM's two views made replayable, plus a solver panel and the three-language
comparison. It has five parts:

- **The header:**
  - the bundle's sequence and mode;
  - the backend toggle;
  - a frame scrubber, with play and speed controls;
  - the run's numbers: ATE and RPE where there is ground truth, frames tracked, keyframes,
    points, latency p50 and p95 against the deadline, and peak memory;
  - for a phone video, where its intrinsics came from.
- **The image panel** shows the frame as the pipeline saw it, in colour where the source has
  it. With feature records (opt-in), each keypoint is coloured by its fate:
  - unmatched;
  - matched, then rejected as an outlier;
  - tracked;
  - made into a new map point.

  Without feature records, the panel shows the image alone.
- **The 3D panel** (three.js) shows:
  - the trajectory as it was tracked;
  - the ground truth, aligned to it;
  - the keyframe frustums and the map points alive at the current frame;
  - the current camera.

  The view can follow the camera or move freely.
- **The solver panel** shows the frame's solver summary (section 2). Plots over the whole run
  show tracked points, inliers, and step times against the deadline, with the current frame
  marked. Hovering over a field shows a one-line explanation.
- **The backend toggle** compares C++ and Rust on whole runs. Python joins only on the runs it
  covers, its first 300 frames, because it isn't real time. The page:
  - overlays the other backends' trajectories;
  - sets their step times side by side;
  - marks any frame where their decisions differ. There have been none so far, across PR 9's
    100 runs.

## 2. Recording (PR 10, all three languages): run format v3

**Always recorded:**
- **Each frame's solver summary,** in its `diagnostics.jsonl` line. It sits beside the fields
  already there (`state`, `tracked`, `keyframe` and the step times):
  - `placed_by`: how the frame was placed. One of:
    - `initialization`;
    - `motion`: the constant-velocity prediction, ORB-SLAM's `TrackWithMotionModel`;
    - `local_pnp`: the fallback;
    - null, when the frame was not placed.
  - `motion`: the matches, whether the search window was widened, and the inliers after
    refinement.
  - `pnp`, for the fallback only: the candidates, the matches by descriptor, RANSAC's
    iterations, why it stopped and its inliers, and the inliers refinement kept.
  - `local_map`: the points in view, the matches and the inliers.
  - `init`, for frames before the map exists: the matches to the reference frame, the
    initial BA's solves, and the outcome (`initialization.Outcome`, as built): `initialized`;
    `too_few_features`, `too_little_depth`; for monocular, `reference_set` and
    `reference_replaced`, or two-view's `too_few_matches`, `degenerate`, `pure_rotation`,
    `ambiguous`, `low_parallax`, `too_few_points` and `bad_scale`.
  - `mapping`, for keyframes only:
    - the points made from depth, triangulated, fused and culled, and the keyframes culled;
    - each local BA solve's initial and final cost, its iterations and why it stopped. Rung 0
      already returns these summaries; `edge.adjust` drops them today.
    - the observations dropped as outliers.

  The reasons a solver stopped are rung 0's names (`nls.Termination`, `ransac.Termination`),
  which all three languages share.
- **Lifetimes.** For every point and keyframe the run ever made, the run records:
  - the frame that made it;
  - the frame that removed it, if one did;
  - how it was removed: culled, fused into another point, or erased with its views.

  Frames are counted by their line in `diagnostics.jsonl`. A removed point keeps its last
  position. The map already gives each id only once, so it can keep removed rows aside as it
  erases them, without renumbering anything. The files:
  - `points.npy` (N, 3): every point ever made.
  - `point_lives.npy` (N, 3), int64: the frame that made it; the frame that removed it, or -1
    if it is in the final map; and how it was removed.
  - `keyframes.npy` and `keyframe_poses.npy`: every keyframe ever made, with
    `keyframe_lives.npy` beside them in the same layout.

  The final map is the rows still alive. `Run.final_points()` and `Run.final_keyframes()`
  return them, and the evaluation and the rerun recording read those.

**Opt-in** (`--record-features`; `vo.run(..., record_features=True)`): each frame's keypoints,
with their pixel, level, fate and map point (-1 for none). The records are streamed to disk as
the run goes, so memory use stays flat. Budget runs leave them off. As built (decided
2026-10-10):
- **Two plain arrays:** `feature_pixels.npy` (N, 2) float32 and `feature_records.npy` (N, 4)
  int32 of frame, level, fate and point, their headers rewritten after every frame. MH_01
  stereo's 3,682 frames, at 1,200 features, come to 107 MB. `run.json`'s `record_features`
  says whether a run recorded them. The flag travels with the request, as `--first` does: it
  changes no decision, so it is not one of the pipeline's settings.
- **The fates** are the first that applies of made (a point made of it on the frame), tracked
  (an inlier of the final pose) and rejected (matched by the frame's last tracking stage, then
  not tracked); else unmatched. The point is the one the fate concerns: the new one, the
  tracked one, or the one it was matched to and rejected from. `docs/notes/viewer.md` has the
  details.

**Why lifetimes, not snapshots** (decided): showing BA's later corrections would mean saving the
window after every local BA. That costs tens of MB more per run, and more code in three
languages. With lifetimes, each point sits where BA finally put it, which is the run's actual
map.

**Format v3.** `runs.VERSION` becomes 3. Under the no-shims rule, v2 runs no longer load; PR 9's
results stay on disk as files. The native writers gain int64 `.npy` arrays and the streamed
feature files.

**Checks:**
- **No decision moves.** Runs from before and after this change are compared on the rename's
  set: C++ and Rust whole runs on five sequence-modes, and Python's first 300 frames. They must
  match byte for byte in:
  - the trajectory;
  - the final keyframes and map;
  - every existing field of every frame's diagnostics.

  The room record (`room_runs.json`), once regenerated, may differ only by the new fields.
- **The languages agree on the new records.** Counts, fates and lifetimes must match exactly.
  Costs must match to PR 9's BA tolerance, roundoff. This is checked on the room runs in layer
  1, and on the dataset runs offline.
- **Still real time.** The budget runs (`pytest -m budget`) pass with summaries and lifetimes
  on and features off.

## 3. The bake (PR 10, Python, `slam_it.orchestration.bundles`)

Built in orchestration, not eval (decided 2026-10-10): it takes each frame's image from the
run's own source, so a stereo frame's image is the rectified one its keypoints were found in.

The bake turns one sequence's runs, one per backend with the same sequence and mode, into a
**bundle** together with the dataset. A bundle is a folder that any plain file server can
serve. Like runs, bundles live outside the repo (`../bundles/`). The page reads nothing else.

A bundle holds:
- **`bundle.json`:**
  - the format and its version (`slam-it-bundle`, version 1);
  - the sequence, mode and camera, and the frame count and times;
  - the backends, the frames each covers, and the alignment each was moved by;
  - whether there is ground truth;
  - the run format's fates and step names, which the page checks against its own.

  A phone video's intrinsics' source is named by its status page (PR 11), not yet by the
  bundle.
- **The images:** the frames as JPEG, written through `eval.images` (the one module allowed to
  use Pillow). They are stored in chunks of 100 frames with their byte offsets, so scrubbing
  fetches a few files rather than thousands. They are in colour where the source has it.
- **Per backend:**
  - the per-frame records, as columns, in JSON;
  - the trajectory, keyframes, points, lifetimes and features, as little-endian float32 and
    int32 arrays that the page reads straight into typed arrays: `.npy` files, each read after
    its 128-byte header (decided 2026-10-10).

**Alignment:**
- **With ground truth,** each run is moved into the ground truth's frame by its ATE alignment
  (`eval.metrics`): Sim(3) for monocular, SE(3) otherwise. Its points and keyframes move with it,
  so the backends overlay. The header shows the same ATE and RPE that `eval.runs.evaluate`
  gives.
- **Without ground truth** (a plain phone video), the run stays in its own frame, and the page
  says the scale is unknown. A Stray Scanner recording has ARKit's trajectory as its ground
  truth (section 5), so it is aligned and scored like a dataset run.

**Command:** `python -m slam_it.orchestration.bundles RUN_DIR... --sequence PATH --out
BUNDLE`. The bake refuses a folder already in use: it never deletes.

**Sizes:** the JPEGs are at quality 75: MH_01 stereo's 3,682 rectified frames come to 163 MB,
and a 30-second phone clip to about 30 MB. A whole run's feature records take about as much
again (`docs/notes/viewer.md`).

## 4. The page (PR 10, `viewer/`)

- **Built like reifyscene's app:** plain JavaScript modules with no build step, IBM Plex and its
  panel layout. three.js comes from a CDN through an import map.
- **Logic is kept apart from drawing.** The modules that read bundles and work out what to draw
  (lifetimes, fates, the per-frame records) never touch the page, so Node can test them.
- **Points:** one buffer holds every point with its made and removed frames. A small shader hides
  each point outside its lifetime, so scrubbing changes one number and never rebuilds the
  buffer.
- **Images:** chunks are fetched around the playhead, decoded once and cached.
- **Serving:** locally, `scripts/view_bundle.py BUNDLE` serves the page and the bundle together
  and opens the browser. Deployed, the server of section 5 serves both.
- **Layer 1** gains JavaScript checks. Node 26 is already in the env, for jscpd.
  - `node --check` on every module;
  - `node --test` on the tests in `viewer/tests/`. They include reading a small golden bundle
    that the Python tests write (`tests/fixtures/orchestration/bundle/`), so the format is held
    from both sides;
  - `tsc --checkJs` on the modules' JSDoc types: the JavaScript counterpart of `mypy --strict`.
    TypeScript and three.js's types are pinned in `viewer/package-lock.json` and installed into
    `viewer/node_modules` by `npm ci` (decided 2026-10-10), so the 3D module is checked against
    three.js's own API.

## 5. Your own video (PR 11)

**What people upload.** The upload page offers two choices:
- **A video,** the default: any phone, and no app to install. On a phone, the page's file button
  offers to record a video now with the phone's own camera, or to choose one already on it. Its
  intrinsics come from sources 2 to 4 of section 5.2.
- **A Stray Scanner recording,** from an iPhone or iPad Pro with LiDAR. Stray Scanner is free
  and open source (MIT). Since version 1.4, its Share button packs a recording into one zip,
  which can be saved to the Files app and chosen on the page. The zip holds:
  - `rgb.mp4`, the video;
  - `camera_matrix.csv`, the exact intrinsics;
  - `odometry.csv`, ARKit's pose for each frame, which becomes the run's ground truth;
  - depth, confidence, IMU and distortion data, which are not read.

  Record3D does the same job, but it is paid after three recordings, so it is not used.

Recording inside the page, with a live camera view, is not in these PRs. A browser's recording
carries no lens metadata, so its intrinsics would always fall to the nominal guess.

**Test devices:** iPhone 14 Pro, Pixel 8, Galaxy A15.
- Gyroflow holds main-camera profiles for the iPhone 14 Pro and the Pixel 8. It holds none for
  the Galaxy A15 (it does for the A13 and A14), so the A15 shows how the demo does on a phone
  nobody has calibrated.
- The iPhone 14 Pro has LiDAR, so it can also make Stray Scanner recordings.

**Checked first,** on a short clip from each test device and one Stray Scanner recording, all
kept outside the repo:
- which metadata each phone's video carries, and whether an Android video names its phone;
- whether iOS keeps the lens tags when it hands a video to the browser;
- Stray Scanner's export: the frame size `camera_matrix.csv` refers to, ARKit's axes, what the
  distortion data holds, and the zip's size for 30 seconds.

If Android videos don't name their phone, the fallback is a "which phone is this?" menu on the
upload page, decided once the clips have been read.

### 5.1 Import (`slam_it.eval.video`)

The import turns an upload into a TUM-format folder (`rgb/`, `rgb.txt` and `camera.txt`, plus
`groundtruth.txt` for a Stray Scanner recording). The readers in all three languages already
accept such a folder without depth, so the pipeline doesn't change. Steps and limits:
- **Decoding** uses PyAV (FFmpeg). A new import-linter contract keeps it in this module, the way
  Pillow is kept in `eval.images`.
- **Rotation:** the rotation the file declares is applied, since phones store portrait video on
  its side.
- **Timestamps** come from each frame's presentation time, because a phone's frame rate varies.
  60 fps is thinned to 30.
- **Size:** frames are scaled so the short side is 480 pixels, as TUM's and EuRoC's are. For
  16:9 video that is 854×480.
- **Undistortion** to pinhole (section 5.2). The output is the largest rectangle with no empty
  border, so no false corners appear at its edge.
- **Caps:** 60 seconds and 500 MB.
- **Stray Scanner zips** are read with Python's `zipfile` and never unpacked under the names
  they contain:
  - only `rgb.mp4`, `camera_matrix.csv` and `odometry.csv` are opened, and anything else is
    ignored;
  - each entry's size is checked before it is read, against its own cap;
  - the CSVs are parsed strictly: numbers only, and one `odometry.csv` row per video frame;
  - ARKit's poses are converted to our camera convention and written as `groundtruth.txt`.
    `odometry.csv` also holds each frame's intrinsics as ARKit refines them; the import uses
    `camera_matrix.csv` and records how far the per-frame values stray from it.
- **`source.json`**, beside the frames, records the intrinsics' source and the metadata fields
  used. Location tags and other metadata are never copied.
- **The same import runs as a command** (`python -m slam_it.eval.video`), and the command can
  be told which intrinsics source to use. That is how the tests drive it, and how the fallbacks
  are measured (section 5.2).

### 5.2 Intrinsics: the best source wins (`slam_it.eval.lenses`)

The sources are tried in order, and the results page names the one used:
1. **A Stray Scanner recording's camera matrix:** exact. Its ARKit trajectory becomes the
   ground truth the run is scored against.
2. **A Gyroflow lens profile**, matched from the video's metadata.
   - **The database:** Gyroflow's `lens_profiles`, released under CC0. It holds 1,164 phone
     profiles, 209 of them Apple. Each profile is a camera matrix plus OpenCV fisheye
     coefficients for one phone, lens and video mode, with its calibration error.
   - **Matching:** first on brand, model and lens (iPhone files carry
     `com.apple.quicktime.camera.lens_model`), then on the frame's aspect. The closest
     resolution and frame rate win, then the lowest calibration error. The matrix is scaled to
     the video's size.
   - Profiles in other distortion models are skipped.
   - The setup script and the worker image fetch the database at a pinned commit. It is not
     committed.
3. **The metadata's 35mm-equivalent focal length**
   (`com.apple.quicktime.camera.focal_length.35mm_equivalent`). It gives a focal length in
   pixels; the centre is put in the middle and distortion taken as none.
4. **A nominal field of view:** the median of the database's main-camera profiles.

**Undistortion.** OpenCV's fisheye model (Kannala-Brandt, θ_d = θ(1 + k1 θ² + k2 θ⁴ + k3 θ⁶ +
k4 θ⁸)) maps each pinhole output pixel directly to its source pixel, so no inversion is needed.
Rung 0's bilinear `image.remap` then resamples.
- Our camera module (`pinhole`, `radtan`, `radial`) needs no fisheye model, because the pipeline
  only ever sees pinhole frames.
- OpenCV stays out of `slam_it`. The demos use it only as a reference.

**Measured, not assumed:**
- whether the 35mm equivalent is taken across the frame's diagonal or its width;
- how far sources 3 and 4 fall from source 2, on the database's own phones;
- on Stray Scanner recordings, how much each fallback costs in ATE against ARKit's trajectory.
  This is measured once, with the import command, and written up in a note. The web app shows
  only the run with exact intrinsics.

The clips, the numbers measured on them and that note stay outside the repo.

### 5.3 The job list and the worker

This is the split Railway's docs describe for a CPU site whose heavy work may run elsewhere: the
site and the job list run on Railway, and workers connect out to them. PR 11's worker runs
on Railway. A GPU worker for reifyscene can later run the same protocol on the workstation or on
an on-demand cloud GPU.

- **The server** (`server/`) is its own small package, `slam_it_server`, built on FastAPI and
  the S3 client. It never imports `slam_it`, and a contract enforces that, so the process
  holding the secrets never parses a video. It:
  - serves the viewer and the upload page;
  - keeps the job list in SQLite on a Railway volume. A job has a state (waiting for upload,
    queued, the worker's stages from downloading to uploading, done, failed, expired), what was
    uploaded (a video or a Stray Scanner zip), its error, its link token and its expiry;
  - hands out presigned links:
    - an upload link, a PUT signed for the size the browser declares, which the server first
      checks against the cap, so the browser sends the video straight to the bucket (decided
      2026-10-11; a POST policy was planned);
    - links to each file, for workers.

  Buckets are private, and Railway charges nothing for bucket downloads. So the server
  redirects requests for bundle files to presigned links rather than proxying them.
- **The worker** (`slam_it.orchestration.worker`):
  1. claims a job over HTTPS with a worker token (long polling);
  2. downloads the upload;
  3. imports it (section 5.1);
  4. runs the C++ program (`vo.run(..., backend="cpp")`), with features recorded;
  5. bakes the bundle and uploads it;
  6. reports done, or the reason it failed.

  Each job has a timeout and a memory limit, set with `setrlimit` and a clock, since containers
  have no systemd. The worker holds only its token: no bucket keys and no database. Each job
  names its kind (`vo` now), and a worker claims only the kinds it can run.
- **Locally,** `scripts/serve_demo.py` runs the server with a folder as storage, plus one worker.
  It can listen on the local network, so the demo can be shown in person, on the same Wi-Fi,
  before anything is deployed.

### 5.4 Guards

- An invite code is needed to upload. Viewing a result needs only its link. On the staging
  deploy (section 5.6), every page needs the code.
- Size and length are capped. The size is checked in the browser, when the job is made and by
  its signed upload link, and again on import; the length in the browser, where it can read
  it, and on import.
- One job runs at a time. Beyond a short queue (5 jobs), the page asks people to try later.
- The upload, video or zip, is deleted when its import ends. A result is deleted after 7 days
  (decision 6 of section 7).
- A result is reachable only through an unguessable link (a 128-bit token). There is no gallery
  or listing, and pages are marked `noindex`.
- Uploads are never served back; only the frames the bake makes are.
- The upload page says what is kept, for how long, and who can see it.

### 5.5 What to expect, stated on the page

- **Monocular:** the scale is unknown, and there is no ground truth unless the upload is a Stray
  Scanner recording.
- **Stabilisation:** phone stabilisation crops and warps every frame, so the camera stops
  behaving like one fixed lens. Turn it off where the phone allows; apps such as Blackmagic
  Camera can.
- **Getting started:** move sideways at first, not only turning. Texture and good light help.
  A lost track ends the run, and the viewer shows where and why.
- **Rolling shutter** is ignored. Moving slowly keeps its effect small.

### 5.6 Packaging

- **Two images:**
  - a slim one for the server;
  - one for the worker, with the built package and the C++ program.

  `docker build` works locally (Docker is installed).
- **On Railway,** in the `timedilated` project, after PR 11 merges:
  - a `slam-it` service: the server and its volume;
  - a `slam-it-worker` service: no domain, on the private network;
  - a bucket, with CORS set for the service's address.
- **Staging, then go-live:**
  - **Staging** comes first, on Railway's own `*.up.railway.app` address, with every page behind
    the invite code. Each Railway step (a service, the volume, the bucket, a setting) is taken
    one at a time, each with its own Yes.
  - **The domain,** `slam-it.timedilated.com`, is attached only on the owner's go-live call.
- **Cost:** Railway bills per minute of use, and storage costs $0.015 per GB-month. A polling
  worker never sleeps, so it is billed while it waits.

## 6. Plan

### PR 10: the viewer (0.3.4.0 → 0.3.5.0)

Commits:
1. `feat(edge)`: per-frame solver summaries, in all three languages.
2. `feat(edge)`: point and keyframe lifetimes; run format v3, with its readers and writers in
   all three languages.
3. `feat(edge)`: per-frame feature records, opt-in; the languages agree exactly.
4. `feat(orchestration)`: the bake, and the golden bundle.
5. `feat(viewer)`: the page, and layer 1's JavaScript checks.
6. `docs`:
   - the viewer note (`docs/notes/viewer.md`);
   - bundles of the edge note's runs, kept outside the repo;
   - CHANGELOG and VERSION.

Checks:
- layer 1;
- the before-and-after byte comparison;
- the budget runs;
- offline CI;
- the full panel.

### PR 11: your own video (0.3.5.0 → 0.3.6.0)

Before the code, the test clips are read (section 5, "Checked first").

Commits:
1. `feat(eval)`: video import, and Stray Scanner recordings.
2. `feat(eval)`: intrinsics and undistortion.
3. `feat(orchestration)`: the worker.
4. `feat(server)`: the server and its storage, the guards, and the upload and status pages.
5. `build`, `docs`: the images, the local demo script, the demo checklist, CHANGELOG and
   VERSION.

Tests:
- **Video import:** the tests encode the room sequence as a video with PyAV, adding a rotation
  and Apple-style tags. Imported and tracked, it must stay within the room's ATE bound.
- **Stray Scanner import:** the tests build a zip from the room sequence in Stray Scanner's
  layout, with its poses in ARKit's convention. Imported, its ground truth must match the
  room's. Hostile zips must be refused: oversized entries, names that climb out of a folder,
  and malformed or mismatched CSVs.
- **Undistortion:** the room is rendered through a fisheye model, and undistorting it must give
  back the pinhole frames, to a tolerance.
- **Profile matching:** a few CC0 profiles are kept in the fixtures, with attribution.
- **The server,** with FastAPI's test client and folder storage: the job lifecycle, the guards
  and expiry.
- **Presigned policies,** checked offline: boto3 signs without a network.

Checks:
- layer 1;
- offline CI;
- the full panel, plus a security pass on the upload path, zip reading included. Its report
  is saved beside the panel's.

### After PR 11

- Staging on Railway, one step at a time (section 5.6).
- The domain waits for the owner's go-live call.
- reifyscene's hosting is planned separately.

**Not in these PRs:**
- recording inside the page, with a live camera view;
- streaming a live camera;
- comparing intrinsics sources in the web app;
- GPU workers;
- checkerboard calibration (the calibration track);
- map snapshots after every BA;
- the showcase page;
- rungs side by side;
- a phone's LiDAR depth as RGB-D.

## 7. Decisions of 2026-10-09

Decided with evan-wes:
1. **Images in bundles:** always baked as JPEG chunks. A plain file server rooted at the bundle
   can't reach `../datasets/`, and one path is simpler than two.
2. **JavaScript checks:** `node --check`, `node --test` and `tsc --checkJs` (section 4).
3. **Test devices and capture:** the test devices are listed in section 5. The upload page
   takes a plain video by default, and a Stray Scanner zip as a second choice (section 5).
   Record3D was dropped: no paid dependencies.
4. **Comparing intrinsics sources:** measured once on Stray Scanner recordings with the import
   command, and written up outside the repo. The web app shows only the exact run.
5. **PR 11's review:** the full panel, plus a security pass on the upload path, with its report
   saved beside the panel's. The panel has no security reviewer of its own.
6. **Retention:** an upload is deleted when its import ends, and a result after 7 days.
7. **Before go-live:** a staging deploy on Railway's own address after PR 11 merges, with every
   page behind the invite code. The domain waits for the go-live call (section 5.6).
8. **The ladder:** PR 11 is milestone 2's last PR, and hosting moves from GitHub Pages to
   Railway. The ladder is updated in the same docs commit as this doc.

## 8. Decisions of 2026-10-10

Decided with evan-wes, while building PR 10:
1. **Solver summaries** are returned by each stage with its result, and nest in each
   diagnostics line as JSON objects.
2. **C++ runs repeat on every core:** the nondeterminism the byte comparison found (Eigen's
   product blocking, read from a hybrid CPU's core) is fixed first, inside PR 10.
3. **Feature records** are two plain arrays, float32 pixels and int32 records.
4. **A keypoint's point** is the one its fate concerns (made, tracked, or rejected from).
5. **The bake** lives in orchestration, and a bundle's arrays are `.npy` files.
6. **The page's type check** uses a pinned `viewer/package.json`, with three.js's own types.

## 9. Decisions of 2026-10-11

Decided with evan-wes, while building PR 11:
1. **Synthetic recordings first:** the import, the intrinsics, the worker and the server are
   built and tested on the room's synthetic videos and Stray Scanner zips while the real clips
   are gathered. What the clips will check is listed in
   [the note](../notes/your-own-video.md#what-waits-for-the-real-clips).
2. **Uploads are a signed PUT** of the size the browser declares, which the server checks
   against the cap first, rather than a POST policy (section 5.3). It is as strict, needs no
   multipart parsing, and works the same for the bucket and for folder storage.
3. **The worker's heartbeat:** it repeats its stage every 30 s, and the server fails a job not
   heard from for 2 minutes.
4. **The upload and status pages live in `viewer/`,** under the viewer's checks. A result's link
   is `/r/<token>`.
5. **Settings come from environment variables,** listed with placeholders in
   `server/settings.template.env`.
6. **An upload's run is named `your-video`** in the viewer's header.

Made while building, within the design, and reported to evan-wes:
- a queued job expires after a day if no worker takes it, so an upload never waits in storage
  for ever;
- a failed or expired job is kept 7 days, as a done one is, so that its link says how it ended;
- invite codes need 12 characters, the worker token and the signing key 32;
- a result holds at most 1,000 files and 1 GB;
- the server listens on IPv6 and IPv4 at once, for Railway's private network, and a stopping
  server waits at most 3 s for open requests, so that a worker's held claim does not hold up a
  redeploy.

PR 11's review panel and its security pass, and how each of their findings was resolved, are in
[the panel's report](../reviews/2026-10-11-feature-your-own-video.md) and
[the security pass's](../reviews/2026-10-11-feature-your-own-video-security.md). Decided with
evan-wes on 2026-10-11: every must-fix fixed; the should-fix findings and notes as recommended,
a few parts deferred to before going public (the ladder's open items); and an invite code
capped at two open jobs, a waiting job expiring with its 15-minute upload link.
