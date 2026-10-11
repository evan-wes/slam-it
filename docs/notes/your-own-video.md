# Your own video: the import, the intrinsics, the worker and the upload server

Design note for PR 11: a phone video, or a Stray Scanner recording, uploaded through a page,
run through the C++ pipeline and opened in the run viewer. The import
(`slam_it_all.eval.video`), the intrinsics (`slam_it_all.eval.lenses`), the worker
(`slam_it_all.orchestration.worker`), the upload server (`server/`, `slam_it_all_server`) and its
pages (`viewer/`). The plan was `docs/design/2026-10-09-run-viewer.md`, section 5; the choices
made while building it are its section 9. This note is what was built, how it was checked, and
what still waits for real phone clips.

## The problem

The pipeline reads dataset folders: frames as PNGs, a camera file, and ground truth when there is
some. A phone gives a video, often stored on its side, at a varying frame rate, through a lens
nobody calibrated for us. A demo on someone's own footage needs that video turned into such a
folder with the best intrinsics to be had, a place to run it that is not the visitor's browser,
and a way back to the result, without the server that holds the storage keys ever parsing a
video.

## The import

`slam_it_all.eval.video` turns a recording into a TUM-format folder without depth, which the
readers in all three languages already take.

- **A video** is decoded with PyAV (an import-linter contract keeps PyAV in this module), turned
  upright by the rotation the file declares, timed by each frame's presentation time (a frame
  closer than 1/40 s to the last kept is skipped, so 60 fps becomes 30), and scaled down, never
  up, to a 480-pixel short side (16:9 becomes 854×480).
- **A Stray Scanner zip** is read in place: only `rgb.mp4`, `camera_matrix.csv` and
  `odometry.csv` are opened, each against its own cap; a name that is absolute, climbs out of its
  folder or repeats refuses the zip. ARKit's poses become `groundtruth.txt`, in our camera axes.
- **Caps:** 500 MB an upload and 60 s a video. The folder is written beside its own and moved
  onto it once whole, so a refused import leaves nothing.
- **`source.json`** records the intrinsics and their source and at most four metadata fields
  (make, model, lens, 35mm-equivalent focal length). Location tags are never copied.

## The intrinsics

The best source wins, and the status page names it:
1. a Stray Scanner recording's camera matrix;
2. a Gyroflow lens profile (CC0), matched from the video's metadata;
3. the metadata's 35mm-equivalent focal length, taken across the diagonal;
4. a nominal field of view: 71.66 degrees across the diagonal, the median of the 766 usable
   main-camera profiles.

The profiles are fetched at a pinned commit, never committed. A profile is usable if it is in
OpenCV's fisheye model, unstretched, uncropped, calibrated within 2 px RMS, and its model holds
out to its image's corners: 1,047 of the 1,164 phone profiles. A video matches a profile on its
brand and model, whole; on its lens's kind (main, ultrawide, telephoto, front); and on its
aspect and orientation. The nearest resolution, frame rate and calibration error then decide.
A fisheye video is undistorted to pinhole at import, so the pipeline never sees a fisheye.

## The worker

`slam_it_all.orchestration.worker` connects out to the server, holding only its token. For each
job it claims, it downloads the upload, then in a child process under a memory limit (6 GiB of
address space) and a 20-minute timeout imports it, runs the C++ pipeline, monocular, with
features recorded, and bakes the bundle; it uploads the bundle's files to the links the server
gives, and reports the job done or failed. It reports each stage as it starts, and again every
30 s while the child works on it: the job's heartbeat. A reason fit to show is an import's or
the pipeline's own message, the worker's paths taken out; anything else is "an internal error".
An upload is imported into a folder named `your-video`, which is the name the viewer shows.

## The server

`slam_it_all_server` keeps the job list in SQLite and hands out storage links; it never imports
`slam_it_all`, and a contract holds it to that.

- **A job** waits for its upload (1 hour), is queued (1 day), goes through the worker's stages,
  and ends done, failed or expired. It is deleted with its files 7 days after it ends. It has two
  names: an id for workers and storage, and a 128-bit token for its owner's link.
- **Storage** is a private S3 bucket, or a folder on this machine whose links the server signs
  and serves. An upload link is a PUT signed for the size the owner declared, so storage refuses
  any other; the server checks that size against the 500 MB cap first. A done job's files are
  redirected to storage's own links, never proxied.
- **The guards:** an invite code to upload; one job worked on at a time and 5 open at most; a job
  not heard from for 2 minutes fails; the upload deleted when the import ends; at most 1,000
  files and 1 GB a result; request bodies capped at 64 KiB outside storage; every answer marked
  `noindex` and sending no `Referer`; on staging, every page behind a code. Invite codes need 12
  characters, the worker token and the signing key 32.
- **Settings** come from environment variables, listed with placeholders in
  `server/settings.template.env`. Nothing about a deployment is in the repository.
- **`python -m slam_it_all_server`** listens on IPv6 and IPv4 at once when given `::`, as the
  image is, since Railway's public port is reached on IPv4 and an older project's private
  network on IPv6 alone. Stopping, it waits at most 3 s for open requests: a worker's claim is
  held open for up to 25 s, and a redeploy should not wait for it.

## The pages

The upload page (`/`) checks a file against the caps, sends it straight to storage with a
progress bar, and says what to expect and what is kept. The status page (`/r/<token>`) follows
the job's stages, then shows its summary with the intrinsics' source and the viewer's link, or
why it failed. Both sit in `viewer/` beside the run viewer, under its checks, and share its
style; their logic is `viewer/src/jobs.js`. The server serves only the viewer's pages, its
stylesheet and its modules.

## Running it

**On this machine:** `python scripts/serve_demo.py` runs the server, with a folder as storage,
and one worker, and prints the page's address and a new invite code. With `--lan` it listens on
the local network too, so a phone on the same Wi-Fi can upload. Everything goes in one folder
(`--data`, by default a new temporary one). Ctrl-C, or a SIGTERM, stops the server and the
worker together.

**The images** build from the repository's root:

```bash
docker build -f docker/server.Dockerfile -t slam-it-all-server .
docker build -f docker/worker.Dockerfile -t slam-it-all-worker .
```

- **The server's** holds `slam_it_all_server` and the pages, and no `slam_it_all`, compiler or
  decoder.
- **The worker's** is built in two stages: the first compiles the wheel, C++ extension and
  programs included, against Eigen 5.0.1 checked out at its tag's commit; the second installs it
  with PyAV and fetches the lens profiles, so a job never reaches out for them.
- The env's lock (`scripts/locks/slam-it-all.lock`) pins every Python package both install,
  and the base image is pinned by digest.

## Deploying to staging

On Railway, in the `timedilated` project, after PR 11 merges. Each step is taken on its own, with
its own Yes, and the domain waits for the go-live call.

1. **The bucket**, private. Its CORS lets the server's address PUT (with `Content-Type`) and GET.
2. **`slam-it-all`**, the server: `docker/server.Dockerfile`, a volume at `/data`,
   `RAILWAY_RUN_UID=0` (Railway mounts volumes as root, and the image runs as its own user),
   `PORT=8000`, and the variables of `server/settings.template.env`, with `SLAM_IT_ALL_STORAGE=s3`,
   the bucket's credentials, `SLAM_IT_ALL_DATABASE=/data/jobs.sqlite3`, the Railway address as
   `SLAM_IT_ALL_PUBLIC_URL`, and `SLAM_IT_ALL_STAGING=1`. The signing key is needed with S3 too.
3. **`slam-it-all-worker`**: `docker/worker.Dockerfile`, no domain,
   `SLAM_IT_ALL_WORKER_TOKEN` as the server's, and
   `SLAM_IT_ALL_SERVER=http://slam-it-all.railway.internal:8000`.
4. **The check:** `/healthz`; a clip uploaded from a phone; the job done; the viewer opened from
   its link.

## The demo checklist

Before showing it in person:
- run `python scripts/serve_demo.py --lan` on the laptop, open the printed address on a phone
  on the same Wi-Fi, and upload a short clip of the room through to the viewer;
- keep a baked bundle at hand (`scripts/view_bundle.py`) in case the network fails;
- give the invite code to whoever records.

Recording, as the upload page says: stabilisation off where the phone allows; move sideways at
first, not only turning; texture and good light; slow, for the rolling shutter; under 60 s.

Afterwards: Ctrl-C stops the server and the worker; the data folder still holds the results,
and deleting it is the last step.

## Checks

- **The import:** the room encoded with PyAV, losslessly, imported upright from its side, thinned
  from 60 fps, scaled, and tracked within the room's monocular ATE bound (3 cm); a Stray Scanner
  zip of the room imports with the room's poses as its ground truth (to 1e-9); hostile zips and
  malformed CSVs refused, leaving nothing.
- **The intrinsics:** profiles matched or not by phone, lens, orientation and aspect; the room
  rendered through a fisheye undistorts to its pinhole view.
- **The worker,** against a stand-in server on a local port: the room's zip run end to end, its
  stages and heartbeat, refusals, a timeout, a server letting the job go, an oversized upload, a
  child out of memory.
- **The server,** with FastAPI's test client and folder storage: every route and guard, the job
  list's life, queue and sweep, S3's links signed offline; the caps and names that the import,
  worker, server and pages each hold, held equal; and an end-to-end run with the server under
  uvicorn and the real worker on the room's zip.
- **The pages' logic** in Node, and the whole path by hand in a headless browser: upload, status,
  the worker, the viewer.
- **The images,** by hand: one job through both containers on a private Docker network, from the
  room's Stray Scanner zip and from its video, read back through the result's link.

## What waits for the real clips

The tests run on the room's synthetic recordings. A short clip from each test phone (iPhone 14
Pro, Pixel 8, Galaxy A15) and a Stray Scanner recording, kept outside the repo, will check:
- the rotation's direction on a real portrait video;
- Stray Scanner's zip layout, the frame size `camera_matrix.csv` describes, `odometry.csv`'s
  columns and ARKit's axes;
- whether Android videos name their phone, and how lenses are named in real metadata;
- whether the 35mm-equivalent focal length is taken across the diagonal;
- how far `odometry.csv`'s per-frame intrinsics stray from the camera matrix, which the import
  does not record yet.

## What it does not do yet

- Recording in the page, with a live camera view, or streaming a live camera.
- Comparing the intrinsics' sources in the web app: measured once, on Stray Scanner recordings,
  and written up outside the repo.
- A GPU worker: the same protocol, run elsewhere, for reifyscene.
