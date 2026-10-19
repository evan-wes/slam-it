# Your own video: the import, the intrinsics, the worker and the upload server

Design note for PR 11: a phone video, or a Stray Scanner recording, uploaded through a page,
run through the C++ pipeline and opened in the run viewer. The import
(`slam_it.eval.video`), the intrinsics (`slam_it.eval.lenses`), the worker
(`slam_it.orchestration.worker`), the upload server (`server/`, `slam_it_server`) and its
pages (`viewer/`). The plan was `docs/design/2026-10-09-run-viewer.md`, section 5; the choices
made while building it are its section 9. This note is what was built, how it was checked, and
what still waits for real phone clips. The review panel and a security pass read it all
([report](../reviews/2026-10-11-feature-your-own-video.md),
[security](../reviews/2026-10-11-feature-your-own-video-security.md)); what they changed is
described here as built.

## The problem

The pipeline reads dataset folders: frames as PNGs, a camera file, and ground truth when there is
some. A phone gives a video, often stored on its side, at a varying frame rate, through a lens
nobody calibrated for us. A demo on someone's own footage needs that video turned into such a
folder with the best intrinsics to be had, a place to run it that is not the visitor's browser,
and a way back to the result, without the server that holds the storage keys ever parsing a
video.

## The import

`slam_it.eval.video` turns a recording into a TUM-format folder without depth, which the
readers in all three languages already take (written by `eval.tum`'s writers, beside its
readers).

- **A video** is opened with FFmpeg's mov/mp4 demuxer alone, as a file alone: any other
  container is refused, so the playlist and reference formats that open other files never read
  an upload. It is decoded with PyAV (an import-linter contract keeps PyAV in this module),
  turned upright by the rotation the file declares, timed by each frame's presentation time (a
  frame closer than 1/40 s to the last kept is skipped, so 60 fps becomes 30), and scaled down,
  never up, to a 480-pixel short side (16:9 becomes 854×480).
- **A Stray Scanner zip** is read in place: only `rgb.mp4`, `camera_matrix.csv` and
  `odometry.csv` are opened, each against its own cap; a name that is absolute, climbs out of its
  folder or repeats refuses the zip, and so does an entry it cannot give back whole. Stray
  Scanner's conventions were read in its source: its `OdometryEncoder.swift` writes each pose's
  rotation already in our camera's axes (ARKit's rotation times a half turn about x), so the
  poses become `groundtruth.txt` as written; and its encoders stamp the first frame before the
  video's session starts, which an MP4 hides behind an edit list, so `rgb.mp4` is decoded with
  its edit list ignored. At its default 60 fps the import keeps every other frame, each with
  its own pose.
- **Caps:** 500 MB an upload and 60 s a video; a frame at most 7680 pixels long and 3 to 1, and
  every frame the first's size, so that a frame written is at most 1440 by 480; at most 14,400
  frames. The folder is written
  beside its own and moved onto it once whole, so a refused import leaves nothing.
- **`source.json`** records the intrinsics and their source and at most four metadata fields
  (make, model, lens, 35mm-equivalent focal length). Location tags are never copied.

## The intrinsics

The best source wins, and the status page names it:
1. a Stray Scanner recording's camera matrix;
2. a Gyroflow lens profile (CC0), matched from the video's metadata;
3. the metadata's 35mm-equivalent focal length, taken across the diagonal;
4. a nominal field of view: 72.37 degrees across the diagonal, the median of the 828 usable
   main-camera profiles.

The profiles are fetched at a pinned commit, never committed, and required: a missing database
is an error, never a silent fall to the fallbacks. A profile is usable if it is in OpenCV's
fisheye model, unstretched, uncropped, calibrated within 2 px RMS, and its model holds out to its
image's corners: 1,047 of the 1,164 phone profiles. A video matches a profile on its brand and
model, whole; on its lens's kind (main, ultrawide, telephoto, front), from Apple's lens name, or,
where the name says none (the Camera app's "back triple camera"), from the 35mm-equivalent focal
length, a name and a focal length that disagree trusting no profile; on the profile's own
35mm-equivalent focal length, within 0.8 to 1.6 times the video's, since some profiles calibrate
another lens than their names say and Apple's 2x is the main sensor cropped (without one, a
main-camera profile within 1.5 times, either way, of the nominal field of view's 29.6 mm); and on
its aspect and orientation. The nearest resolution, frame rate and calibration error then decide.
"Ultra" alone is a phone's name ("Galaxy S23 Ultra"), never a lens's. A fisheye video is undistorted
to pinhole at import, so the pipeline never sees a fisheye; the model and its undistortion map agree
with OpenCV's `cv2.fisheye`.

## The worker

`slam_it.orchestration.worker` connects out to the server, holding only its token. For each
job it claims, it downloads the upload, then in a child process under a memory limit (6 GiB of
address space), a file-size limit (2 GiB) and a 20-minute timeout imports it, runs the C++
pipeline, monocular, with features recorded, and bakes the bundle; it uploads the bundle's files
to the links the server gives, and reports the job done or failed. An upload is imported into a
folder named `your-video`, which is the name the viewer shows.

- **Its heartbeat:** each stage is reported as it starts and again every 30 s while it lasts,
  the download and uploads included. A report that does not reach the server is logged and the
  job goes on; only a 409 lets it go.
- **Supervision:** the child starts a session of its own, so it and the C++ program it starts are
  one process group, killed whole whenever the job ends; in its image, tini reaps whatever a killed
  job leaves. A SIGTERM stops the worker as a Ctrl-C does, and a second signal is ignored while it
  cleans up, so the job's processes and folder go with it however it is stopped. The child inherits
  only the environment it needs, never the token.
- **What is said:** a reason fit to show is an import's or the pipeline's own message, the
  worker's paths taken out; anything else is "an internal error" to the owner, and logged with
  the job's id and its traceback for the operator.
- **Trust:** it refuses a server or link that is not `http` or `https`, and plain `http` to a
  host that may be on the public internet: only to loopback and private addresses (not
  link-local, where a cloud keeps its metadata service), `localhost`, a container's one-label
  name or `*.railway.internal`. The host is judged as urllib connects to it, its escapes undone,
  and an address written as a number as the address it is. It follows no redirect from the
  server or an upload link, and a download's only to a link it would accept.

## The server

`slam_it_server` keeps the job list in SQLite and hands out storage links; it never imports
`slam_it` (nor a decoder, an image library or reifyscene), and a contract holds it to that.

- **A job** waits for its upload (15 minutes, its upload link's life, which the upload page
  states: 500 MB needs about 4.5 Mbit/s), is queued (1 day), goes
  through the worker's stages, and ends done, failed or expired. It is deleted 7 days after it
  ends, its files first: a deletion that fails is logged, and the job stays listed for the next
  sweep to try again. It has two names: an id for workers, storage and the log, and a 128-bit
  token for its owner's link.
- **Storage** is a private S3 bucket, or a folder on this machine whose links the server signs and
  serves, every file as a download. An upload link is a PUT signed for the size the owner declared,
  so storage refuses any other; the server checks that size against the 500 MB cap first. A
  confirmed upload is moved where its owner's link cannot reach it. A job's files, its upload, the
  upload's copy and its result, are in one folder of storage (`jobs/<id>/`), deleted whole with the
  job. A link is checked when an upload starts, so whatever a job's links write later is deleted
  when it ends, and again once every link it signed has expired (an hour after it ended) and an hour
  more has passed; folder storage keeps nothing that arrives whole only after its link expired. A
  done job's files, only a bundle's kinds of file, are redirected to storage's own links, never
  proxied.
- **The guards:** an invite code to upload, at most 2 jobs open per code and 5 in all; one job
  worked on at a time; a job not heard from for 2 minutes fails; the upload deleted when the import
  ends (the demo keeps it, below); at most 1,000 files and 1 GB a result; request bodies capped at
  64 KiB outside storage; every answer marked `noindex`, sending no `Referer`, under a Content
  Security Policy (scripts from the site, three.js's own path on its CDN and the import map by its
  hash; connections to the site and storage); on staging, every page behind a code. Invite codes
  need 12 characters, made at random, the worker token and the signing key 32.
- **The log** names jobs by their id, and a code by its keyed hash; uvicorn's access log, which
  would hold tokens and signed links, is off.
- **Settings** come from environment variables, listed with placeholders in
  `server/settings.template.env`. Nothing about a deployment is in the repository.
- **The job list keeps its format** in its file, and a server refuses one of another, or another
  program's file, at startup, naming it, rather than migrating or adopting it: deleting it starts
  a new one. A test holds the table's columns to the format's number.
- **`python -m slam_it_server`** refuses to run as root. It listens on IPv6 and IPv4 at once
  when given `::`, as the image is: IPv4, as before, and IPv6, since Railway's private network is
  IPv6 alone in an environment made before 16 October 2025 (Railway's docs, "How private networking
  works"). Stopping, it waits at most 3 s for open requests: a worker's claim is held open for up to
  25 s, and a redeploy should not wait for it.

## The pages

The upload page (`/upload`, and `/` unless there is a showcase, below) checks a file against the
caps, sends it straight to storage with a progress bar, and says what to expect, what is kept, and
who sees what: the service the server runs on logs requests, and the fonts' and three.js's CDNs
see each visitor's address. The status
page (`/r/<token>`) follows the job's stages, then shows its summary with the intrinsics' source
and the viewer's link, or why it failed. Both sit in `viewer/` beside the run viewer, under its
checks, and share its style; their logic is `viewer/src/jobs.js`. The viewer reads a bundle from
its own site only, and loads three.js pinned by its hashes. The server serves only the viewer's
pages, its stylesheet and its modules.

**A result in stretches.** A monocular run tracks nothing until its first map, and a lost track
closes its map and the next frame starts a new one (since milestone 2A's A2), once
relocalisation has tried for up to 3 s to find it again in the map it had (since 2B's B1). So a
result can cover the video in stretches, which would otherwise look cropped. Both pages say so:
- the status page in words, from the worker's summary (`worker.summarize`: the clip's length,
  each map's first and last tracked frames' times, the losses, the maps lost too soon to keep,
  and the frames relocalised): "Tracked 0.0–15.9 s and 17.0–27.1 s of your 27.1 s clip.
  Tracking failed on one frame and found it again in the same map, by recognising a place it had
  seen. The track was lost twice, and each time a new map was started, so the rest of the clip
  was still processed; one map was lost too soon to keep.", and, where finding a first map took
  a while, "The first 2.5 s went to finding a first map; moving sideways at the start shortens
  it.";
- the viewer on its timeline: the frame slider spans the part the runs ran, beside the parts
  of the sequence none reached, each as wide as its time and labelled "not run"; under the
  slider, each backend's frames are coloured by state (faded before its first map, its colour
  while tracking, broken where its map was reset, red where it lost the track, green on each
  frame found again), with a legend. The
  bundle records the whole sequence for this (`bundle.json`'s `extent`: its frames, and its
  first and last frames' times), which the page requires.

## The showcase

Runs we baked ourselves, shown to anyone: `SLAM_IT_SHOWCASE` names them, comma-separated, each
name of lowercase letters and digits joined by single hyphens, at most 40 characters (a name
malformed or listed twice stops the server at startup). Each run's bundle is in storage under
`showcase/<name>/`, its folder's files as the bake wrote them (`showcase/<name>/bundle.json`,
`showcase/<name>/cpp/poses.npy`, ...), beside its calibration page
(`showcase/<name>/calibration.html`), put there by hand with any S3 client; nothing the server
does writes or deletes there, and a bucket's lifecycle rule for `jobs/` leaves it alone. A new
bundle format (`slam_it.orchestration.bundles.VERSION`) means baking the showcase's runs again and
uploading them again: the viewer reads only its own format. Unset or empty, `/` is the upload page,
as before.

- **`/`** lists the runs, each by its name, linked to `/s/<name>/`, and links the upload page,
  which moves to `/upload`, its invite code as before. The page is `viewer/showcase.html`, in the
  upload page's style and with its head, which the server reads once at startup and fills with
  each run's link in place of its `<!-- runs -->` marker; a page missing, or its marker missing or
  repeated, stops the server. It has no script and no inline style, so the content policy holds
  it as it is, and it is not served by its own name. The titles are the names: `bundle.json`'s
  sequence would mean the server reading a file from storage, which it never does.
- **`/s/<name>/`** (or without the slash) opens the run viewer on the run's bundle, as the status
  page's link does: `/index.html?bundle=%2Fs%2F<name>%2Fbundle%2F`.
- **`/s/<name>/bundle/<path>`** is redirected to storage's link for `showcase/<name>/<path>`,
  only for a bundle's names and kinds of file, as a job's result is (`RESULT_PATH`). Each link is
  made for its request, so a page left open never holds one expired.
- **`/s/<name>/calibration.html`**, the run's calibration page, is redirected to a link that asks
  the bucket to answer it as a page (`text/html; charset=utf-8`). It is the only file beside the
  bundle that is served: the page is self-contained, its images embedded, and has no script. It
  renders at storage's own address, which its address bar shows, without the site's headers (its
  content policy among them): accepted, because the page is the owner's own static file with no
  script. Folder storage serves every file as a download, this page too, so it shows only from a
  bucket.
- **Anything else** answers 404: a name not listed (matched exactly), a file of another name,
  kind or folder (images kept beside the bundle among them), and a path that climbs or is
  absolute, escaped or not. The routes read the settings and storage's links alone: never the job
  list, so never a token, and nothing lists storage. On staging, they are behind its Basic auth,
  as every page is.

## Running it

**On this machine:** `python scripts/serve_demo.py` runs the server, with a folder as storage, and
one worker, and prints the page's address and a new invite code, three short words such as
`moss-lamp-tide` (`--code` gives another). With `--lan` it listens on the local network too, so a
phone on the same Wi-Fi can upload, and every page then asks for the code (any name, the code as the
password); the page's address is printed as a QR code too, for the phone's camera, black on white
whatever the terminal's colours. Everything goes in one folder, readable by its owner alone
(`--data`, by default `~/.local/share/slam-it/demo`, the same every run). The demo keeps each
upload with its result, in the job's folder of storage (`files/jobs/<id>/`), so that a recording can
be run again (`python -m slam_it.eval.video`, `scripts/run_vo.py`, the bake,
`scripts/view_bundle.py`): `SLAM_IT_KEEP_UPLOADS`, which the server takes with folder storage
alone. The server deletes both once 7 days old, while it runs; since the folder is the same every
run, each start deletes what passed its time while no demo ran. The upload page says so, once the
server has said it keeps uploads. With uploads kept, a new job is refused while the disk under the
folder has less than 5 GB free, ten uploads' worth. Copy out a recording to keep it longer. A job
list of an earlier format, the folders' layout before one folder a job among them, is refused,
naming its file. Ctrl-C, or a SIGTERM, stops the server and the worker together, and the server
stops if the worker does.

**The images** build from the repository's root:

```bash
docker build -f docker/server.Dockerfile -t slam-it-server .
docker build -f docker/worker.Dockerfile -t slam-it-worker .
```

- **The server's** holds `slam_it_server` and the pages, and no `slam_it`, compiler or
  decoder. Its entrypoint, run as root, gives the job list's folder to the server's own user and
  drops to it, unable to gain privileges again; it refuses a database path that is relative or
  whose folder is `/`, which it would otherwise give away.
- **The worker's** is built in two stages: the first compiles the wheel, C++ extension and
  programs included, against Eigen 5.0.1 checked out at its tag's commit; the second installs it
  with PyAV and tini and fetches the lens profiles, so a job never reaches out for them.
- The env's lock (`scripts/locks/slam-it.lock`) pins every Python package both install,
  and the base image is pinned by digest.

## Deploying to staging

On Railway, in the `timedilated` project, after PR 11 merges. Each step is taken on its own, with
its own Yes, and the domain waits for the go-live call.

1. **The bucket**, private. Its CORS lets exactly the server's address, never `*`, PUT (with
   `Content-Type`) and GET. If Railway's buckets take a lifecycle rule, one deletes `jobs/` after
   8 days, behind the server's own deletions: a job's files share its folder, so the rule cannot
   delete an upload any sooner than its result, should the server's deletion at import fail.
2. **`slam-it`**, the server: `docker/server.Dockerfile`, a volume at `/data`,
   `RAILWAY_RUN_UID=0` (Railway mounts volumes as root; the entrypoint gives the folder to the
   server's user and drops to it at once), `PORT=8000`, and the variables of
   `server/settings.template.env`, with `SLAM_IT_STORAGE=s3`, the bucket's credentials,
   `SLAM_IT_DATABASE=/data/jobs.sqlite3`, the Railway address as `SLAM_IT_PUBLIC_URL`,
   and `SLAM_IT_STAGING=1`. The signing key is needed with S3 too.
3. **`slam-it-worker`**: `docker/worker.Dockerfile`, no domain,
   `SLAM_IT_WORKER_TOKEN` as the server's, and
   `SLAM_IT_SERVER=http://slam-it.railway.internal:8000`.
4. **The check:**
   - `/healthz`, and `curl -I http://` to the Railway address redirecting to `https://`, so
     that staging's Basic auth never crosses in clear;
   - the server running: it refuses to run as root, so running, it is the entrypoint's user
     (uid 10001);
   - a PUT of the wrong size through a fresh upload link, refused by the bucket;
   - a PUT through a fresh upload link, begun before the link expires and finished after: if the
     bucket keeps it, an upload slower than about 0.5 Mbit/s (500 MB over the 135 minutes before
     its job's last clearing) stays until the job's 7 days, or the bucket's lifecycle rule, and
     the note should say so;
   - a clip uploaded from a phone; the job done; the viewer opened from its link, with no
     Content Security Policy error in the browser's console.

## The demo checklist

Before showing it in person:
- run `python scripts/serve_demo.py --lan` on the laptop, on a network you trust (a phone's
  hotspot rather than a café's Wi-Fi: the demo is plain HTTP), scan the printed QR code (or open
  the printed address) on a phone, give the code as the password, and upload a short clip of the
  room through to the viewer;
- keep a baked bundle at hand (`scripts/view_bundle.py`) in case the network fails;
- give the invite code to whoever records.

Recording, as the upload page says: stabilisation off where the phone allows; move sideways at
first, not only turning; texture and good light; slow, for the rolling shutter; under 60 s.

Afterwards: Ctrl-C stops the server and the worker; the data folder still holds the results,
and deleting it is the last step.

## Checks

- **The import:** the room encoded with PyAV, losslessly, imported upright from its side, thinned
  from 60 fps, scaled, and tracked within the room's monocular ATE bound (3 cm); blobs at known
  rays through a phone's profile, stored with a rotation of 90, -90 or 180, land within 0.1 px of
  where the written camera projects them; a Stray Scanner zip written as its encoders write one
  imports with the room's poses as its ground truth (to 1e-9), at Stray's 60 fps in HEVC and at
  its 30 fps setting; other containers, oversized or too many frames, frames that change size,
  hostile zips and malformed CSVs refused, leaving nothing.
- **The intrinsics:** profiles matched or not by phone, lens, focal length, orientation and aspect,
  a profile named for another lens than it calibrates and Apple's 2x among them; the fisheye model's
  turning point, refusals and failure to hold; the projection and the map against OpenCV; the room
  rendered through a fisheye undistorts to its pinhole view.
- **The worker,** against a stand-in server on a local port: the room's zip run end to end, its
  stages and heartbeat, transfers included, refusals, a timeout, a server letting the job go, a
  report lost or a failure report refused, an oversized upload, a child out of memory; a job's
  processes gone after a timeout, a 409 and an orphan, and with its folder after the worker is
  stopped; its environment, the URLs it trusts and the redirects it follows.
- **The server,** with FastAPI's test client and folder storage: every route and guard, the job
  list's life, format, caps and sweep, a deletion retried, a replayed upload link and one that lands
  late, the logs, the Content Security Policy; S3's links signed, and its errors read, offline
  (botocore's Stubber); the caps and names that the import, worker, server and pages each hold, held
  equal; and an end-to-end run with the server under uvicorn and the real worker on the room's zip.
- **The pages' logic** in Node, and the whole path by hand in a headless browser: upload, status,
  the worker, the viewer.
- **The images,** by hand: one job through both containers on a private Docker network, from the
  room's Stray Scanner zip and from its video, read back through the result's link.
- **Breaking each fix** the review panel asked for, one at a time, fails a test.

## What waits for the real clips

The tests run on the room's synthetic recordings. A short clip from each test phone (iPhone 14
Pro, Pixel 8, Galaxy A15) and a Stray Scanner recording, kept outside the repo, will check:
- the rotation's direction on a real portrait video;
- Stray Scanner's zip layout, the frame size `camera_matrix.csv` describes, `odometry.csv`'s
  columns, and the conventions read in its source (its poses' axes, its first frame's stamp);
- whether Android videos name their phone, and how lenses are named in real metadata;
- whether the 35mm-equivalent focal length is taken across the diagonal, and whether the nominal
  field of view should be one per aspect: the pooled median, 72.37 degrees, mixes 16:9 and 4:3
  main-camera profiles, whose own medians differ by about 13.5 degrees;
- what Apple's 35mm-equivalent tag gives for each lens and for the 2x crop: the still photo's
  figure, as one published iPhone clip suggests, or the video frame's own (the focal-length
  thresholds sit between the still photo's figures; the band a profile is held to holds under
  either). Against Apple's published figures, at 16:9, a profile's own focal length is 1.21
  times its lens's figure for main cameras (median of 134), 1.23 for telephotos and 1.28 for
  ultrawides, and 1.02 to 1.05 at 4:3: the 16:9 crop explains 1.09 of it, stabilisation the
  rest. If the tag is the still photo's figure, the 35mm source is about 17% low on 16:9 video;
- whether a profile calibrated at a 16-row-padded size (3840x2176, the Pixel 8's main among them)
  is cropped, not stretched, to a 16:9 video;
- whether a phone ever writes a mirror in its display matrix, which reads as a half turn;
- how far `odometry.csv`'s per-frame intrinsics stray from the camera matrix, which the import
  does not record yet.

## What it does not do yet

- Joining a run's maps: each map a lost track closed stays apart from the next (milestone 2B's
  B4).
- Recording in the page, with a live camera view, or streaming a live camera.
- Comparing the intrinsics' sources in the web app: measured once, on Stray Scanner recordings,
  and written up outside the repo.
- A GPU worker: the same protocol, run elsewhere, for reifyscene.
- Before going public (the security pass, and the ladder's open items): the result's token out
  of every path, so that no host's log holds it; the fonts self-hosted and HSTS on the domain; a
  token per worker; the lock's hashes required in the images.
