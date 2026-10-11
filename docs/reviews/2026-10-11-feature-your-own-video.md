# Review panel: `feature/your-own-video`

<!-- Written by /slam-it-all-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-11

| | |
|---|---|
| **Branch** | `feature/your-own-video` |
| **Range** | `63b61da...cbf7b3d` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus); a security pass (opus) beside them, in [its own report](2026-10-11-feature-your-own-video-security.md) |
| **How they ran** | registered slam-it-all-* agents; the security pass as a general-purpose agent with its own brief (the panel has no security reviewer) |
| **Layer 1** | all checks passed (`scripts/check.sh`, exit 0; pytest 3431 passed), on the tree committed as `cbf7b3d` |

Findings that more than one reviewer raised share a row, with all their IDs, at the highest
severity given; the security report's IDs (S*) are named where a row overlaps one of its
findings. Two of the algorithm reviewer's findings rest on Stray Scanner's source, which was
read again before this report: `OdometryEncoder.swift` writes `q_WA * q_AC` with
`q_AC = simd_quatf(ix: 1, iy: 0, iz: 0, r: 0)` (L1), and `DatasetEncoder.swift` passes the
frame counter before its increment, from -1, to a video encoder that stamps frames at
`CMTime(value: frameNumber, timescale: 60)` after `startSession(atSourceTime: .zero)` (L3).

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 / A1 (S1) | must-fix | hygiene, architecture, security | `orchestration/worker.py:317-354, 390-398, 425` | the timeout kills only the Python child, never the C++ program it started; any exception from a heartbeat (a 409, the server away) leaves both running; a failed `failed()` report escapes `handle` and stops the worker | fix | fixed (68aaf3c) -- decided by evan-wes, 2026-10-11 |
| A2 (S7) | must-fix | architecture, security | `server/.../app.py:121-134, 275-279`; `jobs.py:347-353` | a sweep deletes the job rows before their files, so a storage failure loses those files for good, past the 7 days, silently | fix | fixed (f616342) -- decided by evan-wes, 2026-10-11 |
| H2 / A7 | must-fix | hygiene, architecture | `app.py:275-279`; `worker.py:320, 394-396` | the sweeper, the worker's "an internal error" and the child's stderr swallow exceptions without a log | fix | fixed (68aaf3c: the worker; f616342: the sweep) -- decided by evan-wes, 2026-10-11 |
| L1 | must-fix | algorithm | `eval/video.py:109, 446-456`; `tests/eval/recordings.py:87` | Stray Scanner's rotations are already in OpenCV's camera axes; the import turns them a second time, so a Stray recording's ground truth is a half turn about x from its frames (RPE wrong, ATE not) | fix | fixed (0a26105) -- decided by evan-wes, 2026-10-11 |
| L2 | must-fix | algorithm | `eval/lenses.py:242-290, 355-356, 480` | the fisheye model's failure paths (`rising`, `unbend`'s refusals, `holds`, no rectangle) are untested, though `holds()` drops 84 real profiles; `rising()` returns about pi/2 when the bend falls from the start | fix | fixed (d8556cc) -- decided by evan-wes, 2026-10-11 |
| H3 | should-fix | hygiene | `eval/video.py` `_write_frames` | one function sets up, filters, resamples and writes, with eight mutable locals; `probe` is read twice | fix | fixed (0a26105) -- decided by evan-wes, 2026-10-11 |
| H4 | should-fix | hygiene | `server/.../storage.py` | `S3Storage.size` reads any client error as "absent"; delete errors are ignored, in S3 and in the folder | fix | fixed (f616342) -- decided by evan-wes, 2026-10-11 |
| H5 | should-fix | hygiene | `lenses.py`, `worker.py`, `upload_view.js`, `status_view.js` | magic numbers: lens-kind focal thresholds, timeouts, the page's 5 s and "7 days" | fix | fixed (d8556cc, 68aaf3c, a5b122c) -- decided by evan-wes, 2026-10-11 |
| H6 (S8) | should-fix | hygiene, security | `jobs.py:175-260`; `app.py:391-401, 461-483` | every read takes SQLite's write lock; the claim's SQLite work and folder storage's writes run on the event loop; `create` raises `StateError` for bad input | fix | fixed (f616342) -- decided by evan-wes, 2026-10-11 |
| H7 (S16) | should-fix | hygiene, security | `worker.py:Client`; `docker/worker.Dockerfile` | the worker accepts any server URL: an unset one retries forever, and `file://` or plain HTTP links are followed | fix | fixed (68aaf3c) -- decided by evan-wes, 2026-10-11 |
| H8 | should-fix | hygiene | `lenses.py`, `jobs.py`, `viewer/src/*_view.js` | the centred pinhole built twice; `Job.working` and `Jobs.working` mean different things; the pages' `main()` undocumented | fix | fixed (d8556cc, f616342, a5b122c) -- decided by evan-wes, 2026-10-11 |
| A3 | should-fix | architecture | `CLAUDE.md:39, 57-60`; ladder :88, :357-414; run-viewer design :168; `viewer/src/jobs.js:7` | the docs have not caught up: no server in CLAUDE.md or the ladder's tables and contracts, pages said to read only bundles, `bundle.json`'s intrinsics source promised, a wrong constant cited | fix | fixed (1504bd5) -- decided by evan-wes, 2026-10-11 |
| A4 | should-fix | architecture | `pyproject.toml` | three contracts missing: the server may import `av`, `PIL`, `cv2` or reifyscene; nothing keeps `eval.video` (the optional extra) out of `orchestration.vo` and `.bundles` | fix | fixed (cd1a668) -- decided by evan-wes, 2026-10-11 |
| A5 | should-fix | architecture | `worker.py:373-388`; `config.py`; `app.py:149` | the worker's own stages (downloading, uploading) send no heartbeat, so a transfer over 2 minutes fails the job; the timing constants' relation is untested | fix | fixed (68aaf3c) -- decided by evan-wes, 2026-10-11 |
| A6 / H10 | should-fix | architecture, hygiene | `worker.py:241-242`; `eval/video.py:547-548`; `lenses.py:708-719` | a missing lens database silently degrades every video to the fallbacks; the lookup is written twice | fix | fixed (d8556cc, 68aaf3c) -- decided by evan-wes, 2026-10-11 |
| L3 | should-fix | algorithm | `eval/video.py:187-207, 497-505`; `tests/eval/recordings.py:69-103` | a Stray recording's first frame is stamped at -1/60 s behind an edit list, so FFmpeg drops it and the import would refuse every real recording | fix | fixed (0a26105) -- decided by evan-wes, 2026-10-11 |
| L4 | should-fix | algorithm | `lenses.py:400-430, 558` | Apple's own Camera app names every rear lens "back triple camera", which reads as the main camera, so an ultrawide or telephoto clip gets the main lens's profile | fix | fixed (d8556cc) -- decided by evan-wes, 2026-10-11 |
| L5 | should-fix | algorithm | `tests/eval/test_video.py:40-121`; `tests/eval/test_lenses.py:201-286` | scaling, undistortion and turning are tested by arithmetic, not on image content; only +90 is turned; no OpenCV parity | fix | fixed (d8556cc: OpenCV parity; 0a26105: rays through the import) -- decided by evan-wes, 2026-10-11 |
| H9 (S6) | note | hygiene, security | `config.py`; `jobs.py:212-232` | an upload that never arrives holds one of the 5 places for an hour | see S6 | fixed (f616342, as S6) -- decided by evan-wes, 2026-10-11 |
| H11 | note | hygiene | `scripts/serve_demo.py` | the demo does not notice its worker dying | fix | fixed (a5b122c) -- decided by evan-wes, 2026-10-11 |
| A8 | note | architecture | `app.py:17-18`; `jobs.py:37`; `app.py:412-413` | the protocol is specified by its client, and the server's states are the `vo` worker's stages | defer (until a second kind is planned) | deferred (the ladder's open items, PR 11's: until a second kind of job is planned) -- decided by evan-wes, 2026-10-11 |
| A9 | note | architecture | `eval/lenses.py:128-318` | a Kannala-Brandt camera now lives outside rung 0; TUM-VI (milestone 6) will need it there | defer (ladder open item) | deferred (the ladder's open items, PR 11's: with TUM-VI, milestone 6) -- decided by evan-wes, 2026-10-11 |
| A10 | note | architecture | `eval/video.py:115-117, 210-215` | a third copy of "the fastest built backend", in another order | defer (PR 7's H18/A13, this copy recorded) | deferred (the ladder's open items, PR 7's H18/A13, this copy recorded) -- decided by evan-wes, 2026-10-11 |
| A11 | note | architecture | `eval/video.py:254-267`; `eval/tum.py` | the TUM folder is written by hand in `eval.video` but read in `eval.tum` | fix | fixed (0a26105) -- decided by evan-wes, 2026-10-11 |
| A12 | note | architecture | `server/pyproject.toml:15`; `app.py:49, 52` | `pydantic` and `botocore` are imported but not declared | fix | fixed (a5b122c; the ladder in 1504bd5) -- decided by evan-wes, 2026-10-11 |
| L6 | note | algorithm | `lenses.py:90, 577` | profiles at 16-row-padded sizes (88, the Pixel 8's main among them) are stretched to 16:9, not cropped | defer (to the real clips; listed in the note) | deferred (to the real clips: the note's list, and the ladder's open items, PR 11's) -- decided by evan-wes, 2026-10-11 |
| L7 | note | algorithm | `eval/video.py:179-184, 237` | a mirror in the display matrix reads as a half turn | defer (to the real clips; listed in the note) | deferred (to the real clips: the note's list, and the ladder's open items, PR 11's) -- decided by evan-wes, 2026-10-11 |
| L8 | note | algorithm | `lenses.py:96, 678-705` | the nominal field of view pools aspects (70.16 degrees for 16:9 alone), and the database cannot settle the 35mm convention | defer (to the real clips) | deferred (to the real clips: the note's list, and the ladder's open items, PR 11's) -- decided by evan-wes, 2026-10-11 |
| L9 | note | algorithm | -- | where the field is on uncertain intrinsics: focal refinement in BA (COLMAP), learned calibration (GeoCalib) | no action (context for later rungs) | no action: context for later rungs, not a defect -- decided by evan-wes, 2026-10-11 |

### Hygiene review

## Hygiene review

**Scope:** PR 11 (`feature/your-own-video`), 63b61da...cbf7b3d. I read in full: `server/src/slam_it_all_server/{app,jobs,storage,config,__main__}.py`, `src/slam_it_all/orchestration/worker.py`, `src/slam_it_all/eval/{video,lenses}.py`, `scripts/serve_demo.py`, `viewer/src/{jobs,upload_view,status_view,page_view}.js`, both Dockerfiles, `.dockerignore`, `server/settings.template.env`, `tests/server/test_shared_names.py`. For the test files I read only the test names, not the bodies. I did not run anything.

**Summary:** The code is very well documented and carefully validated at its boundaries: size caps, zip hygiene, strict CSV parsing, constant-time secret comparison, atomic staged writes. I found two real failure-handling bugs in the worker and one swallowed exception in the server's sweep. Everything else is small.

### Findings

#### [must-fix] A job abandoned or failed mid-run leaves the child process running, and a failed report can crash the worker
- **Where:** `src/slam_it_all/orchestration/worker.py` `_run_child` (about line 300), `handle` (about lines 352-368).
- **Issue:** There are two defects.
  1. `_run_child` has no `try/finally` around the `Popen` loop. `report(...)` calls `client.progress`, which raises `AbandonedError` on a 409 or a `URLError` on a network fault. The exception leaves `_run_child` with the child still alive, and `handle`'s `finally` then `rmtree`s the job folder under it.
  2. In `handle`, the `except RefusedError` branch calls `client.failed(...)` unguarded, unlike the `except Exception` branch, which wraps it in `suppress`. `Client._post` turns a 409 into `AbandonedError`, and a network fault is a `URLError`. Both escape `handle`, and `serve` does not catch them around `handle(...)`, so the worker process dies.
- **Why it matters:**
  - A cancelled or expired job keeps a 6 GiB-limited C++ pipeline running for up to 20 minutes after the server let it go. The next claim then runs beside it. The server allows one job at a time, so the machine has two.
  - A job whose `failed` call hits a 409 or a hiccup kills the worker. This happens when the server expired the job for silence while the child timed out, or when the server blipped as the refusal was reported. In the container this means a restart loop.
- **Evidence:** `_run_child` has no `try`, `finally` or `child.kill()` outside the timeout branch. `handle`: `except RefusedError as refusal: client.failed(job, str(refusal))`. `test_a_job_the_server_lets_go_is_dropped_quietly` only asserts that no done or failed call is made; it does not assert that the child is gone.
- **Suggested fix:**
  - Wrap the loop in `try/finally` that kills and waits on a still-running child.
  - Put every `client.failed` call behind one helper that catches `AbandonedError` and `OSError`, and logs.
  - Add tests for a 409 on `failed`, and for the child being gone after a 409 on progress.

#### [must-fix] Swallowed exceptions with no log: the sweep loop and the worker's "internal error"
- **Where:**
  - `server/src/slam_it_all_server/app.py` lifespan `sweeping()` (`contextlib.suppress(Exception)  # a sweep that fails is tried again`).
  - `worker.py` `handle` (`except Exception: with contextlib.suppress(Exception): client.failed(job, "an internal error")`).
  - `worker.py` `_run_child` (`stderr=subprocess.DEVNULL`), and `process`, which only catches `SlamItAllError` and `FileNotFoundError`.
- **Issue:** An unexpected error is dropped without a trace in each place.
- **Why it matters:**
  - A sweep that fails every time (a storage credentials fault, a locked database) is invisible. The privacy promise (uploads and results deleted on time) then silently fails.
  - An operator seeing "an internal error" has nothing to debug with: the exception, the child's traceback and the stderr are all discarded.
- **Evidence:** The lines quoted above. There is no `logging` and no `print(..., file=sys.stderr)` on those paths.
- **Suggested fix:**
  - Log with `logging.exception` or `traceback.print_exc()` to stderr in both places.
  - Send the child's stderr to the worker's own stderr, or keep its tail for the log.
  - The reason shown to the owner can stay as it is.

#### [should-fix] `_write_frames` does several jobs in one 55-line function
- **Where:** `src/slam_it_all/eval/video.py` `_write_frames`.
- **Issue:** The function does first-frame setup (size check, rotation, scaling, building the undistortion map), per-frame spacing and ordering filters, resize and undistort and write, then `rgb.txt`/`camera.txt` output. It tracks eight mutable locals (`upright`, `mapping`, `size`, `turns`, `count`, `times`, `decoded`, `stamps`).
- **Why it matters:** It is the hardest function to read in the PR, and it is the one the real-clip assumptions will land on.
- **Suggested fix:**
  - Extract a `_setup(frame, lens)` that returns `(upright, mapping, size, turns)`.
  - Extract `_write_metadata(out, upright, stamps)`.
  - Extract the spacing and ordering filter into one predicate. The two checks `elapsed - times[-1] < MIN_SPACING` and `elapsed <= times[-1]` are split across an `elif` and a later `if`.
  - `probe(video)` is called again just for the metadata; take it from the caller instead.

#### [should-fix] `S3Storage.size` and the deletes treat every error as "absent" or "done"
- **Where:** `server/src/slam_it_all_server/storage.py:S3Storage.size`, `S3Storage.delete_prefix`, `FolderStorage.delete_prefix`.
- **Issue:**
  - `size` catches `ClientError` broadly. A 403 (bad credentials) or a 5xx reads as "not there", so the owner sees "the upload has not arrived whole" and a worker's result looks missing. It should return None only for a 404.
  - `delete_objects` ignores the `Errors` list in its response.
  - `shutil.rmtree(..., ignore_errors=True)` hides a failed delete of data the retention policy promises to remove.
- **Suggested fix:** Check the `404` code in `size`, report or raise on the delete errors, and rely on the sweep (once it logs) to retry.

#### [should-fix] Magic numbers without a name
- **Where:**
  - `lenses.py` `lens_kind`: `focal <= 16` and `focal >= 50` (the docstring explains them but they are not constants).
  - `upload_view.js` `setTimeout(() => done(null), 5000)`.
  - `status_view.js`: "results are kept 7 days" hard-coded, while `RETENTION` lives in `config.py` and is not covered by `test_shared_names`.
  - `worker.py`: the 60.0 s `_post` and download timeouts, the 300.0 s upload timeout and the `wait + 30.0` margin.
- **Suggested fix:** Name them as documented constants. Add `RETENTION` days to the shared-names test, or have the status response carry the days.

#### [should-fix] Reads take a write transaction
- **Where:** `jobs.py` `by_token`, `by_id`, `working`, `ahead`.
- **Issue:** All of them go through `_changing()`, which runs `BEGIN IMMEDIATE`. Every 2 s status poll and every `owned()` lookup takes SQLite's write lock.
- **Why it matters:** At QUEUE=5 this is harmless, but it serialises reads behind writes and a long-held claim. A read-only helper, `BEGIN DEFERRED` or a plain `connect().execute()`, is the standard shape.
- **Also:** `Jobs.create` raises `StateError` for an unknown upload or kind, which should be `ValueError`/`InvalidInputError`. `StateError` means "not the worker's any more" everywhere else.

#### [should-fix] The worker image does not check `SLAM_IT_ALL_SERVER`
- **Where:** `docker/worker.Dockerfile` CMD and `worker.py:main`.
- **Issue:** An unset `SLAM_IT_ALL_SERVER` gives `--server ""`. `claim` then fails with a `ValueError` ("unknown url type"), which `serve` catches and retries every 5 s forever, printing "worker: no job".
- **Suggested fix:** Have `Client.__init__` validate the server URL (http or https, with a host), as it validates the token.

#### [should-fix] Smaller readability and duplication points
- **Where:** `lenses.py`, `jobs.py`, the JS pages.
- **Issue:**
  - `lenses.from_35mm` and `lenses.nominal` both end with the same `Pinhole(focal, focal, (w-1)/2, (h-1)/2, w, h)`. A `_centred(focal, w, h)` helper would remove it.
  - `Job.working` (property) and `Jobs.working` (method) share a name and mean different things.
  - `upload_view.js` `main()` and `status_view.js` `main()` have no JSDoc, unlike every other function in the JS.

#### [note] A failed upload holds a queue slot for up to an hour
- **Where:** `config.py` (QUEUE=5, UPLOAD_WAIT=1 h) and `upload_view.js`.
- **Issue:** If a browser makes a job and the PUT then fails, the job stays `waiting`. Five abandoned uploads block everyone for an hour. This is probably acceptable for a private demo, but it is worth stating in the note.

#### [note] Worker falls back to "nominal" intrinsics silently when the profile folder is missing
- **Where:** `worker._work` (`profiles = lenses.load(folder) if folder.is_dir() else []`).
- **Issue:** The fallback is visible in the summary's `intrinsics` field, so it is not hidden from the owner. A misconfigured worker would still quietly degrade every job, so a one-line stderr warning would help.

#### [note] `serve_demo` does not notice the worker dying
- **Where:** `scripts/serve_demo.py`.
- **Issue:** It does `return server.wait()` and never polls `worker`. If the worker dies, the demo keeps serving jobs that queue forever.

### Checked and fine
- **Input validation at the server's boundaries:** Pydantic bounds on every body, the size cap checked against the declared size and signed into the link, `check_key` on every storage key, result paths and bytes capped, the stage order enforced.
- **Auth and request hygiene:** constant-time secret comparison (`hmac.compare_digest`); `Authorization` parsing; the request-body cap with a 411 for chunked bodies; response headers; `noindex`.
- **Folder storage:** atomic PUT through a `.part` file, the signed size enforced while streaming, and the signature covering method, key, expiry and size.
- **Zip import:** names that climb out are refused, duplicates and caps are checked before and during the read, and only three files are ever opened. Frame, metadata and timing handling in `video.py` is sound: upright rotation, a PNG per frame, and only the named metadata keys recorded. The staged-folder pattern leaves nothing behind on failure.
- **`lenses.py`:** the maths is documented with conventions (pixel centres at integers, the `numpy.rot90` direction). Bisection avoids Newton's overshoot. The fetch bounds (archive size, profile size, path filtering, staging) are good. Constants are documented and cite the data they were measured on.
- **Documentation:** numpy-style docstrings carry shapes, units and frame conventions. Module docstrings state each module's purpose and its open assumptions, as the brief asked.
- **Docker images:** digest-pinned base images, a non-root user, and the lock as pip constraints. The server image has no compiler or decoder.
- **Cross-language copies:** a test (`test_shared_names.py`) holds the Python and JS caps and names equal. This is the right answer to the server not importing `slam_it_all`.
- **Tests:** the names show coverage of the happy path, hostile and malformed zips, auth, caps, sweep and expiry, 409 handling, OOM and timeout, staging auth and the entry point. The gaps are the two worker cases in finding 1.
- **Frontend:** `textContent` is used throughout, with no `innerHTML`; `encodeURIComponent` on the token.

Out-of-lane notes:
- Security reviewer: the viewer takes `?bundle=<url>`, and `viewerLink` builds it from the server's `status.bundle`, so its trust model deserves a second look.
- Correctness reviewer: the real-clip assumptions listed in the PR note, which I left alone as instructed.

### Architecture review

## Architecture review

**Scope:** PR 11 "your own video", `63b61da..cbf7b3d` on `feature/your-own-video` (11 commits, 66 files). I read every changed module in full: `eval.video`, `eval.lenses`, `eval.images`, `orchestration.worker`, the `slam_it_all_server` package, the pages and modules in `viewer/`, `docker/`, `scripts/serve_demo.py`, and the import-linter contracts in `pyproject.toml`. I also read the docs (ladder, run-viewer design sections 3, 5 and 9, the your-own-video note, `CLAUDE.md`, README) and enough of the neighbouring code to check the fit (`orchestration.vo`/`bundles`, `eval.tum`, `foundations.camera`/`image`, `backends`). Small probes were run in the `slam-it-all` env under a memory cap. They are saved outside the repo at `pr11_worker_server_probes.py`.

**Summary:** The package split is right, and the two-way contracts hold it. The server never imports `slam_it_all`, `slam_it_all` never imports the server or its web stack, and the constants that must agree across that line are held equal by a test rather than imported. Each new piece sits on a sensible rung, and fisheye undistortion reuses rung 0's `remap` instead of adding a kernel. Two things are not production quality: the worker's supervision of a job's processes, and the server's sweep, which can silently break the 7-day retention promise. Both are reproduced below. The docs (`CLAUDE.md`, the ladder, run-viewer section 3) have not caught up with what was built, and three contracts that the PR's own reasoning relies on are not declared.

### Findings

#### [must-fix] The worker's timeout and a 409 leave the job's processes running, and a failed report stops the worker
- **Where:** `src/slam_it_all/orchestration/worker.py:317-354` (`_run_child`). The `Popen` at 317-323 has no process group of its own; `child.kill()` at 334 is the only cleanup; `report(...)` at 338 and 346 can raise with no `finally`. Also `worker.py:393` (`client.failed` unguarded inside `except RefusedError`) and `worker.py:425` (`serve` calls `handle` with no guard). The C++ program is the child's own subprocess (`orchestration/vo.py:680`).
- **Issue:**
  1. The timeout kills only the Python child. The native `slam_it_all_vo` it started keeps running.
  2. Any exception from a heartbeat leaves `_run_child` without stopping even the Python child. That includes the 409 `AbandonedError`, a `URLError` while the server redeploys, and an HTTP 500 (for example, the upload delete in `app.py:412-413` failing).
  3. If `client.failed` raises in the `RefusedError` branch (a 409, or the server away), the exception escapes `handle` and ends `serve`. This contradicts the comment at 394, "a job's failure must never stop the worker".
- **Why it matters:** Design section 5.3's guards ("one job at a time", "each job has a timeout and a memory limit") do not hold for the process that does the work. After a timeout, a hung or compute-bound C++ run keeps its CPU and up to 6 GiB while the worker claims the next job. `RLIMIT_AS` is per process, so two runs can coexist in one container. A server redeploy during a job aborts it and orphans its processes; decision 9 already plans for redeploys with the 3 s shutdown. Today the orphans die only by accident: a BrokenPipe on their next stage line, or reading files that `handle`'s `rmtree` removed. A C++ run busy in a long BA does neither. The tests miss all three paths. `test_a_job_the_server_lets_go_is_dropped_quietly` cancels at "downloading", before a child exists, and `test_a_job_past_its_time_fails` never checks for surviving processes.
- **Evidence:** The real `_run_child` was driven with a stand-in child that reports a stage and then waits on a grandchild (`sleep 30`, standing in for `vo.run`'s `subprocess.run`):
  ```
  1. grandchild alive after the timeout: ['1540225']
  2. child/grandchild alive after a 409: ['1540278']
  3. escaped handle(), so serve() ends: AbandonedError
  ```
  An earlier run of probe 2 also found the Python child itself still alive.
- **Suggested fix:**
  - Start the child with `start_new_session=True`. It combines with the existing `preexec_fn`.
  - In a `try/finally` around the select loop, `os.killpg(child.pid, SIGKILL)` and `wait()` on every exit that is not a clean end.
  - Treat a heartbeat's connection error as transient: log it and keep working. Only a 409 abandons.
  - Guard `client.failed` in the `RefusedError` branch as the generic branch does.
  - Add tests: a 409 at "running" and a timeout during "running", each asserting that nothing in the job's process group survives.

#### [must-fix] A storage failure during a sweep loses the deleted jobs' files for good, silently
- **Where:** `server/src/slam_it_all_server/app.py:121-134` (`sweep`: the job list is swept and committed first, then the files are deleted). `jobs.py:347-353` deletes the rows in a committed transaction. `app.py:277` is `contextlib.suppress(Exception)` with the comment "a sweep that fails is tried again".
- **Issue:** The rows of jobs past retention are deleted and committed before their files are. If storage raises (an outage, bad or rotated S3 credentials), the rest of `swept.deleted` keeps its files forever. No later sweep can find them, and nothing is logged.
- **Why it matters:** "A result is deleted after 7 days" (design section 5.4, and stated on the upload page) silently stops being true, and the bucket grows without bound. The operator gets no signal, because the sweeper swallows the exception. Ended jobs' uploads only get a second chance at their 7-day deletion.
- **Evidence:** Folder storage whose deletes fail once, a done job whose 7 days are up, a sweep under the lifespan's `suppress`, then a healthy sweep:
  ```
  4. job row kept: False | result file still in storage: True
  ```
- **Suggested fix:** Delete a job's files before its row. For example, keep the row (or mark it `deleting`) until its storage deletes succeed, then delete it in a second transaction, so a failed sweep is retried. Log the sweeper's exception instead of suppressing it. Add a test with a store that fails once.

#### [should-fix] The docs have not caught up with what was built
- **Where:**
  - `CLAUDE.md:39`: the JavaScript row says the viewer "reads only a bundle".
  - `CLAUDE.md:57-60`: the layout and contracts never mention `slam_it_all_server`, a second import-linter root, or its two-way contracts.
  - Ladder `:88`: the upload and status pages are "reading only bundles".
  - Ladder `:357-371`: the package table has no server row. The eval row lacks the import and intrinsics, and the orchestration row lacks the worker. Line 369 still says `viewer/` "reads only the bundles the bake writes".
  - Ladder `:399-414`: "Dependency contracts" lists neither server contract, and its "Inside packages" list omits the server's internal order.
  - `docs/design/2026-10-09-run-viewer.md:168`: `bundle.json` should hold "for a phone video, the intrinsics' source". It does not: `bundles.py` is unchanged, and `docs/notes/viewer.md` now lists it under "not yet".
  - `viewer/src/jobs.js:7` cites `slam_it_all_server.config.MAX_SECONDS`, which does not exist. The cap is `slam_it_all.eval.video.MAX_SECONDS`, which is what `test_shared_names.py` holds it to.
- **Issue:** The upload and status pages call the server's API (`/api/jobs`) and PUT to storage, so "only bundles" is now false. The server package and its rules are invisible in the files future sessions read first.
- **Why it matters:** `CLAUDE.md` is the entry point for every session. A session that has not been told that a second root package exists, and must never import `slam_it_all`, will not know the rule until import-linter fails.
- **Evidence:** The lines above. `grep -n server CLAUDE.md` finds nothing.
- **Suggested fix:** Change the docs, not the code:
  - add the server to `CLAUDE.md`'s Languages/Layout and the ladder's package table and contract list;
  - say what the pages read;
  - make run-viewer section 3 say the intrinsics' source is shown by the status page and not yet in `bundle.json`;
  - point `jobs.js`'s comment at `eval.video`.

#### [should-fix] Three contracts the PR's own reasoning needs are not declared
- **Where:** `pyproject.toml:256-263` (PyAV), `:318-321` (reifyscene), `:324-331` (the server).
- **Issue:**
  - (a) The server contract's stated purpose is "the process holding the secrets never parses a video". It bans only `slam_it_all`; nothing stops `slam_it_all_server` importing `av`, `PIL` or `cv2` directly.
  - (b) "slam_it_all never imports reifyscene" has `source_modules = ["slam_it_all"]`, so the new root package sits outside the one-way boundary. It is also the package most likely to gain reifyscene job kinds (design section 5.3).
  - (c) The PyAV allow-list ignores `slam_it_all.eval.video -> av`, which hides every chain through `eval.video`. That is the gap `:276-281` closes for pycolmap with "Pipelines never reach the COLMAP reference". Nothing stops `orchestration.vo` (the program the budget runs time) or `orchestration.bundles` from importing `eval.video` and so making the optional `video` extra mandatory for the pipeline.
- **Why it matters:** Each rule is stated in the docs or a comment but is checkable, and today a regression would pass layer 1.
- **Evidence:** The contract texts above. `slam_it_all` does not prefix-match `slam_it_all_server` in import-linter.
- **Suggested fix:**
  - Add `av`, `PIL`, `cv2` and `reifyscene` to the server's forbidden modules (or add `slam_it_all_server` to the reifyscene contract).
  - Add a forbidden contract: `slam_it_all.orchestration.vo` and `.bundles` may not import `slam_it_all.eval.video`.

#### [should-fix] The heartbeat covers only the child's stages, and its timing contract with the server is untested
- **Where:** `worker.py:373-376` (the "downloading" report, then a download with no heartbeat) and `:380-388` ("uploading", the links, then sequential PUTs with no heartbeat). `config.py` sets `SILENCE = 120`, `worker.py` sets `HEARTBEAT = 30` and `WAIT = 25`, and `app.py:149` bounds `Claim.wait` at 60. `tests/server/test_shared_names.py` holds caps and names only.
- **Issue:** Decision 9.3 says the worker repeats its stage every 30 s. The parent's own stages, downloading and uploading, never repeat. A transfer longer than 120 s gets the job failed as "the worker stopped answering". The worker's next report then gets a 409 and the job is silently dropped, with its result deleted by the sweep. `HEARTBEAT` well under `SILENCE`, and `WAIT` within the claim's bound, are cross-process invariants that no test states.
- **Why it matters:** This is fine on Railway's internal network. It breaks for the off-Railway GPU worker of section 5.3 on a home uplink (a 500 MB upload at 20 Mbit/s takes over 3 minutes), and a later edit to either constant fails every long job.
- **Evidence:** The code cited. `grep HEARTBEAT tests` finds only the monkeypatch in `test_worker.py:159`.
- **Suggested fix:** Report the stage between download chunks and between uploaded files, which keeps the worker single-threaded; its `preexec_fn` relies on there being no threads. Add `config.SILENCE >= 3 * worker.HEARTBEAT` and the claim-wait bound (exposed as a constant) to `test_shared_names.py`.

#### [should-fix] A missing lens database silently degrades every video's intrinsics, and the lookup is duplicated
- **Where:** `worker.py:241-242` and `eval/video.py:547-548`, both `profiles = lenses.load(folder) if folder.is_dir() else []`. Also `eval/lenses.py:708-719` (`default_folder`, from `$SLAM_IT_ALL_LENS_PROFILES` or `~/.cache`).
- **Issue:** With the variable unset or wrong, or the fetch skipped, every import falls to the 35mm or nominal source without a word. `source.json` and the status page then say "nominal", which reads the same as "this phone has no profile".
- **Why it matters:** This is environment-dependent behaviour that fails silently. CLAUDE.md asks for observable failure. The worker image sets the variable and fetches at build time, but the local demo and any other host depend on `$HOME`'s cache.
- **Evidence:** The two lines cited. Neither logs nor raises when the folder is absent or holds no `.json` files.
- **Suggested fix:** One `lenses` function that locates and loads the database and raises when it is absent or empty. The worker calls it once in `serve` and refuses to start without it. The CLI uses it unless `--source nominal`, `35mm` or `--intrinsics` makes the database unnecessary.

#### [should-fix] An internal failure in the worker leaves no trace for the operator
- **Where:** `worker.py:320` (`stderr=subprocess.DEVNULL`), `:394-396` (`except Exception` reports "an internal error" and logs nothing), `:354`.
- **Issue:** The child's traceback is discarded and the parent logs nothing. By design the owner sees a sanitised "an internal error", but the operator's logs on Railway get nothing either.
- **Why it matters:** CLAUDE.md says "no swallowed errors". On staging, the first job that fails for an unexpected reason (the C++ program killed by `RLIMIT_AS` lands here) cannot be diagnosed without reproducing it.
- **Evidence:** The lines cited, together with `vo.py:680-697`, where a signal-killed or unexpected exit raises `RuntimeError`, which `process` does not catch.
- **Suggested fix:** Send the child's stderr to the worker's own stderr, and log the exception with the job id in the generic branch. The protocol's `reason` stays sanitised.

#### [note] The protocol is specified by its client, and the server's job states are the `vo` worker's stages
- **Where:** `app.py:17-18` ("the worker's API is the one `slam_it_all.orchestration.worker` describes"); `jobs.py:37` (`STAGES` global, not per kind); `app.py:412-413` (the upload is deleted when the stage leaves `"importing"`); the summary keys that `viewer/src/jobs.js` `summaryRows` reads.
- **Issue:**
  - The API's owner, the server, points at a docstring in a package it may not import.
  - The server's state machine, and its upload-deletion guard, are keyed to one kind's internal stage names, as section 5.3 decided.
  - The summary's keys pass through the server opaquely, and no test holds them.
  - There is no protocol version, although the two services deploy separately.
- **Why it matters:** None of this bites while `vo` is the only kind. It will when the planned second client (reifyscene's GPU worker) and a second kind arrive.
- **Suggested fix:** When the second kind is planned, move the protocol into design section 5.3 with the server as its owner. Declare stages per kind, or make "upload consumed" an explicit message.

#### [note] A camera model now lives outside rung 0
- **Where:** `eval/lenses.py:128-318`. `Fisheye` is a full Kannala-Brandt model (`bend`, `unbend`, `source`, `pinhole`, `holds`). `Pinhole` is a fourth camera-with-size type beside `edge.frames.Camera`, `eval.tum.Camera` and `eval.euroc.Camera`, and it uses neither `foundations.camera`'s `(4,)` encoding nor `check_intrinsics`.
- **Issue:** Design section 5.2 decided rung 0 needs no fisheye model because the pipeline only sees pinhole frames. That is right for PR 11. But milestone 6 measures on TUM-VI, whose cameras are calibrated in Kalibr's equidistant (Kannala-Brandt) model, and `foundations/camera.py`'s docstring expects wide-angle models to join there.
- **Suggested fix:** Record an open item in the ladder: when TUM-VI arrives, Kannala-Brandt joins `foundations.camera` in all three languages and `lenses` uses it, rather than gaining a second copy. Fold `Pinhole` into the camera lift already planned (PR 8 panel, A8).

#### [note] A third copy of "the fastest built backend", the first in library code, in a different order
- **Where:** `eval/video.py:115-117` and `:210-215` use `("rust", "cpp", "python")`; `scripts/demo_two_view.py:63` and `scripts/demo_absolute_pose.py:78` use `("cpp", "rust", "python")`.
- **Issue:** This is open item PR 7 H18/A13. The output does not depend on the choice, since `remap` is bit-exact across backends, but the choice is now copied into the library.
- **Suggested fix:** Close the item with one helper in `backends` that takes a preference order and knows nothing of what the module computes. Or record this copy against the item.

#### [note] The TUM folder is written by hand in `eval.video` but read in `eval.tum`
- **Where:** `eval/video.py:254-267` writes `rgb.txt` and `camera.txt`; `eval/tum.py` owns `CAMERA_FILE` and the reader; `tests/orchestration/datasets.py:78-79` has another hand-written copy.
- **Issue:** The format's writer and reader live in different modules. Writing `groundtruth.txt` already goes through `runs.write_trajectory`. The round trip is tested, so the risk is low.
- **Suggested fix:** Put `tum.write_camera` and `tum.write_frames` beside the reader.

#### [note] The server imports dependencies it does not declare
- **Where:** `server/pyproject.toml:15` declares fastapi, uvicorn and boto3, but `app.py:49` imports `botocore.config` and `app.py:52` imports `pydantic` directly. The ladder's runtime-dependency list (`:100`) names neither.
- **Issue:** Both arrive transitively today and the lock pins them.
- **Suggested fix:** Declare both, and name pydantic in the ladder's runtime-dependency list.

### Checked and fine
- **Dependency direction.** The worker sits on top of orchestration (`worker > bundles > vo`). In eval, `video` is at the top beside `budgets`, `lenses` is at the bottom and depends only on `foundations.errors`, and every new import respects its layer. No reifyscene import anywhere.
- **Package split.** `slam_it_all_server` is its own distribution with its own `pyproject`, two-way forbidden contracts and an exhaustive internal order (`__main__ > app > jobs | storage > config`). The server image holds no `slam_it_all`, compiler or decoder.
- **Constants across the process boundary.** Caps, stages, kinds, uploads and intrinsics sources are held equal by `tests/server/test_shared_names.py` rather than imported across the line. The end-to-end test runs the real worker against the real server under uvicorn.
- **Optional dependencies.** PyAV is an optional extra confined to `eval.video` by contract. Pillow stays in `eval.images`: `write_png` was added there, sharing `_check_picture` with `encode_jpeg`. The runtime-dependency changes were written into the ladder first.
- **Language split.** No C++ or Rust was added, so no parity fixture or `Implementation` Protocol is owed. Fisheye resampling reuses rung 0's lock-step `image.remap` rather than adding a kernel.
- **Edge budgets.** Nothing in `foundations`, `edge` or `backends` gains a dependency. The worker runs the unchanged C++ program through `vo.run(..., backend="cpp")`.
- **Viewer.** `jobs.js` is a logic module, inside the DOM-free `tsc` check. DOM code stays in `*_view.js`, with `element` shared through `page_view.js`. 9cda8db's `ran`/`shownAt`/`spanText` are logic in `compare.js`, wired by `main.js`. `<base href="/">` makes `/r/<token>` resolve the pages' assets.
- **Settings.** Explicit environment variables with a placeholder template, no deployment specifics in the repo, `SettingsError` at startup, and staging an explicit switch.
- **Packaging.** Two images as section 5.6 plans, the base pinned by digest, the lock used as constraints, and lens profiles fetched at image build.
- **Tests layout.** Tests mirror the packages (`tests/server/`, `tests/orchestration/test_worker.py`, `tests/test_serve_demo.py` beside the other script tests). VERSION is 0.3.6.0, as the plan says.

### Algorithm review

## Algorithm review

**Scope:** PR 11 "your own video", `63b61da...cbf7b3d` (HEAD `cbf7b3d0`). I read `src/slam_it_all/eval/video.py`, `src/slam_it_all/eval/lenses.py` and `eval/images.py`, their tests (`tests/eval/test_video.py`, `tests/eval/test_lenses.py`, `tests/eval/recordings.py`) and the fixture profiles. I also read how the worker calls the import (`orchestration/worker.py:238-244`). I checked them against:
- OpenCV's fisheye model (Kannala & Brandt, TPAMI 2006) and OpenCV 5.0's `cv2.fisheye` itself;
- FFmpeg's display-matrix semantics;
- Apple's ARKit and AVFoundation documentation;
- Stray Scanner's published Swift source and its author's StrayVisualizer;
- CIPA DCG-001 for the 35mm equivalent;
- the whole Gyroflow database at the pinned commit.

The two test files pass (63 tests). The review was read-only. Scratch scripts are in `/tmp/scratch/review-algorithm/`. Importing `slam_it_all` triggered scikit-build's editable rebuild in the git-ignored `build/` tree; the working tree is clean.

**Summary:** The geometry is right, and I checked it on image content:
- the fisheye projection matches OpenCV to 2e-13 px, and the undistortion map matches `initUndistortRectifyMap` to float32 rounding;
- the pixel-centre scaling matches swscale's resampling to 0.008 px;
- the quarter turns follow FFmpeg's display-matrix semantics;
- the whole scale, undistort and turn chain lands known rays within 0.04 px.

One must-fix is a convention error that is checkable now against Stray Scanner's source: its `odometry.csv` rotations are already in OpenCV camera axes, and the import turns them a second time. A perfect estimate then scores an RPE of 1.33 cm a frame on 1.79 cm steps. The other must-fix is that none of the fisheye model's failure paths is tested, although one of them (`holds()`) decides 84 real profiles. Three should-fixes: a Stray video's first frame is very likely dropped by FFmpeg, so every real recording would be refused; Apple's default-app lens names would be read as the main camera; and the import's content-level checks are missing.

### Findings

#### [must-fix] Stray Scanner's poses are converted to our camera axes twice
- **Where:** `src/slam_it_all/eval/video.py:109` and `:446-456` (`ARKIT_TO_CAMERA`, `_arkit_poses`); module docstring `:46-51`. The fixture repeats the same assumption at `tests/eval/recordings.py:87`.
- **Issue:** Stray Scanner does not write ARKit's raw camera rotation. Its `OdometryEncoder.swift` writes `q = q_WA * q_AC`, with `q_AC = simd_quatf(ix: 1, iy: 0, iz: 0, r: 0)`. That is a half turn about x, `diag(1,-1,-1)`, which turns ARKit's axes (x right, y up, z out of the screen) into OpenCV's. This conversion has been in every recording in the `timestamp, frame, x, y, z, qx, qy, qz, qw` format since that format was introduced (commit 4947d46, 2021-08-24). The import right-multiplies by `diag(1,-1,-1)` again, which puts the rotation back in ARKit's axes.
- **Why it matters:**
  - **Wrong ground truth:** every Stray `groundtruth.txt` has its orientations a half turn about x from the frames' camera.
  - **Scores:** positions are untouched, so ATE is unaffected. The RPE composes full relative poses (`eval/metrics.py:222-227`), so it is wrong, and the viewer displays it (`viewer/src/main.js:51-53`, from `bundles.py:226-227`).
  - **Tests:** the test passes only because `stray_zip` writes raw ARKit rotations, the same assumption the import makes.
- **Evidence:**
  - Stray Scanner's source, [OdometryEncoder.swift](https://github.com/strayrobots/scanner/blob/main/StrayScanner/Helpers/OdometryEncoder.swift): `let q: vector_float4 = (q_WA * q_AC).vector`.
  - The author's own reader, [StrayVisualizer](https://github.com/kekeblom/StrayVisualizer) `stray_visualize.py` (`read_data`, `point_clouds`), uses the quaternion directly as `T_WC`. It passes `inv(T_WC)` to Open3D's `create_from_rgbd_image`/`integrate`, which use OpenCV axes.
  - ARKit's raw axes are given in [ARCamera.transform](https://developer.apple.com/documentation/arkit/arcamera/transform).
  - Reproduced with the import's own code (`stray_axes.py`):
    ```
    odometry rotation == R_W_cv already: True
    import's rotation == R_W_cv:         False
    angle of error (deg): 180.0
    ```
  - Effect on scores, on the room (`stray_rpe.py`), for a perfect estimate:
    ```
    correct truth: ATE rmse 4.11e-16 m; RPE translation mean 7.56e-18 m, rotation mean 0.000 deg; mean step 1.791e-02 m
    as imported : ATE rmse 4.11e-16 m; RPE translation mean 1.33e-02 m, rotation mean 0.438 deg
    ```
- **Suggested fix:**
  - Use the quaternion as written: drop the right-multiplication, and `ARKIT_TO_CAMERA` with it.
  - Make `stray_zip` emulate the encoder literally: take the ARKit transform, then write `q_WA * q_AC`. The current code should then fail the test.
  - Restate the docstring's assumption as Stray's conversion, citing the file.

#### [must-fix] The fisheye model's failure paths are untested
- **Where:** `src/slam_it_all/eval/lenses.py`:
  - `:242-252`: `rising`;
  - `:266-269`: `unbend`'s refusal;
  - `:278-290`: `holds`, and its use at `:480`;
  - `:355-356`: `undistortion`'s "no rectangle".
- **Issue:**
  - Every fixture profile and every test lens passes `holds()`.
  - Nothing tests a polynomial that turns back before the corners, `unbend` beyond the rising part or below zero, or `undistortion` raising. A grep of `tests/` for `holds|rising|unbend|does not hold|no rectangle` finds none.
  - These are the model's degenerate cases. The project's standard makes untested behaviour a must-fix, and so does the panel's rule for algorithm code.
- **Why it matters:**
  - **It decides real data:** `holds()` is the guard that keeps a non-invertible calibration from producing a garbage undistortion map, and it is the largest single reason profiles are dropped.
  - **A latent bug a test would catch:** `rising()` returns `theta[falling[0] - 1]`. When the slope is already non-positive at the first grid angle, that index is `-1`, and it returns about pi/2 instead of about 0. Today `holds()` still rejects such a lens, but only by accident.
- **Evidence:**
  - `holds.py` over the database at `886fc61`: `dropped for RMS: 26  dropped by holds(): 84`. One example is `Apple_iPhone 11 Pro Max_Wide Lens__4k_16by9_3840x2160-60.00fps`, whose bend stops rising at 1.126 rad.
  - `rising()` for `k1=-1e9`: `1.5707004529956536 (pi/2 = 1.5707963267948966)`, `bend there: -3875074949.4`, `holds(): False`.
- **Suggested fix:**
  - Add a fixture profile that `holds()` drops. The example above is CC0, or a synthetic polynomial that turns back inside the frame will do.
  - Test `rising()` against the analytic turning point of a simple polynomial, e.g. `k1 < 0` only, where the turn is at `theta^2 = -1/(3 k1)`.
  - Test that `unbend` refuses a negative angle and an angle beyond the bend's top.
  - Test that `undistortion` raises for a lens that leaves no rectangle.
  - In `rising()`, return `0.0` when `falling[0] == 0`.

#### [should-fix] A Stray recording's first frame is stamped before its session, so FFmpeg very likely drops it and the import refuses the zip
- **Where:** `src/slam_it_all/eval/video.py:187-207` (`_decode` opens with no options) and `:497-505` (the row count check); the fixture is `tests/eval/recordings.py:69-103` (30 fps, first frame at 0).
- **Issue:**
  - Stray's `DatasetEncoder` starts `currentFrame` at -1 and passes `totalFrames` (the value before the increment) to the video encoder. `VideoEncoder` stamps each frame at `CMTime(value: frameNumber, timescale: 60)` after `startSession(atSourceTime: .zero)`. So the first frame of every recording is stamped at -1/60 s. This has been so since commit 9e470db, "Fps button", 2022-02-15.
  - Apple documents that "the writer adds samples with timestamps earlier than the start time to the output file, but they don't display during playback". In an MP4 that means an edit list that starts after the first sample.
  - FFmpeg honours edit lists and drops that frame. The import then decodes N-1 frames against N odometry rows and refuses the recording.
- **Why it matters:** Source 1, the only source with exact intrinsics and ground truth, would fail on every real Stray Scanner zip. The failure is loud: "odometry.csv: N rows for the video's N-1 frames".
- **Evidence:**
  - [DatasetEncoder.swift](https://github.com/strayrobots/scanner/blob/main/StrayScanner/Helpers/DatasetEncoder.swift): `let totalFrames: Int = currentFrame` (starts at -1) … `rgbEncoder.add(..., currentFrame: totalFrames)`.
  - [VideoEncoder.swift](https://github.com/strayrobots/scanner/blob/main/StrayScanner/Helpers/VideoEncoder.swift) does the stamping.
  - Apple: [startSession(atSourceTime:)](https://developer.apple.com/documentation/avfoundation/avassetwriter/startsession(atsourcetime:)).
  - Emulated with PyAV, 10 frames with the first stamped -1 tick (`edit_list.py`, `ignore_editlist.py`):
    ```
    from -1, editlist | elst ... media_time=256 | decoded 9
    {'ignore_editlist': '1'} 10 [(0.0, 0), (0.0167, 20)]
    ```
  - Stray has had frame-count trouble before: [StrayVisualizer #3](https://github.com/kekeblom/StrayVisualizer/issues/3).
- **Suggested fix:**
  - Open a Stray `rgb.mp4` with `av.open(..., options={"ignore_editlist": "1"})` and keep the strict row count check.
  - Make `stray_zip` reproduce Stray's timing: 60 fps, the first frame at -1/60 behind an edit list.
  - Confirm on the real recording.

#### [should-fix] Apple's default Camera app names every rear lens "back triple camera", which the lens-kind rule reads as the main camera
- **Where:** `src/slam_it_all/eval/lenses.py:400-430` (`lens_kind`, `equivalent=False` reads words alone) and `:558` (`match`).
- **Issue:**
  - Third-party apps that pick one lens get names such as "back ultra wide camera". The default Camera app reports the virtual multi-camera instead, as `"<model> back triple camera <physical mm> f/<n>"` (or `dual`).
  - Those names carry no kind word, so every lens reads as "main".
  - An ultrawide or telephoto clip is then matched to the main camera's profile. That is silently wrong intrinsics, labelled with the best source. The 35mm tag in the same metadata would tell the lenses apart.
- **Why it matters:** For an ultrawide clip the field of view is off by tens of degrees, and the status page still names a matched profile.
- **Evidence:**
  - Published EXIF for an iPhone 11 Pro ([alexwlchan.net](https://alexwlchan.net/2020/how-do-i-use-my-iphone-cameras/)): ultrawide `"iPhone 11 Pro back triple camera 1.54mm f/2.4"`, telephoto `"... back triple camera 6mm f/2"`.
  - `probes.py`:
    ```
    'iPhone 11 Pro back triple camera 1.54mm f/2.4' -> main
    'iPhone 14 Pro back triple camera 9mm f/2.8' -> main
    telephoto clip (35mm tag 77) matched to: Apple_iPhone 14 Pro_24mm__1080p_16by9_1920x1080-25.00fps kind main
    ```
  - Whether video tags follow the photo naming is one of the note's open items. The rule as written fails whenever they do.
- **Suggested fix:**
  - When Apple's lens text has no kind word, classify by `APPLE_FOCAL_35MM` with the profiles' thresholds (16 mm and under: ultrawide; 50 mm and over: telephoto).
  - Refuse a profile whose kind disagrees with the 35mm tag.
  - Add the "triple camera" form to the `lens_kind` parametrisation.

#### [should-fix] The import's scaling, undistortion and turning are checked by arithmetic, not on image content
- **Where:**
  - `tests/eval/test_video.py:40-60` (only +90, at native size);
  - `:74-91` (scaling: intrinsics arithmetic on noise);
  - `:94-121` (a fisheye import on noise: sizes and intrinsics only);
  - `tests/eval/test_lenses.py:201-286` (no OpenCV reference).
- **Issue:**
  - Every real phone video goes through `frame.reformat(..., interpolation="AREA")`, then the remap, then `np.rot90`. No test checks that the frames land where the written `camera.txt` puts them.
  - The camera depends on swscale's pixel-centre convention, a dependency that a PyAV or FFmpeg change could alter.
  - The rotation test covers only +90. An iPhone portrait clip declares -90.
  - OpenCV is in the lock and already serves as a reference in `test_pnp`, `test_orb` and `test_image`, but not here.
- **Why it matters:** These steps decide the intrinsics of every upload. All are correct today: I checked them, as listed below. Nothing holds them there.
- **Evidence (my checks, which could become the tests):**
  - `scale_convention.py`: blobs at known centres in 1280x720 land within 0.008 px of `(c+0.5)s-0.5`. The `x*s` convention would be off by 0.17 px.
  - `chain.py`: rays drawn through the iPhone 14 Pro profile at 1920x1080, stored with rotation +90, -90 and 0, then imported. Each lands within 0.040 px of where the written camera projects it.
  - `opencv_parity.py`: `Fisheye.source` against `cv2.fisheye.distortPoints` is within 2.27e-13 px. The `undistortion` map against `cv2.fisheye.initUndistortRectifyMap`, with the output camera as P, is within 3.05e-05 px (OpenCV's float32 map).
- **Suggested fix:**
  - Add a content-level import test with blobs at known rays, through a profile, parametrised over rotations 90, -90 (the iPhone's matrix) and 180, scaled down.
  - Add a `cv2.fisheye` parity test for the projection and the map, with `rtol=0`, an explicit `atol` and a stated reason.

#### [note] Profiles calibrated at a 16-row-padded size are stretched to fit, not cropped
- **Where:** `src/slam_it_all/eval/lenses.py:90` (`ASPECT_TOLERANCE`) and `:577` (`best.fisheye.scaled(width, height)`).
- **Issue:**
  - 88 of the 1,047 usable profiles are at 3840x2176 (59 of them), 1920x1088 or 960x544. Those heights are 2160, 1080 and 540 rounded up to the 16-pixel coding block.
  - They match 16:9 videos within the 0.02 aspect tolerance, and are scaled with a different factor on each axis.
  - If the extra rows are coding padding, the right mapping is a crop at the same scale. The stretch puts fy 0.74% low and shifts cy by about 8 px at 4K (about 1.8 px at 480p).
- **Why it matters:** The Pixel 8 is a test device, and its main profile is 3840x2176. No real camera stretches one axis by 0.74%.
- **Suggested fix:** Add this to "What waits for the real clips". When the widths agree and the heights differ by under 16 rows, scale both axes by the width ratio and treat the difference as padding.

#### [note] A mirror in the display matrix is read as a half turn
- **Where:** `src/slam_it_all/eval/video.py:179-184` and `:237`.
- **Issue:** `frame.rotation` is FFmpeg's `av_display_rotation_get`, which ignores flips. A horizontal mirror alone reads as -180 (`probes.py`: `hflip only: frame.rotation = -180 -> quarter turns 2`). Such frames come out upside down. Their geometry stays consistent, because the stored pixels are an unmirrored camera image.
- **Suggested fix:** Read the `DISPLAYMATRIX` side data, and refuse or handle a negative determinant. This is rare in practice.

#### [note] The nominal field of view pools aspects, and the database cannot settle the 35mm convention
- **Where:** `src/slam_it_all/eval/lenses.py:96` and `:678-705`.
- **Issue:**
  - The 71.66-degree median reproduces (766 profiles), but it pools aspects. The 16:9 profiles (589) alone give 70.16 degrees, and the 4:3 ones 81.91. For the usual 16:9 phone video the difference is a 2.8% focal.
  - Separately, the profile names quote 4:3 still-photo equivalents: for 4:3 profiles fx sits at 0.985 of the diagonal reading of the name, for 16:9 profiles at 1.12 (`equiv.py`). So the database cannot measure which convention Apple's video tag uses; only the clips can. The diagonal itself is the standard (CIPA DCG-001, [summary](https://en.wikipedia.org/wiki/35_mm_equivalent_focal_length)).
- **Suggested fix:** Optional: a median per aspect.

#### [note] Where the field is on uncertain intrinsics
- The fallbacks are classic, which suits this CPU rung.
- The standard classic remedy for phone intrinsics is to refine the focal length (and distortion) in bundle adjustment. COLMAP does this by default: Schönberger & Frahm, "Structure-from-Motion Revisited", CVPR 2016.
- Learned single-image calibration now estimates focal and distortion far better than a median field of view: Veicht, Sarlin, Lindenberger & Pollefeys, "GeoCalib", ECCV 2024, [arXiv:2409.06704](https://arxiv.org/abs/2409.06704). MegaSaM (Li et al., CVPR 2025) optimises focal length jointly.
- Context for later rungs (focal refinement for sources 3 and 4, or the learned rungs), not a defect here.

### Checked and fine
- **The KB model:** `Fisheye.source` is OpenCV's model ([cv::fisheye](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html); Kannala & Brandt, TPAMI 2006), including the `r = 0` limit. `rising()`'s derivative `1 + 3k1θ² + 5k2θ⁴ + 7k3θ⁶ + 9k4θ⁸` is correct.
- **The inverse by bisection:** `unbend` is exact to 4e-16 even where OpenCV's `undistortPoints` gives up. On the ultrawide fixture OpenCV fails on 15 of 2,000 points at 1e-14 criteria, and 41 with default criteria, all at θ ≈ 45° on a rising stretch.
- **The undistortion rectangle:** it lies within the source's outermost pixel centres. The clip moves no pixel by more than 4.7e-8 px on the fixtures. The output keeps the lens's focal lengths, and the remap's 1/32-px fixed point never weights a neighbour outside the image.
- **Turning the camera:** `Pinhole.turned` matches `np.rot90`'s `(x, y) → (y, W-1-x)`.
- **Rotation direction:** this is FFmpeg's convention. `av_display_rotation_get` is counter-clockwise, and `fftools` autorotates it to `transpose=cclock` (see [display.h](https://ffmpeg.org/doxygen/trunk/display_8h.html)). An iPhone portrait clip's matrix gives -90, which the import turns clockwise, as players do.
- **Pixel-centre scaling:** `Pinhole.scaled` and `Fisheye.scaled` are exact for the centre convention. A profile scaled to the video, then to the output, composes to one scaling.
- **Timestamps:** presentation times relative to the first frame; 60 fps thinned to 30 by `MIN_SPACING`; jittered 30 fps keeps every frame; times strictly ascending.
- **Stray Scanner's layout, checked against its source:**
  - `camera_matrix.csv` is row-major (`cameraMatrix.transpose.columns`) and describes `rgb.mp4`'s stored size, since `VideoEncoder` is created at `videoFormat.imageResolution`.
  - The share zip nests the files one folder deep (`NSFileCoordinator .forUploading`).
  - The newer 15-column `odometry.csv`, with empty distortion cells, parses by column name.
  - `rgb.mp4` declares no rotation.
- **The 35mm-equivalent formula:** the focal is scaled from the frame diagonal (CIPA DCG-001), with the centre at `(W-1)/2, (H-1)/2`.
- **The profile database at `886fc61`:** all 1,164 phone profiles are OpenCV fisheye (no `distortion_model` set). None uses a digital lens or interpolation, and `calib_dimension` equals `orig_dimension` throughout, so ignoring those fields is safe. Loading all 1,047 usable profiles takes 0.21 s.
- **Quaternions and matching:** they are normalised, xyzw Hamilton; a zero quaternion is refused. Matching whole on brand, model, lens, orientation and aspect behaves as documented on the fixtures, and the `_FOCAL_MM`/`_WIDER` regexes misclassify no usable profile.
- **ATE for Stray runs:** Sim(3) for monocular (`runs.evaluate`), as the rung needs. The tests seed their generators.

## Run 2 -- 2026-10-11

| | |
|---|---|
| **Branch** | `feature/your-own-video` |
| **Range** | `cbf7b3d...1359afb`, run 1's fixes, read against the whole PR `63b61da...1359afb` |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus); the security pass (opus) beside them again, in [its own report](2026-10-11-feature-your-own-video-security.md), run 2 |
| **How they ran** | as in run 1: the registered slam-it-all-* agents, the security pass as a general-purpose agent with its own brief |
| **Layer 1** | all checks passed (`scripts/check.sh`, exit 0; pytest 3510 passed), on `1359afb` |

IDs go on from run 1's, and findings raised by more than one reviewer share a row at the highest
severity given. Each review opens with run 1's findings as it found them after the fixes: each
is resolved, or deferred as decided. Both must-fixes were reproduced before this report, with the
reviewers' own probes run again on `1359afb`:
- A13: after a SIGTERM to the worker, the job's grandchild was still alive; after a Ctrl-C to a
  demo-style parent, the job survived in 6 of 6 trials.
- L10: of 82 main-camera clips at 30 fps (Apple's phones in the database, at 4K and 1080p), 3
  were matched to a profile whose own focal length is an ultrawide's (14.0 to 14.4 mm), and all
  12 clips of the 2x crop (the 48 MP phones at both sizes) to a 1x profile.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A13 (S22) | must-fix | architecture, security | `orchestration/worker.py:468-474, 620-656`; `scripts/serve_demo.py:177-180` | since run 1's fix the job runs in a session of its own and is killed only when Python unwinds through `_run_child`; a SIGTERM to the worker skips that, and the demo's Ctrl-C mostly does too, so the job runs on unsupervised and its folder stays | fix (a SIGTERM unwinds the worker as a Ctrl-C does; the demo lets the worker stop on its own before terminating it; a test) | fixed (4ee7c67: a SIGTERM stops the worker as a Ctrl-C does, a second signal ignored while it unwinds; the demo gives each process 10 s, then kills it) -- decided by evan-wes, 2026-10-11 |
| L10 | must-fix | algorithm | `eval/lenses.py:462-487, 548-553, 628-645` | matching never compares a profile's own focal length with the clip's: mislabelled ultrawide calibrations win 30 fps main clips on the iPhone 12 Pro, 14 and 15 Pro, and a 2x-crop clip gets the 1x profile, each with fx about half the truth | fix (a candidate's own 35mm-equivalent kept within a band around the clip's tag; without a tag, outliers from the same phone's median dropped; tests on the database's own pairs and a 2x clip) | fixed (53ed1f4: the band around the tag; without one, the phone's median, used only where the median is itself a main camera's) -- decided by evan-wes, 2026-10-11 |
| A14 | should-fix | architecture | `server/.../jobs.py:69-85, 192-200` | the job list carries no schema version: a file from an earlier server opens, then fails every request and every sweep | fix (`PRAGMA user_version` set; another version refused at startup, naming the file; a test) | fixed (d66da0f) -- decided by evan-wes, 2026-10-11 |
| A15 (S21) | should-fix | architecture, security | `server/.../jobs.py:425-428`; `app.py:184-187, 603-630`; `storage.py:179-191` | an upload still arriving when its link expires lands after the link's only clean-up and stays 7 days; the 15-minute window also caps an upload's speed (about 4.4 Mbit/s for 500 MB, an estimate), which the page does not say | fix (the link cleared again an hour past its expiry; folder storage checking the expiry again before keeping a file; the window stated on the page and in the note; a test) | fixed (d66da0f: cleared when the job ends and again after an hour; folder storage keeps no late upload; the page states the window) -- decided by evan-wes, 2026-10-11 |
| H12 | should-fix | hygiene | `server/.../app.py:305-306` | the content policy silently loses the import map's hash if `index.html` is missing or its tag changes, and the viewer then breaks in the browser alone | fix (refused when the app is made; a test) | fixed (d66da0f) -- decided by evan-wes, 2026-10-11 |
| H13 | should-fix | hygiene | `server/.../app.py:471` | a failed move in `uploaded` answers a bare 500 with no line by job | fix (caught, logged by job id, answered 409 if the file has gone and 503 on a storage error; a test) | fixed (d66da0f) -- decided by evan-wes, 2026-10-11 |
| L11 | should-fix | algorithm | `tests/eval/recordings.py:81-101`; `eval/video.py:623` | the Stray fixture emulates its 30 fps setting in H.264; Stray's default is 60 fps in HEVC, so which truth rows go with the kept frames is untested (a mutant pairing them wrongly passes 32 tests) | fix (`encode_stray` with Stray's divider, 1 by default, in HEVC with B-frames; the kept frames' poses asserted; its docstring corrected) | fixed (53ed1f4) -- decided by evan-wes, 2026-10-11 |
| L12 | should-fix | algorithm | `eval/video.py:290-324, 494-497` | only the first frame's size is checked: frames of another size later in the stream are squeezed into it and given its camera, and the caps see the header alone | fix (a frame of another size refused; a test with two joined streams) | fixed (53ed1f4) -- decided by evan-wes, 2026-10-11 |
| H14 | should-fix | hygiene | `eval/lenses.py:438-439` | `lens_kind` calls `focal_kind` twice | fix (bound once) | fixed (53ed1f4) -- decided by evan-wes, 2026-10-11 |
| H15 | should-fix | hygiene | `eval/video.py:29-32, 365-373` | `_write_source` takes an anonymous `(source, profile)` tuple named like `lenses.Chosen`; the Caps paragraph is unclear and repeats derived numbers | fix (two parameters; the paragraph reworded) | fixed (53ed1f4: lenses.Chosen passed whole) -- decided by evan-wes, 2026-10-11 |
| H16 | should-fix | hygiene | `orchestration/worker.py:499, 517, 551` | `report` catches `(URLError, OSError)`, one a subclass of the other; the deadline's message is written twice | fix (`OSError` alone; the message from one place) | fixed (4ee7c67) -- decided by evan-wes, 2026-10-11 |
| A16 | note | architecture | `orchestration/worker.py:129, 304-314`; `server/.../config.py:37` | a stalled upload sends no heartbeat, and `UPLOAD_TIMEOUT` (300 s) runs past `SILENCE` (120 s) | fix (`UPLOAD_TIMEOUT <= SILENCE - HEARTBEAT`, held by `test_shared_names`) | fixed (4ee7c67) -- decided by evan-wes, 2026-10-11 |
| A17 | note | architecture | `server/.../app.py:311` | the content policy finds storage's origin by signing a link for a made-up key | fix (an `origin` on `Storage`, tested for both stores) | fixed (d66da0f) -- decided by evan-wes, 2026-10-11 |
| A18 | note | architecture | `pyproject.toml:344-347` | the contract keeping `eval.video` out of the pipeline names two modules, where its rule is "only the worker" | fix (`slam_it_all.orchestration` as its source, the worker's import ignored) | fixed (e944d8e) -- decided by evan-wes, 2026-10-11 |
| L13 | note | algorithm | `eval/lenses.py:433` | "Ultra" in a phone's model name reads all its profiles as ultrawide (119 usable profiles) | fix (only "ultra wide", "ultrawide" or "ultra-wide" read as ultrawide; checked against the database) | fixed (53ed1f4) -- decided by evan-wes, 2026-10-11 |
| L14 | note | algorithm | `eval/lenses.py` (`focal_kind`); `docs/notes/your-own-video.md` | the thresholds depend on whether Apple's video tag quotes the still photo's focal length or the video frame's | fix (added to the note's "What waits for the real clips"; L10's band holds under either) | fixed (53ed1f4: the module docstring; 8f53353: the note) -- decided by evan-wes, 2026-10-11 |
| H17 | note | hygiene | `orchestration/worker.py:474` | `killpg` runs after `_supervise` has reaped the child; a group id recycled in those milliseconds is theoretical | no action (the reviewer asks none) | no action: the reviewer asks none -- decided by evan-wes, 2026-10-11 |
| H18 | note | hygiene | `tests/test_serve_demo.py` | test stand-ins without docstrings | no action (test helpers) | no action: test helpers -- decided by evan-wes, 2026-10-11 |

### Hygiene review

## Hygiene review

**Scope:** Run 2 of PR 11, the fixes `cbf7b3d...1359afb`. I read the full source diff (`src/`, `server/src/`, `scripts/`, `viewer/src/`) and the tests that cover it. I compared each run 1 hygiene finding with what the code now does. I did not re-run Layer 1, which passed on HEAD's tree.

**Summary:** All eleven hygiene findings (H1-H11) are resolved in production quality. The fixes are tight: errors are logged and re-raised in the right places, the constants are named and pinned by tests, and the tests assert on values. I found no must-fix issues. The new should-fix items are small and mostly about unchecked startup assumptions and readability.

### Run 1's findings
- **H1:** resolved. `_run_child` starts the child in its own session and kills the process group in a `finally`. `handle` can no longer raise, because `_report_failure` logs a failed report instead of letting it escape. `test_a_job_leaves_no_process_behind` checks the grandchild really is gone after a timeout, a 409 and an orphan.
- **H2:** resolved. The worker logs with the job id and traceback, the child's stderr is no longer sent to DEVNULL, and `contextlib.suppress` on the sweep is replaced by logging. Tests check the log text.
- **H3:** resolved. The work is split into `_geometry`, `_kept`, `_picture` and `_Geometry`, and `probe` is called once.
- **H4:** resolved. S3 `size` treats only 404 and NoSuchKey as absent. `delete_prefix` raises `StorageError` on partial errors, and the folder `rmtree` no longer ignores errors. Tests cover it.
- **H5:** resolved. `ULTRAWIDE_MM`, `TELEPHOTO_MM`, `REQUEST_TIMEOUT`, `UPLOAD_TIMEOUT`, `CLAIM_MARGIN`, `CLAIM_WAIT` and `RETENTION_DAYS` are named. `test_shared_names` pins the cross-language ones, including SILENCE >= 3 x HEARTBEAT and WAIT <= CLAIM_WAIT.
- **H6:** resolved. Reads use `_connected` (WAL, no write lock), the claim runs through `asyncio.to_thread`, and the folder-storage write runs through `to_thread`. `create` raises `ValueError` for bad input. `test_a_status_read_does_not_wait_on_a_change` covers this.
- **H7:** resolved. `check_url` is applied to the server URL, the download link and the upload links, with a table-driven test. `main` returns 2 on an invalid server address.
- **H8:** resolved. `_centred` is shared by `from_35mm` and `nominal`, and `Jobs.working` is now `held`. The pages' `main()` functions are documented.
- **H9:** resolved. `UPLOAD_WAIT` is 15 minutes and the upload link has the same life. Links to close and the per-code cap are covered by tests.
- **H10:** resolved. `lenses.database()` is the single lookup, and the worker's `serve` and the video CLI both use it.
- **H11:** resolved. `watch()` ends the demo when the worker dies, and a test covers it.

### Findings

#### [should-fix] The content policy silently loses its import-map hashes if `index.html` is missing or changes
- **Where:** `server/src/slam_it_all_server/app.py:389-395` (`content_policy`)
- **Issue:** `text = index.read_text() if index.is_file() else ""` yields an empty hash list without a word. The regex `<script type="importmap">(.*?)</script>` also finds nothing if the tag gains an attribute or different spacing.
- **Why it matters:** The failure is silent and happens at runtime. The run viewer's inline import map is blocked by the CSP, and the page breaks in the browser with no server-side signal. This is the "environment-dependent behaviour that fails silently" class that run 1 raised for lens profiles.
- **Evidence:** The one test (`test_the_pages_carry_a_content_policy...`) uses the real `index.html` and so cannot see the degraded case.
- **Suggested fix:** Raise at `make_app` time when `index.html` is not a file. Optionally also raise when the page contains `importmap` but the pattern found no match.

#### [should-fix] `uploaded` has an unlogged 500 path around the new move
- **Where:** `server/src/slam_it_all_server/app.py:477` (`server.store.move(...)` before `server.jobs.queue`)
- **Issue:** A storage error or a vanished source file (for example two concurrent confirms) raises out of the handler. The handler does not catch `StorageError` or `FileNotFoundError`, and does not log anything with the job id.
- **Why it matters:** The owner gets a bare 500 and the operator gets no job-id line, unlike every other new storage call, which is wrapped by `_best_effort` or the sweep.
- **Suggested fix:** Catch the move failure, log it with the job id, and answer 409/503 with a clear message. Add one test for a concurrent confirm.

#### [should-fix] `lens_kind` calls `focal_kind` twice per loop pass
- **Where:** `src/slam_it_all/eval/lenses.py:1227-1228`
- **Issue:** The code reads `if focal_kind(focal) != "main": return focal_kind(focal)`.
- **Suggested fix:** Bind the result once: `if (kind := focal_kind(focal)) != "main": return kind`.

#### [should-fix] `_write_source` takes `(source, profile)` as an anonymous tuple, and the video module docstring is hard to read
- **Where:** `src/slam_it_all/eval/video.py` (`_write_source(out, kind, chosen: tuple[str, str | None], frames, metadata)`; module docstring "Caps")
- **Issue:** The `chosen` tuple was added only to keep the argument count down, and its name collides with `lenses.Chosen`, which is a different type with a different shape. In the Caps paragraph, "narrower than MAX_ASPECT to one" is unclear, and "at most 1440 by 480" repeats numbers that derive from `MAX_ASPECT * SHORT_SIDE`.
- **Suggested fix:** Pass `source` and `profile` as two parameters, or pass the `lenses.Chosen`. Say "more than MAX_ASPECT to 1" and refer to the derived size instead of quoting it.

#### [should-fix] The worker's `report` catches `(URLError, OSError)`, and the timeout message is written twice
- **Where:** `src/slam_it_all/orchestration/worker.py:2346` (`handle.report`), and the two `took longer than {timeout / 60:g} minutes` messages in `_supervise`.
- **Issue:** `URLError` is a subclass of `OSError`, so the pair is redundant. The two message strings should come from one place.
- **Suggested fix:** Catch `OSError` only, and use a small helper for the deadline error.

#### [note] `os.killpg(child.pid, ...)` runs after the child has been reaped
- **Where:** `worker.py` `_run_child`, the `finally` block.
- **Issue:** The `with Popen` block does not exit until after the `finally`, so the kill happens before the final reap and the pid is not yet recycled in the normal path. `child.wait()` inside `_supervise` has already reaped it by then, though. A recycled process-group id is theoretically possible, but extremely unlikely in a few milliseconds. I do not ask for a change.

#### [note] `_Process.poll` and the `_Reading` stand-ins in tests have no docstrings
- **Where:** `tests/test_serve_demo.py` (`_Process.poll`)
- **Issue:** This is a test helper and is fine for tests. I mention it only for consistency with the repository's docstring habit.

### Checked and fine
- Sweep design in `app.py` and `jobs.py`: the files go before the job, the `link_open` and `files_open` flags make a failed deletion retry on the next sweep, and the failures are counted and logged.
- `Jobs`: `held`/`by_id` reads without a lock, `create` validates its arguments, and the per-owner cap is checked together with the queue cap in one transaction. The owner is only stored as a keyed hash.
- The `RESULT_PATH` whole-match regex replaces `check_key` for result paths. `_KEY.fullmatch` and the `.`/`..` check close the trailing-newline and dot-segment holes.
- Worker supervision: the environment is allow-listed and the token is not inherited. `RLIMIT_FSIZE` is added. `_Beat` and `_Reading` keep the worker single-threaded, as `preexec_fn` requires.
- `video.py`: `_open` restricts the demuxer and protocols, and the edit-list handling for Stray Scanner is documented. `_ZIP_ERRORS` and `_csv_rows` turn corrupt inputs into `InvalidInputError`. `MAX_LONG_SIDE`, `MAX_ASPECT` and `MAX_DECODED` are all tested through the parametrised refusal test.
- `tum.write_list` and `write_camera` have numpy docstrings and a round-trip test.
- `lenses.database` and `video_lens` have precise docstrings and tests for the disagreement case, with a fixture-based telephoto test. `Fisheye` turning-point fix (`falling[0] > 0`) is covered by a test.
- Viewer: `bundleAddress` has JSDoc and a Node test. `RETENTION_DAYS` is pinned against `config.RETENTION`.
- Docs: CLAUDE.md and the module docstrings match the new behaviour (the second package root, CSP, the sweep).
- Tests assert on values, not just that nothing crashes.

### Architecture review

## Architecture review

**Scope:** Run 2 of PR 11, `feature/your-own-video`. I reviewed the fix commits `cbf7b3d...1359afb` (8 commits, 51 files) against run 1's findings and the design.
- **Read in full:** `orchestration/worker.py`; the server's `app.py`, `jobs.py`, `storage.py`, `config.py` and `__main__.py`; the `eval.video`, `eval.lenses` and `eval.tum` diffs; the contracts in `pyproject.toml`; both Dockerfiles and `docker/server-entrypoint.sh`; `scripts/serve_demo.py`; the viewer diffs; the docs diffs (`CLAUDE.md`, the ladder, run-viewer design, both notes, CHANGELOG).
- **Tests:** names, plus the bodies of the new worker, shared-names and demo tests.
- **Probes:** run in the env under the memory cap. `lint-imports` gives 21 kept, 0 broken. No server was started.
  - My run-1 probes are updated for the new signatures, with two signal cases added. They are saved at `pr11_worker_server_probes.py`, which replaces the cbf7b3d-only version.
  - Scratch files are in `/tmp/scratch/review2-architecture/`.
  - No probe process was left running.

**Summary:** Every architecture finding of run 1 is resolved or recorded as deferred. My run-1 probes now show the fixed behaviour. The sweep's redesign (files before rows, two "still to clear" flags, every early deletion best-effort and backed by a later sweep) is sound, and the docs match what was built. One gap in A1's fix is new:
- The job's processes are killed only when Python unwinds through `_run_child`.
- A SIGTERM to the worker skips that, and since 68aaf3c the job runs in a session of its own that no other signal reaches.
- So stopping the demo usually leaves the pipeline running.

Also new: the job list has no schema version, which f616342's schema change makes visible. Four notes follow.

### Run 1's findings
- **A1:** resolved for the three paths it named: the timeout, a 409 on a heartbeat, and a failure report that is refused or never arrives. Probes 1-3 now print "none" and "handle returned". A path it did not name is still open; see the new must-fix below.
- **A2:** resolved. Probe 4: after a failing sweep the row is kept (3 failures logged); the next sweep deletes the row and the result file.
- **A3:** resolved (1504bd5). `CLAUDE.md`'s Languages and Layout sections, the ladder's package table and contract list, run-viewer section 3 and `jobs.js:7` now match what was built.
- **A4:** resolved (cd1a668). All three contracts are declared and kept. The new one's reach is in note 4.
- **A5:** resolved. Downloads and uploads beat; `SILENCE >= 3 * HEARTBEAT` and `WAIT <= CLAIM_WAIT` are tested. A stalled upload still sends no beat (note 2).
- **A6:** resolved.
  - `lenses.database()` is the one lookup and raises when the database is missing or empty.
  - `serve` refuses to start without it (exit 2).
  - The CLI requires it only when a profile may be chosen.
  - The child finds the same folder, since `HOME` and `SLAM_IT_ALL_LENS_PROFILES` are in `CHILD_ENVIRONMENT`.
- **A7:** resolved. The worker logs the traceback with the job's id, the child's stderr is the worker's, and the sweeper logs. Logging is configured only in the two entry points, not at import.
- **A8:** deferred; recorded in the ladder's open items (PR 11's).
- **A9:** deferred; recorded in the ladder's open items (PR 11's).
- **A10:** deferred; recorded against PR 7's H18/A13.
- **A11:** resolved. `tum.write_list` and `tum.write_camera` sit beside the readers, and both `eval.video` and `tests/orchestration/datasets.py` use them.
- **A12:** resolved. `pydantic` and `botocore` are declared in `server/pyproject.toml`, and the ladder names them.

### Findings

#### [must-fix] A SIGTERM to the worker leaves its job running, and the demo's Ctrl-C usually does too
- **Where:**
  - `src/slam_it_all/orchestration/worker.py:468` and `:474`: the child starts its own session, and its group is killed only in `_run_child`'s `finally`.
  - `worker.py:620-656`: `main` installs no SIGTERM handler.
  - `scripts/serve_demo.py:177-180`: the `finally` calls `terminate()` on the worker first.
  - The claims: `worker.py:28-31` ("a job never outlives its turn"), `serve_demo.py:11` and `docs/notes/your-own-video.md:139` ("Ctrl-C, or a SIGTERM, stops the server and the worker together").
- **Issue:**
  1. The group kill only runs if Python unwinds through `_run_child`. A SIGTERM keeps Python's default action, which ends the process without running any `finally`.
  2. Since 68aaf3c the job is in a session of its own, so no signal sent to the worker or to its terminal reaches the job any more.
  3. Before that, a Ctrl-C reached the child and the C++ program directly, because they shared the terminal's process group.
  4. On a Ctrl-C, the demo and its worker both get the SIGINT. The demo's `finally` then sends the worker a SIGTERM. If that lands before the worker has reached its own `finally`, the worker dies and the job runs on.
- **Why it matters:** The guarantee fails on the worker's ordinary stop paths outside a container: `serve_demo.py`'s SIGTERM, its Ctrl-C in most trials, systemd, or `kill`.
  - The orphan runs the pipeline to its end, up to 6 GiB.
  - A demo restarted straight away claims the next job beside it.
  - `handle`'s `rmtree` never runs, so job folders pile up under `--data/work`.
  - The server only finds out after `SILENCE`.
  - Railway's image is safe: when tini exits, the kernel kills the rest of the container's PID namespace.
  - The demo is the path the in-person showing depends on, and its docs promise the opposite.
- **Evidence:** the saved probe, run on 1359afb. Probe 5 sends a SIGTERM to a worker mid-job. Probe 6 stands in for `serve_demo.py`: a Ctrl-C to its process group, then `terminate()` of the worker in its `finally`.
  ```
  5. after a SIGTERM to the worker, the job's grandchild alive: True
  6. after Ctrl-C to a demo-style parent, the job survived in 6 of 8
  ```
  An earlier ad-hoc run gave 4 of 6. After the SIGTERM, `ps` showed the job's child re-parented to 2731, still running with its `sleep` grandchild in its group.
- **Suggested fix:**
  - In `worker.main`, for `serve`, turn a SIGTERM into the existing `KeyboardInterrupt` path (as `serve_demo._stop` does), so that `_run_child`'s `finally` kills the group and `handle`'s `finally` removes the folder.
  - In `serve_demo.py`, after a Ctrl-C, give the worker a few seconds to finish its own interrupt before calling `terminate()`.
  - Add a fourth case to `test_a_job_leaves_no_process_behind`: SIGTERM a worker subprocess mid-job, and assert the grandchild is gone.

#### [should-fix] The job list has no schema version: a file made by an earlier server opens, then fails every request and every sweep
- **Where:** `server/src/slam_it_all_server/jobs.py:69-85` (`CREATE TABLE IF NOT EXISTS`) and `:193-200` (`Jobs.__init__`).
- **Issue:** f616342 adds the `owner`, `link_open` and `files_open` columns. On an existing file, `CREATE TABLE IF NOT EXISTS` does nothing and nothing compares the schema, so an old file is accepted at startup and then breaks at its first use.
- **Why it matters:** The job list is the one piece of state kept across deploys: a Railway volume (staging comes straight after the merge) and the demo's `--data` folder.
  - The next column added once staging is live would show up as 500s on making and reading jobs.
  - The sweep would log an error every 15 s and delete nothing, so the 7-day promise would stop holding.
  - Every other format the project keeps carries a version: the run format, and `bundle.json`'s `format` and `version`.
  - Clean breaks are the policy, but a clean break should be refused at startup with a message, not found one request at a time.
- **Evidence:** a file made with cbf7b3d's `_SCHEMA`, opened by 1359afb's `Jobs` (`schema_probe.py` in scratch):
  ```
  opened: no error at startup
  first job: OperationalError no such column: owner
  sweep: OperationalError no such column: files_open
  ```
- **Suggested fix:** Set `PRAGMA user_version` when the table is made. At open, refuse any other version with an error at startup that names the file and says to delete it (no migration, per policy). Add a test with a file of another version.

#### [note] The 15-minute upload window now also caps how long an upload may take, and the page does not say so
- **Where:**
  - `server/src/slam_it_all_server/config.py:33` (`UPLOAD_WAIT`) and `:24` (`MAX_BYTES`)
  - `jobs.py:425-428` (`links_to_close`)
  - `app.py:603` and `storage.py:188`: folder storage checks a link's expiry before it streams the body.
- **Issue:**
  - A waiting job expires at 15 minutes even while its PUT is still running. A full 500 MB upload therefore needs about 4.4 Mbit/s sustained (500e6 × 8 / 900 s). A slower one completes, then `uploaded` gets a 409.
  - The late file lands after `links_to_close` and `files_to_clear` have already run for the expired job, so it stays until `_delete`, 7 days later. That is within retention, but the file is never used.
  - S3 is expected to behave the same way (it authenticates a request when it arrives); I did not test that here.
- **Suggested fix:** The window was the owner's S6 decision, so this is not a defect. Consider stating the window or a minimum upload speed on the upload page and in the note, or having the page warn from its measured rate. Optionally, delete `uploads/<id>` again for an expired job once a longest-allowed PUT could have finished.

#### [note] A stalled upload sends no heartbeat, and `UPLOAD_TIMEOUT` reaches well past `SILENCE`
- **Where:** `worker.py:129` (`UPLOAD_TIMEOUT = 300`), `:304-314` (`_Reading` beats only when a block is read) and `config.py:37` (`SILENCE = 120`).
- **Issue:** The heartbeat is driven by data flow, so a socket that blocks sends none. If an upload stalls for more than about 90 s, the server fails the job while the worker keeps waiting for up to 300 s, and the worker learns of it only at its next 409. Download reads, with `REQUEST_TIMEOUT = 60`, stay inside the window.
- **Suggested fix:** Make `UPLOAD_TIMEOUT <= SILENCE - HEARTBEAT`, and hold that relation in `tests/server/test_shared_names.py` beside the heartbeat test.

#### [note] The content policy finds storage's origin by signing a dummy link
- **Where:** `server/src/slam_it_all_server/app.py:311` (`store.download_link("content-policy/probe")`).
- **Issue:** The `Storage` protocol has no way to say where its links point, so `content_policy` signs a link for a made-up key just to read its scheme and host. It works for both stores, but it goes around the interface rather than through it.
- **Suggested fix:** Add an `origin` property to `Storage`, implemented and tested by both stores.

#### [note] The new contract names two modules, but its rule is "only the worker"
- **Where:** `pyproject.toml:344-347`.
- **Issue:** `source_modules` lists `orchestration.vo` and `.bundles`. A future pipeline module placed beside `vo` would get a layer in the exhaustive order contract, but not this protection.
- **Suggested fix:** Use `source_modules = ["slam_it_all.orchestration"]` with `ignore_imports = ["slam_it_all.orchestration.worker -> slam_it_all.eval.video"]`. That states the rule for every module to come.

### Checked and fine
- **Dependency direction:**
  - 21 contracts kept.
  - The server's forbidden list now covers `av`, `PIL`, `cv2` and `reifyscene`.
  - `eval.video` uses `tum`'s writers, which respects eval's internal order.
  - The worker stays at the top of orchestration.
  - `scripts/serve_demo.py` importing the worker for its constants is a script above everything.
  - No C++ or Rust changed, and nothing in `foundations`, `edge` or `backends` gains a dependency.
- **The sweep:**
  - It runs in this order: end the overdue jobs, close expired upload links, clear ended jobs' files, then delete each job after its files.
  - Every early deletion (in `progress`, `done`, `failed`) is best-effort, and a later flag-driven step covers it.
  - I traced the races: `uploaded` at the moment of expiry, two `uploaded` calls at once, and a replayed link after the move. Each ends with its files cleared.
- **Storage keys:**
  - `uploads/` is only ever the owner's link's target; `imports/` is the worker's copy, out of the link's reach; `results/` holds bundles.
  - All three are deleted by `_delete`, and the bucket lifecycle rule in the checklist covers all three.
  - `Storage` gains `move` and explicit link lifetimes, and its errors are raised rather than read as "absent" or "done".
- **The server/worker split:** the process holding the keys still parses nothing. `RESULT_PATH` mirrors the bake's layout, and the layer-1 end-to-end test (the real worker and bake against the real server) holds the two equal.
- **The worker's supervision otherwise:** a group kill on every exit Python unwinds through, including an orphan holding the pipe open. It runs with a minimal environment and `RLIMIT_FSIZE`, and `check_url` is applied to the server, the claim's link and each upload link.
- **Server internals:** reads are lock-free in WAL mode; the claim runs off the event loop; folder storage writes through a thread; the per-code cap is a `FullError` subtype mapped to 429; the code is kept only as a keyed hash.
- **Images:** tini as the worker's PID 1. The server's entrypoint chowns only the job list's folder, then drops to uid 10001 with `setpriv`.
- **Docs against code:**
  - `CLAUDE.md` (the server, its contracts, and layer 1's mypy and ruff scope match `check.sh` and `[tool.mypy] files`).
  - The ladder's package table, contract list and open items (A8, A9, A10, S2, S11, S15, S20, L6-L8).
  - Run-viewer section 3, the closing decisions of section 9, the viewer note and the your-own-video note.
  - CHANGELOG. VERSION stays 0.3.6.0 within the PR.
  - The one place they disagree with the code is the SIGTERM claim in the must-fix above.
- **Viewer:** `bundleAddress` keeps bundles on the page's own site as a DOM-free logic module, with `URL` declared in `logic.d.ts`. `RETENTION_DAYS` is held to `config.RETENTION` by test.

### Algorithm review

### Run 1's findings
- **L1** resolved. The import now uses Stray's quaternions as written. Against an independent scipy emulation of `q_WA * q_AC`, the worst error over 200 random poses is 2.6e-14 deg. Through the whole import, a perfect estimate scores ATE 4.1e-16 m and RPE 6.2e-17 m / 9.5e-16 deg.
- **L2** resolved. `rising()` now returns 0.0 for a bend that falls from the start (k1 = -1e9). `holds()` still drops 84 real profiles. The turning point, `unbend`'s refusals, `undistortion`'s refusals and a real dropped profile are all tested.
- **L3** resolved. The import gets every frame back, in order, with its timing. This also holds for HEVC with B-frames (the codec Stray actually writes) and for Stray's default 60 fps and its 5 fps setting. The fixture covers only the 30 fps setting in H.264 (should-fix below).
- **L4** resolved as suggested. A "back triple camera" clip tagged 77 now gets the 3x profile. The same rule still trusts the database's names, though (must-fix below).
- **L5** resolved. The OpenCV parity test uses `rtol=0`, an explicit `atol` and a stated reason. The ray test covers rotations 90, -90 and 180 and has 2.5x margin (see Checked and fine).
- **L6 to L9** stay deferred or need no action. Not re-raised.

## Algorithm review

**Scope:** the fixes in `cbf7b3d...1359afb`, mainly d8556cc (`src/slam_it_all/eval/lenses.py`, `tests/eval/test_lenses.py`) and 0a26105 (`src/slam_it_all/eval/video.py`, `tum.py`, `tests/eval/recordings.py`, `tests/eval/test_video.py`). I checked them against:
- Stray Scanner's current source: `DatasetEncoder`, `VideoEncoder`, `OdometryEncoder` and `RecordSessionViewController.swift`;
- the Gyroflow database at `886fc61` (`~/.cache/slam-it-all/lens_profiles/`);
- Apple's published lens specs, and one published iPhone video's metadata.

`tests/eval/test_lenses.py`, `test_video.py` and `test_tum.py` pass (124 tests). The review was read-only and the tree is still clean at 1359afb. The probes are in `/tmp/scratch/review2-algorithm/`.

**Summary:** L1 to L5 are fixed correctly. I re-ran each probe and widened the Stray ones to the codec and frame rate Stray really uses. One must-fix is new, and I should have caught it in run 1: profile matching never compares a profile's own focal length with the clip's. On real database entries, a 30 fps main-camera clip from three common iPhones is matched to an ultrawide calibration, and a 2x clip gets the 1x profile. Both put fx at roughly half the truth, labelled "profile". There are two should-fixes:
- the Stray fixture does not exercise Stray's default 60 fps, so how ground truth is picked when frames are dropped is untested;
- only the first frame's size is checked, so the new caps apply to the header alone.

### Findings

#### [must-fix] Matching never compares a profile's own focal length with the clip's, so mislabelled and 2x-cropped clips get about half the true fx
- **Where:**
  - `src/slam_it_all/eval/lenses.py:628-645` (`match`): it filters candidates by the kind their name says, then ranks by resolution, frame rate and RMS.
  - `:548-553`: a profile's kind comes from its free-text name alone.
  - `:462-487` (`video_lens`): it uses the 35mm tag to decide the kind, never its value.
- **Issue:**
  - *Mislabelled profiles.* Some ultrawide calibrations in the database have names that say "main", or "24mm", or no lens at all. Their own focal length (from fx over the frame's diagonal) is 14.0 to 14.4 mm. The same phones' real main profiles are 24.6 to 31.4 mm. When one of them sits at exactly 30 fps, the "nearest frame rate" rule picks it for a 30 fps main clip.
  - *The 2x crop.* Every iPhone with a 48 MP main sensor offers a one-tap 2x that is the middle 12 MP of the main sensor, "for … 4K videos" too. Apple specifies it as 48 mm f/1.78. `focal_kind(48)` says "main", so the clip gets the 1x main profile. For a centre crop at the same output size the true fx is twice that profile's.
- **Why it matters:**
  - **Wrong intrinsics, labelled as the best source:** fx comes out at about half the truth, and the status page names the matched profile.
  - **Common settings:** it hits the default 30 fps modes on the iPhone 12 Pro (4K), iPhone 14 (4K) and iPhone 15 Pro (1080p). The 2x case hits every 14 Pro and later, including the iPhone 14 Pro test phone.
  - **Not a deferred question:** the mislabelled entries match whatever Apple writes in the tag, because the clip only needs to read as "main". So this is not part of L8, which waits on the real clips.
- **Evidence:**
  - `mislabelled.py`, `apple_mismatch2.py`, at the pinned commit:
    ```
    iPhone 12 Pro 4K30 -> 'iPhone 12 Pro___4k_16by9_3840x2160-30.00fps' (14.0 mm); its other main profiles 26.9-31.4 mm
    iPhone 14 4K30     -> 'iPhone 14___4k_16by9_3840x2160-30.00fps' (14.4 mm); others 26.1-32.6 mm
    iPhone 15 Pro 1080p30 -> 'iPhone 15 Pro_24mm_GyroCam iOS_1080p...' (14.4 mm), named "24mm"
    2x (tag 48), iPhone 14 Pro 4K: kind main -> 'iPhone 14 Pro_1x__4k...' fx 2994.5 (true about 2x that)
    ```
  - Apple, the 14 Pro's 2x: [newsroom](https://www.apple.com/newsroom/2022/09/apple-debuts-iphone-14-pro-and-iphone-14-pro-max/) ("middle 12 megapixels … for full-resolution photos and 4K videos"); [tech specs](https://support.apple.com/en-ie/111849) (48 mm f/1.78).
  - `band_filter.py` simulates the fix outside the repo: drop candidates whose own focal is outside 0.8 to 1.6 times the tag. The three clips then get proper main profiles (30.7, 31.4 and 30.4 mm). The 14 Pro's 2x falls to the 35mm source. Only 7 of 116 Apple matches change at all.
  - Correct lens and tag pairs sit at 1.1 to 1.4 times the tag (`apple_uw_tele.py`: ultrawides 1.10 to 1.36 at tag 13).
- **Suggested fix:**
  - In `match`, compute each candidate's 35mm-equivalent from its own fx over the diagonal. When the clip has a tag, drop candidates outside a band around it before ranking, e.g. 0.8 to 1.6, wide enough for L8's still-photo versus video-frame gap.
  - For clips without a tag, drop profiles far from the median focal length of the same phone and kind.
  - Test it with the iPhone 12 Pro pair from the CC0 database (the 30.00 fps profile at 14.0 mm and a proper main profile) and with a 2x clip tagged 48.

#### [should-fix] The Stray fixture emulates its 30 fps setting in H.264; Stray's default is 60 fps in HEVC, and the ground-truth selection when frames are dropped is untested
- **Where:**
  - `tests/eval/recordings.py:81-101` (`encode_stray`): stamps fixed at `2 * i - 1`, libx264 with `bf=0`. Its docstring says the encoder "keeps every other ARKit frame".
  - `src/slam_it_all/eval/video.py:623`: `poses = _stray_poses(table)[frames.decoded]`.
- **Issue:**
  - Stray's default frame rate is 60 fps. `RecordSessionViewController.swift` has `FpsDividers = [1, 2, 4, 12, 60]`, and the setting is read from `UserDefaults`, which gives 0 when unset, so the divider is 1. Stamps are then -1, 0, 1, …, and the import keeps every other frame.
  - `VideoEncoder.swift` encodes `AVVideoCodecType.hevc`, and its encoder may reorder frames.
  - The fixture keeps every frame, so `frames.decoded` is always `0..N-1`. Nothing tests which ground-truth rows go with the kept frames.
- **Why it matters:** this one line pairs every kept frame with its truth for the only source with exact ground truth. A regression here would silently pair frame 2k's image with frame k's pose on every default recording.
- **Evidence:**
  - `stray_60fps.py` (HEVC, `bframes=4`, edit list): works today. At 60 fps: `decoded 24, kept 12; shown == truth's frame: True`. At the 5 fps setting: also True.
  - Same script with `decoded` replaced by `range(len(times))`: `shown == truth's frame: False ([0, 2, 4, 6, 8, 10] vs [0, 1, 2, 3, 4, 5])`.
  - With that replacement, the Stray and worker tests still pass: `pytest -p mutate_decoded tests/eval/test_video.py tests/orchestration/test_worker.py -k "stray or Stray or worker"` gives `32 passed`.
  - `stray_bframes.py`: with frame reordering, the ignored edit list starts presentation times at 0.0667 s rather than 0. The import is correct only because it subtracts the first time, and no test covers that.
- **Suggested fix:**
  - Give `encode_stray` a divider, defaulting to 1 as Stray does, with stamps `divider * i - 1`, and keep a 30 fps case.
  - Encode with libx265 and B-frames; the lock's PyAV has the encoder.
  - Assert `{"decoded": 2N, "kept": N}` and that the kept frames' poses are the room's even frames.
  - Correct the docstring.

#### [should-fix] Only the first frame's size is checked; later frames of any size are squeezed into its geometry, and the new caps read only the header
- **Where:**
  - `src/slam_it_all/eval/video.py:290-324`: `_geometry` runs on the first frame only, and `_picture` resizes every frame to `geometry.size` without looking at its size.
  - `:494-497`: `probe`'s caps read the stream header.
  - The module docstring at `:29-32` says "a frame longer than MAX_LONG_SIDE … is refused".
- **Issue:** a stream whose resolution changes mid-stream is accepted. The frames after the change are resized into the first frame's size and get its camera. The new caps never see them.
- **Why it matters:**
  - **Wrong geometry, silently:** those frames' intrinsics are wrong. In the probe below they are squeezed 4x horizontally.
  - **The caps' promise does not hold:** S9's bound covers the header only. The decoder's allocation for such frames is for the security pass to judge.
- **Evidence:**
  - `size_change.py`: `decoded sizes: [(320, 240) x3, (1280, 240) x3]`, `probe: 320 240`, `import accepted: {'decoded': 6, 'kept': 6}`.
  - `size_change_long.py`: frames of `(8192, 64)` are accepted too, over `MAX_LONG_SIDE` and 128 to 1.
- **Suggested fix:** in `_write_frames` or `_picture`, refuse a frame whose `(width, height)` differs from the first frame's, with an `InvalidInputError`. Test it with the concatenated H.264 streams from the probe.

#### [note] "Ultra" in a phone's model name makes all its profiles read as ultrawide
- **Where:** `src/slam_it_all/eval/lenses.py:433` (`"ultra" in words`), applied to text that includes the profile's file name (`:548-553`).
- **Issue:** 119 usable profiles from Galaxy S2x Ultra and Xiaomi Ultra phones read as ultrawide. With the model's "Ultra" removed, they read as main 69, telephoto 20 and ultrawide 30. Examples: `S25Ultra_117mm` (113.6 mm) and `Samsung_galaxy s23 ultra_230mm` (428.6 mm). This corrects my run-1 "checked and fine" line, which covered only the regexes.
- **Why it matters:**
  - **No wrong intrinsics today:** Android clips always read as main, and Android model names wait on the real clips.
  - **Still a loss:** these profiles can never be matched, and the main-camera ones are missing from the nominal field-of-view median (`ultra_model.py`).
- **Suggested fix:** match "ultra wide", "ultrawide" or "ultra-wide" rather than "ultra", or remove the brand and model from the text before `lens_kind`.

#### [note] focal_kind's thresholds depend on which convention Apple's video tag uses
- **Issue:**
  - `TELEPHOTO_MM = 50` is only 4% above the 2x crop's 48 mm.
  - One published iPhone 15 Pro clip has lens `"iPhone 15 Pro back camera 6.765mm f/1.78"` and tag `23` ([remotion media-parser docs](https://www.remotion.dev/docs/media-parser/tags)). That suggests the tag quotes the sensor's still-photo figure, where 13, 24, 48, 77 and 120 all land on the right side of the thresholds.
  - If the tag instead measured the video frame's own diagonal, an ultrawide named "triple camera" would read as main. The database measures Apple's 16:9 ultrawides at 16.4 to 17.7 mm (n = 24).
- **Suggested fix:**
  - Add "the tag's value for each lens and for the 2x crop" to the note's "What waits for the real clips".
  - The band check suggested above works under either convention: about 1.0 to 1.4 for correct pairs, about 0.6 for a 2x crop or a mislabelled profile.

### Checked and fine
- **L1 conversion:** `_stray_poses` matches `q_WA * q_AC`, and the fixture's `stray_quaternion` matches an independent scipy emulation to 3.6e-16. The Stray truth test checks poses at `atol=1e-9`, so the old double turn fails it.
- **L3 decoding:** `ignore_editlist` returns all N samples, in presentation order, for H.264 and HEVC, with and without B-frames. With edits honoured, one frame is dropped, as run 1 found.
- **The fisheye model's failure paths:** `rising()`'s grid result lies within one step below the analytic turn. `holds()` and `unbend` are conservative and refuse beyond the turn.
- **OpenCV parity:** `distortPoints` agrees within `atol=1e-9`, `rtol=0`. `initUndistortRectifyMap` agrees within `atol=1e-4`, about 3 float32 steps at 320 px, inside a 1 px border, with the reason stated.
- **Ray test tolerance** (`ray_test_margin.py`): the honest error is 0.0396 px at 0, ±90 and 180. It fails the x·s scaling convention (0.40 px) and a principal point 0.25 px off (0.26 px).
- **`video_lens` on Apple's names:**
  - "triple camera" or "back camera" defers to the tag;
  - a name and tag that disagree trust no profile, so the import falls to the 35mm source, visibly in `source.json`;
  - "front" is never overridden;
  - a non-numeric, infinite or negative tag is ignored.
- **Bounds:**
  - with `MAX_ASPECT = 3`, `output_size` writes at most 1440 by 480;
  - `MAX_DECODED` = 60 s × 240 fps = 14,400, and the boundary is right;
  - 7680 admits a phone's 8K.
- **Containers:** the mov demuxer handles every phone container. Edit lists are honoured for phone videos, as players do, and ignored for Stray alone.
- **TUM writers:** `tum.write_camera` writes floats with `repr`, so they read back exactly.
- **Profile database:** `database()` refuses a missing or empty folder, and the worker requires it.
- **A broken Pixel 6 profile** (1771 mm equivalent, fx/fy 1.44) passes the RMS and `holds()` filters but loses on calibration error at every Pixel 6 size, so it is harmless today.
