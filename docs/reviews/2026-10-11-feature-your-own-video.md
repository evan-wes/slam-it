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
