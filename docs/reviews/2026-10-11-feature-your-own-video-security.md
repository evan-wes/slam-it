# Security pass: `feature/your-own-video`

<!-- The security pass on PR 11's upload path (docs/design/2026-10-09-run-viewer.md, section 7,
decision 5), saved beside the panel's report. Later runs append a "Run N" section. -->

## Run 1 -- 2026-10-11

| | |
|---|---|
| **Branch** | `feature/your-own-video` |
| **Range** | `63b61da...cbf7b3d` (base: `origin/develop`) |
| **Reviewer** | security (opus), a general-purpose agent with its own brief: the server, the worker, hostile uploads, the pages, the images and the demo, against attackers without a code, with one, with a result link, and with the worker token |
| **Probes** | the server on 127.0.0.1 with folder storage and staging on, probed with curl; an S3 link signed offline; two process probes on the room's Stray Scanner zip; no images built, no external service contacted |
| **Panel** | [the panel's report](2026-10-11-feature-your-own-video.md), whose IDs (H*, A*, L*) are named where a finding overlaps one of its own |

### Findings and resolutions

| ID | Severity | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|
| S1 (H1 / A1) | must-fix | `orchestration/worker.py:317-354, 390-398`; `vo.py:680` | the timeout and a failed heartbeat leave the job's processes running, the C++ program always; the worker is PID 1 in its image, so orphans are never reaped | fix (the panel's row, plus an init in the worker image) | fixed (68aaf3c, tini in the worker's image included) -- decided by evan-wes, 2026-10-11 |
| S2 | should-fix | `server/.../__main__.py`; every route with a token | uvicorn's access log records result tokens and folder storage's signed links, and client addresses | fix (the access log off, the server's own lines keyed by job id; the page says what the host logs); defer the token out of the path (`/r#<token>`) to before going public | fixed (a5b122c: the access log off, the server's own lines by id, the page says what the host logs); deferred (the ladder's open items, PR 11's, before going public): the token out of the path -- decided by evan-wes, 2026-10-11 |
| S3 | should-fix | `eval/video.py:190, 385` | uploads are opened with every FFmpeg input format, reference formats (`hls`, `concat`, `dash`) among them | fix (an allow-list of the mov/mp4 family, anything else refused, tested) | fixed (0a26105) -- decided by evan-wes, 2026-10-11 |
| S4 | should-fix | `worker.py:317-323`; `scripts/serve_demo.py:150-153` | the decoding child inherits the worker token, and on the demo the owner's whole environment | fix (a minimal environment for the child and the demo's worker) | fixed (68aaf3c; the demo's worker in a5b122c) -- decided by evan-wes, 2026-10-11 |
| S5 | should-fix | `app.py:325-343, 403-414`; `storage.py:155-161` | the owner's upload link can be replayed for its hour: after the upload is confirmed, and after the import deleted it | fix (on confirmation, the upload moved to a key the owner holds no link for) | fixed (f616342) -- decided by evan-wes, 2026-10-11 |
| S6 (H9) | should-fix | `jobs.py:212-232`; `app.py:314-329` | one code holder can fill the queue for an hour with five tiny requests, and a job does not record its code | fix (scope to decide: waiting jobs expire with their link, open jobs per code capped, the code's hash stored) | fixed (f616342: a waiting job expires with its 15-minute link; at most 2 jobs open per code, its keyed hash kept) -- decided by evan-wes, 2026-10-11 |
| S7 (A2) | should-fix | `app.py:121-134, 275-279` | a failed deletion is lost, and the files outlive the 7 days | fix (the panel's A2, plus a bucket lifecycle rule in the staging checklist if Railway supports one) | fixed (f616342; the lifecycle rule in the staging checklist, 1504bd5) -- decided by evan-wes, 2026-10-11 |
| S8 (H6) | should-fix | `app.py:391-401, 461-483`; `jobs.py:175-189` | synchronous SQLite work blocks the event loop, and every read takes the write lock | fix (the panel's H6) | fixed (f616342) -- decided by evan-wes, 2026-10-11 |
| S9 | should-fix | `eval/video.py:120-257`; `worker.py:228-230` | the import bounds memory and time, but not the frame's long side, the frames decoded, or the disk written | fix (the long side and aspect refused past a limit, decoded frames capped, `RLIMIT_FSIZE`) | fixed (0a26105; the file-size limit in 68aaf3c) -- decided by evan-wes, 2026-10-11 |
| S10 | should-fix | `docs/notes/your-own-video.md:127`; `docker/server.Dockerfile` | `RAILWAY_RUN_UID=0` runs the internet-facing server as root on Railway | fix (an entrypoint that gives `/data` to the server's user, then drops to it) | fixed (a5b122c) -- decided by evan-wes, 2026-10-11 |
| S11 | should-fix | `app.py:83-88`; `viewer/*.html`; `viewer/src/main.js:206` | no Content Security Policy; three.js from a CDN without `integrity`; Google Fonts; `?bundle=` takes any URL; no HSTS | fix now: a CSP, `integrity` on three.js, `?bundle=` same-origin only; defer self-hosted fonts and HSTS to going public | fixed (a5b122c: the policy, three.js pinned, the bundle on its own site); deferred (the ladder's open items, PR 11's, before going public): fonts self-hosted, HSTS -- decided by evan-wes, 2026-10-11 |
| S12 | note | `docs/notes/your-own-video.md:125-135` | the staging check should test the bucket's own guards: a wrong-size PUT refused, an exact CORS origin, HTTPS only | fix (the checklist) | fixed (1504bd5) -- decided by evan-wes, 2026-10-11 |
| S13 | note | `config.py:96-100`; `server/settings.template.env` | invite codes are checked for length only; the template does not say to make them at random | fix (the template) | fixed (a5b122c) -- decided by evan-wes, 2026-10-11 |
| S14 | note | `storage.py:29-41, 190-192` | storage keys accept `.` parts and a trailing newline; a result path `.` escapes deletion | fix (`fullmatch`, `.` refused) | fixed (f616342) -- decided by evan-wes, 2026-10-11 |
| S15 | note | `app.py:383-445, 485-492` | the worker token is one all-powerful credential; folder storage serves a result's `*.html` from the server's origin | fix: result paths limited to the bake's names, folder files served as attachments; defer a token per worker | fixed (f616342: a result's files a bundle's alone, served as downloads); deferred (the ladder's open items, PR 11's, before going public): a token per worker -- decided by evan-wes, 2026-10-11 |
| S16 (H7) | note | `worker.py:145-150, 207-225` | the worker follows any URL the server gives, `file://` and plain HTTP included | fix (the panel's H7) | fixed (68aaf3c) -- decided by evan-wes, 2026-10-11 |
| S17 | note | `.gitignore`, `.dockerignore` | a filled-in `server/settings.env` would be committed and copied into the image | fix (`*.env` ignored, the template excepted) | fixed (a5b122c) -- decided by evan-wes, 2026-10-11 |
| S18 | note | `eval/video.py:345-353, 485-496`; `worker.py:291` | a hostile zip's CRC, encryption or CSV-field errors reach the owner as "an internal error" | fix (refused with the import's own message) | fixed (0a26105) -- decided by evan-wes, 2026-10-11 |
| S19 | note | `scripts/serve_demo.py` | with `--lan`, every page is open on plain HTTP to the whole network | fix (`--lan` puts every page behind the code; the checklist advises a trusted network) | fixed (a5b122c; the demo checklist in 1504bd5) -- decided by evan-wes, 2026-10-11 |
| S20 | note | `scripts/locks/slam-it-all.lock` | the lock pins versions without hashes, and only a PyAV bump updates its FFmpeg | defer (to going public) | deferred (the ladder's open items, PR 11's, before going public) -- decided by evan-wes, 2026-10-11 |

### Security review

## Security review

**Scope:** The upload path of PR 11 at `cbf7b3d` against `63b61da` (origin/develop):
- `server/src/slam_it_all_server/` (all modules) and `server/settings.template.env`
- `src/slam_it_all/orchestration/worker.py`
- `src/slam_it_all/eval/video.py` and `lenses.py` (the lens fetch and metadata parts)
- `vo.run`'s launch of the C++ program
- `viewer/upload.html`, `status.html`, `index.html`, `src/jobs.js`, `upload_view.js`, `status_view.js`
- the Dockerfiles, `.dockerignore`, `scripts/serve_demo.py`, and the staging steps in `docs/notes/your-own-video.md`

**How I checked:**
- Ran the server on 127.0.0.1:8790 with folder storage and staging on, probed it with curl, then stopped it.
- Signed an S3 link offline with boto3.
- Ran two process probes on the room's Stray Scanner zip.
- Built no Docker images and contacted no external service.

**Summary:** The server's core is sound:
- staging auth leaves open only the paths that carry their own credential;
- every secret is compared in constant time;
- tokens and ids carry 128 bits each;
- every state change is one transaction;
- signed links cover method, key, expiry and size;
- the pages set every server string with `textContent`;
- the zip is never unpacked, and each file is capped on its actual bytes.

The one must-fix is in the worker: the 20-minute timeout, and any failed heartbeat, leave the job's processes running. The C++ program always survives, so "one job at a time" and the timeout do not hold for the most expensive stage. The should-fixes are mostly hardening before going public:
- result tokens land in the logs;
- the decoder accepts any FFmpeg input format;
- the decoding child inherits the worker token;
- the owner's upload link can be replayed after use;
- one code can fill the queue;
- a failed deletion is lost for good;
- the server runs as root on Railway;
- there is no CSP.

### Findings

#### [must-fix] The timeout and a failed heartbeat leave the job's processes running
- **Where:** `src/slam_it_all/orchestration/worker.py:317-354` (`_run_child`), `worker.py:390-398` (`handle`), `src/slam_it_all/orchestration/vo.py:680`
- **Issue:** There are two paths.
  - **The timeout** calls `child.kill()`, which signals only the Python child. The C++ program it started (`subprocess.run(argv, capture_output=True, text=True, check=False)`, no timeout, same process group) keeps running.
  - **A failed report.** Any exception raised by `report(...)` leaves `_run_child` with no clean-up at all, and the child and the C++ program both keep going. That covers a 409 (`AbandonedError`) and a `URLError` while the server redeploys.

  In both cases `handle` then deletes the folder, and `serve` claims the next job while the old pipeline still runs. The worker runs as PID 1 in its image (`exec python ...`), so the orphans are reparented to it and never reaped.
- **Why it matters:**
  - The design's guards fail exactly at the C++ stage. One job at a time stops holding: each orphan keeps its own 6 GiB `RLIMIT_AS`, but nothing bounds how many there are. The 20-minute timeout bounds only the Python side.
  - An invited visitor whose upload drives the pipeline past 20 minutes, or any server redeploy during a job, gets two pipelines running at once on the worker. On Railway that risks killing the container when it runs out of memory.
  - Neither the timeout test (which kills at 0.5 s, during the import) nor the 409 test (cancelled at "downloading") reaches this path.
- **Evidence:** Two probes on the room's zip, run on this machine.
  - **Probe 1:** `report("running")` raises `ConnectionError`, as a heartbeat does during a redeploy. One second after `_run_child` raised, both processes were still alive:
    ```
    1536181 1535333 .../python -m slam_it_all.orchestration.worker process .../orphan-job/upload.zip ...
    1539858 1536181 .../slam_it_all/bin/slam_it_all_vo --dataset tum --sequence .../orphan-job/your-video ...
    ```
  - **Probe 2:** SIGKILL to the Python child alone, which is what `child.kill()` does at the deadline. The C++ program was still alive afterwards, reparented, and then finished on its own (the room's run is short):
    ```
    killed the Python child 1540647 ; the C++ program is 1544360
      1 s later, C++ program alive: True 1544360    2731 .../slam_it_all_vo ...
    ```
- **Suggested fix:**
  - Start the child with `start_new_session=True`.
  - Wrap `_run_child`'s loop in `try/finally` that runs `os.killpg(child.pid, SIGKILL)` and `child.wait()` while the child still runs. Use the same group kill at the deadline.
  - Add tini (or another init) as the worker image's entrypoint so PID 1 reaps orphans.
  - Add a test: a report that raises at "running", and a deadline reached during "running", each leave no process of the job behind.

#### [should-fix] Result tokens and signed links are written to the server's log
- **Where:** `server/src/slam_it_all_server/__main__.py:76-78` (uvicorn's access log on by default); routes `/r/<token>`, `/api/jobs/<token>`, `/r/<token>/bundle/...`, `/storage/<key>?...`
- **Issue:** Every request line is logged with its path and query string. That includes:
  - the owner's 128-bit token, which grants 7 days of access to the result;
  - folder storage's complete signed links, valid for 1 hour.

  Railway's edge also keeps its own HTTP request logs, which by their nature hold the path. Confirm this in the dashboard on staging.
- **Why it matters:**
  - The logs become a second, unlisted store of bearer links, kept on the host's retention rather than the 7-day promise.
  - Client IP addresses are logged too. The page's "What is kept, and who can see it" section does not mention either.
  - On staging only the owner reads the logs, so this is hardening, not an exposure today.
- **Evidence:** From the probe server's log:
  ```
  "PUT /storage/uploads/069a…eae?expires=1791714309&size=10&signature=62f5237f…7a09 HTTP/1.1" 200 OK
  "POST /api/jobs/WU-Lo6EakiVG34m1fQ0SfA/uploaded HTTP/1.1" 200 OK
  "GET /api/jobs/WU-Lo6EakiVG34m1fQ0SfA HTTP/1.1" 200 OK
  ```
- **Suggested fix:**
  - Now: `uvicorn.Config(..., access_log=False)`. Log your own lines keyed by job id, never by token or query string. A logging filter that redacts `/r/<token>`, `/api/jobs/<token>` and query strings also works.
  - Before going public: consider keeping the token out of every path. Use `/r#<token>`, with the status page reading `location.hash` and sending the token in a header or body. Then neither uvicorn nor the edge ever sees it.
  - Add a line to the page saying the host keeps request logs, including IP addresses, and for how long.

#### [should-fix] Untrusted uploads are opened with every FFmpeg input format
- **Where:** `src/slam_it_all/eval/video.py:190` and `:385` (`av.open(str(path))`, with no `format` and no `options`)
- **Issue:** FFmpeg chooses the demuxer from the file's contents. This PyAV build (av 18.1.0) includes playlist and reference formats that open other files named inside the input: `hls`, `concat`, `dash` and `tee` all report available. No format allow-list is set, and no `protocol_whitelist`.
- **Why it matters:**
  - Reference formats are a known FFmpeg risk; it ships safety options (`concat`'s `safe`, `hls`'s `allowed_extensions`) for that reason. Those defaults narrow the risk but are the only barrier.
  - The upload's worst case is on the LAN demo. There the worker runs as the owner, with the owner's home directory readable, and the frames it decodes are published in the result.
  - I deliberately did not build a proof file, so whether this exact build can be led to read another file is **untested**.
- **Evidence:** `av.open(str(path))` at `video.py:190` and `:385`. On this env, `av.formats_available` contains `hls`, `concat`, `dash`, `tee` (checked with `python -I -c "import av; ..."`).
- **Suggested fix:**
  - Open with an allow-list, for example `av.open(path, format="mov")` (the mov/mp4 demuxer covers iPhone `.mov` and Android/Stray `.mp4`), or `options={"format_whitelist": "mov,mp4,m4a,3gp,3g2,mj2", "protocol_whitelist": "file"}`.
  - Refuse any other container with an `InvalidInputError`.
  - Add a test that a non-mp4 container (for example a Matroska file) is refused.
  - Do this before the in-person demo.

#### [should-fix] The decoding child inherits the worker token, and on the demo the owner's whole environment
- **Where:** `worker.py:317-323` (`subprocess.Popen(...)` with no `env=`); `scripts/serve_demo.py:150-153` (`env={**os.environ, **token}`)
- **Issue:** The child that runs FFmpeg on hostile input, and the C++ program, both inherit `SLAM_IT_ALL_WORKER_TOKEN`. On the laptop demo they also inherit every variable the owner's shell has (API keys, cloud credentials, the SSH agent socket).
- **Why it matters:** A memory-safety bug in a decoder (FFmpeg has them every year) becomes a stolen worker token. With that token an attacker can download every queued upload, forge results, and fail jobs. On the demo it becomes the owner's credentials.
- **Evidence:** `child = subprocess.Popen([*command, str(upload_path), kind, str(out)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, preexec_fn=_limit_memory)`, with no `env`.
- **Suggested fix:**
  - Pass the child an explicit minimal environment: `PATH`, `HOME`, `SLAM_IT_ALL_LENS_PROFILES`, the thread caps, and no token.
  - In `serve_demo.py`, give the worker a minimal environment too.
  - Running the demo's worker in its Docker image would isolate it from the home directory.
  - Same-uid processes can still read the parent's `/proc/<pid>/environ`, so this raises the bar rather than closing the hole. Keep PyAV current for FFmpeg fixes.

#### [should-fix] The owner's upload link can be replayed after the upload is confirmed and after the import deleted it
- **Where:** `server/src/slam_it_all_server/app.py:325-343` and `:403-414`; `storage.py:155-161` (links last `LINK_SECONDS` = 3600)
- **Issue:** The signed PUT stays valid for an hour, and nothing stops a repeat.
  - After `uploaded` has checked the size and queued the job, the owner can replace the file the worker is about to download.
  - After the import ends and the server deletes the upload, the same link writes it back. It then stays until the job is deleted 7 days later.
- **Why it matters:**
  - The page says "Your upload is deleted as soon as its frames have been imported". A replayed link breaks that, and up to 500 MB per job sits in the bucket for a week.
  - It also makes the server's one size check at `uploaded` a time-of-check/time-of-use check. This is a weakness in how the decided signed PUT was built, not in the decision itself.
- **Evidence:** On the probe server:
  - PUT 200, then `uploaded` gave `{"state":"queued"}`;
  - the same link was accepted again: `PUT again after queued: 200`;
  - after the worker's `importing` and `running` reports, the uploads folder was empty;
  - `owner re-PUT after the import ended: 200` recreated `uploads/069a…eae` (10 bytes).
- **Suggested fix:**
  - On `uploaded`, move or copy the object server-side to a key the owner holds no link for (S3 `CopyObject` plus delete, or a rename in folder storage), and give the worker that key.
  - Alternatively, shorten the upload link's life and delete `uploads/<id>` again once the link has expired.

#### [should-fix] One code holder can fill the queue for an hour with five tiny requests
- **Where:** `server/src/slam_it_all_server/jobs.py:212-232` (`waiting` jobs count toward `QUEUE`, `UPLOAD_WAIT` = 1 h); `app.py:314-329`
- **Issue:** A job that never receives its upload holds one of the 5 open places for an hour. No limit applies per invite code, and the job does not record which code made it.
- **Why it matters:**
  - Anyone who holds or learns a code shuts everyone out for an hour, repeatably, at 5 requests an hour.
  - Without the code on the job, a leaked code's abuse cannot be traced or cut off short of redeploying.
- **Evidence:** On the probe server, `POST /api/jobs` with `"bytes":1` five times gave `201` each time; the sixth gave `503`.
- **Suggested fix:**
  - Give waiting jobs their own short limit (for example 15 minutes, or the link's life) and a separate cap.
  - Cap open jobs per code, storing a hash of the code on the job.
  - Log the code's hash, not the code, with each job.

#### [should-fix] A deletion that fails is lost, and the files outlive the 7 days
- **Where:** `server/src/slam_it_all_server/app.py:121-134` (`sweep`), `:275-279` (`contextlib.suppress(Exception)`)
- **Issue:** `Jobs.sweep` commits the deletion of the rows first; the files are deleted afterwards. If a storage call raises (an S3 hiccup, or the server stopping mid-sweep), the rest of that sweep's files are skipped. Their jobs are already gone from the list, so no later sweep retries them. The error is swallowed silently.
- **Why it matters:** Results and uploads can stay in the bucket indefinitely, contrary to the page's "kept for 7 days, then deleted", and nothing records that it happened.
- **Evidence:** `swept = job_list.sweep()` (committed) precedes the `for job in swept.deleted: store.delete(...); store.delete_prefix(...)` loop; the background task wraps it in `with contextlib.suppress(Exception):`.
- **Suggested fix:**
  - Delete the files first, then the row, or keep a "deleting" state that is retried.
  - Log failures.
  - Add a bucket lifecycle rule as a backstop (for example `uploads/` after 2 days, `results/` after 8). Add it to the staging checklist if Railway's buckets support one.

#### [should-fix] Synchronous SQLite work blocks the event loop, and every read takes the write lock
- **Where:** `app.py:391-401` (`async def claim` calls `server.jobs.claim(...)` directly, once a second for up to 60 s); `jobs.py:175-189` (`BEGIN IMMEDIATE`, `timeout=10.0`, also used by `by_token`, `by_id`, `ahead`); `app.py:461-483` (sync file writes in `async def put_file`)
- **Issue:**
  - `claim` runs in the event loop. While another thread holds SQLite's write lock (the sweep, a status poll), every request on the server waits, for up to the 10 s busy timeout.
  - Every status read, including one for a made-up token (unauthenticated once staging auth is off), takes that write lock.
- **Why it matters:** A flood of `GET /api/jobs/<random>` turns into stalls of the whole server whenever the worker's long-poll contends with it. Contention from normal status polling causes the same stalls on a smaller scale.
- **Evidence:** The quoted lines. `while (job := server.jobs.claim(request.kinds)) is None:` sits inside `async def claim`.
- **Suggested fix:**
  - Use `await asyncio.to_thread(server.jobs.claim, request.kinds)`.
  - Do reads without `BEGIN IMMEDIATE` (a plain connection, WAL mode).
  - In `put_file`, write the file through `asyncio.to_thread` or anyio's file API.

#### [should-fix] The import bounds memory and time, but not frame size, frame count or disk
- **Where:** `src/slam_it_all/eval/video.py:120-137` (`output_size`), `:187-207` (`_decode`), `:218-257`; `worker.py:228-230` (only `RLIMIT_AS`)
- **Issue:**
  - A frame whose short side is 480 or less is kept at its stored size, so the long side is bounded only by FFmpeg's own limits.
  - Frames closer together than 1/40 s, or not increasing in time, are decoded and skipped without limit.
  - The PNGs are written with no `RLIMIT_FSIZE` and no free-disk check.
- **Why it matters:** An upload of very wide, high-entropy frames can write tens of GB of PNGs within the 20 minutes and fill the worker's disk. It also feeds the C++ program huge frames, which feeds the must-fix above. The job's folder is removed afterwards, so this is a denial of service of bounded length.
- **Evidence:** `if short <= SHORT_SIDE: return width, height`, and there is no check on `stream.width`/`stream.height` in `probe`.
- **Suggested fix:**
  - At `probe`, refuse a stored long side over 4096 (or an aspect ratio over about 3:1).
  - Cap decoded frames (for example `MAX_SECONDS` × 240).
  - Set `RLIMIT_FSIZE` in `_limit_memory`, or check free space before the import.

#### [should-fix] The server runs as root on Railway
- **Where:** `docs/notes/your-own-video.md:127` (`RAILWAY_RUN_UID=0`); `docker/server.Dockerfile:24-25`
- **Issue:** The image drops to uid 10001, but the staging step overrides that with uid 0 so the server can write to the volume. The internet-facing process that holds the bucket keys then runs as root in its container.
- **Why it matters:** It removes the protection the image's `USER server` was there to give.
- **Suggested fix:**
  - Keep the image's user and add a small entrypoint that runs as root only to `chown 10001 /data`, then `exec setpriv --reuid=10001 --regid=10001 --init-groups python -m slam_it_all_server --host ::`.
  - `setpriv` is in util-linux, which the Debian base image already has.

#### [should-fix] No Content Security Policy, an unpinned third-party script, and Google Fonts
- **Where:** `app.py:83-88` (`_HEADERS`); `viewer/index.html:13-20` (three.js from jsDelivr with no `integrity`); `upload.html:8-12`, `status.html:112-116`, `index.html:7-11` (Google Fonts); `viewer/src/main.js:206` (any `?bundle=` URL)
- **Issue:**
  - A compromised CDN response runs as the server's origin. That origin includes the page where invite codes are typed, and on staging the browser's cached Basic-auth access.
  - The viewer fetches a bundle from any URL given in `?bundle=`, so anyone can make a link on the server's origin show an attacker's frames and numbers. This is content spoofing, not XSS: everything is set as text.
  - Every page sends visitors' IP addresses to Google, and the privacy section does not say so.
  - There is no HSTS header for the future custom domain.
- **Suggested fix:**
  - Send a CSP: `default-src 'self'; script-src 'self' https://cdn.jsdelivr.net 'sha256-<importmap>'; style-src 'self' https://fonts.googleapis.com; font-src https://fonts.gstatic.com; connect-src 'self' <bucket origin>; img-src 'self' blob: data:; base-uri 'self'; form-action 'self'; frame-ancestors 'none'`.
  - Add `integrity` to the import map, or vendor three.js. Self-host the two fonts.
  - Restrict `?bundle=` to same-origin paths.
  - Add `Strict-Transport-Security` once on a domain served only over HTTPS.

#### [note] The staging checklist should verify the bucket's own guards
- **Where:** `docs/notes/your-own-video.md:125-135`
- **Issue:** boto3 does sign the size into the upload link, so enforcing it rests with the bucket. Offline: `X-Amz-SignedHeaders: ['content-length;host']`. Whether Railway's S3-compatible bucket rejects a PUT of a different size is untested here.
- **Suggested fix:** Add to "The check":
  - a PUT of a wrong size through a fresh link must fail;
  - the CORS origin must be the exact server address, never `*`;
  - the lifecycle rule above;
  - `curl -I http://<app>.up.railway.app/` should redirect to HTTPS, since a 401 over plain HTTP would invite Basic credentials in clear text.

#### [note] Invite codes are checked for length only
- **Where:** `server/src/slam_it_all_server/config.py:96-100`; `server/settings.template.env:3-7`
- **Issue:** Nothing makes a code random, and with no rate limit (as decided) a human-chosen 12-character code is guessable. The template's "make the secrets with `secrets.token_urlsafe(32)`" does not clearly cover the codes.
- **Suggested fix:** State in the template that codes are generated randomly (for example `secrets.token_urlsafe(9)`, 72 bits, as `serve_demo.py` does).

#### [note] Storage keys accept `.` parts and a trailing newline; a result path `.` escapes deletion
- **Where:** `server/src/slam_it_all_server/storage.py:29-41` (`$` matches before a final `\n`; `.` parts are allowed), `:190-192` (`rmtree(..., ignore_errors=True)`); `app.py:59` (`PAGES` also uses `$`, harmless behind its `is_file` check)
- **Issue:** A worker path `.` gets a link to `results/<id>/.`. pathlib turns that into the file `results/<id>`, which `delete_prefix`'s `rmtree` then silently fails to remove. Only a buggy or compromised worker can do this.
- **Evidence:** In-process, `check_key('uploads/abc\n')` and `check_key('results/x/.')` are accepted. On the probe server, a PUT to the `.` link wrote `results/069a…eae`, and the file was still there after `/failed`.
- **Suggested fix:** Use `\Z` (or `re.fullmatch`) and refuse `.` parts, as well as `..`.

#### [note] The worker token is one all-powerful credential
- **Where:** `app.py:383-445`; `app.py:485-492` (`FileResponse(path)` guesses the media type from the key)
- **Issue:** The holder of the token can:
  - claim and download every queued upload;
  - re-request result links, including after `done`, since earlier links stay valid for an hour;
  - in folder storage, upload a `*.html` result that `/storage/` serves from the server's own origin as `text/html`.
- **Suggested fix:**
  - Restrict result paths to the bake's names and extensions.
  - Serve folder storage files as `application/octet-stream` with `Content-Disposition: attachment`.
  - Later, consider a token per worker.

#### [note] The worker trusts the server's URLs
- **Where:** `worker.py:207-225`, `:145-150`
- **Issue:** `urllib.request.urlopen` accepts `file://`, so a malicious or compromised server could hand the worker a local file as the "upload". The `--server` address may also be plain HTTP. That is fine on Railway's private network, but not for a later GPU worker elsewhere.
- **Suggested fix:** Require `https` for links and the server, allowing `http` only for loopback and `*.railway.internal`.

#### [note] Filled-in settings files are not ignored
- **Where:** `.dockerignore`, `.gitignore`, `docker/server.Dockerfile:13-17`
- **Issue:** `COPY server /opt/slam-it-all-server` keeps whatever is in `server/` in that image layer; the later `rm -rf` does not remove it. A filled-in `server/settings.env` would therefore ship in the image, and it is also not git-ignored. Railway builds from git, so this concerns local builds and accidental commits.
- **Suggested fix:** Add `*.env` with `!server/settings.template.env` to both files.

#### [note] Some refusals of hostile zips surface as "an internal error"
- **Where:** `video.py:345-353`, `:485-496`; `worker.py:291`
- **Issue:** A CRC failure, an encrypted entry, an unsupported compression method or an over-long CSV field raises `zipfile.BadZipFile`, `RuntimeError`, `NotImplementedError` or `csv.Error`. None of these is wrapped, so the owner sees "an internal error". Clean-up still happens, so this is safe; it belongs to the hygiene reviewer.
- **Evidence:** A zip with one byte corrupted inside `camera_matrix.csv` raised `zipfile BadZipFile - Bad CRC-32 for file '2026-10-10--12-00-00/camera_matrix.csv'`, which `process()` does not catch.

#### [note] `serve_demo.py --lan` exposure
- **Where:** `scripts/serve_demo.py:75`, `:133-136`
- **Issue:** With `--lan`:
  - the demo serves plain HTTP on `0.0.0.0`, so every interface is reachable, a VPN's included;
  - staging is off, so every page is open to the network, and the invite code and result links cross the Wi-Fi in clear text;
  - together with the decoder findings above, the worker parses strangers' files as the owner.
- **Suggested fix:**
  - Use a phone hotspot or another trusted network.
  - Consider `SLAM_IT_ALL_STAGING=1` with `--lan`.
  - Run the worker in its image.

#### [note] Supply chain
- **Where:** `scripts/locks/slam-it-all.lock`
- **Issue:** The lock pins versions without hashes (0 `--hash` lines). PyAV bundles its own FFmpeg, which only a PyAV bump updates.
- **Suggested fix:** Consider `--require-hashes` for the images, and a periodic PyAV bump for FFmpeg's security fixes.

### Checked and fine
- **Staging Basic auth:** only `/healthz`, `/storage/` (signature checked) and `/api/worker/` (Bearer token) are open. Probed: `/`, `/index.html`, `/r/…`, `/api/jobs/…` and `/robots.txt` all give 401. `/healthz/../upload.html`, `/storage/../upload.html`, `/api/worker/../jobs/abc` and the `%2e%2e` form give 404 or 403.
- **Constant-time comparisons:** `hmac.compare_digest` for invite codes (each code compared), the worker token and folder-storage signatures.
- **Names:** the token is `token_urlsafe(16)` and the id `token_hex(16)`, 128 bits each; neither is made from the other, and the worker never learns the token.
- **Job state machine:** every change runs in one `BEGIN IMMEDIATE` transaction that re-checks the state. Queueing, one-at-a-time claims, monotonic stages, and done/failed only from a worker's stage all hold.
- **Folder-storage links:** the HMAC covers method, key, expiry and size; expiry is checked; the key is validated before any path is built.
- **Folder-storage uploads:** writes go to a staged file with exact length, then an atomic replace. Probed: 11 bytes gave 400, chunked encoding 400, and the 64 KiB body cap 413.
- **S3 upload link:** the size is signed in; the size is re-checked at `uploaded` and for each result file at `done`.
- **Bundle files:** only for done jobs and exactly the recorded paths, redirected to storage (307), never proxied. The `PAGES` allow-list is strict.
- **Response headers:** `noindex`, `no-referrer`, `nosniff` and `DENY` on every response, `no-store` on `/api/` (probed). The docs and OpenAPI routes are off, and the server header is off.
- **Pages:** every reason, summary and error is set with `textContent`. The viewer link is relative. The invite code is a password field, sent only in a JSON body and never stored.
- **Zip reading:** never extracted and names never used as paths. Each entry is capped on its declared size and on the bytes actually read, so a zip bomb stops at its cap. Repeated names and absolute or climbing names are refused, and symlink entries are only read as bytes.
- **CSVs:** numbers only, finite, frames in order and one row per frame.
- **Metadata:** only `METADATA_KEYS` are copied, never a location. `source.json` is not part of the bundle, and the bake re-encodes frames from pixels with Pillow, so no file metadata is carried.
- **Lens profiles:** fetched over HTTPS at a pinned commit, the size capped, members never extracted under their own names, `..` refused.
- **What owners see:** the worker's paths are stripped from reasons; anything unexpected becomes "an internal error"; the child's stderr is discarded. The download cap is enforced while streaming.
- **Images:** the base image is pinned by digest, Eigen is checked by its commit, both run as non-root users, the server image holds no decoder, and the worker image holds no secret.
- **Retention:** the upload is deleted when the import ends, fails or expires, and results after 7 days, apart from the cases above.

Scratch files from the probes are in `/tmp/scratch/review-security/` (`orphan_probe.py`, `timeout_probe.py`, `server.log`). The probe server on 127.0.0.1:8790 is stopped, and no process from the probes was left running. The repository was not modified.

## Run 2 -- 2026-10-11

| | |
|---|---|
| **Branch** | `feature/your-own-video` |
| **Range** | `cbf7b3d...1359afb`, run 1's fixes, read against the whole PR `63b61da...1359afb` |
| **Reviewer** | security (opus), a general-purpose agent with its own brief, as in run 1 |
| **Probes** | the server on 127.0.0.1 with folder storage and staging on, its upload wait cut to 30 s and its sweep to 2 s; a real `worker serve` and three `serve_demo.py` runs on local ports; the locally built images with no network; in-process probes of `check_url`, the demuxer and the key rules; no image built, no external service contacted |
| **Panel** | [the panel's report](2026-10-11-feature-your-own-video.md), run 2, whose IDs (H*, A*, L*) are named where a finding overlaps one of its own |

### Findings and resolutions

| ID | Severity | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|
| S22 (A13) | should-fix | `orchestration/worker.py:462-474, 652-655`; `scripts/serve_demo.py:176-180` | a SIGTERM to the worker, or the demo's Ctrl-C, can leave a job running with no deadline and its folder on disk; on Railway, tini and the container's teardown cover it | fix (the panel's row, a must-fix there) | fixed (4ee7c67, as the panel's row) -- decided by evan-wes, 2026-10-11 |
| S21 (A15) | should-fix | `server/.../jobs.py:425-428`; `app.py:184-187, 608-630`; `storage.py:179-191` | an upload still arriving when its link expires lands after the link's only clean-up and stays 7 days, whether an honest slow upload or one sent late on purpose | fix (the panel's row) | fixed (d66da0f, as the panel's row) -- decided by evan-wes, 2026-10-11 |
| S23 | should-fix | `server/.../app.py:99-100, 315` | the content policy allows any script on jsDelivr, not only three.js | fix (`script-src` narrowed to `https://cdn.jsdelivr.net/npm/three@0.186.0/`; a test) | fixed (d66da0f) -- decided by evan-wes, 2026-10-11 |
| S24 | note | `orchestration/worker.py:205-211, 243, 321` | `check_url` passes numeric one-label hosts (`http://134744072/` is 8.8.8.8) and never checks a redirect, which urllib follows with the worker's token | fix (a host the C library reads as a number refused; redirects refused on the API calls and checked on downloads; tests) | fixed (4ee7c67: a one-label name still accepted, a container's) -- decided by evan-wes, 2026-10-11 |
| S25 | note | `docker/server-entrypoint.sh:7-10`; `server/.../__main__.py` | the entrypoint's `chown -R` follows the database setting, `/` included; `setpriv` leaves `no_new_privs` off; a bypassed entrypoint would run the server as root unnoticed | fix (a root or relative database folder refused; `--no-new-privs`; the server refusing uid 0; the staging check names uid 10001) | fixed (d66da0f; the staging check in 8f53353) -- decided by evan-wes, 2026-10-11 |
| S26 | note | `orchestration/worker.py:412-413` | any FileNotFoundError in a job reaches the owner verbatim, worker paths included | fix (caught only around the upload's own check; any other one "an internal error", logged) | fixed (4ee7c67) -- decided by evan-wes, 2026-10-11 |

### Security review

## Security review

**Run 1's findings:**
- **S1:** resolved. I reran both probes. A 409 raised at "running", and a deadline reached while the C++ program was held stopped, each left no process of the job 1 s later. tini is PID 1 in the worker image. One gap remains: the fix's guarantee fails on SIGTERM (new should-fix below).
- **S2:** resolved. The probe server's log holds no token, signature or query string, only job ids and an 8-hex prefix of the owner hash. The token out of the path is deferred as decided.
- **S3:** resolved. Matroska, AVI, ffconcat and m3u8 files are refused; MP4 opens. The open options are `protocol_whitelist=file` and `enable_drefs=0`.
- **S4:** resolved. The child gets only `CHILD_ENVIRONMENT`; the demo's worker gets the same plus its token. Run 1's caveat stands as recorded: the same uid can still read `/proc/<ppid>/environ`.
- **S5:** resolved for replays. A replayed PUT after confirmation returned 200 and wrote `uploads/<id>`, but `imports/<id>` was unchanged. `uploads/<id>` was deleted at the link's expiry, and a PUT after that got 403. **Still wrong for a PUT already in flight at the expiry** (should-fix 1).
- **S6:** resolved. The third job on one code got 429. A waiting job expired at `UPLOAD_WAIT`, which freed its place. The code itself is not stored, only its HMAC.
- **S7, S8:** resolved. Files are deleted before rows, a failed deletion is logged and retried, the database is in WAL mode with lock-free reads, and claims and writes run in threads.
- **S9:** resolved. Limits: long side 7680, aspect 3:1, 14,400 decoded frames, `RLIMIT_FSIZE` 2 GiB.
- **S10:** resolved. I ran the locally built `slam-it-all-server:pr11` with `--user 0`: `uid=10001(server)`, and CapPrm, CapEff and CapAmb are all 0.
- **S11:** resolved. The policy carries the import map's hash, and my own recomputation matches the header. three.js's three files have integrity values, and `bundleAddress` keeps bundles same-origin. Fonts and HSTS are deferred as decided. One part is still too wide: the policy allows all of jsDelivr (should-fix 3).
- **S12, S13, S14, S17, S18, S19:** resolved.
- **S15:** resolved. Every bad result path I tried was refused, and folder files are served as `application/octet-stream` attachments. A token per worker is deferred as decided.
- **S16:** resolved for schemes and named public hosts. The one-label rule still lets numeric hosts through to public addresses (note).
- **S20:** deferred as decided.

**Scope:** the fixes `cbf7b3d...1359afb` (0a26105, 68aaf3c, f616342, a5b122c, cd1a668, 1504bd5). That covers `server/src/slam_it_all_server/*`, `src/slam_it_all/orchestration/worker.py`, `src/slam_it_all/eval/video.py` and `lenses.py`, `docker/*`, `scripts/serve_demo.py`, `viewer/index.html` and `src/bundle.js`, read against the whole PR `63b61da...1359afb`.

**How I checked:**
- Reran run 1's orphan and timeout probes, rewritten for the new `_run_child`.
- Ran the server on 127.0.0.1:8790 with folder storage and staging on, through `launch_server.py`, which shortens `UPLOAD_WAIT` to 30 s and the sweep to 2 s.
- Ran a real `worker serve` and three `serve_demo.py` instances on ports 8771-8773.
- Ran the locally built `pr11` images with `--rm --pull never --network none`.
- Ran in-process probes of `check_url`, the demuxer and the key rules.
- Ran `tests/server`, `tests/orchestration/test_worker.py` and `tests/test_serve_demo.py`: 145 passed.
- Built no image and contacted no external service. The repository is unchanged.

**Summary:** The fixes hold. Every run-1 finding is resolved or deferred as decided, and there is no must-fix. The fixes leave or introduce three should-fixes:
1. An upload still arriving when its link expires escapes the link's clean-up and stays 7 days.
2. Moving the job into its own session took it out of Ctrl-C's reach, while SIGTERM still skips the worker's clean-up.
3. The new Content Security Policy allows any script on jsDelivr.

### Findings

#### [should-fix] An upload still arriving when its link expires lands after the sweep has closed the link, and stays 7 days
- **Where:**
  - `server/src/slam_it_all_server/jobs.py:425-428` (`links_to_close`: `created <= now - UPLOAD_WAIT`)
  - `app.py:184-187` (`_close_link`)
  - `app.py:608-630` (`put_file` checks the link once, before reading the body)
  - `storage.py:179-191`
- **Issue:** The sweep deletes `uploads/<id>` once, at the link's expiry, and sets `link_open = 0`. But a link is checked when a PUT starts, not when it ends:
  - folder storage verifies the link before streaming the body;
  - S3 checks a presigned URL's expiry when the request arrives (whether Railway's bucket does the same is untested here).

  So a PUT that starts before the expiry and finishes after the sweep writes `uploads/<id>` after its only deletion. Nothing deletes it again until `_delete`, 7 days after the job ended.
- **Why it matters:**
  - **Honest owners.** A 500 MB upload needs about 4.4 Mbit/s sustained to finish within 15 minutes (estimate: 500e6 × 8 / 900 s). A slower phone upload runs past the window. The job expires ("the upload did not arrive in time"), and then the video lands and stays in the bucket for 7 days, against the page's promise.
  - **Abuse.** A code holder can do the same on purpose, 500 MB per job. Expired jobs no longer count toward the per-code cap, so that is 2 such jobs every 15 minutes per code.
  - **Bound.** The 7-day deletion limits it, as does the bucket's lifecycle rule if Railway's buckets support one. It is a broken retention promise, not an exposure: no one else can read the file.
- **Evidence:** `flow_probe.py` against `launch_server.py` (`UPLOAD_WAIT` 30 s, sweep 2 s, folder storage), with the PUT trickled at 1 byte/s:
  ```
  [27.1s] job 4: trickled PUT started; link expires at +30 s after creation
  [42.1s] job 4: trickled PUT (started before expiry) finished -> 200 {}
  [45.1s] job 4 (trickled): expired link_open 0 ; uploads/ there after the link was closed: True
  [50.1s] job 4 after more sweeps: uploads/ there: True size 15
  ```
- **Suggested fix:**
  - Close the link only after a grace period past its expiry that covers the longest PUT storage lets run (for example `created + UPLOAD_WAIT + 1 h`). The waiting job's own 15-minute expiry stays as it is.
  - In folder storage, check the link's expiry again before `staged.replace(target)`.
  - Add a test: a PUT that finishes after the link's expiry leaves nothing behind once the sweep has run.

#### [should-fix] Stopping the worker with SIGTERM, or the demo with Ctrl-C, can leave a job running unsupervised
- **Where:**
  - `src/slam_it_all/orchestration/worker.py:462-474` (`start_new_session=True`)
  - `worker.py:652-655` (only `KeyboardInterrupt` is handled; there is no SIGTERM handler)
  - `scripts/serve_demo.py:176-180` (`child.terminate()` right after its own Ctrl-C)
- **Issue:** S1's fix gives the job its own session, which is what makes the group kill work. It also takes the job out of the terminal's process group, so Ctrl-C no longer reaches it. The job's only clean-up is now the worker's `finally` blocks: `killpg`, then `rmtree`.
  - The worker turns SIGINT into `KeyboardInterrupt`, but SIGTERM's default action kills it without running any `finally`.
  - `serve_demo.py` sends SIGTERM to the worker as soon as its own Ctrl-C arrives, which races the worker's own unwinding.
- **Why it matters:**
  - Any SIGTERM to the worker orphans its job: serve_demo's `terminate()`, `kill`, a systemd stop, or the later GPU worker outside a container.
  - The import and the C++ program then run on with no deadline; only the inherited rlimits still apply.
  - The job's folder, with the upload in it, stays on disk.
  - Before the fix, Ctrl-C reached the job's processes directly, so the demo's Ctrl-C path is a regression.
  - On Railway this is covered: tini forwards SIGTERM, and once the worker exits, the container's teardown kills the rest.
- **Evidence:**
  - **SIGTERM to `worker serve` while the C++ program ran** (`sigterm_probe.sh`): the worker died. 1 s later the child (reparented to pid 2731) and `slam_it_all_vo` were still running, and `work/job-ed051681-98a7pihb/{upload.zip,run,your-video}` remained.
  - **Ctrl-C to serve_demo's process group** (`/bin/kill -INT -- -PGID`) as the C++ program started, three runs (`ctrlc_probe.sh`):
    - In run 3, the child (ppid 2731) and `slam_it_all_vo` were still alive 1 s later. The orphaned child later died on `BrokenPipeError` when it printed "baking".
    - All three runs left the job's folder, with `upload.zip`, in `data/work`.
- **Suggested fix:**
  - In the worker's `main`, make SIGTERM unwind the same way as Ctrl-C: a handler that raises `KeyboardInterrupt` or `SystemExit`, as `serve_demo._stop` does. Then `_run_child`'s `killpg` and `handle`'s `rmtree` run however the worker is stopped.
  - In `serve_demo.py`, give the worker a few seconds to exit on its own SIGINT before terminating it.
  - Add a test: SIGTERM to `serve` during a job leaves no process of the job and no job folder.

#### [should-fix] The Content Security Policy allows any script on jsDelivr
- **Where:** `server/src/slam_it_all_server/app.py:99-100` (`CDN = "https://cdn.jsdelivr.net"`) and `:315`
- **Issue:**
  - `script-src` names the whole jsDelivr host, which serves every npm package and GitHub repository. The import map pins three.js's path and its sha384 values, but the policy itself would let any `https://cdn.jsdelivr.net/npm/<anything>` run on the origin where invite codes are typed.
  - The import map's `integrity` is honoured only by browsers that implement import-map integrity; other browsers load the modules unchecked.
- **Why it matters:**
  - The policy is the second line of defence against script injection. No injection is known: every server string is set as text. But if one appeared, this policy would not stop it from loading an attacker's package from the allowed CDN.
  - This is hardening before going public.
- **Evidence:** The header the probe server sent: `script-src 'self' https://cdn.jsdelivr.net 'sha256-xRXsj6h8WM4LThHv+2yd/VWRcHXQNR5O0ot1D7veDuM='`.
- **Suggested fix:**
  - Narrow the source to `https://cdn.jsdelivr.net/npm/three@0.186.0/` (a source path ending in `/` matches as a prefix).
  - Or vendor three.js under `src/`, which also ends the third-party request the page has to disclose.
  - Add a test that the policy names no wider script source.

#### [note] `check_url`'s one-label rule lets numeric hosts through to public addresses, and redirects are never checked
- **Where:** `worker.py:205-211` (`"." not in host`), `:243` and `:321` (`urllib.request.urlopen`)
- **Issue:**
  - `ipaddress.ip_address` refuses `134744072` and `0x8080808`, so they pass as one-label names. The C library then reads them as IPv4 addresses, so both resolve to 8.8.8.8. `http://0/` passes too.
  - The resolver's search domains can also turn a one-label name into a public name.
  - `check_url` checks only the first URL. urllib follows redirects to http, https or ftp and keeps every header except the content ones. A 301, 302 or 303 on a worker's POST would therefore carry `Authorization: Bearer <token>` wherever it points, plain http included.
- **Why it matters:** Exploiting either needs a server, or something in front of it, that gives out or redirects to such a URL. So the worker's "plain http only to a host that cannot be on the public internet" does not hold as written. It matters for the later GPU worker, which will connect over the internet.
- **Evidence:**
  - `url_probe.py` on the env's Python 3.11.17: `http://134744072/ ACCEPTED 8.8.8.8`, `http://0x8080808/ ACCEPTED 8.8.8.8`, `http://0/ ACCEPTED 0.0.0.0`.
  - Named and IP-literal public hosts are refused.
  - The redirect behaviour is in `urllib/request.py:680-681` (only content headers dropped) and `:707` (http, https, ftp allowed).
- **Suggested fix:**
  - Allow plain http only to loopback or private IP literals, `localhost` and `*.railway.internal`, and drop the bare one-label rule.
  - Give the worker an opener that refuses redirects on its API calls and re-runs `check_url` on a download's redirect.

#### [note] The server entrypoint's root step could be hardened
- **Where:** `docker/server-entrypoint.sh:7-10`; `server/src/slam_it_all_server/__main__.py`
- **Issue:**
  - **`chown -R` follows a setting.** Its target is the folder of `SLAM_IT_ALL_DATABASE`. A database at `/`, or a relative path (the working directory is `/`), makes it run `chown -R server:server /` until `set -e` stops it at `/sys`. A path under `/opt` would give the server's user the pages it serves. The operator sets this value, so it is a configuration hazard, not an attack.
  - **`setpriv` leaves room.** It leaves `no_new_privs` off and the bounding set full, and the image has 11 setuid or setgid programs (`su`, `mount`, `passwd`, ...).
  - **Nothing would notice a bypassed entrypoint.** For example, a start command set in Railway's service settings might replace it; how such a command interacts with `ENTRYPOINT` is not checked here. The server would then run as root (`RAILWAY_RUN_UID=0`) without a word.
- **Evidence:** In `slam-it-all-server:pr11`:
  - Run with `--user 0`: `uid=10001(server) gid=999(server)`, `CapPrm/CapEff/CapAmb 0`, `NoNewPrivs: 0`, `CapBnd 00000000a80425fb`.
  - With `SLAM_IT_ALL_DATABASE=/jobs.sqlite3`, the entrypoint ran `chown -R` over `/` and stopped at `chown: changing ownership of '/sys': Read-only file system`.
- **Suggested fix:**
  - Refuse a database folder that is `/` or relative.
  - Add `--no-new-privs` to `setpriv`.
  - Make the server refuse to start as uid 0.
  - Add "the server runs as uid 10001" to the staging check.

#### [note] A FileNotFoundError's message reaches the owner verbatim
- **Where:** `worker.py:412-413` (`except (SlamItAllError, FileNotFoundError)`); `_safe` strips only the job's folder and the upload's path.
- **Issue:** This catch is meant for a missing upload. But any FileNotFoundError in the job, such as a missing program of the worker's own, becomes the owner's reason, worker paths included. It is mainly the hygiene reviewer's area.
- **Evidence:** I first ran the worker without its env on `PATH`. That job ended `"reason":"[Errno 2] No such file or directory: 'cmake'"`, from the editable install's rebuild.
- **Suggested fix:** Catch FileNotFoundError only around the import's own check. Map any other one to "an internal error" and log it.

### Checked and fine
- **S1 supervision:** after a 409 at "running", and after a deadline with `slam_it_all_vo` held stopped, the whole group (child and C++ program, one pgid) was gone 1 s later. The worker image runs `/usr/bin/tini --` as PID 1, as uid 10001.
- **Confirmation's move:**
  - The size is checked before the move.
  - A failed S3 copy or delete leaves the job waiting and retriable, and its expiry deletes both keys.
  - A second `uploaded` gets 409.
  - Concurrent confirmations can only move the owner's own bytes, of the signed size.
  - `uploads/` and `imports/` share one filesystem, so `os.replace` holds.
- **Owner hash:** HMAC-SHA256 with the signing key, over a code that has already matched. The API never returns it, and the log shows 8 hex characters. Its message never overlaps folder storage's signatures in a way that exposes either.
- **Sweep flags:** `link_open` and `files_open` each change in their own transaction. A failure is logged and retried. The cap counts only jobs that have not ended, inside the creating transaction.
- **Logs:** with the access log off, the server writes only "job <id>: ..." lines and uvicorn's lifecycle lines.
- **Content Security Policy:**
  - It is on every response, and its import-map hash matches my recomputation.
  - `connect-src` covers storage's origin, including the 307 redirects for bundles, and `media-src blob:` covers the upload page's length check.
  - The pages have no inline script, style or event handlers.
  - The three integrity values are well-formed sha384 hashes. I could not compare them with the CDN's files offline.
- **`bundleAddress`:** cross-origin, protocol-relative, `javascript:` and `data:` addresses are refused (their origin is "null"). Same-origin paths lead only to routes guarded by a token or a signature.
- **Import bounds:**
  - A mid-stream change of frame size is decoded within `RLIMIT_AS` and the deadline, then rescaled to the first frame's output size.
  - At most 2,401 frames are kept (60 s at 1/40 s), each at most 1440×480. In the worst case that is about 5 GB of PNGs (estimate, for incompressible frames), plus C++ files under 2 GiB each. Worth comparing with the worker's disk on Railway.
- **Transient reports:** an HTTP error (401 and 5xx included) or a network error is logged and the job runs on to its deadline, and a 409 ends it. A 401 never loops hot: claims retry every 5 s, and failed `done`/`failed` reports are logged.
- **Key and path rules:**
  - Keys use `fullmatch` and refuse `.`, `..` and a trailing newline.
  - `RESULT_PATH` refused `index.html`, `cpp/x.html`, `a/b/c.json`, `../x.json`, `X.json`, `cpp/./x.json` and `info.json\n`.
  - Folder downloads are sent as octet-stream attachments.
  - Staging auth and path tricks behave as in run 1.
- **`serve_demo --lan`:** staging is on, and the worker inherits only `CHILD_ENVIRONMENT` and its token.
- **Settings files:** `*.env` is ignored by both git and Docker, except the template.
- **Tests:** 145 passed. The repository is unchanged.

Scratch files are in `/tmp/scratch/review2-security/`:
- probes: `orphan_probe.py`, `timeout_probe.py`, `flow_probe.py`, `url_probe.py`, `container_probe.py`, `sigterm_probe.sh`, `ctrlc_probe.sh`;
- the server: `launch_server.py`, `server.env`, `server.log`;
- the demo runs: `demo-877*/`, `demo-877*.log`, `work/`.

The probe server on 8790 and the three demos on 8771-8773 are stopped, no process of any job is left, and no container is running. The probes would be worth keeping in a private scripts folder if wanted; I left them in the scratch folder, as my brief said.
