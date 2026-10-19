# Review panel: `feature/showcase-route`

## Run 1 -- 2026-10-19

| | |
|---|---|
| **Branch** | `feature/showcase-route` |
| **Range** | `8c9872ba...7dc66449` (base: `origin/develop`); the fixes then rebased onto develop `7c1ca659` (the run viewer's PR) |
| **Reviewers** | hygiene (sonnet), architecture (opus) with a security pass; the algorithm reviewer not run (no algorithm in this PR) |
| **How they ran** | registered slam-it-* agents |
| **Layer 1** | at `5ce05ab2` (the fixes, before the rebase): every check passed (pytest 4,928 passed, 1 skipped: the dashcam overlay's OCR test, the env having no Tesseract) except jscpd, on the clone already on develop that the run viewer's PR removed; after the rebase, jscpd finds 0 clones |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | should-fix | hygiene | `server/src/slam_it_server/app.py:187` | The showcase page's HTML head copies `upload.html`'s, with no test tying them together | fix (with A2) | fixed (21fbf78b: the page in `viewer/showcase.html`, its head tested against `upload.html`'s) -- decided by evan-wes, 2026-10-19 |
| H2 | should-fix | hygiene | `tests/server/test_showcase.py:217,228` | Loose status assertions in two showcase tests | fix | fixed (21fbf78b: exact `(path, status)` pairs) -- decided by evan-wes, 2026-10-19 |
| H3 | should-fix | hygiene | `server/src/slam_it_server/app.py:907-914` | `showcase` in `_pages` is the page's HTML; a repeated header literal | fix | fixed (21fbf78b: `showcase_html`, `_PAGE_HEADERS`) -- decided by evan-wes, 2026-10-19 |
| H4 | note | hygiene | `VERSION`, `CHANGELOG.md` | VERSION jumps 0.3.14.0 to 0.3.16.0, so merge order matters | note | resolved: the run viewer's PR (0.3.15.0) merged first and this branch was rebased onto it -- decided by evan-wes, 2026-10-19 |
| H5 | note | hygiene | `docs/notes/your-own-video.md:146` | One overlong line in the notes' paragraph edit | fix | fixed (3293d948) -- decided by evan-wes, 2026-10-19 |
| H6 | note | hygiene | environment | The shared conda env imports the server package from another checkout | note | noted (layer 1 ran in an env with this worktree installed) -- decided by evan-wes, 2026-10-19 |
| A1 | should-fix | architecture | `server/src/slam_it_server/app.py:101-113,895-902` | The calibration page and its images are served from storage's origin, outside the site's headers, with no producer or link | narrow, and record | fixed (21fbf78b: the route serves `calibration.html` alone, a self-contained page with no script; no images, no SVG; 3293d948: its origin accepted and recorded in the design doc and the note) -- decided by evan-wes, 2026-10-19 |
| A2 | should-fix | architecture | `server/src/slam_it_server/app.py:187-252` | The showcase page is an HTML template in `app.py`, outside `viewer/` and its checks | fix | fixed (21fbf78b: `viewer/showcase.html`, read at startup, its marker checked; the image copies it, checked by a test; prettier covers it) -- decided by evan-wes, 2026-10-19 |
| A3 | should-fix | architecture | `docs/design/2026-10-09-run-viewer.md` | The design doc does not record the showcase; the ladder's "showcase page" names something else | fix | fixed (3293d948: section 10, 2026-10-19 decisions; section 5.4 qualified; the ladder's walkthrough page kept distinct) -- decided by evan-wes, 2026-10-19 |
| A4 | should-fix | architecture | `scripts/serve_demo.py:152-182` | `serve_demo.py` inherits `SLAM_IT_SHOWCASE` from the caller's shell | fix | fixed (30ae2ce0) -- decided by evan-wes, 2026-10-19 |
| A5 | should-fix | architecture | `viewer/src/bundle.js:258-259` | Long-lived showcase bundles meet a viewer that accepts exactly one bundle version | fix | fixed (3293d948: the note and `bundles.VERSION`'s docstring say a bump means re-baking and re-uploading) -- decided by evan-wes, 2026-10-19 |
| A6 | note | architecture | `server/src/slam_it_server/app.py:893` | The bundle files' content type is whatever the uploading client stored | note | noted -- decided by evan-wes, 2026-10-19 |
| A7 | note | architecture | -- | A presigned link that changes on every request defeats the browser cache | note | noted -- decided by evan-wes, 2026-10-19 |
| A8 | note | architecture | `server/src/slam_it_server/app.py:101-113` | `SHOWCASE_FILE` and `SHOWCASE_TYPES` are kept in step by hand | note | resolved by A1 (one page, one type) -- decided by evan-wes, 2026-10-19 |
| A9 | note | architecture | `tests/server/test_showcase.py:134-135` | A listed run without its bundle is a dead link on the home page | note | noted -- decided by evan-wes, 2026-10-19 |
| A10 | note | architecture | `server/src/slam_it_server/app.py` | The showcase as its own module, if it grows | note | noted -- decided by evan-wes, 2026-10-19 |
| A11 | note | architecture | `VERSION`, `CHANGELOG.md` | Merge order | note | resolved: the 0.3.15.0 PR merged first -- decided by evan-wes, 2026-10-19 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope.** `git diff 8c9872bae423633ade8bb33b0b9dbfda5867aa0d...7dc664490f1b34faa91b6923235520c6c4decbaa` (3 commits, 13 files). Read in full: `server/src/slam_it_server/app.py`, `config.py` and `storage.py`, `tests/server/test_showcase.py`, `conftest.py`, `test_config.py`, `test_storage.py` and `test_shared_names.py`, the CHANGELOG entry, `server/settings.template.env` and the notes section; for context `viewer/upload.html`, `viewer/app.css`, `viewer/src/bundle.js`, `viewer/src/main.js`, `viewer/src/jobs.js` and `src/slam_it/orchestration/bundles.py`.

**Summary.** The change is clean and production quality, with no defects found. Route inputs are validated by whole-match regexes; names are validated at startup, with messages that name the bad value; docstrings state their conventions, and the tests assert exact values. No wrong behaviour, security hole, crash or broken contract. The should-fix and note items are small. `tests/server`: 181 passed; `test_showcase.py` has 34 tests (one environment caveat in the notes).

### Findings

#### [should-fix] The showcase page's HTML head is a Python string that copies `upload.html`'s, with no test tying them together
- **Where:** `server/src/slam_it_server/app.py:187` (`_SHOWCASE_PAGE`), against `viewer/upload.html:1-14`
- **Issue:** The `<head>` (viewport, robots meta, the Google Fonts `<link>` pair, `app.css`) is copied from `upload.html`, and the body reuses its classes: the fourth copy of the fonts URL (`index.html`, `status.html` and `upload.html` hold the others) and the only one outside `viewer/`. Prettier and the viewer checks cover the other three but not a Python string, whose `<!-- runs -->` marker sits at column 0. A change to the pages' head (a new font weight, a renamed stylesheet) will not reach this page, and nothing fails.
- **Why it matters:** silent drift between the home page and the pages it sits beside; duplication tooling cannot see it across HTML and Python.
- **Evidence:** the fonts `href` appears at `viewer/upload.html:11` and at `app.py:196-197`.
- **Suggested fix:** keep the template as `viewer/showcase.html` (prettier-formatted; not served, as `PAGES` does not match it) and let the server fill the `<!-- runs -->` marker; or a test in the style of `tests/server/test_shared_names.py` that the `<head>` lines of `showcase_page(...)` equal `upload.html`'s.

#### [should-fix] Loose status assertions in two showcase tests
- **Where:** `tests/server/test_showcase.py:217` and `:228`
- **Issue:** `test_the_showcase_never_reads_the_job_list` asserts `status_code in (200, 307, 404)` for five known paths; `test_staging_puts_the_showcase_behind_a_code` asserts `in (200, 307)`. Each path's status is fixed by the design (`/` and `/upload` 200, `/s/room/` and `/s/room/calibration.html` 307, `/s/other/` 404; in staging, `/` and `/upload` 200 and the `/s/...` paths 307).
- **Why it matters:** a route that regressed from 307 to 404, or from 200 to 307, would still pass; the first test then no longer proves the real handlers ran without touching the job list.
- **Evidence:** `assert client.get(path, follow_redirects=False).status_code in (200, 307, 404)`.
- **Suggested fix:** parametrize or loop over `(path, status)` pairs and assert equality.

#### [should-fix] `showcase` in `_pages` is the page's HTML, and `showcase is None` reads backwards
- **Where:** `server/src/slam_it_server/app.py:907-914`
- **Issue:** the closure variable `showcase` holds `str | None`, the rendered page; `if showcase is None` reads as "no showcase" but stands for "no showcase page". The name also collides with `Settings.showcase` (a tuple of names), `_showcase(app, server)` and the test fixture `showcase`. The literal `{"Cache-Control": "no-cache"}` at `app.py:914` repeats `_Server.page`'s (`app.py:526`).
- **Why it matters:** readability only, but `showcase` already means three things in this change.
- **Suggested fix:** rename the variable to `showcase_html`; optionally one named constant for the header dict shared with `_Server.page`.

#### [note] VERSION jumps 0.3.14.0 to 0.3.16.0, so merge order matters
- **Where:** `VERSION`, `CHANGELOG.md:7`
- **Issue:** `origin/develop` is at 0.3.14.0; 0.3.15.0 exists only on the unmerged local branch `feature/showcase`. This branch's changelog entry sits directly above 0.3.14.0 with no 0.3.15.0 entry, so the two branches conflict at the top of `CHANGELOG.md` and in `VERSION`.
- **Why it matters:** no code dependency; the order is the owner's. If this PR merges first the tag sequence skips 0.3.15.0 until `feature/showcase` lands; if `feature/showcase` merges first this branch needs a rebase and a changelog resolution.
- **Suggested handling:** say which merges first when the PR goes up; suggested `feature/showcase` (0.3.15.0) first.

#### [note] One overlong line in the notes' paragraph edit
- **Where:** `docs/notes/your-own-video.md:146` (136 characters; the rest of the file wraps at about 100)
- **Issue:** the reflow of "The upload page (`/upload`, and `/` unless there is a showcase, below) ..." left the second line unwrapped. Cosmetic.

#### [note] The shared conda env imports the server package from another checkout
- **Where:** environment, not code
- **Issue:** in this worktree, `conda run -n slam-it python -m pytest tests/server` fails at collection (`ImportError: cannot import name 'SHOWCASE_LENGTH' from 'slam_it_server.config'`, resolved to the main checkout's server package): the env's editable install points at the main checkout. With `PYTHONPATH=<worktree>/server/src:<worktree>`: 181 passed. Layer 1 and `scripts/offline_ci.sh` build their own env, so this should not affect the merge, but a layer-1 run from this worktree with the shared env would test the wrong code.

### Checked and fine
- **Route validation:** `RESULT_PATH` and `SHOWCASE_FILE` are matched with `fullmatch`, the name exactly against the settings' tuple, and `SHOWCASE_TYPES[suffix]` cannot raise `KeyError` for any path the regex lets through. The 27 parametrized 404 cases cover escaped and unescaped climbs, absolute paths, case, nested folders and a NUL byte, and assert that no `location` header is sent.
- **Setting validation:** `config._showcase` refuses a malformed, over-long or repeated name at startup with a message naming the value; empty entries are skipped, tested; `SHOWCASE_LENGTH` and `SHOWCASE_NAME` are named and justified.
- **No debug prints, swallowed errors or stray TODOs.** `del content_type` in `FolderStorage.download_link` is explained in its docstring and by the module docstring, and a test covers it.
- **Docs coverage:** docstrings carry the conventions (what each prefix holds, which file kinds are served, escaping, who can reach what); the module docstrings of `app.py` and `config.py` and the `Storage` protocol docstring are updated, as are the settings template, the notes ("The showcase" section) and the changelog.
- **Changelog counts:** 181 tests, 34 in `test_showcase.py`, 27 in the 404 parametrization, 13 added in `test_config.py` (1 + 12), all consistent with what was collected and run.
- **Page relative paths:** the moved upload page's relative references (`app.css`, `src/upload_view.js`) resolve at `/upload`, and its API calls are absolute (`/api/kept`, `/api/jobs`).
- **Bundle paths:** every file name and folder a bundle writes (`bundles.py`: `cpp/`, `rust/`, `python/`, `images/`, `*.npy`, `records.json`, `chunk_NNNN.bin`) matches `RESULT_PATH`.
- **Refactors:** `_stored` is a single helper used by both job results and the showcase; moving `make_s3` and `BUCKET` to `tests/server/conftest.py` is a clean extraction, with `test_storage.py` updated.
- **Shared names:** the `VIEWER_PAGE` / `viewerLink` coupling is pinned by `test_shared_names.py`, and the escaping equivalence on `[a-z0-9-]` names holds.
- **Resources:** each file link is made per request, with no caching of expiring links; a test shows a new link after the first expires.

### Architecture review (with a security pass)

**Scope.** `git diff 8c9872ba...7dc66449` on `feature/showcase-route` (13 files: `server/src/slam_it_server/{app,config,storage}.py`, `server/settings.template.env`, `tests/server/*`, `docs/notes/your-own-video.md`, `CHANGELOG.md`, `VERSION`). Neighbouring code read: all of `app.py`, `storage.py`, `__main__.py`, `viewer/src/bundle.js`, `viewer/src/jobs.js`, `scripts/serve_demo.py`, `tests/test_serve_demo.py`, the server's import-linter contract, `docs/design/2026-10-09-run-viewer.md` (sections 4, 5.3, 5.4, 6, 9) and the ladder's "Public presentation". Two probes outside the repo: path probes against the app through raw ASGI with S3 storage signed offline, and boto3 1.43.111's addressing style for custom endpoints.

**Summary.** The showcase routes are sound where it matters most: every key they can produce is `showcase/<listed name>/<regex-gated path>`, checked again by `check_key`; they never touch the job list, never list storage, never redirect off-site except to a link the server signed itself, and leave the upload gate and staging's Basic auth exactly as they were. No must-fix. Five should-fixes: the calibration page is served from storage's origin, outside the site's headers, and nothing in the repo produces it or links to it; the new home page is an HTML template inside `app.py`, against the "pages live in `viewer/`" decision; the design doc doesn't record the showcase, and its own "showcase page" means something else; the demo script now inherits `SLAM_IT_SHOWCASE` from the caller's shell; long-lived showcase bundles meet a viewer that accepts exactly one bundle version.

### Findings

#### [should-fix] The calibration page and its images are served from storage's origin, outside the site's headers, with no producer or link
- **Where:** `server/src/slam_it_server/app.py:101-113` (`SHOWCASE_FILE`, `SHOWCASE_TYPES`), `:895-902` (`showcase_file`), `server/src/slam_it_server/storage.py:113-120` (`ResponseContentType`), `docs/notes/your-own-video.md:194-202`
- **Issue:** `/s/<name>/calibration.html` redirects to a presigned GET signed with `response-content-type=text/html; charset=utf-8`, so the page renders on the bucket's origin. That response carries none of the site's headers (CSP, `X-Frame-Options`, `nosniff`, `Referrer-Policy`), and S3 cannot attach them. boto3 signs custom endpoints path-style, so that origin is the storage endpoint's host, shared by every bucket behind it. SVG gets the same treatment; opened at top level, an SVG runs script just as the HTML page does. Nothing in the repo or the docs produces `calibration.html` or `calibration/*.png|jpg|svg`, and no page links to them.
- **Why it matters (the security pass's answer):** no hole today: only holders of the bucket's keys can write under `showcase/` (the worker's PUTs are signed for `jobs/<id>/result/<RESULT_PATH>`, `app.py:722-727`; the owner's for `jobs/<id>/upload`, `app.py:674`); a script in a malicious or mistaken page there still cannot read the site's DOM, API answers or storage, and holds no other presigned link. It is still the one route whose output the site cannot constrain: a third-party script or a mistake in that file runs with no policy, on an origin other tenants can also serve HTML from. Visitors land on the storage host, not the site, with a signature that dies after `LINK_SECONDS` (3600 s, `config.py:62`), so a shared or bookmarked calibration link breaks within the hour. The stored file is tied to one deployment (the note requires the page to link its images by the site's whole address, `docs/notes/your-own-video.md:199`). And it is speculative structure: it widens the `Storage` protocol (`download_link(key, content_type)`) for files nothing produces.
- **Evidence:** signed offline with a custom endpoint, the link is `https://storage.railway.app/my-bucket/showcase/room/calibration.html?response-content-type=text%2Fhtml%3B%20charset%3Dutf-8&X-Amz-...`: path-style, origin = the endpoint host. `grep -rn "calibration\.html" src/ scripts/ viewer/ docs/design/` finds nothing.
- **Suggested fix:** drop `/s/<name>/<file>`, `SHOWCASE_FILE`, `SHOWCASE_TYPES` and `download_link`'s `content_type` from this PR. When the calibration page's producer lands, make it a page of the site, in `viewer/` under the CSP, reading its data and images through `/s/<name>/...` redirects as the viewer reads a bundle (needs `img-src` to admit `store.origin`, as `connect-src` already does, `app.py:485-487`). If the owner keeps the route instead: record in the design doc that pages on storage's origin are accepted, and why. Either way, keep `.svg` out.

#### [should-fix] The showcase page is an HTML template in `app.py`, outside `viewer/` and its checks
- **Where:** `server/src/slam_it_server/app.py:187-252` (`_SHOWCASE_PAGE`, `showcase_page`)
- **Issue:** a page now lives as a Python string duplicating `viewer/upload.html`'s head (the fonts' URL, `app.css`, and the classes `sheet`, `bar`, `brand`, `column`, `panel card`, `primary`); this departs from decision 9.4 of the run-viewer doc ("The upload and status pages live in `viewer/`, under the viewer's checks", `docs/design/2026-10-09-run-viewer.md:518`) and from CLAUDE.md's Languages table (`viewer/` holds the pages; the server holds "the job list, storage's links and the guards").
- **Why it matters:** prettier, the viewer's checks and jscpd never see it; a change to the fonts, the stylesheet's classes or the CSP's font sources must be remembered in two languages.
- **Evidence:** `app.py:195-199` repeats `viewer/upload.html:8-13` verbatim.
- **Suggested fix (code, not doc):** move the template to `viewer/showcase.html` with its `<!-- runs -->` marker, kept out of `PAGES`; read it once at startup, as `import_maps` reads `index.html`, and refuse a missing marker with `SettingsError`; fill it with the escaped names as now.

#### [should-fix] The design doc does not record the showcase, and the ladder's "showcase page" names something else
- **Where:** `docs/design/2026-10-02-slam-it-ladder.md:848-856` ("a static showcase page… walks through the SLAM process on one sequence, stage by stage"); `docs/design/2026-10-09-run-viewer.md:368` ("There is no gallery or listing"); `:471` ("the showcase page" out of scope); the PR documents the routes only in `docs/notes/your-own-video.md:174-207`.
- **Issue:** the PR adds a public listing at `/` under the name "showcase", with routes and a storage layout (`showcase/<name>/`); the design doc has none of it, its own "showcase page" is a different artifact, and its guard against a gallery or listing is not qualified.
- **Why it matters:** the design doc is the authority; a reader can't tell whether this listing is the ladder's showcase page, its first step, or a separate thing, and section 5.4 reads as if the listing breaks it.
- **Evidence:** the lines cited; `git diff --stat` shows no change under `docs/design/`.
- **Suggested fix (doc):** a short decisions entry (2026-10-19) in the run-viewer doc covering the routes, the `showcase/<name>/` layout, and that nothing the server does writes there; qualify section 5.4's "no gallery or listing" as concerning uploaded results; say how this relates to the ladder's showcase page (the walkthrough will live here, or the concept gets another name, e.g. "featured runs").

#### [should-fix] `serve_demo.py` now inherits `SLAM_IT_SHOWCASE` from the caller's shell
- **Where:** `scripts/serve_demo.py:152-182` (`environment`), `:276-279`
- **Issue:** the demo pins every behavioural `SLAM_IT_*` setting, then starts the server with `env={**os.environ, **settings}`; `SLAM_IT_SHOWCASE` is the first behavioural setting it doesn't pin. A shell that exported it turns the demo's `/` into a showcase page whose runs would need `showcase/<name>/` in the demo's data folder, which isn't there.
- **Why it matters:** the demo's printed address and QR code would land on dead links; `environment()`'s docstring ("The `SLAM_IT_*` variables") is no longer true.
- **Evidence:** `"SLAM_IT_KEEP_UPLOADS": "1"` is the last key at `scripts/serve_demo.py:181`; no `SLAM_IT_SHOWCASE`.
- **Suggested fix:** pin `"SLAM_IT_SHOWCASE": ""` in `environment()` and assert it in `tests/test_serve_demo.py`; a `--showcase` flag later if a local demo should show baked runs.

#### [should-fix] Long-lived showcase bundles meet a viewer that accepts exactly one bundle version
- **Where:** `viewer/src/bundle.js:258-259`, `src/slam_it/orchestration/bundles.py:93`, `docs/notes/your-own-video.md:174-207`
- **Issue:** the viewer refuses any `bundle.json` whose version isn't `VERSION`. A bundle used to live 7 days; showcase bundles are uploaded by hand and stay indefinitely; the server never reads storage, so nothing checks them, and no doc ties a `VERSION` bump to re-baking them.
- **Why it matters:** the first deploy that bumps the bundle format breaks every public showcase run with "expected slam-it-bundle version N", with no warning beforehand.
- **Evidence:** `if (info.format !== FORMAT || info.version !== VERSION) { throw ... }`
- **Suggested fix:** one sentence in the note's showcase section and in `bundles.VERSION`'s docstring: a bump means re-baking and re-uploading the showcase's runs.

#### [note] The bundle files' content type is whatever the uploading client stored
- **Where:** `server/src/slam_it_server/app.py:893` (and the job route at `:933`). Showcase bundles are put there "with any S3 client", and the bucket answers a top-level navigation with whatever type that client stored; the viewer reads bundles with `fetch`, which ignores the type. Passing `application/octet-stream` for bundle files would let the server name every type a browser receives from the bucket.

#### [note] A presigned link that changes on every request defeats the browser cache
- `X-Amz-Date` changes every second and a 307 isn't cached, so a returning visitor downloads every bundle file again; harmless now (Railway charges nothing for bucket downloads, run-viewer doc section 5.3). If load times matter later, sign within a fixed window.

#### [note] `SHOWCASE_FILE` and `SHOWCASE_TYPES` are kept in step by hand
- **Where:** `server/src/slam_it_server/app.py:101-113`, `:901`. A kind added to the regex without a type in the map becomes a `KeyError` (a 500). Derive the extension alternation from the map's keys, or add a test. Moot if the first finding drops the route.

#### [note] A listed run without its bundle is a dead link on the home page
- Deliberate and tested (`tests/server/test_showcase.py:134-135`). A startup `store.size(f"showcase/{name}/bundle.json")` that logs a warning, without refusing to start, would make the misconfiguration observable without hanging startup on storage.

#### [note] The showcase as its own module, if it grows
- `app.py` is 950 lines and the showcase is a coherent unit; if it gains titles or a calibration page, split out `slam_it_server.showcase` between `app` and `jobs | storage`; the layers contract is exhaustive (`pyproject.toml:448-458`), so the split must be added there.

#### [note] Merge order (hygiene reviewer's lane)
- `VERSION` skips 0.3.15.0, which sits on the unmerged `feature/showcase` / `fix/showcase-review`; the two PRs' code doesn't overlap but both touch `VERSION` and `CHANGELOG.md`. Merge the 0.3.15.0 PR first, so tags and changelog order stay ascending.

### Checked and fine
- **Path traversal and key confusion:** names are validated at startup (`config.py:71-75`, `:164-175`) and matched exactly against the tuple (`app.py:873-877`); the path is gated by `fullmatch` against `RESULT_PATH` or `SHOWCASE_FILE`, which allow no dot-only segment, no leading `/` and no `%`; `check_key` then checks the whole key again (`storage.py:39-43`); the two regexes' extensions are disjoint, so no file is reachable under two content types; the 27 tested paths and raw ASGI probes (`./`, `../room/`, `\x00`, `%252e%252e`, backslash, trailing slash) all answered 404. One Starlette quirk: a raw trailing `\n` is dropped by its route regex's `$` (`/s/room/bundle/bundle.json\n` gives the key `showcase/room/bundle.json`), harmless since the key is a legitimate showcase key. No request can reach `jobs/`.
- **Job tokens:** the showcase routes never touch `server.jobs`; `_NoJobs` in `tests/server/test_showcase.py:205-218` holds this.
- **Bucket listing:** the showcase makes no list call; `delete_prefix` lists only `jobs/<id>/` (`storage.py:145`), so nothing deletes under `showcase/`.
- **Open redirect:** every `Location` is either the same-site `VIEWER_PAGE` plus the quoted `/s/<listed name>/bundle/`, or a link the server signed itself; the viewer's `bundleAddress` refuses another origin (`viewer/src/bundle.js:170-176`).
- **Content policy:** the middleware puts the same CSP and headers on every answer, 307s included (`app.py:589-605`); the home page has no script and no inline style (tested), and its names are validated and escaped; the viewer page's policy is unchanged.
- **Link lifetimes:** each link is a GET for one key, lasting 3600 s, made per request; the response type is part of the SigV4-signed query (tested), so it can't be altered.
- **Startup validation:** a malformed, over-long or repeated name raises `SettingsError`, and the server exits with code 2 (`__main__.py:110-113`).
- **The upload gate:** `/upload` serves the same page; making a job still needs a code (`app.py:656`); `_OPEN` is unchanged; the showcase is behind staging's Basic auth (tested).
- **Folder storage:** every file, the calibration page included, is still served as an attachment (`storage.py:200-207`, `app.py:854-858`), so nothing renders on the site's own origin.
- **Dependency direction:** the PR adds only stdlib imports, never imports `slam_it`, and leaves the server's internal layer order intact.
- **Shared names:** `VIEWER_PAGE` is held equal to `viewerLink` by `tests/server/test_shared_names.py`.
- **Small changes:** `_stored` gives both bundle routes one redirect, and `make_s3` moving to `conftest.py` is sound.
