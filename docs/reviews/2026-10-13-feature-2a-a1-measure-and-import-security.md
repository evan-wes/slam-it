# Security pass: `feature/2a-a1-measure-and-import`

<!-- The security pass on A1's upload changes (a Stray Scanner zip's LiDAR maps, the worker's
RGB-D run), saved beside the panel's report. Later runs append a "Run N" section. -->

## Run 1 -- 2026-10-13

| | |
|---|---|
| **Branch** | `feature/2a-a1-measure-and-import` |
| **Range** | `2aa96fe...a533b13` (base: `origin/develop`) |
| **Reviewer** | security (opus), a general-purpose agent with its own brief: a Stray Scanner zip's LiDAR maps, the worker's RGB-D run and feature count, the video import's new options against the upload path, the status page's warning; the dataset importers if ever pointed at untrusted data |
| **Probes** | P1-P10 below, in the `slam-it` env, the heavy ones memory-capped; no external service contacted; no private data read |
| **Panel** | [the panel's report](2026-10-13-feature-2a-a1-measure-and-import.md), whose IDs (H*, A*, L*) are named where a finding overlaps one of its own |

### Findings and resolutions

| ID | Severity | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|
| S1 | should-fix | `video.py:610-619`; `images.py:56-61` | The map's size check reads the first IHDR, Pillow decodes at the last: about 1.1 GiB per job from a 430 KB zip | fix (`images` refuses a side over a bound after Pillow's header parse, before decoding; maps decoded from bytes; the bomb warning an error; a two-IHDR test) | fixed (84508ce: a two-header map refused at its decoded size; the bomb warning an error) -- decided by evan-wes, 2026-10-13 |
| S2 | note | `images.py:60`; `video.py:618-622` | Pillow's `ValueError`/`SyntaxError` on malformed chunks escape as "an internal error" | fix (caught in `images`; tests for each) | fixed (84508ce) -- decided by evan-wes, 2026-10-13 |
| S3 | note | `video.py:545-547` | Map names accept Unicode digits: `'²'` fails as an internal error | fix (six ASCII digits; a test) | fixed (84508ce) -- decided by evan-wes, 2026-10-13 |
| S4 | note | `video.py:563-566`; `worker.py:572-579, 692-697`; `app.py:280` | An over-cap map's raw name reaches the refusal: forged log lines, and over 10,000 characters the failure report itself fails | fix (the map named by kind and frame; the worker cuts a reason to 500 before posting) | fixed (84508ce: maps named by kind and frame; `worker.REASON_LENGTH`, held to the server's) -- decided by evan-wes, 2026-10-13 |
| S5 | note | `zju.py`, `arkitscenes.py`, `advio.py` | Dataset importers, if ever pointed at untrusted data | fix ZJU's containment (with H4); waive the rest: local, trusted datasets | fixed in part (84508ce: ZJU's names kept in its images folder); waived the rest: local, trusted datasets -- decided by evan-wes, 2026-10-13 |

### Security review

## Security review: feature/2a-a1-measure-and-import (2aa96fe...a533b13)

**Summary.** The new LiDAR path mostly holds up:
- Map names never become paths.
- Each map is capped at 1 MiB, declared and streamed.
- Duplicates across folder levels are refused.
- The scratch file is the import's own and is removed on every failure path I tried.
- Total work and disk across the 2,401 frames an upload can keep stay small. The worst I measured was 41 s and 0.58 GB of depth for a full 500 MB zip.

The upload path's defaults are unchanged, and nothing from the dataset path (Limits, clock, RadTan, quarter_turns) is reachable from an upload. The status page's new warning is set with `textContent`. There are no must-fixes.

The one real gap is the guard this PR advertises. The 1024-a-side check reads only the **first** IHDR chunk, and Pillow 12.3 decodes at the **last** one. So a map can be decoded at up to Pillow's own ceiling of about 179M pixels: 1.1 GiB of memory per job from a 430 KB zip, or a MemoryError at the worker's 6 GiB address-space limit. The damage is bounded to one pair of maps per job, because `registered_depth` then refuses the size. Three notes cover:
- Pillow exceptions that escape as "an internal error";
- Unicode digits in map names;
- a map's raw zip name echoed into the refusal, which lets an uploader forge server log lines or make the failure report itself fail.

### Findings

**S1 — should-fix — the map size check reads the first IHDR; Pillow decodes the last**
- **Where:** `src/slam_it/eval/video.py:610-619` (`_read_map`); `src/slam_it/eval/images.py:56-61` (`_open`).
- **Issue:** `_read_map` checks `data[16:24]`, the first chunk's width and height, against `STRAY_MAP_SIDE`. Pillow's `PngStream.chunk_IHDR` overwrites `im_size` on every IHDR it meets before IDAT. So a PNG of `IHDR(256x192) IHDR(13370x13370) IDAT(...)` passes the check and is decoded at 13370x13370. The only limit left is Pillow's `MAX_IMAGE_PIXELS`: a warning above 89.5M pixels, an error only above 2x that. `DecompressionBombWarning` is just a warning.
- **Why it matters:** an invite holder gets a ~1 GiB, ~1 s decode per job from a few hundred KB, contrary to the module docstring ("checked ... before it is decoded").
  - With an RGBA confidence map, the child hits `RLIMIT_AS` and the owner sees "an internal error".
  - The cost is bounded to the first kept frame: `registered_depth` refuses any map larger than the frame, and `RLIMIT_AS` holds.
  - That keeps this short of must-fix. It is still the PR's stated guard not holding.
- **Evidence (P2, under RLIMIT_AS 6 GiB, MemoryMax 4G):**
  - Depth map 328,623 bytes (zip 430,270): first IHDR 256x192, second 13000x13000, 16-bit. Result: `InvalidInputError: ... depth (13000, 13000) and confidence (192, 256)`. 0.68 s; maxrss 145 → 1071 MiB; `DecompressionBombWarning: Image size (169000000 pixels)`.
  - Both maps 13370x13370, gray (zip 622,630): 1.14 s, maxrss 156 → 1129 MiB, two bomb warnings, refused on aspect.
  - Depth plus an RGBA confidence map (zip 1,143,861): `MemoryError` after 1.30 s. Through `worker._run_child` (P9) the owner sees "an internal error".
- **Suggested fix:**
  - Check the size Pillow will decode, not the bytes' first chunk. Give `images._open` (and `read_depth`/`read_gray`) an optional `max_side`, and refuse `max(opened.size) > max_side` after `Image.open(...)` and before `opened.load()`. Pillow has parsed every chunk up to IDAT by then, so `size` is the last IHDR.
  - Optionally decode from `io.BytesIO(data)` and drop the scratch file.
  - Add a test: a map with two IHDR chunks (256x192, then 2000x1500) is refused, and no `DecompressionBombWarning` is raised.
  - Defence in depth: turn `DecompressionBombWarning` into an error inside `images`.

**S2 — note — Pillow's parse errors escape `images._open` as "an internal error"**
- **Where:** `images.py:60` (catches only `OSError` and `DecompressionBombError`); `video.py:618-622` (catches only `InvalidInputError`).
- **Issue:** malformed PNG chunks raise `ValueError` or `SyntaxError`, not `OSError`:
  - a truncated IHDR (`ValueError: Truncated IHDR chunk`);
  - an iCCP chunk over `MAX_TEXT_CHUNK`;
  - zTXt chunks over `MAX_TEXT_MEMORY` (64 MiB, which also bounds them);
  - a bad chunk after IDAT, read by `load_end` (`SyntaxError: unknown filter category`).

  The new map path exposes `images` to uploads for the first time.
- **Why it matters:** the owner gets "an internal error" instead of "is not a 16-bit depth map", and the worker logs a full traceback per such upload. This is the class the earlier S18 fixed for the zip itself. Cleanup holds: no staging folder or `.map.png` was left in any case.
- **Evidence:** P3 shows each case raising `ValueError` in about 0.02-0.08 s; P10 shows the `SyntaxError`. P9, through the worker's child: `truncated_ihdr`, `iccp_over_chunk` and `ztxt_over_memory` each report "an internal error (RuntimeError: the job's process ended with 1 and no result)".
- **Suggested fix:** catch `(OSError, ValueError, SyntaxError, Image.DecompressionBombError)` in `images._open`. Add tests for a truncated IHDR, an oversized iCCP and a bad chunk after IDAT.

**S3 — note — map names accept Unicode digits**
- **Where:** `video.py:545-547` (`_stray_map`: `name.stem.isdigit()`, then `int(name.stem)`).
- **Issue:** `str.isdigit()` accepts `'²'`, which `int()` rejects. It also accepts Arabic-Indic and full-width digits, which `int()` reads, so they stand in for ASCII frame numbers. A cp437 name byte `0xFD` decodes to `'²'`, so this happens without the UTF-8 flag too.
- **Why it matters:** any zip containing such an entry, even a monocular one, fails as "an internal error" with a traceback. `depth/٠٠٠٠٠١.png` aliases frame 1. That is harmless (duplicates are refused), but not what the docstring's "six digits" means.
- **Evidence:**
  - P3 `unicode_digit_name`: `ValueError: invalid literal for int() with base 10: '00000²'`.
  - P3 `arabic_digit_alias`: `InvalidInputError: zip: depth/000001.png appears twice`.
  - P9: the first reaches the owner as "an internal error".
- **Suggested fix:** `re.fullmatch(r"[0-9]{6}", name.stem)` (or `stem.isascii() and stem.isdigit()`), and a test.

**S4 — note — an over-cap map's raw zip name goes into the refusal: forged log lines, and a lost failure report**
- **Where:** `video.py:563-566` (`f"zip: {info.filename} is over its cap ..."`, no `!r`). The other ends are `worker.py:572-579, 692-697` and `server/src/slam_it_server/app.py:280, 691-692` (`Failed.reason` has `max_length=10_000`, and the server logs `"job %s: failed (%s)"`).
- **Issue:** a map name is `[folder/]depth/NNNNNN.png`, and the folder part is free text up to about 64 KB, newlines included.
  - **A newline** reaches the server's log raw: an uploader can write a forged line, cut to 500 characters.
  - **Over 10,000 characters**, the server refuses the `failed` report with 422. The worker logs that and moves on, and the job sits "importing" until `SILENCE` (120 s), then ends as "the worker stopped answering".
  - The existing message at `video.py:557` uses `!r` (escapes newlines) but has the same length problem; that part is pre-existing.
- **Evidence:**
  - P8: the refusal is `'zip: rec\nINFO:slam_it_server.app:job 0000: done (forged)/depth/000000.png is over its cap of 1048576'` (it contains a newline). A 12,000-character folder gives a 12,049-character reason.
  - Server model check: `Failed(reason='a'*12049)` gives `string_too_long`; 9,000 is accepted.
- **Suggested fix:**
  - Name the map as the other map messages do: `f"zip: {kind}/{frame:06d}.png is over its cap ..."`.
  - Have the worker cut a reason to the server's `REASON_LENGTH` (500) before posting, so no upload can make the failure report fail.

**S5 — note — dataset importers: matters only if one is ever pointed at untrusted data**
- **`zju.py:212, 224`:** `images / row[1]` takes the path from `data.csv`, so an absolute path or `..` symlinks any readable file into `rgb/`. Only `is_file()` is checked. Its frames would then be read and could be baked into a bundle. Fix if ever needed: require `image.resolve()` inside the images folder.
- **`zju.py:115-124`:** `_image_size` reads the first IHDR only, the same pattern as S1. Here it only sets `camera.txt`'s size.
- **`arkitscenes.py:170-171`:** maps are decoded with no size bound beyond Pillow's 179M pixels, and the capture's own symlinks are followed (`:164`).
- **`advio.py:243`:** `Limits(600 s, ...)` and a caller-chosen `--short-side` would lift the upload's 60 s and 480-pixel caps.
- **`sensor_yaml`:** reads with regular expressions only, no YAML loader. **`measure_testset.py`:** uses `tomllib` on a local config and runs no shell.

### Checked and fine
- **Per-map byte caps:**
  - The declared `file_size` is checked at listing.
  - `ZipExtFile` stops at the declared size, and the streaming cap re-checks it.
  - A map's buffer is at most 2 MiB.
  - A 1x1 map whose IDAT (978 KB) inflates to about 1 GB takes 0.03 s: Pillow stops once the image is full.
- **Total work and disk across the frame cap** (2,401 kept frames: 60 s at 1/40 s; 4,802 maps):
  - Random 1023x341 maps at a 1440x480 frame: 57 ms and 811 KB written per frame. Only 714 frames' maps fit a 500 MB zip, so about 41 s and 0.58 GB in all (P4).
  - Maps padded to exactly 1 MiB, 2.6 KB zipped: 7.7 ms per frame, 19 s for all 2,401 frames (P7).
  - The worst compressible pattern I found (xor) amplifies 17.9x but totals 0.10 GB (P4b).
  - All of this is well under the RGB frames' existing amplification and the per-file `RLIMIT_FSIZE`.
- **Listing:** 1M map-named entries (a 113 MB zip) cost `ZipFile()` 2.29 s plus `_stray_entries` 2.12 s at 737 MiB, against 1.22 s and 666 MiB for 1M other names (P6). Map keys are bounded at 2M (six digits, two kinds). The rest of the cost is the existing `ZipFile` parse.
- **Scratch file:** it is `.map.png` in a freshly made `.<name>.<uuid>.importing` folder that holds only the import's own files. Its name never comes from the zip, so nothing can plant a symlink there without the worker's uid. `finally` unlinks it, and `_staged` removes the folder on any failure. After P2, P3 and P8, including the MemoryError and ValueError cases, no `.importing` folder or `.map.png` was left.
- **Map path handling:**
  - Absolute names, `..` and `\` are refused before map parsing.
  - Duplicates across folder levels are refused by the `(kind, frame)` key.
  - Directories are skipped, and only kept frames' maps are opened.
  - Decode errors name the entry, not the scratch path.
- **Modes:**
  - Depth must be 16-bit single-channel (LA, 16-bit RGB and palette maps are refused after decode), and its values are range-checked.
  - Confidence must be 8-bit; a palette is expanded.
  - The existing tests cover the wrong-bit-depth cases.
- **Worker (item 2):**
  - `depth.txt` exists only if the import wrote it into its fresh folder, and the mode is only `rgbd` or `monocular`.
  - `n_features = features_for(size)` is bounded to [1000, 2000]: upload frames are at most 1440x480, which gives 2250, capped at 2000.
  - `camera.txt` is the import's own output.
- **Extreme fx (`bf = 0.08 fx`) through the C++ RGB-D run (P5):**
  - fx = 1e-300: `InvalidInputError: xyz: expected finite points` in 0.26 s.
  - fx = 1e300 and 1.7e308: the run ends after 6 frames with 1 tracked, in about 0.35 s.
  - No hang and no memory growth. The Python and C++ readers both refuse a non-finite or non-positive `bf`.
- **Item 3:**
  - The worker calls `import_video(upload, seq, chosen)` and `import_stray(upload, seq)` without `limits` or `clock`.
  - `UPLOAD = Limits()` is (60.0, 14400, 480), equal to the old constants, and `_decode`'s frame and time caps are unchanged.
  - `probe`'s long-side and aspect caps and the mov-only demuxer are unchanged, and the stray video is still decoded through `_write_frames` with the defaults.
  - `RadTan` is built only in `advio.read_calibration`; `lenses.choose` returns only a profile, 35mm or nominal Pinhole or Fisheye.
  - `quarter_turns` is called only by advio, and `--min-confidence` is CLI-only.
  - `source.json`'s `**read` holds only `metadata` and `depth`.
- **Item 4:**
  - `sizeWarning` passes `Number()` values into a template string, set with `textContent`.
  - `MODES` and `INTRINSICS` values are also set with `textContent`, and nothing in `viewer/src` or the pages uses innerHTML or any other HTML sink.
  - `mode` and `size` come from the worker, not from upload strings, and the server caps a summary at 4096 bytes.

### Probes run
All ran in the slam-it env with `python -I` from `/tmp/secprobe`, the heavy ones under `systemd-run --user --scope -p MemoryMax=4G -p MemorySwapMax=1G`. Scripts are in `/tmp/secprobe/scripts/`.
- **P1:** baseline 4-frame RGB-D Stray import.
- **P2:** double-IHDR maps under `RLIMIT_AS` 6 GiB: depth 13000²; both 13370² gray; depth plus RGBA confidence.
- **P3:** truncated IHDR, iCCP over `MAX_TEXT_CHUNK`, 70 zTXt chunks over `MAX_TEXT_MEMORY`, a `'²'` name, an Arabic-Indic alias, and a 1x1 IDAT over-inflating to about 1 GB.
- **P4 and P4b:** per-frame time and bytes for a random max-size map, and for zeros, gradient, xor and LCG patterns.
- **P5:** extreme fx through `import_stray`, `worker.options_for` and `vo.run(..., "rgbd", backend="cpp")` on 30 room frames.
- **P6:** listing cost of 1M map-named entries against 1M other names.
- **P7:** maps padded to exactly 1 MiB.
- **P8:** an over-cap map name with a newline, and one of 12,000 characters.
- **Server model check:** `Failed` validation at 9,000 and 12,049 characters.
- **P9:** the hostile zips through `worker._run_child` (the real child, its rlimits and minimal environment).
- **P10:** a bad IHDR after IDAT, read with `images.read_depth`.

### Housekeeping
- **Repo:** untouched; `git status` is clean and HEAD is still a533b13. Importing slam_it triggers the editable install's rebuild check, which re-ran `cmake --install` into the git-ignored `build/slam-it` ("ninja: no work to do").
- **External services:** none contacted.
- **Private data:** the private datasets were not read.
- **Probe data:** about 252 MB, mostly P6's two 1M-entry zips, is left under `/tmp/secprobe/data/`. It is safe to delete.

## Run 2 -- 2026-10-13

| | |
|---|---|
| **Branch** | `feature/2a-a1-measure-and-import` |
| **Range** | `a533b13...c832973` (the fixes of run 1's findings) |
| **Reviewer** | security (opus), a general-purpose agent: run 1's probes rerun against the fixes; what the fixes introduced |
| **Probes** | P2, P3, P8, P9, P10 rerun; P11-P13 new; no external service contacted; no private data read |

S1-S4 and S5's ZJU part hold. One new finding, S6, came from the fix of S1 itself.

### Findings and resolutions

| ID | Severity | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|
| S6 | should-fix | `images.py:81-91`; `video.py:603-606` | With the hand-written PNG check gone, a map is decoded as whatever its content is: TIFF, JPEG, and EPS through Ghostscript (on the workstation; not in the worker's image) | fix (`images` opens PNG alone; a test with EPS and TIFF maps) | fixed (73a0195) -- decided by evan-wes, 2026-10-13 |

### Security review

## Security review (run 2): feature/2a-a1-measure-and-import, fixes a533b13...c832973

**Summary.** The S1-S4 fixes and S5's ZJU part all hold: every probe that previously demonstrated a problem now returns a clean `InvalidInputError` (or, for the Unicode/Arabic frame names, a benign ignore), with no runaway memory, no scratch file, no forged-log-line or report-breaking reason. Re-ran P2 (double-IHDR under RLIMIT_AS), P3, P8, P9 (through `worker._run_child`) and P10; all confirm the resolutions. The one thing the fixes introduced is a genuine surface-expansion: `images._opened` now does `Image.open` on the map's **bytes** with no format restriction, and Pillow identifies by content, so the removed hand-written PNG-signature check means a zip entry named `depth/NNNNNN.png` is decoded as whatever format its bytes actually are — TIFF, JPEG, BMP, GIF, WebP, JPEG2000, ICO/ICNS, and crucially **EPS, which invokes an external Ghostscript process on upload-controlled PostScript**. That is S6 (one new finding, should-fix). The size guard, byte caps, staging, warnings-filter scope, and the reason cut are all sound; the import path is single-threaded so `warnings.catch_warnings` thread-safety does not apply here.

---

### S1 — map size check reads first IHDR; Pillow decodes the last — FIX HOLDS
`images._opened` (`src/slam_it/eval/images.py:68-99`) now reads `opened.size` after `Image.open` has parsed every header chunk up to IDAT, and refuses `max(width,height) > max_side` before `opened.load()`. `_read_map` (`video.py:589-608`) passes `STRAY_MAP_SIDE` (1024) and decodes from bytes in memory — the scratch file and the hand-written `data[16:24]` check are gone.
Evidence (P2, RLIMIT_AS 6 GiB, MemoryMax 4G):
- depth first-IHDR 256x192 then 13000x13000 (zip 430 KB): `InvalidInputError: zip: depth/000000.png: image: not a readable image (Image size (169000000 pixels) exceeds limit of 89478485 pixels ... decompression bomb ...)`, 0.02 s, maxrss 157→157 MiB.
- both maps 13370x13370 (zip 622 KB): same refusal, 0.02 s, maxrss 155→155 MiB.
- depth + RGBA-confidence 13000x13000 (zip 1.09 MB): same refusal, maxrss 148→148 MiB (the old MemoryError is gone).
- 9000x9000, below the bomb ceiling (zip 574 KB): `InvalidInputError: image: 9000x9000, over 1024 pixels a side` — the max_side guard itself fires.
- 2000x1500 (zip 108 KB): `image: 2000x1500, over 1024 pixels a side`.
No decode occurs in any case (maxrss flat ~150 MiB vs the old 1071-1129 MiB); no `.importing` folder or `.map.png` left behind.

### S2 — Pillow's ValueError/SyntaxError escaping as "an internal error" — FIX HOLDS
`_PILLOW_ERRORS = (OSError, ValueError, SyntaxError, Image.DecompressionBombError, Image.DecompressionBombWarning)` caught in `_opened` (`images.py:55-99`); `UnidentifiedImageError` named separately. The bomb warning is turned into an error inside a scoped `warnings.catch_warnings()` (`images.py:83-84`).
Evidence (P3): `truncated_ihdr` → `InvalidInputError: ... (Truncated IHDR chunk)`; `iccp_over_chunk` → `... (Decompressed data too large for ... MAX_TEXT_CHUNK)`; `ztxt_over_memory` → `... (Too much memory used in text chunks: 68157440>MAX_TEXT_MEMORY)`; P10 bad-chunk-after-IDAT via `images.read_depth` → `InvalidInputError: ... (unknown filter category)`. P9 through the real child: each reaches the owner as a proper `reason:` string, no longer "an internal error".

### S3 — Unicode digits in map names — FIX HOLDS
`_FRAME_NUMBER = re.compile(r"[0-9]{6}")` with `.fullmatch` in `_stray_map` (`video.py:532-535`). A non-ASCII-digit name no longer matches, so the entry is simply ignored rather than raising.
Evidence (P3): `unicode_digit_name` (`00000²`) → **IMPORTED** (entry ignored, import proceeds on the real maps); `arabic_digit_alias` (`٠٠٠٠٠١`) → **IMPORTED** (no alias, no error). Previously both raised an internal error. P9: `unicode_digit_name/z.zip` → `done`.

### S4 — raw zip name in the refusal (log forgery / report-breaking length) — FIX HOLDS
Over-cap message now `f"zip: {kind}/{frame:06d}.png is over its cap of ..."` (`video.py:550-557`), and `Client.failed` posts `reason[:REASON_LENGTH]` with `REASON_LENGTH=500` (`worker.py:136-139, 381-383`); `test_shared_names.py:52-54` asserts `worker.REASON_LENGTH == app.REASON_LENGTH`, and the server also stores `request.reason[:REASON_LENGTH]` (`app.py:691`).
Evidence (P8): both the newline folder and the 12,000-char folder produce exactly `'zip: depth/000000.png is over its cap of 1048576'` (48 chars, no newline). Multi-byte safety: `reason[:500]` slices by code point (never splits a character) and 500 code points ≤ the model's `max_length=10_000`, so the `Failed` report can no longer be refused. `test_worker.py:692-695` confirms a `2*REASON_LENGTH` reason is cut to 500 on the wire.

### S5 (ZJU part) — FIX HOLDS
`zju.py:121` now takes the image size from `images.size(first)` (which opens and parses the full header, no first-IHDR shortcut), replacing the old hand-rolled parser (H2/A4). The ZJU `data.csv` path containment (H4/S5) was also added (refusing absolute / foldered / `..` names). The rest of S5 (arkitscenes/advio untrusted-data notes) was waived as local trusted data — unchanged and out of scope for this run.

---

### New findings

**S6 — should-fix — map decoding is no longer restricted to PNG; EPS content invokes Ghostscript**
- **Location:** `src/slam_it/eval/images.py:81-91` (`_opened`: `Image.open(stream)` with no `formats=` restriction); reached from `src/slam_it/eval/video.py:603-606` (`_read_map` → `read_depth`/`read_gray`). The old `_read_map` refused anything whose first 8 bytes were not the PNG signature (`data[:8] != b"\x89PNG\r\n\x1a\n"`); that check was deleted with the dimension parse in 84508ce.
- **Issue:** `Image.open` identifies by content, not by the `.png` name the entry passed `_stray_entries`. So a zip entry `depth/NNNNNN.png` / `confidence/NNNNNN.png` whose bytes are any Pillow-supported format is decoded as that format. The sharp case is **EPS**: a map beginning `%!PS` is identified as EPS, and `read_gray`/`read_depth` calling `.load()` runs `gs` (Ghostscript) as a subprocess on the uploader's PostScript.
- **Why it matters:** Ghostscript is a large external interpreter with a history of `-dSAFER` bypasses and RCE. Running it at all on upload bytes is exactly the surface the PNG-signature check used to exclude. Scope and mitigations:
  - **Deployed worker is NOT exposed:** `docker/worker.Dockerfile` installs only `tini` + the wheel at runtime; no ghostscript apt package, so `has_ghostscript()` is False and EPS `.load()` raises `OSError` → caught as `InvalidInputError`. (Verified by reading the Dockerfile; the EpsImagePlugin raises "Unable to locate Ghostscript on paths".)
  - **Local/dev exposure is real:** `scripts/serve_demo.py --lan` accepts phone uploads over Wi-Fi gated only by an invite code, and the design doc anticipates a GPU worker run on "the workstation." A dev box has `gs` (this one: /usr/bin/gs 10.02.1), so an invite holder can make the worker child spawn `gs` on arbitrary PostScript.
  - Blast radius is bounded: the child runs with RLIMIT_AS 6 GiB, RLIMIT_FSIZE 2 GiB, a 20-min timeout, a minimal env, its own session killed as a group; the map input is capped at 1 MiB (`STRAY_MAPS`); Pillow renders with `-dSAFER`; and an EPS whose BoundingBox exceeds the max_side/bomb limits is refused before `gs` runs (P13b: a 50000x50000 bbox is refused at open; only a bbox ≤1024/side reaches `gs`, so the raster is bounded — but the PostScript program itself still executes).
- **Evidence:**
  - P13: `read_gray(<EPS bytes>, 1024)` → `ACCEPTED (48,64)` with "gs on PATH: True" — Ghostscript ran. End-to-end `import_stray` with EPS depth maps → rejected afterward (`expected a 16-bit single-channel image, got Pillow mode L`) **but only after gs had already rendered**; an EPS *confidence* map (read via `read_gray`, which accepts mode L) would be accepted outright.
  - P12a: a Stray zip with **TIFF depth + JPEG confidence** maps imported successfully (`depth.txt` written) — confirming non-PNG raster formats are silently accepted (a JPEG "depth" map is lossy garbage; a robustness bug beyond the EPS security point).
  - P11: from bytes at side 1024, `read_gray` accepts JPEG/BMP/GIF/TIFF/WebP/ICO/PGM/TGA/SGI/QOI/JPEG2000/PNG; `read_depth` accepts 16-bit TIFF and 16-bit PNG. P12b: ICO/ICNS containers wrapping an oversized embedded PNG are size-checked correctly (refused at 6000x6000 by max_side / allowed-size rules), so container nesting does not bypass the guard.
- **Suggested fix:** restrict the decoder to PNG, the only format maps are ever supposed to be. Either pass `formats=["PNG"]` to `Image.open` in `images._opened` (every real caller — the maps, TUM/ARKit/ZJU reads, vo replay — reads PNGs, so this is safe project-wide), or, if `images.py` is meant to stay a general reader, restore a cheap PNG-signature check on `data` in `video._read_map` before calling the readers. The former also closes the TIFF/JPEG-accepted-as-a-map robustness gap in one line. A test: an EPS and a TIFF named `.png` as a stray map are both refused.

---

### Checked and fine
- **Size guard and byte caps:** the decoded size (last IHDR / format-native size) is checked against `STRAY_MAP_SIDE` before `opened.load()`; map input is read through `_read_entry(..., STRAY_MAPS[kind])` so bytes are ≤ 1 MiB before any decode; the bomb warning is an error and the hard bomb error is caught. No decode runs for an over-size or over-pixel map (P2 maxrss flat).
- **`warnings.catch_warnings` scope / thread-safety:** the filter mutation is confined to `images._opened` with a `with` block. The map decode runs inside `import_stray` in the worker's single-threaded child (the `_limit_resources` preexec comment confirms "the worker runs no threads"), before `vo.run` starts any C++/ORB-SLAM threads; no Python thread calls `read_gray`/`read_depth` concurrently, so the known non-thread-safety of `catch_warnings` cannot bite here. (All call sites in `vo.py`, `tum.py`, `zju.py`, `arkitscenes.py` are sequential.)
- **`tum.staged` (moved from `video._staged`, `tum.py:308-349`):** logic is byte-for-byte the same; folder `.<out.name>.<uuid>.importing` created with `.mkdir()` (fails if present), `staging.replace(target)` is an atomic rename onto a verified-new/empty `target`, `shutil.rmtree(staging)` on failure removes only the import's own fresh folder. The hidden name comes from `out.name` (for uploads, the fixed `"your-video"`), never the zip. No scratch file now (maps decoded in memory), so nothing in the staging folder is attacker-named; no symlink or TOCTOU avenue.
- **`tum.write_source` (`tum.py:352-362`):** writes `source.json` via `json.dumps`; the record is built by the importer (kind, frame counts, ffprobe metadata), properly escaped; no zip-entry text flows into it.
- **Reason cut:** the child prints the reason as a `json.dumps` line (newlines escaped, stays one line); `_message` parses it back; `Client.failed` cuts to 500 before posting; the server cuts again. No upload-shaped reason can forge a log line or break the `failed` report.
- **Frame-number regex** `[0-9]{6}` fullmatch: exactly six ASCII digits; non-matching names are ignored, not errored (P3).
- **`images.size`** (used by ZJU) opens the full header via `_opened(source, None)`; `max_side=None` means it never refuses on size, which is correct for its trusted-dataset use.

### Probes run
In the `slam-it` env with `python -I` from `/tmp/secprobe`, heavy ones under `systemd-run --user --scope -p MemoryMax=4G/8G -p MemorySwapMax=1G`; no external service contacted; no private data read.
- **P2** (rerun): double-IHDR maps 13000², 13370² gray, 13000² with RGBA confidence, plus 9000² and 2000x1500 to exercise the max_side branch.
- **P3** (rerun): truncated IHDR, iCCP over MAX_TEXT_CHUNK, 70 zTXt over MAX_TEXT_MEMORY, `²` name, Arabic-Indic alias, 1x1 over-inflating IDAT.
- **P8** (rerun): over-cap map name with a newline, and 12,000 characters.
- **P9** (rerun): the hostile zips through `worker._run_child` (real child, rlimits, minimal env).
- **P10** (rerun): bad chunk after IDAT via `images.read_depth`.
- **P11 (new):** which image formats `read_gray`/`read_depth` accept from bytes at side 1024.
- **P12 (new):** (a) TIFF-depth + JPEG-confidence stray zip through `import_stray`; (b) ICO/ICNS containers wrapping an oversized embedded PNG — size-check ordering.
- **P13 / P13b (new):** an EPS map (name `.png`, content `%!PS`) through `read_gray` and `import_stray`; and a large-BoundingBox EPS to confirm size-bounding before Ghostscript.
- **test_images.py:** ran the repo's own module, 32 passed (exit 0), corroborating the S1/S2 fix tests.

### Housekeeping
Repo untouched; read-only throughout; no git state moved. New probe scripts under `/tmp/secprobe/run2/scripts/`, data under `/tmp/secprobe/run2/data/` (safe to delete). No external services contacted; private datasets not read.

### Recommendation
One should-fix (S6) to decide: restrict map decoding to PNG (`formats=["PNG"]` in `images._opened`, or a PNG-signature check in `video._read_map`). It closes the EPS→Ghostscript surface and the TIFF/JPEG-accepted-as-a-map robustness gap. The deployed worker is not currently exposed (no `gs` in its image); the local/dev demo on a box with `gs` is. S1-S5 are confirmed resolved.
