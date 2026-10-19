# Review panel: `feature/showcase`

## Run 1 -- 2026-10-19

| | |
|---|---|
| **Branch** | `feature/showcase` |
| **Range** | `8c9872ba...6fe202df` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet); a front-end PR, reviewed by the hygiene reviewer alone (owner's choice) |
| **How they ran** | registered slam-it-* agents |
| **Layer 1** | at `6fe202df`: every check passed (pytest 4,871 passed, 1 skipped: the dashcam overlay's OCR test, the env having no Tesseract) except jscpd, which found one clone already on develop (`tests/edge/test_pipeline_threads.py`, from B3b's review fix H1); `8072c889` carries B4's fix for it (cherry-picked from `037dd361`), after which jscpd finds 0 clones |

The owner looked at the viewer on a real drive bundle before the review and approved its
behaviour; the review was asked about code quality, robustness and tests.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | should-fix | hygiene | `viewer/src/loop_view.js:261-266` | A failed inset decode is never retried for the same earlier frame | fix | fixed (9f504b62) -- decided by evan-wes, 2026-10-19 |
| H2 | should-fix | hygiene | `viewer/src/loops.js:147-158` | A second binary search beside `frames.js`'s `lowerBound` | fix | fixed (86aafd24) -- decided by evan-wes, 2026-10-19 |
| H3 | should-fix | hygiene | `viewer/tests/loops.test.js:26,66-67` | The loop tests pin literals to a golden record regenerated whenever decisions change | fix | fixed (1906f0bb: expectations from the record's own columns) -- decided by evan-wes, 2026-10-19 |
| H4 | should-fix | hygiene | `viewer/src/loops.js:174-222` | `insets` is one long function and `before` is not a good name | fix | fixed (df126891: `candidateRows`, `nearestRow`, `cutoff`) -- decided by evan-wes, 2026-10-19 |
| H5 | should-fix | hygiene | `viewer/tests/loops.test.js` | The "no loops" path and an empty inset window are not tested | fix | fixed (1906f0bb) -- decided by evan-wes, 2026-10-19 |
| H6 | note | hygiene | `viewer/src/solver_view.js:136` | Per-frame work that does not change per frame in the Solver panel | note | noted -- decided by evan-wes, 2026-10-19 |
| H7 | note | hygiene | `viewer/src/loop_view.js:137-144` | Tick placement and the timeline's height have two owners | note | noted -- decided by evan-wes, 2026-10-19 |
| H8 | note | hygiene | `viewer/index.html:143` | The inset's `aria-label` replaces its live label | note | noted -- decided by evan-wes, 2026-10-19 |
| H9 | note | hygiene | several | Small literals, names and doc nits | note | noted -- decided by evan-wes, 2026-10-19 |
| H10 | note | hygiene | `viewer/src/loops.js` | `loops.js` holds four concerns; the Plot's label arithmetic cannot be tested | note | noted -- decided by evan-wes, 2026-10-19 |
| H11 | note | hygiene | `docs/design/2026-10-09-run-viewer.md:54-58` | The run-viewer design doc does not mention the new controls | note | deferred (a docs-only update on develop after this PR) -- decided by evan-wes, 2026-10-19 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope.** `git diff 8c9872bae423633ade8bb33b0b9dbfda5867aa0d...6fe202df771e8f41b5e02167e4f2640c26370944` (feature/showcase, v0.3.15.0). All of the changed `viewer/` source, tests, `index.html` and `app.css`, `CHANGELOG.md` and `VERSION` were read in full. `node --test viewer/tests/*.test.js`: 82/82 pass. Probe scripts, kept outside the repo, ran against `axis.js` and `loops.js`.

**Summary.** The code is clean and well documented, and robust on the paths listed. The pure logic sits in Node-testable modules, and the tests assert on values. No real defect, so no must-fix findings. The should-fix items are one small failure-path bug (a failed inset decode is never retried), one duplicated binary search, one fragile test coupling, one long function, and two test gaps. The rest are notes.

### Findings

#### [should-fix] A failed inset decode is never retried for the same earlier frame
- **Where:** `viewer/src/loop_view.js:261-266` (`showInset`)
- **Issue:** `this.drawn = earlier` is set before `await this.images.image(earlier)`. If the await rejects, `drawn` stays `earlier`, so every later call with the same `earlier` takes `if (earlier === this.drawn) return;` and never draws.
- **Why it matters:** `FrameImages.image` and `ChunkCache` were built to retry (the failed promise is deleted from `decoded`, a failed chunk fetch is "tried again"), and `ImageView.show` retries because it has no such guard. Here the retry is defeated while the playhead rests inside a window: the inset keeps the previous bitmap under the new label until `earlier` changes. The error is reported once through `report`.
- **Evidence:** lines 261-266 (`if (earlier === this.drawn) { return; } this.drawn = earlier; ... const bitmap = await this.images.image(earlier);`).
- **Suggested fix:** wrap the load in a try/catch that resets `this.drawn = -1` (if still `earlier`) and rethrows.

#### [should-fix] A second binary search beside `lowerBound`
- **Where:** `viewer/src/loops.js:147-158` (`firstFrom`) and `:264-266` (`cycleRows`), against `viewer/src/frames.js:51-62` (`lowerBound`) and `:110` (`posesUpTo`)
- **Issue:** `firstFrom` is the same lower-bound search as `lowerBound`, with an `after` flag; `cycleRows` computes `firstFrom(poseFrames, loop.frame, true)`, which is `posesUpTo(poseFrames, loop.frame)`.
- **Why it matters:** one search written two ways in one language; jscpd cannot see it because the shapes differ.
- **Evidence:** `loops.js:149-156` against `frames.js:53-60`.
- **Suggested fix:** export one `lowerBound` over an `ArrayLike<number>` with a stride (or without one) from `frames.js`; build `firstFrom`, and `cycleRows`'s end via `posesUpTo`, on it.

#### [should-fix] The loop tests pin literals to a golden record that is regenerated whenever decisions change
- **Where:** `viewer/tests/loops.test.js:26` and `:66-67` (frame 261 with keyframes 0 and 28; frame 260 with keyframes 2 and 30 and `olderMade` 13); also `:178-190`
- **Issue:** the other viewer tests read the baked bundles under `tests/fixtures/orchestration/`; these read `tests/fixtures/edge/room_runs_cpp.json`, which CLAUDE.md says is regenerated whenever a decision changes (touched twice on 2026-10-17, `381b4268`, `740a8415`).
- **Why it matters:** the next regeneration moves the loop's frame or keyframes and the viewer tests fail, with nothing in that workflow pointing at them and nothing to do with the viewer.
- **Evidence:** the literal rows `["loop/tum/rgbd", 261, 0, 28, 0]` and `["loop/euroc/stereo", 260, 2, 30, 13]`.
- **Suggested fix:** derive the expectations from the record's own columns: for each `summaries[k].loop.correction`, expect a loop at that frame whose ids equal the `keyframe_loops` row (a stronger cross-check than literals); or bake the loop room into a small bundle fixture with its own generator.

#### [should-fix] `insets` is one long function and `before` is not a good name
- **Where:** `viewer/src/loops.js:174-222`
- **Issue:** one function builds `rowOf`, selects candidate rows, finds the nearest candidate per frame, enforces "never steps back", and shapes the result. `before` (line 192) is a time cutoff, the midpoint between the matched keyframe's making and the window's start; the name reads like a frame count beside `first`, `end` and `last`, and the doc says only "nearer it in time than the window". The nearest-candidate loop is nested three deep.
- **Why it matters:** the trickiest logic in the PR and the point of the "seen before" feature; the part most likely to be changed wrongly.
- **Evidence:** lines 192-217.
- **Suggested fix:** extract `candidateRows(loop, times, poseFrames, poseMaps, cutoff)` and `nearestRow(row, candidates, apart)`; rename `before` to `cutoff`, and state "midway between the matched keyframe's making and the window's first frame" in the doc.

#### [should-fix] The "no loops" path and an empty inset window are not tested
- **Where:** `viewer/tests/loops.test.js`
- **Issue:** no test runs `loopsOf` on a run with no `loop` column (the `records.loop?.[k] ?? null` and `records.map?.[k]` branches), nor `insets` for a loop whose map has no tracked poses, or with `loops = []`.
- **Why it matters:** these are the "bundle with no loops" and "nothing to match" paths. A probe returns `[]` and an all `-1` window as designed: coverage only, not a bug.
- **Evidence:** `loopsOf({records:{frame:[0,1,2]}, ...empty})` gives `[]`; `insets([loop], times, <no poses>)` gives `matches` all `-1` with `insetAt(...) === null`.
- **Suggested fix:** add those cases; also drop `assert.equal(EFFECT_HOLD, 1.5)` and `assert.equal(EFFECT_FADE, 1)` at `:210-211`, which only restate the constants.

#### [note] Per-frame work that does not change per frame in the Solver panel
- **Where:** `viewer/src/solver_view.js:136` (`ticks` in `draw`), `:128-130` and `:171-196` (colour lookups), `:395` (`cell.title = ...`)
- **Issue:** `draw` runs on every shown frame; it recomputes `ticks()` (about 0.1 ms per plot, node microbenchmark) and adds about five `getComputedStyle` colour lookups per plot; `showSummary` assigns `cell.title` for every cell every frame, although `setText` guards the text so that "a frame shown rewrites only what moved".
- **Why it matters:** not measurable jank; slightly at odds with the PR's own "holds its height while it plays" design.
- **Suggested fix:** compute the axis in `Plot.set()` (it depends only on `top` and the axis), and guard the `title` write like `setText`.

#### [note] Tick placement and the timeline's height have two owners
- **Where:** `viewer/src/loop_view.js:137-144`, `viewer/src/main.js:188-200` and `:363`
- **Issue:** `LoopView.mark` repeats `showTimeline`'s tick placement (`left`, `width`, `top` styles); both set `spans.style.height`, and `main.js` computes `tracks.length * SPAN_ROW` in two places.
- **Suggested fix:** a small `placeTick(el, frame, frames, top)` helper, and one owner for the height.

#### [note] The inset's `aria-label` replaces its live label
- **Where:** `viewer/index.html:143` (`#inset`)
- **Issue:** a fixed `aria-label` on a `<button>` overrides its content, so a screen reader never hears the dynamic "seen N s earlier · keyframe K" span.
- **Suggested fix:** drop the `aria-label` and keep the `title`, or set the label from `seenText`.

#### [note] Small literals, names and doc nits
- `viewer/src/solver_view.js:225`: the label backing's `0.85` is unexplained.
- `viewer/src/axis.js:18` and `:55`: the `decimals < 20` cap and the `decade ± 1` window are unexplained.
- `viewer/src/format.js:50`: the parameter `factor` shadows its own function.
- `viewer/src/scene_view.js:238`: `this.size` is really a depth factor; `sizeFactor` would say what it is.
- `viewer/src/loops.js:312`: "again each time" is wrong for a re-seek to the frame already shown; `effectChange` returns `null` when `to === from`, and the test agrees.
- `viewer/src/loop_view.js`: the local `tick` (element) shadows the method `tick`.

#### [note] `loops.js` holds four concerns, and the Plot's label arithmetic cannot be tested
- **Where:** `viewer/src/loops.js`, `viewer/src/solver_view.js:209-232`
- **Issue:** loop discovery, inset matching, effect timing and cycle rows share one file; the effect functions (`effectOpacity`, `effectOver`, `effectFrame`, `effectChange` and their constants) would sit well in an `effect.js` logic module. The label above/below flip, the gutter and the label clamp are plain arithmetic inside canvas drawing, so Node cannot test them (the project's convention for `*_view.js`).
- **Suggested fix:** an optional split now, or when the file next grows; extract the label placement into `axis.js` if its edge cases start to matter.

#### [note] The run-viewer design doc does not mention the new controls
- **Where:** `docs/design/2026-10-09-run-viewer.md:54-58`
- **Issue:** the doc does not describe the sliders, the loops row, the "seen before" inset or the loop flash; a docs-only update that CLAUDE.md allows straight on `develop`.

### Checked and fine
- **No debug leftovers:** no console calls, TODOs, absolute paths or lines over 100 characters in the diff. VERSION and CHANGELOG agree (0.3.15.0, 2026-10-19). The CHANGELOG test counts (loops 10, axis 4, settings 2, solver +2, geometry +2, playback +1, so 61 to 82) match the files.
- **Import map:** the sha384 of all five `lines/` modules recomputed from the CDN each match `viewer/index.html`; their only relative imports are among those five, so no transitive module is unpinned; all new URLs sit under `server_app.CDN`, so the policy test holds.
- **localStorage failing:** `remembered` and `remember` in `viewer/src/page_view.js:19-44` catch the failure, with the reason in the doc and a comment; a refused or blank value falls back to the default, and `settings.test.js` covers `null`, `""`, `" "`, `"NaN"` and out-of-range text.
- **No loops, or an invalid loop:** `loopsOf` throws a message that names the frame, keyframes and map; `main.choose` catches it, shows it, and shows the run without loops; `mark()` runs in a `finally`, and buttons and toggle follow `this.loops`.
- **Culled keyframes:** ids are never reused; `atlas.py` builds run ids as `_kf_base + id` and keeps culled keyframes in the `keyframe_*` arrays, so `first + matched` and `first + current` hold for culled keyframes and multi-map runs, and `loopsOf` cross-checks them against `keyframe_loops`.
- **Several backends:** `summaryLayout` is per run, and the step table is rebuilt per `setRun` from `tracks`; loops, windows and effect belong to the chosen run, and `setRun` ends a running effect and clears the scene's lines synchronously, with no render in between; `show()` between a frame move and a backend change cannot start an effect on the wrong run's loops, because loops come only from the run that ran the frame.
- **Resize:** `Plot.draw` re-measures the canvas and gutter on every draw, with a `ResizeObserver` behind it; `LineMaterial.resolution` is set from the viewport in `onBeforeRender` (three@0.186.0), widths are CSS pixels; the inset is sized by percentage CSS; the panel head wraps.
- **Disposal:** `clearLoop` removes the lines and disposes each geometry and material; `FrameImages` bounds its chunk and bitmap caches (2 chunks and 60 bitmaps for the inset).
- **Effect clock:** `startEffect` chains two animation frames, and `endEffect` cancels whichever is pending; `effect.start ??= now` keeps a hidden-tab return sane.
- **Ticks:** a 200k-case fuzz of `ticks()` found no throw and no violation: every case started at 0, ascended, stayed within the top, had matching label and value counts and at most 15 ticks.
- **Sliders:** `bindSlider` sets min, max and step before the value, snaps and clamps through `onSlider`, and persists the snapped value; `scene.show(playhead.frame)` is safe before any run is loaded (`run === null` is guarded); the key handler ignores non-scrubber inputs, so the sliders' arrow keys do not move the playhead.
- **Contract changes:** the `mapDepths` and `Playhead` `onFrame` signatures, and the removal of `ImageView.image`, have no other callers (grep).
- **Docs and layering:** the JSDoc is thorough on every new function and typedef; `loops.js`, `axis.js` and `settings.js` never touch the DOM and are covered by the no-DOM `tsc` glob; the tests assert values (the depth cap at 1 m and `MIN_DEPTH`, tick sets, the effect's opacity and frame schedule).
