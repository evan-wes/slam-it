# Review panel: `feature/ba-spine`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/ba-spine` |
| **Range** | `0d9366fcf8a282ea61ea8327f03cff8d0ca6b3b4...e3c60fc5167d6567c3029ba021595dc5d0940609` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents (symlinked into reifyscene's `.claude/`) |
| **Layer 1** | all checks passed (`scripts/check.sh --env slamlab`, 16 checks) |

### Findings and resolutions

Findings two reviewers raised about the same issue are marked "same as": they are decided
once.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `cpp/src/foundations/ba.cpp` (`check_problem`) | `check_anchors` reads `pt_idx[a]` before `pt_idx`'s length is checked (out-of-bounds read for C++ callers) | fix | fixed (aa41aec) -- decided by evan-wes on 2026-10-02 |
| H2 | should-fix | hygiene | `ba/problem.py` | `BAProblem` validation bypassed by the constructor and `dataclasses.replace` (same as A2) | fix (with A2) | fixed with A2 (aa41aec) -- decided by evan-wes on 2026-10-02 |
| H3 | should-fix | hygiene | `ba/solve.py`, `bindings/foundations/ba.cpp` | packed `settings`/`codes`/`counts` held by position and enum order (same as A4) | fix (with A4) | fixed with A4 (aa41aec) -- decided by evan-wes on 2026-10-02 |
| H4 | should-fix | hygiene | `readers.py`, `colmap_reference.py`, `demo_slice.py` | COLMAP pixel offset and principal-point slices copied (same as A8) | fix (with A8) | fixed with A8 (3348a47) -- decided by evan-wes on 2026-10-02 |
| H5 | should-fix | hygiene | `ba/readers.py` (`read_colmap_text`) | raw `KeyError`/`IndexError` on malformed COLMAP files; image names with spaces truncated | fix | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| H6 | should-fix | hygiene | `eval/colmap_reference.py` | inline `import time` with a `noqa` | fix | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| H7 | should-fix | hygiene | `scripts/demo_slice.py`, `scripts/demo_bal.py` | cwd-relative default path; interrupted download leaves a corrupt file | fix | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| H8 | should-fix | hygiene | `eval/reprojection.py` | no direct tests; empty mask gives NaN; per-camera loop is quadratic | fix | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| H9 | should-fix | hygiene | `ba/problem.py` (`BAOptions.gauge`), `ba.solve` | docstring points to a module docstring that does not describe the gauge; Raises incomplete | fix | fixed (b45dce5) -- decided by evan-wes on 2026-10-02 |
| H10 | should-fix | hygiene | `ba/bundle.py` | per-group masked loops make the Python reference O(K x M) | fix | fixed (b45dce5) -- decided by evan-wes on 2026-10-02 |
| H11 | should-fix | hygiene | `nls.py` (`solve`), `ba/bundle.py` | unnamed damping constants; long `solve`, `Bundle.__init__`, `linearize` | fix | fixed (b45dce5) -- decided by evan-wes on 2026-10-02 |
| H12 | note | hygiene | various | smaller items: triangular solves, `numeric_jacobian` arguments, BAL negative counts and encoding, Umeyama scale via cbrt, `Bundle` method docstrings | fix the BAL counts/encoding and the `Bundle` docstrings; waive the rest | BAL counts and encoding fixed (3348a47), `Bundle` docstrings fixed (b45dce5); waived: the general solves on triangular factors serve only the dense reference and tests; `numeric_jacobian` is a test helper; Umeyama's cbrt scale is correct, as the review says -- decided by evan-wes on 2026-10-02 |
| A1 | must-fix | architecture | `tests/fixtures/foundations/` | BA has no parity fixture; the live parity test covers one scene, whose pose priors make `auto` pick `none` (gauges, GN, degeneracy, radial unchecked across backends) | fix | fixed (dbfb56a) -- decided by evan-wes on 2026-10-02 |
| A2 | must-fix | architecture | `ba/problem.py`, `ba/solve.py` | reference and C++ fail differently on an unvalidated problem (`IndexError` vs `InvalidInputError`) | fix | fixed (aa41aec) -- decided by evan-wes on 2026-10-02 |
| A3 | should-fix | architecture | `nls.py`, `nls.hpp`, design doc | solve summary lives in `nls`, not the doc's `foundations.solve`; doc's open items and Status line stale | fix the doc now; defer the move to PR 2 (RANSAC) | doc fixed (9178ddd); deferred: the summary's move to `foundations.solve`, to PR 2 (RANSAC), recorded in the design doc's open items -- decided by evan-wes on 2026-10-02 |
| A4 | should-fix | architecture | `ba/solve.py`, `bindings/foundations/ba.cpp`, `nls.hpp` | cross-language options held by position; implicit enum values; hard-coded enum sizes | fix | fixed (aa41aec) -- decided by evan-wes on 2026-10-02 |
| A5 | should-fix | architecture | `eval/colmap_reference.py`, `pyproject.toml` | pycolmap loaded via `importlib` (invisible to import-linter); no optional extra | fix | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| A6 | should-fix | architecture | `ba/readers.py`, `eval/__init__.py` | file-format readers in rung 0's solver package; eval docstring stale ("Stub") | fix (move to `slamlab.eval`) | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| A7 | should-fix | architecture | `scripts/demo_slice.py` | reifyscene's slice list and run layout built into slamlab | fix (generic script; slice list documented, not coded) | fixed (3348a47): `compare_ba_colmap.py` is generic; the slice invocation lives in the BA note, reifyscene unchanged -- decided by evan-wes on 2026-10-02 |
| A8 | should-fix | architecture | readers, reference, demo | model constants copied (same as H4) | fix | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| A9 | should-fix | architecture | `ba/problem.py` | general validators belong in `arrays.py`; `_depth_prior` skips `as_real` | fix | fixed (aa41aec) -- decided by evan-wes on 2026-10-02 |
| A10 | should-fix | architecture | `scripts/check_layers.py` | rung-0 internal-order contract not applied to C++ includes | fix | fixed (d28f41d) -- decided by evan-wes on 2026-10-02 |
| A11 | note | architecture | `cpp/bindings/common.hpp` | batching lives in the bindings; standalone C++/Rust callers have no batched API | defer (design doc open question, before the edge rung) | deferred (design doc, "Deferred from PR 1's review panel"; before the edge rung) -- decided by evan-wes on 2026-10-02 |
| A12 | note | architecture | `ba.hpp` | the sparse reduced solver belongs in rung 0 behind `ReducedSystemSolver` | defer (record in the design doc for milestone 4) | deferred (design doc, "Deferred from PR 1's review panel"; milestone 4) -- decided by evan-wes on 2026-10-02 |
| A13 | note | architecture | `pyproject.toml` | no contract orders the modules inside `ba` | fix (add the contract) | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| A14 | note | architecture | `tests/foundations/test_ba_readers.py` | `eval` tests live under `tests/foundations/` | fix (move to `tests/eval/`) | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| L1 | must-fix | algorithm | `ba/layout.py`, `ba/layout.cpp` | `auto` gauge ignores fixed points (motion-only BA freezes a wrong pose) and treats any pose prior as a full anchor (scale left free; focal falsely unobservable) | fix | fixed (3d0a5a0) -- decided by evan-wes on 2026-10-02 |
| L2 | must-fix | algorithm | `ba/bundle.py` (`_rays`) | an anchor pixel leaving the invertible region raises out of the Python solve; C++ rejects the step; at the start the two raise different kinds | fix | fixed (3d0a5a0) -- decided by evan-wes on 2026-10-02 |
| L3 | should-fix | algorithm | `ba/solve.py`, `schur.py` | focal deviations not scaled by the a-posteriori variance factor; the 1% rule rests on arbitrary weights | fix | fixed (b45dce5) -- decided by evan-wes on 2026-10-02 |
| L4 | should-fix | algorithm | `ba/problem.py`, `ba/layout.py` | "first observation" relies on NumPy's unspecified last-write order | fix | fixed (aa41aec) -- decided by evan-wes on 2026-10-02 |
| L5 | should-fix | algorithm | `eval/colmap_reference.py` | reference silently drops pose priors, weights, fixed poses/points, per-parameter masks | fix | fixed (3348a47) -- decided by evan-wes on 2026-10-02 |
| L6 | note | algorithm | `nls.py` | absolute step tolerance never fires on BA's mixed scales | defer (backlog: per-block relative test) | deferred (design doc, "Deferred from PR 1's review panel") -- decided by evan-wes on 2026-10-02 |
| L7 | note | algorithm | `nls.py` | Triggs correction is the standard upgrade over plain IRLS | defer (backlog, milestone 3) | deferred (design doc, "Deferred from PR 1's review panel"; milestone 3) -- decided by evan-wes on 2026-10-02 |
| L8 | note | algorithm | reduced solve | state of the art for milestone 4 (sparse Cholesky / PCG, root BA, power BA) | fix (cite in the BA note) | fixed (9178ddd) -- decided by evan-wes on 2026-10-02 |
| L9 | note | algorithm | `bundle-adjustment.md` | the slice's focal result is the classic critical-motion degeneracy (Sturm 1997; Kahl, Triggs & Åström 2000) | fix (cite) | fixed (9178ddd) -- decided by evan-wes on 2026-10-02 |
| L10 | note | algorithm | `ba/problem.py` | a fixed anchored point still moves with its host | fix (reject in validation) | fixed (aa41aec) -- decided by evan-wes on 2026-10-02 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `git diff 0d9366f...e3c60fc` (89 files). I read in full: `foundations/nls.py`, `ba/{problem,bundle,solve,readers}.py`, `alignment.py`, `eval/{colmap_reference,reprojection}.py`, `backends.py`, both demos, `cpp/bindings/foundations/ba.cpp` and `cpp/src/foundations/ba.cpp`. I skimmed the other C++ files, the tests (by test name, plus greps for edge cases), the CHANGELOG and VERSION. I did not read `so3.py`, `se3.py`, `camera.py`, `layout.py`, `schur.py`, the C++ `bundle.cpp`/`schur.hpp`/`nls.hpp`, the fixtures or the docs notes in full.

**Summary:** The code is well documented and readable. Conventions are stated, validation is centralised, and C++ and Python validation mirror each other. I found one real defect: a C++ validation order that can read out of bounds. The other findings are should-fix items: validation bypass via `dataclasses.replace`, a fragile packed-argument protocol, duplicated constants, and thin edge-case tests for the readers and the COLMAP reference.

### Findings

#### [must-fix] `check_problem` reads `pt_idx` before its length is checked
- **Where:** `cpp/src/foundations/ba.cpp:136` (`check_anchors`, which indexes `problem.pt_idx[a]` at line ~84) versus the length check at line ~140 (`problem.cam_idx.size() != n_obs || problem.pt_idx.size() != n_obs ...`).
- **Issue:** `check_anchors` runs first and does `problem.pt_idx[static_cast<std::size_t>(a)]` with `a < observations.rows()`. It does not check that `pt_idx.size() == n_obs`. That check comes afterwards.
- **Why it matters:** `ba::solve` is a public core entry point, and its checks exist so the core can reject bad input without Python's help. A caller that supplies a short `pt_idx` and a valid anchor index gets an out-of-bounds `std::vector` read (undefined behaviour) instead of `InvalidInput`. The nanobind path is shielded only because Python validates first. `test_core_checks_indices_too` covers index range, not this ordering.
- **Evidence:**
  ```
  if (auto checked = check_anchors(problem); !checked) {   // reads pt_idx[a]
  ...
  if (problem.cam_idx.size() != n_obs || problem.pt_idx.size() != n_obs ||   // checked after
  ```
- **Suggested fix:** Move the size checks for `cam_idx`, `pt_idx` and `intrinsics_idx` (and the index range checks) above `check_anchors`. Add a core test with `pt_idx.size() != n_obs` and an anchor.

#### [should-fix] `BAProblem` validation is bypassed by `dataclasses.replace`, and `ba.solve` does not re-validate
- **Where:** `src/slamlab/foundations/ba/problem.py` (`BAProblem` has no `__post_init__`; `anchor_points` uses `replace`). Callers: `scripts/demo_slice.py:focal_free` and `scripts/demo_bal.py` (`replace(problem, ...)`), `eval/colmap_reference.py`, `eval/reprojection.py`.
- **Issue:** The module says "`create` validates it once, so the solvers can trust it". The frozen dataclass can be built or `replace`d into any shape. The Python reference then fails with an obscure numpy error on bad shapes. The C++ backend re-checks, but the two backends then disagree on the error type.
- **Why it matters:** Production boundary, and the documented contract does not hold for a public type. The arrays are also mutable in place (`problem.poses[...] = ...`).
- **Suggested fix:** Either validate in `__post_init__` by moving the checks out of `create`, or call a `check` at the top of `ba.solve`. At minimum, document that `replace` skips validation and offer `BAProblem.with_(...)`.

#### [should-fix] Packed `settings`/`codes`/`counts` arguments depend on unnamed positions and on enum order
- **Where:** `ba/solve.py:_backend_arguments`, `_result_from_backend` (`list(nls.Termination)[int(counts[0])]`, `GAUGES[int(counts[1])]`, `counts[2:]`); `cpp/bindings/foundations/ba.cpp:options_from` (`codes(1)`, `codes(2)`, `settings(2)`, and the magic range limits `3`, `1`, `2`, and `codes(0) > 2`).
- **Issue:** Python and C++ agree only by index convention. The upper bounds `3`, `1` and `2` are hard-coded copies of enum sizes. Reordering `nls.Termination`, `GAUGES` or `camera.MODELS` silently mis-decodes the result. `codes(0) > 2` assumes exactly three camera models.
- **Why it matters:** This is silent corruption on a refactor, where a loud failure would be better. Parity tests only catch it if the order differs in tested paths.
- **Suggested fix:** Name the slots in Python (module constants such as `CODE_MODEL = 0`, or an `IntEnum`). In C++, derive the bounds from the enums' `kCount` sentinel values. Add a test that pins the Python tuples to the C++ enum order, for example by exporting the enum names from the extension.

#### [should-fix] The same constant and slice are defined three or four times
- **Where:** `COLMAP_PIXEL_OFFSET = 0.5` in `ba/readers.py:41` and `PIXEL_OFFSET = 0.5` in `eval/colmap_reference.py:25`. Principal-point slices live in three places: `readers._cameras` (`slice(1,3) if radial else slice(2,4)`), `colmap_reference._CENTRE`, and `scripts/demo_slice.py:_CENTRE`. `camera.py` presumably knows these indices already.
- **Why it matters:** If a model's parameter layout changes, the copies drift. This is the kind of duplication jscpd cannot see.
- **Suggested fix:** Expose `camera.PRINCIPAL_POINT_INDICES[model]` (it sits next to the existing `camera.FOCAL_INDICES`). Define the 0.5 offset once, in `readers`, and import it into `colmap_reference`.

#### [should-fix] `read_colmap_text` raises raw `KeyError`/`IndexError`/`ValueError` on malformed files
- **Where:** `ba/readers.py:read_colmap_text` (`point_index[int(pid)]`, `group_of[int(h[8])]`, `header[1:8]` unpacking, `h[9]`, `reshape(-1, 3)` on a ragged keypoint line). `_cameras` indexes `row[4:...]` without a length check.
- **Issue:** The docstring promises only `InvalidInputError` for unsupported models. A dangling point id, an unknown camera id or a truncated row crashes with an unnamed error. `read_bal` handles truncation properly, so the two readers are inconsistent. The test suite covers only the mixed-model rejection. Image names containing spaces are also silently truncated (`h[9]`).
- **Suggested fix:** Wrap lookups in a helper that raises `InvalidInputError` naming the file and the offending id. Add tests for a dangling point id, an unknown camera id and a short line. Join `header[9:]` for the image name.

#### [should-fix] Inline `import time` with a `noqa` in `colmap_reference.solve`
- **Where:** `eval/colmap_reference.py:~162` (`import time  # noqa: PLC0415 -- only the reference's timing needs it`).
- **Issue:** `time` is stdlib and cheap. The lazy import and lint suppression are unjustified; the comment even reads like an excuse.
- **Suggested fix:** Import `time` at module top and delete the `noqa`.

#### [should-fix] `demo_slice.py` default path depends on the working directory, and `demo_bal.py` can leave a corrupt download
- **Where:** `scripts/demo_slice.py:main` (`default=Path("$REIFYSCENE_DATA_DIR/runs").resolve()`). `scripts/demo_bal.py:fetch` (`urllib.request.urlretrieve(url, path)` straight to the final path).
- **Issue:** `resolve()` is applied to a cwd-relative path, so the default is only right when run from the slamlab root, and nothing says so. An interrupted download leaves a truncated `.bz2` that later runs treat as present.
- **Suggested fix:** Anchor the default to `Path(__file__).resolve().parents[N]`, or require `--runs`. Download to `path.with_suffix(".part")` and `rename` on success. Also `SLICE` hard-codes reifyscene scene names; keep it, since it is a documented demo.

#### [should-fix] `eval/reprojection.py` has no direct tests, and `rms` is unvalidated and loops per camera
- **Where:** `src/slamlab/eval/reprojection.py:rms` and `in_front`. There is no `tests/eval/`; the functions are only exercised through `test_ba_readers.py`.
- **Issue:** An all-False `used` mask gives `np.mean([])`, which returns NaN with a RuntimeWarning rather than an error. `rms` loops `for c in np.unique(cam)` with `cam == c` masks, which is O(cameras x observations). It is also not grouped by intrinsics group the way `Bundle` does it (per-camera BAL groups make this quadratic on large problems). The docstring says used observations "must be in front" but nothing checks it.
- **Suggested fix:** Raise `InvalidInputError` on an empty selection or a wrong-length mask. Add small value tests: a hand-computed RMS and a point behind the camera. Group by `intrinsics_idx` (a stable sort) rather than by camera.

#### [should-fix] Docstring pointer is broken: "see the module docstring" for the gauge
- **Where:** `ba/problem.py`, `BAOptions` attribute `gauge` ("How the gauge freedom is removed (see the module docstring)"). The module docstring of `problem.py` says nothing about gauge.
- **Suggested fix:** Describe the gauge choices where they are implemented (`layout.py`'s module docstring) and point there, or add a short description to `BAOptions`. `ba.solve`'s Raises section also omits `ValueError` from `backends.check_name`, and `InvalidInputError` for an unknown option.

#### [should-fix] Python reference does a per-group masked loop in the hot path
- **Where:** `ba/bundle.py:_rays`, `_anchors`, `_reprojection`, `_reprojection_kind`: `for g, params in enumerate(state.intrinsics): sel = group == g`.
- **Issue:** With one intrinsics group per camera (BAL, and the slice demos), every residual and Jacobian evaluation is O(K x M) in Python. `np.add.at` in `linearize` is also known to be slow. The Python reference is not the performance path, so I have not made this a must-fix. It is still a needless quadratic, and a sort-by-group split would make it linear.
- **Suggested fix:** Precompute the per-group index lists once in `Bundle.__init__`, or add a batched `project` that takes per-row intrinsics.

#### [should-fix] Magic numbers and long functions in the solver loop
- **Where:** `nls.py:solve`. The `growth = 2.0` initial value, its `growth *= 2.0` update, and the Nielsen constants `1.0/3.0` and `(2ρ-1)**3` are unnamed. The function is about 60 lines across several phases: initial evaluation, the loop, the accept/reject branches and the summary. Also `ba/bundle.py:Bundle.__init__` (~65 lines) and `Bundle.linearize` (~55 lines).
- **Suggested fix:** Name the constants (`_NIELSEN_MIN_SCALE`, `_DAMPING_GROWTH`) and say where they come from. Split `solve`'s loop body into `_accept` and `_reject` helpers. Split `Bundle.__init__` into `_index_reprojection`, `_index_depth`, `_index_priors` and `_index_anchors`.

#### [note] Smaller items
- `nls.cholesky_solve` uses two general `np.linalg.solve` calls on triangular factors, which is O(n³) each. The comment acknowledges NumPy has no triangular solver; `scipy.linalg.cho_solve` would be one line if SciPy is already a test dependency. `DenseSystem` is only used by the reference and the tests, so this is low priority.
- `nls.numeric_jacobian` takes `dimension` and `step` without validation (non-positive step, negative dimension). It is a test helper, so it is fine, but the docstring should say that.
- `read_bal` opens text files with the platform default encoding and does not reject negative counts in the header. A negative `n_obs` could make `expected` coincide with the number count. Check `n_cams, n_points, n_obs >= 0`.
- `alignment.align` recovers `scale` from `cbrt(det(matrix[:3,:3]))` of the backend output instead of having the backend return it. This is correct, but a roundabout way of reading a value the algorithm already has.
- Public `Bundle` methods (`world_points`, `reduction`, `kinds`) have one-line docstrings with no shapes or frames. They are semi-internal, but the frame ("poses are world-to-camera in `State`, camera-to-world in `BAProblem`") deserves a sentence, since the convention flips there.

### Checked and fine
- Docstring usefulness in `nls.py`, `problem.py`, `readers.py` and `colmap_reference.py`: shapes, frames, units and conventions are stated, and module docstrings explain the algorithm and where each piece sits.
- Input validation in `BAProblem.create`, `BAOptions`, `nls.Options`, `Loss` and `BlockLoss`: errors name the argument and the bad value.
- The C++ core `check_problem` covers sizes, finiteness, index ranges and rigid transforms (apart from the ordering bug above). The binding also range-checks its enum codes.
- No debug prints in library code (the prints are only in the scripts, where they are the output). No swallowed or bare exceptions. `backends._extension` correctly distinguishes a missing extension from a failed load. No `unwrap()` on fallible library input (`unwrap` in the bindings is the Result-to-exception bridge). The NOLINTs are all narrowly scoped and justified.
- `nls.solve` returns the best estimate and a summary, never a worse state than the start, and raises only in strict mode. This matches the CLAUDE.md error contract.
- Parity and oracle tests in `test_ba.py` assert on values: chi-squared, gradient, comparison with SciPy and Ceres, the gauge, dropped counts, strict mode, and invalid-problem parametrisation.
- CHANGELOG and VERSION bumped (0.2.0.0) with a faithful entry.

### Architecture review

## Architecture review

**Scope:** `git diff 0d9366f...e3c60fc` on `feature/ba-spine` (89 files). Read in full or in part: `src/slamlab/backends.py`, `src/slamlab/foundations/{arrays,nls,camera,so3}.py`, the whole `src/slamlab/foundations/ba/` package, `src/slamlab/eval/*`, `cpp/include/slamlab/foundations/{nls,ba,camera}.hpp`, the C++ include graph, `cpp/bindings/{common.hpp,common.cpp,foundations/ba.cpp}`, `rust/slamlab-py/src/foundations/so3.rs`, `pyproject.toml`, `CMakeLists.txt`, `scripts/check_layers.py`, `scripts/demo_slice.py`, `tests/foundations/test_ba.py`, `tests/test_backends.py`, and the design-doc and CLAUDE.md diffs.

**Summary:** The slice is well structured. Rung 0 holds the Lie groups, cameras, NLS and BA, as the design doc requires. The Python `ba` package and the C++ core mirror each other module for module. The generic NLS solver is a Protocol in Python and a concept in C++, and only `ba.solve` is bound. The error kinds cross the binding correctly, and nothing breaks dependency direction. Two rules are broken:
- BA, the PR's main compiled algorithm, has no parity fixture.
- The Python reference and the C++ core fail differently on the same input, because `BAProblem` can skip validation.

The rest are places where a convention is held by position, or by copies of the same constant, instead of in one checked place. There are also some divergences from the design doc and some stale docs.

### Findings

#### [must-fix] Bundle adjustment has no parity fixture; C++ is checked only by a live comparison on one scene
- **Where:** `tests/fixtures/foundations/` (fixtures exist for so3, se3, alignment and camera only); `tests/foundations/test_ba.py:397-432`
- **Issue:** CLAUDE.md says "Every C++/Rust implementation has a Python reference, a parity fixture in `tests/fixtures/<rung>/` that states its tolerance policy". The design doc says the solve-summary fields "go into the parity fixtures". BA has a C++ implementation but no fixture. Its parity test runs the reference and C++ side by side, in-process, on one scene. That scene has pose priors, so `gauge="auto"` resolves to `"none"`. As a result, these C++ paths are never checked against the reference:
  - the `first_pose` and `similarity` gauges (the Householder basis is documented as "deterministic in every backend");
  - Gauss-Newton;
  - degenerate-point dropping and its counts;
  - the `radial` model inside BA.

  The Ceres comparison in `test_ba_readers.py:171` is a correctness check, not parity.
- **Why it matters:** A live comparison needs the Python reference at test time. Native C++ tests and a future Rust BA cannot check themselves against expected outputs without Python, and that check is what fixtures are for. A divergence in the C++ gauge code would go undetected today.
- **Evidence:** `git ls-tree -r --name-only HEAD tests/fixtures` lists no BA fixture. `_every_kind` (test_ba.py:147) sets `pose_prior_weights`. The module docstring says `"auto"` picks `"none"` "when any pose has a prior".
- **Suggested fix:** Add `tests/fixtures/foundations/ba.json`. Generate it from the reference, with a stated tolerance policy, covering at least: each gauge, LM and GN, a degenerate-drop case, radial/BAL, and the every-kind scene. Assert the summary fields too. If the owner prefers live parity for iterative solvers, change the CLAUDE.md and design-doc rule to say so, and parametrise the live test over those cases instead. I recommend the fixture.

#### [must-fix] The reference and C++ core fail differently on the same unvalidated problem
- **Where:** `src/slamlab/foundations/ba/problem.py:33` (a public frozen dataclass with no `__post_init__`); `:96` (`create` is the only validation); `ba/solve.py:1033`
- **Issue:** `BAProblem(...)` and `dataclasses.replace(problem, ...)` build a problem without validation. The tests do this, `anchor_points` does it, and so do callers. `ba.solve` trusts the result. On the same bad problem, the C++ core returns `InvalidInputError`, as the design doc requires, but the Python reference crashes with a raw numpy `IndexError`.
- **Why it matters:** The design doc says "every language fails the same way". The design rule "validates its input once" can be bypassed through the type's own constructor.
- **Evidence:** I built a valid problem, then ran `replace(p, pt_idx=np.array([0,0,1,99]))`:
  ```
  python: IndexError index 99 is out of bounds for axis 0 with size 2
  cpp: InvalidInputError pt_idx: indices must lie in [0, 2)
  ```
- **Suggested fix:** Move the checks into `BAProblem.__post_init__`, so construction and `replace` both validate. Keep `create` for filling defaults and converting array-likes. Add the Python backend to `test_core_checks_indices_too` (test_ba.py:477).

#### [should-fix] The solve summary is not where the design doc puts it, and the doc's open items are stale
- **Where:** `src/slamlab/foundations/nls.py:269-363` (`Termination`, `Iteration`, `SolveSummary`, `Solution`, `CONVERGED`); `cpp/include/slamlab/foundations/nls.hpp:114-190`; design doc lines 3, 108, 121-126, 459-470
- **Issue:** The design doc puts the shared summary in `slamlab.foundations.solve` and `cpp/include/slamlab/foundations/solve.hpp`, and has C++ return `std::expected<Solution<T>, Error>`. The code puts it in `nls`, and C++ `ba::solve` returns its own `ba::Solution`. Two open items are now settled by the code but still listed as open:
  - "first fallible function → error translation";
  - "first solver defines the summary; Ceres-style converged mapping; RANSAC summary decision".

  The Status line still says "No SLAM algorithms exist yet".
- **Why it matters:** RANSAC is the next rung-0 solver and is meant to return this summary. With the summary in `nls`, RANSAC would have to import the least-squares module, and `Termination` includes NLS-only reasons (`DAMPING_LIMIT`, `SINGULAR`). The Python name `ba/solve.py` will also clash conceptually with a future `foundations.solve`.
- **Suggested fix:** Before RANSAC, either move the generic types to `foundations/solve.py` and `solve.hpp` and keep the NLS-specific reasons in `nls`, or change the doc to say the summary lives in `nls`. I recommend the move. In both cases, close the two open items and update the Status line.

#### [should-fix] The cross-language contract of `ba.solve` is held by position, which the Protocol test cannot see
- **Where:** `src/slamlab/foundations/ba/solve.py:137` (`list(nls.Termination)[int(counts[0])]`), `:171-229` (packed `settings`/`codes`/`tolerances`); `cpp/bindings/foundations/ba.cpp:89,199` (`in_range(gauge, 3)`, `codes(0) > 2`); `nls.hpp:31,39,114` (`Method`, `LossKind` and `Termination` have implicit values)
- **Issue:** The design doc says options cross "as scalars, with integer codes for enums". Here, 18 options travel inside three arrays, identified only by index. `test_backends.py` checks parameter *names*, so it cannot catch a reordering inside these arrays. The `Termination`, `Method` and `LossKind` codes come from declaration order in both languages, while `Gauge` and `camera::Model` have explicit values. The binding also hard-codes how many values each enum has, so adding a camera model or a loss means a silent edit in three places.
- **Why it matters:** Swapping, say, `function_tolerance` and `step_tolerance` on one side passes every structural check. Whether parity notices depends on the scene.
- **Suggested fix:** Pass options as named scalar arguments, as the design doc says, so the Protocol test covers them. Give every crossing enum explicit values in C++. Add one test that checks the Python code tables against the compiled side, for example by binding `nls::name(Termination)` or exposing each enum's size. Derive the binding's range checks from the enums.

#### [should-fix] pycolmap is imported through `importlib`, which import-linter cannot see, and is not declared as an extra
- **Where:** `src/slamlab/eval/colmap_reference.py:58-65`; `pyproject.toml` (pycolmap is listed only under `dev`)
- **Issue:** A library module depends on a package that is only in the dev dependencies, and loads it with `importlib.import_module("pycolmap")`. grimp does see function-level `import` statements. The dynamic form hides the dependency, so no contract can stop a lower rung from copying the pattern.
- **Suggested fix:** Use a plain local `import pycolmap` inside the function, and declare an optional extra (e.g. `colmap = ["pycolmap"]`). Add a forbidden contract so only `slamlab.eval` may import `pycolmap`.

#### [should-fix] BAL/COLMAP readers sit in rung 0's solver package, while eval's docstring claims "benchmark loaders"
- **Where:** `src/slamlab/foundations/ba/readers.py`; `src/slamlab/eval/__init__.py:1-9`
- **Issue:** File-format I/O (bz2, text parsing, dataset conventions) lives inside `foundations.ba`, yet no rung-0 algorithm needs it. Only demos, tests and eval use it. `eval/__init__.py` says loaders belong in eval and still reads "Stub: no code yet." even though this PR adds two eval modules. Milestone 2 brings TUM and EuRoC loaders, and this choice sets their precedent.
- **Suggested fix:** Pick one home for dataset and format readers. I recommend `slamlab.eval`, which matches its docstring; the alternative is a dedicated `slamlab.foundations.io`. Record the choice in the design doc's package-layout table. Update eval's docstring: drop "Stub" and state its milestone and budget.

#### [should-fix] `demo_slice.py` builds a reifyscene yardstick into slamlab
- **Where:** `scripts/demo_slice.py:1-16, 31-46, 98`
- **Issue:** The script hard-codes reifyscene's slice list, its run layout (`runs/slice_*`, `pose_refine/colmap_seed`) and a default path to the reifyscene checkout. The design doc says "reifyscene-specific yardsticks stay in reifyscene". This is not an import, but it is a data-level dependency on reifyscene's internals, and it breaks when reifyscene's layout changes.
- **Suggested fix:** Make the slamlab script generic ("slamlab vs COLMAP BA over a list of COLMAP text models"). Move the slice list and run-layout knowledge to reifyscene, which calls it.

#### [should-fix] Model and convention constants are copied across modules
- **Where:** principal-point indices in `ba/readers.py:166`, `eval/colmap_reference.py:32` and `scripts/demo_slice.py:49`; the COLMAP pixel offset in `ba/readers.py:42` (`COLMAP_PIXEL_OFFSET`) and `eval/colmap_reference.py:29` (`PIXEL_OFFSET`); the intrinsics sizes `5/4/9` in `readers.py:153-157`, which duplicate `camera.MODELS`
- **Issue:** One camera-model fact and one convention conversion (design doc, "Pixel coordinates") each have several copies. Adding a model means editing four files.
- **Suggested fix:** Add `camera.CENTRE_INDICES` beside `FOCAL_INDICES`. Take sizes from `camera.MODELS`. Keep one COLMAP offset constant and import it.

#### [should-fix] General validation helpers live in `ba/problem.py` instead of `arrays.py`
- **Where:** `src/slamlab/foundations/ba/problem.py:340-445` (`_shaped`, `_indices`, `_weights`, `_mask`, `_depth_prior`)
- **Issue:** CLAUDE.md puts shared checks in `slamlab.foundations.arrays`. Index-array, mask and weight checks are general; RANSAC, PnP and the edge rung will need them. `_depth_prior` also skips `as_real` and uses `np.asarray(value, dtype=np.float64)`, so booleans are silently converted, which the `arrays` contract says must be rejected.
- **Suggested fix:** Promote `as_indices`, `as_mask` and `as_weights` to `arrays.py`, and route `_depth_prior` through `as_real` with NaN allowed.

#### [should-fix] `check_layers.py` does not apply the new rung-0 internal-order contract to C++
- **Where:** `pyproject.toml` contract "Rung 0's internal order"; `scripts/check_layers.py:92` (reads only `containers == ["slamlab"]` exhaustive contracts)
- **Issue:** CLAUDE.md says `check_layers.py` holds C++ "to the same contracts as import-linter". The new module-order contract is enforced only in Python. Today's C++ include graph happens to follow it (I checked every `#include "slamlab/..."`).
- **Suggested fix:** Extend `check_layers.py` to check includes between the files of `cpp/include/slamlab/foundations/*.hpp` and `cpp/src/foundations/*` against that contract. Otherwise, mark the contract as Python-only in its comment.

#### [note] Batching lives in the bindings, not the cores
- **Where:** `cpp/bindings/common.hpp` (`map_batch`), used by `se3.cpp` and the other module bindings; `rust/slamlab-py/src/foundations/so3.rs`
- The cores are per-item (fixed-size Eigen), and the bindings loop over the batch. This fits the design doc ("fixed-size arrays, spans or slices") and adds no algorithm logic. The consequence: a standalone C++ or Rust caller (the edge budgets) has no batched API, and per-item error semantics (first failure aborts the batch) are defined in the binding. Worth deciding before the edge rung.

#### [note] The sparse reduced solver should land in rung 0
- `ReducedSystemSolver` is a Protocol in Python and a concept in `ba.hpp`, but C++ `ba::solve` is not parametrised on it. When milestone 4's sparse solver arrives, it belongs in rung 0 behind this interface, not in `classic`. The edge rung's local BA will want it too.

#### [note] No contract orders the modules inside the `ba` package
- Today the order is clean: problem → layout|schur → bundle → solve, with readers above problem. An import-linter layers contract on `slamlab.foundations.ba` would keep it that way as the package grows.

#### [note] Tests for `eval.colmap_reference` live under `tests/foundations/`
- `tests/foundations/test_ba_readers.py` exercises `slamlab.eval.colmap_reference`, and there is no `tests/eval/`. The mirroring rule calls for `tests/eval/`.

### Checked and fine
- Ladder direction: `eval` imports only `foundations`; nothing in `foundations` imports `eval`, `orchestration` or reifyscene. Demos use only public package exports.
- Rung placement: SO(3), SE(3), Umeyama, cameras, NLS and BA all belong in rung 0 per the design doc's ladder table, with no copies in other rungs.
- Edge-budget dependencies: rung 0 adds only Eigen (header-only, a PUBLIC dependency, pinned in setup). SciPy, mpmath, manifpy, pycolmap and rerun are dev or test only. No torch or GPU stack.
- Generic in the core, concrete in the bindings: NLS is a Protocol/concept with native tests (`cpp/tests/foundations/test_nls.cpp`), and only `ba.solve` is bound.
- `slamlab.backends` stays generic: `function` and `dispatch` locate and call implementations without knowing what they compute. The `LOCKSTEP_BACKENDS` change to `test_backends.py` correctly enforces "C++ lock-step, Rust subset".
- Error translation: `cpp/bindings/common.cpp` maps all three kinds to `slamlab.foundations.errors`. Each kind is exercised through C++ (alignment for Degenerate, strict BA for NotConverged, core index checks for InvalidInput).
- C++ structure: `ba` private headers stay in `cpp/src/foundations/ba/`, and the public `ba.hpp` is small. The binding `ba.cpp` only converts types, apart from the enum range checks noted above.
- The import-linter layer contracts and `slamlab_add_rung` linking are unchanged and still correct. Eigen's addition to every rung library is documented in `CMakeLists.txt`.
- No globals, hidden singletons, import-time side effects or environment-dependent behaviour in `src/`.
- The design doc's new "Conventions" section is reflected in the code (T_cam_world inside the solver, T_world_cam at the boundary, xyzw quaternions, OpenCV pixel centres via the COLMAP reader).

Key paths:
- src/slamlab/foundations/ba/problem.py
- src/slamlab/foundations/ba/solve.py
- src/slamlab/foundations/nls.py
- cpp/bindings/foundations/ba.cpp
- src/slamlab/eval/colmap_reference.py
- src/slamlab/foundations/ba/readers.py
- scripts/demo_slice.py
- scripts/check_layers.py
- tests/foundations/test_ba.py
- docs/design/2026-10-02-slamlab-ladder.md

(One side effect: the `conda run` I used for evidence triggered the editable install's automatic C++ rebuild into `build/slamlab/`. That directory is git-ignored and the rebuild was a no-op ("no work to do"). No tracked files were touched.)

### Algorithm review

## Algorithm review

**Scope:** `0d9366f...e3c60fc` (feature/ba-spine): `src/slamlab/foundations/{so3,se3,alignment,camera,nls}.py`, `src/slamlab/foundations/ba/*`, `src/slamlab/eval/{colmap_reference,reprojection}.py`, and the matching C++ in `cpp/src/foundations/` (`ba/bundle.cpp`, `ba/layout.cpp`, `camera.cpp`), read against `docs/notes/{lie-groups,camera-models,nls,bundle-adjustment}.md`. Checked against Barfoot (SO(3)/SE(3) Jacobians, Q), Solà et al. (right perturbations), Umeyama 1991, Triggs et al. 1999 (Schur, gauge, robust IRLS), Ceres' LM strategy (Nielsen update, Marquardt diagonal clamp, loss definitions) and the BAL/COLMAP model definitions. I ran experiments in a scratch directory against the HEAD working tree with both backends.

**Summary:** The maths is sound and well tested. The Lie-group series and closed forms are correct; I checked the coefficients by hand. The Schur elimination, the anchored-inverse-depth chain (including the implicit-function intrinsics Jacobian), the Householder scale-gauge basis, the pose-prior `Jr⁻¹` and the readers' frame flips are all correct. There are two must-fix defects in the BA layer, each shown on a small synthetic scene:
- **The `auto` gauge over-constrains or under-constrains common problems.** On motion-only BA with fixed points it freezes a pose at its wrong starting value. With a single pose prior it leaves scale free, and the focal length is then wrongly reported as unobservable.
- **An anchor pixel that cannot be undistorted crashes the Python solve.** The Python reference raises, while the C++ core rejects the step and converges.

### Findings

#### [must-fix] `auto` gauge ignores fixed points and treats any pose prior as a full anchor
- **Where:** `src/slamlab/foundations/ba/layout.py:59-65` (`_resolve_gauge`), used at `layout.py:139-146`. Mirrored at `cpp/src/foundations/ba/layout.cpp:176-185`.
- **Issue:** `auto` decides using only "any pose prior → none" and "metric depth → first_pose", and otherwise picks `similarity`. Two consequences:
  1. **Fixed points are never considered.** Motion-only BA (all points fixed, poses free, the standard PnP refinement and tracking case) or a problem with fixed control points still gets `similarity`. That freezes the first free pose and confines a second one, even though the fixed structure already defines the gauge.
  2. **Any single pose prior selects `none`.** One SE(3) prior pins 6 DoF but not the monocular scale, and a translation-only prior on one camera pins only 3. So the reduced system stays singular.
- **Why it matters:**
  - Case 1 returns a wrong answer with no error raised: pose 0 stays at its perturbed start and the cost cannot reach zero.
  - Case 2 makes `S` singular at the solution. `intrinsics_std` becomes inf, and `unobservable_focal` flags a focal length that is in fact well observed. The note recommends that milestone 3 decide whether to free the focal length on exactly this flag.
  - The tests do not cover either path. All the pose-prior tests put priors on every camera.
- **Evidence:** Synthetic scene `_scene(cameras=4, points=40, perturb=0.01, seed=1)` from `tests/foundations/test_ba.py`, with `points = truth` and `fixed_points = all True`:
  ```
  auto python similarity gauge_pose 0 scale_pose 3 final cost 3.318e+02 pose err per cam [0.008447 0. 0. 0.011194]
  auto cpp    similarity gauge_pose 0 scale_pose 3 final cost 3.318e+02 pose err per cam [0.008447 0. 0. 0.011194]
  none python none ... final cost 2.345e-20 pose err per cam [0. 0. 0. 0.]
  ```
  Second scene: `_scene(cameras=5, points=60, noise=0.5, seed=3, rotation=0.3)` with focal free. Adding one full pose prior (weights 1e4) on camera 0 only:
  ```
  no prior, auto      gauge=similarity  f_std=[1.63 2.16]  unobservable_focal=0
  one pose prior, auto gauge=none       f_std=[inf inf]    unobservable_focal=1
  ```
  Triggs et al. 1999, §9, covers gauge: the constraints must be minimal (no more and no fewer than the gauge's dimension), or the solution is biased or the covariance singular.
- **Suggested fix:** Have `auto` count what the fixed variables and priors actually remove:
  - **Fixed points:** three or more non-collinear fixed points, or fixed points seen by a fixed pose, leave no gauge, so use `none`.
  - **Scale:** it is pinned by metric depth, by two or more fixed or prior-constrained camera centres at distinct places, or by fixed points with spread. Otherwise add the scale constraint even under pose priors, a "scale-only" gauge using the existing Householder machinery.
  - Alternatively, `auto` could raise `InvalidInput` and ask for an explicit gauge when fixed points are present.

  Add tests for motion-only BA and for a single pose prior. Fix both cores.

#### [must-fix] Python BA crashes (and the backends disagree) when an anchor pixel leaves the distortion's invertible region
- **Where:** `src/slamlab/foundations/ba/bundle.py:200-210` (`_rays` calls `camera.unproject`, which raises for the whole batch), reached from `squared_norms` during `nls.solve`. Compare `cpp/src/foundations/ba/bundle.cpp:409-420` (`anchor_ray` returns `nullopt`, so the cost is inf and the step is rejected).
- **Issue:** With free radtan intrinsics and anchored points, an LM trial step that moves k1 far enough makes an anchor pixel non-invertible.
  - **Mid-solve:** the Python reference lets `NotConvergedError` escape out of `ba.solve`. That breaks the solver contract, under which NotConverged is only for strict mode or divergence. C++ rejects the step and converges.
  - **At the start:** the two backends raise different error kinds (Python `NotConvergedError`, C++ `InvalidInputError` "initial cost not finite"). Neither drops the point, unlike the documented handling of an anchor behind its camera.
- **Why it matters:**
  - The reference and the compiled core diverge on the same input.
  - Free distortion with anchored points is exactly the milestone 3 use (refining VGGT cameras with depth anchors).
  - The parity test does not exercise this path.
- **Evidence:** `_scene(cameras=4, points=30, model="radtan", seed=2, perturb=0.02)`, then `ba.anchor_points`, k1 started at -1.0, f and distortion free, cx/cy fixed:
  ```
  -1.0 python: RAISES NotConvergedError | cpp: gradient cost 4.3e-25 k1 -0.0800
  -1.1 python: RAISES NotConvergedError | cpp: RAISES InvalidInputError
  cpp trace: [(1, 31658.163, False), (2, 31658.163, True), ...]   # C++ rejected the bad first step
  python: ... camera.py:232 _undistort -> NotConvergedError: undistort: pixel outside the invertible region
  ```
- **Suggested fix:**
  - **Mid-solve:** in `_rays`/`world_points`, undistort per anchor and catch the failure. Set that point's world position to NaN, so its blocks become inf cost and the step is rejected, matching C++.
  - **At the start:** drop such anchored points as degenerate in `_kept_observations`, with the same rule in both cores, and count them. That gives one error behaviour in both backends.
  - Add a parity test that starts from a k1 like the one above.

#### [should-fix] Focal standard deviation assumes the weights are true informations; the default weight of 1 px⁻² rarely is
- **Where:** `src/slamlab/foundations/ba/solve.py:70-86`, `ba/schur.py:255-279`, and the decision rule `_unobservable_focal` (`focal_std_ratio = 1%`).
- **Issue:** `σ_f = sqrt(diag(S⁻¹))` is not scaled by the a-posteriori variance factor `σ̂₀² = 2·cost/(m − n)`. With the default weights of 1 (σ = 1 px) and real residuals of about 0.4–0.9 px (the slice's RMS column), σ_f is overstated by roughly 1/σ̂₀. The 1% threshold then flags or clears groups on an arbitrary scale.
- **Why it matters:** The note's recommendation for milestone 3 ("hold focal fixed unless `unobservable_focal` is 0") hinges on this threshold.
- **Evidence:** The docstring itself says "Those deviations assume the weights are true informations". The standard practice is to scale by the a-posteriori variance factor: Triggs et al. 1999, §2.2–2.3; Förstner & Wrobel, *Photogrammetric Computer Vision* (2016), §4.2.
- **Suggested fix:** Report `σ̂₀²` in the summary, and either scale the deviations by it or make the scaling an option with a stated default. Keep the unscaled value available.

#### [should-fix] Two "first observation" computations rely on undefined NumPy behaviour
- **Where:** `src/slamlab/foundations/ba/problem.py:420` (`anchor[problem.pt_idx[weighted]] = weighted  # the last write per point is its first`) and `src/slamlab/foundations/ba/layout.py:49` (`first[pt[order]] = order`).
- **Issue:** Both depend on "last write wins" for repeated indices in a fancy assignment.
- **Why it matters:** NumPy documents this order as unspecified. If it changes, the anchor choice and the parallax reference ray change silently, and parity with C++ (which loops explicitly) breaks.
- **Evidence:** NumPy indexing docs, "Detailed notes": "For advanced assignments, there is in general no guarantee for the iteration order. This means that if an element is set more than once, it is not possible to predict the final result." (https://numpy.org/doc/stable/user/basics.indexing.html)
- **Suggested fix:** Use `np.minimum.at(first, pt, np.arange(...))` (initialised to a large value), or `np.unique(pt, return_index=True)`.

#### [should-fix] The COLMAP reference silently ignores parts of the problem it cannot represent
- **Where:** `src/slamlab/eval/colmap_reference.py:68-73` (`_check`), and `to_reconstruction` / `solve`.
- **Issue:** `_check` rejects anchors and depth priors but not the following, which are dropped without warning:
  - pose priors (`pose_prior_weights > 0`);
  - non-unit or zero observation `weights`;
  - `fixed_poses` and `fixed_points`;
  - per-parameter `fixed_intrinsics` beyond "principal point fixed".
- **Why it matters:** The "identical problem" comparisons in `bundle-adjustment.md` hold only when none of these are set. A future comparison could pass or fail on a different problem from the one solved by slamlab.
- **Suggested fix:** Reject these in `_check` (or map the fixed sets onto `BundleAdjustmentConfig` constant poses and points). State in the docstring what the comparison covers.

#### [note] Step tolerance is absolute, unlike Ceres
- **Where:** `src/slamlab/foundations/nls.py`, `Options.step_tolerance`, with the termination test in `_try_step`.
- Ceres' parameter tolerance is relative, `|Δx| ≤ (|x| + tol)·tol`. The note documents the absolute choice ("a manifold state has no norm"). In BA, points (scale ~10–1000) and focal lengths (~1000) share the step vector, so 1e-10 absolute effectively never fires. Termination then falls to the function test or the cap, which matches the notes' "five scenes never converge within 300 iterations". A per-block relative test (pose twist absolute, points and intrinsics relative) would be closer to Ceres. This is not a defect.

#### [note] IRLS without the ρ'' term is documented; Triggs' correction is the standard upgrade
- **Where:** `nls.py` module docstring; `nls.md`.
- Dropping `2ρ'' Jᵀ r rᵀ J` is a valid, PSD-safe choice, and the note measures its cost (108 iterations on Huber outliers). For context, Ceres' `Corrector` implements the Triggs et al. 1999 §4.3 rescaling. It is the natural next step if robust BA convergence matters in milestone 3.

#### [note] State of the art for the reduced solve (milestone 4 context)
- The dense Cholesky on `S` is appropriate for this rung, and the `ReducedSystemSolver` seam is in the right place. When milestone 4 plugs in a sparse solver, the current options are:
  - sparse Cholesky, or Schur-preconditioned CG ("inexact step LM"): Agarwal, Snavely, Seitz & Szeliski, "Bundle Adjustment in the Large", ECCV 2010;
  - QR null-space marginalisation of landmarks, numerically more stable and float32-capable: Demmel, Sommer, Cremers & Usenko, "Square Root Bundle Adjustment for Large-Scale Reconstruction", CVPR 2021, [arXiv:2103.01843](https://arxiv.org/abs/2103.01843v2);
  - a power-series inverse of the Schur complement: Weber, Demmel, Chan & Cremers, "Power Bundle Adjustment for Large-Scale 3D Reconstruction", CVPR 2023, [arXiv:2204.12834](https://arxiv.org/abs/2204.12834v4).
- Anchored inverse depth follows Civera, Davison & Montiel, "Inverse Depth Parametrization for Monocular SLAM", T-RO 2008, and DSO's host frames (Engel, Koltun & Cremers, TPAMI 2018).

#### [note] The slice's focal-unobservability result matches known theory
- The finding that focal length is unobservable on 9 of 10 DAVIS clips is the classic critical-motion degeneracy. Pure translation and small rotation do not determine focal length: Sturm, "Critical Motion Sequences for Monocular Self-Calibration and Uncalibrated Euclidean Reconstruction", CVPR 1997; Kahl, Triggs & Åström, "Critical Motions for Auto-Calibration When Some Intrinsic Parameters Can Vary", JMIV 2000.
- Citing it in `bundle-adjustment.md` would ground the recommendation to milestone 3.

#### [note] A fixed anchored point still moves with its host
- **Where:** `problem.py`, `_anchors` (it accepts `point_anchor >= 0` together with `fixed_points[p] = True`).
- Such a point's ρ is held, but its world position follows the host pose and intrinsics. `anchor_points` never produces this combination. A caller-supplied one is accepted without a stated meaning. Either reject it in validation or document it.

### Checked and fine
- **SO(3):**
  - Rodrigues and the half-angle form of `(1−cos θ)/θ²`.
  - The `atan2` angle in `log`.
  - The near-π axis from the symmetric part, with the sign taken from the skew part and a deterministic choice at exactly π.
  - The series for `(θ−sin θ)/θ³` (1/6, −1/120, 1/5040, −1/362880) and for `1/θ² − cot(θ/2)/(2θ)` (1/12, 1/720, 1/30240, 1/1209600), both checked by hand.
  - Shepperd's quaternion with w ≥ 0, and the SVD projection with its det fix.
- **SE(3):**
  - Barfoot's Q, with all four terms and their signs, and its block inverse.
  - The exact series `c_k` for the three Q coefficients, including `(k+1)/(2k+5)!`.
  - The adjoint for `(ρ, θ)` order, and `exp`/`log` through `Jl(θ)`.
  - The oracle (series of `adⁿ/(n+1)!`) is genuinely independent of the implementation.
- **Umeyama:** the `det(U)det(V)` reflection guard, `s = tr(DS)/σ_x²`, and collinear and too-few-points degeneracy raised as `DegenerateError`.
- **Camera:**
  - The radtan distortion and its 2×2 Jacobian (every entry checked by hand), and the intrinsics Jacobian.
  - The radial-through-radtan map E.
  - Newton undistortion with the det ≤ 0 guard. A 2000-point stress test on three lenses, including the note's k3 = −6 lens, never returned a root beyond the fold. The only round-trip mismatches were points projected from beyond the fold, which correctly map back to the invertible branch.
- **NLS:**
  - The predicted decrease `½δᵀ(Dδ − g)` is derived correctly from `(H+D)δ = −g`.
  - The Nielsen update and doubling ν match Ceres.
  - The Marquardt diagonal is clamped to [1e-6, 1e32].
  - The Huber and Cauchy ρ and ρ' match Ceres' definitions.
  - The best state is always returned, with a summary and strict mode.
  - Gauss-Newton stopping on `no_decrease` and `singular` is right.
- **BA Jacobians:**
  - `dp/dρ = R`, `dp/dθ = −R hat(X)` for a right perturbation of `T_cw`.
  - For anchored points, `dX/d(host) = [−I, hat(X)]` and `dX/dρ = −Rᵀm/ρ²`.
  - The IFT intrinsics term uses `∂π/∂(x,y)` at Z = 1, and the depth-prior affine columns are correct.
  - The pose prior `log(T̄_cw⁻¹ T_cw)` with `Jr⁻¹(e)`. Its translation part approximates the world-frame centre difference, which suits a GNSS prior.
- **Schur:** damping applied before the H_pp inverse and in reduced coordinates, so the step equals the damped full solve. Back-substitution is correct, and repeated columns are accumulated in both cores.
- **Scale gauge:** `dc = −ρ + c×θ` and `a = (−b, b×c)` verified by the scalar triple product. The Householder complement is orthonormal and deterministic, and it is recomputed per linearisation consistently with `retract`.
- **Cheirality:** observations behind a camera at the start are dropped; mid-solve, they give infinite cost and the step is rejected. `ρ ≤ 0` likewise rejects the step.
- **Readers:**
  - BAL's `F = diag(1,−1,−1)` flip gives pixels `(x, −y)`, checked by hand.
  - COLMAP's pose is world-to-camera with a Hamilton quaternion, pixels and principal point shift by −0.5, and the OPENCV and RADIAL family slot maps are correct.
- **Tests:** synthetic scenes are seeded (`default_rng(seed)`). Schur-vs-dense and numeric Jacobians are tested with radtan, free intrinsics, Huber and anchors. Fixtures carry tolerance policies, and mpmath oracles stand independent of the implementations.

Scratch scripts used for the evidence are in `/tmp/scratch/` (`gauge_fixed_points.py`, `prior_gauge.py`, `anchor_undistort4.py`, `anchor_undistort5.py`, `undistort_floor.py`, `undistort_root.py`). Run them from the repo root with `PYTHONPATH=. conda run -n slamlab python <script>`. No repository files were modified.

Sources:
- [NumPy indexing docs](https://numpy.org/doc/stable/user/basics.indexing.html)
- [Square Root Bundle Adjustment, arXiv:2103.01843](https://arxiv.org/abs/2103.01843v2)
- [Square Root Bundle Adjustment, CVPR 2021](https://openaccess.thecvf.com/content/CVPR2021/html/Demmel_Square_Root_Bundle_Adjustment_for_Large-Scale_Reconstruction_CVPR_2021_paper.html)
- [Power Bundle Adjustment, arXiv:2204.12834](https://arxiv.org/abs/2204.12834v4)
- [Power Bundle Adjustment, CVPR 2023](https://openaccess.thecvf.com/content/CVPR2023/html/Weber_Power_Bundle_Adjustment_for_Large-Scale_3D_Reconstruction_CVPR_2023_paper.html)

## Run 2 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/ba-spine` |
| **Range** | `e3c60fc5167d6567c3029ba021595dc5d0940609...cda70e627c44a4910f71c65ed474b7c61fc953be` (Run 1's fixes; full PR range `0d9366f...cda70e6` as context) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents (symlinked into reifyscene's `.claude/`) |
| **Layer 1** | all checks passed at 9178ddd (`scripts/check.sh --env slamlab`); later commits are changelog and report only |

Each reviewer first checked its Run 1 findings: all resolved as recorded, except H8 (partly:
see H13) and L1 (partly: see L11, L12).

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H13 | should-fix | hygiene | `eval/reprojection.py`, `ba/layout.py`, `eval/colmap_reference.py` | per-group / per-camera / per-point masked loops survive (O(K x M), O(N x M)); `_members` is private to `bundle.py` | fix (share `_members`) | fixed (f24d033, b4196f7) -- decided by evan-wes on 2026-10-02 |
| H14 | should-fix | hygiene | `ba/layout.py`, `ba/layout.cpp` | coincident-pose tolerance `1e-12 * (1 + |origin|)` unnamed, written twice per core | fix | fixed (f24d033): the tolerance went with the old gauge code -- decided by evan-wes on 2026-10-02 |
| H15 | should-fix | hygiene | `ba/layout.py` (`_gauge`), `layout.cpp` (`fix_gauge`) | "scale pinned" computed two ways (fixed poses apart vs `_anchoring`) | fix (with L11) | fixed (f24d033): one null-space test replaces both -- decided by evan-wes on 2026-10-02 |
| H16 | should-fix | hygiene | `eval/readers.py` | dangling point id reports the header's line, not the keypoint line; closures with default-argument capture | fix | fixed (b4196f7) -- decided by evan-wes on 2026-10-02 |
| H17 | should-fix | hygiene | `ba/solve.py`, `bindings/foundations/ba.cpp` | the trace still crosses by column position | fix | fixed (f24d033) -- decided by evan-wes on 2026-10-02 |
| H18 | should-fix | hygiene | `scripts/compare_ba_colmap.py` | bare COLMAP tolerance; reports group 0 without saying so; `compare` does too much | fix | fixed (b4196f7) -- decided by evan-wes on 2026-10-02 |
| H19 | should-fix | hygiene | `eval/colmap_reference.py` | module docstring still says pycolmap is a dev dependency (same as part of A20) | fix | fixed (b4196f7) -- decided by evan-wes on 2026-10-02 |
| H20 | note | hygiene | various | `_members` vs `self._members`; `point` passed beside `kind`; shared camera-side helper; `centred` misnamed; `..` in relative includes; `ba.json` one number per line; robust bias in σ̂₀ (same as L14) | fix the rename, `centred`, `..` (with A16), compact fixture; waive the rest | fixed: `_by_group` rename and compact fixture (f24d033), `..` includes (b4196f7), `centred` went with the old code; waived: `point` beside `kind` (keeps mypy's narrowing explicit) and a shared camera-side helper (two short, different kinds) -- decided by evan-wes on 2026-10-02 |
| A15 | should-fix | architecture | `ba/problem.py` | validated arrays are writable and `point_anchor` aliases the caller's array, so a valid `BAProblem` can be made invalid in place | fix (copy; read-only arrays) | fixed (f24d033) -- decided by evan-wes on 2026-10-02 |
| A16 | should-fix | architecture | `scripts/check_layers.py` | internal-order check sees direct includes only, exempts the facade both ways, skips Rust, overwrites duplicate containers -- yet claims to match import-linter | fix (close the C++ gaps; state that Rust is not covered) | fixed (b4196f7): chains, `..`, facade, duplicate containers; Rust stated as not covered -- decided by evan-wes on 2026-10-02 |
| A17 | should-fix | architecture | `ba/layout.py` (`bearings_or_nan`) | a camera capability (per-pixel unproject with NaN) lives in BA's layout | fix (move to `camera`) | fixed (f24d033): `camera.unproject_or_nan` -- decided by evan-wes on 2026-10-02 |
| A18 | should-fix | architecture | `tests/foundations/test_ba.py` | the explicit `scale` gauge error is tested on the reference only | fix | fixed (f24d033) -- decided by evan-wes on 2026-10-02 |
| A19 | note | architecture | `generate_ba.py`, `ba.json` | fixture not self-describing: options and the exact-codes policy live in the generator | fix (full options and an exact policy in the JSON) | fixed (f24d033) -- decided by evan-wes on 2026-10-02 |
| A20 | note | architecture | bindings, `colmap_reference.py`, `pyproject.toml`, design doc | enum tables anchored only on the last enumerator; stale docstring; pycolmap pinned twice; deny-list contract; `eval` row in the package table | fix | fixed (f24d033: kCount sentinels, one pycolmap pin, allow-list contract; 3adfcd4: design-doc eval row, A12 record; b4196f7: docstring) -- decided by evan-wes on 2026-10-02 |
| L11 | must-fix | algorithm | `ba/layout.py`, `layout.cpp` (`_anchoring`, `_resolve_gauge`) | `auto` reduces anchoring to two flags, so partial anchoring (fixed points, translation-only or rotation-only priors) fixes a whole pose and biases the solution (cost 4.7-29 vs 1e-21) | fix (see options) | fixed (f24d033): the general null-space gauge, as the user chose ("General gauge now", design approved separately) -- decided by evan-wes on 2026-10-02 |
| L12 | should-fix | algorithm | `ba/layout.py`, `layout.cpp` | a pose prior on an unobserved camera counts as an anchor: `auto` picks `scale`, leaving rigid motion free (σ_f inf) | fix | fixed (f24d033): priors count only on observed poses; constant prior blocks left out -- decided by evan-wes on 2026-10-02 |
| L13 | should-fix | algorithm | `ba/bundle.py`, `bundle.cpp` (`residual_rows`) | a half pose prior counts 6 rows in m, understating σ̂₀² | fix | fixed (f24d033) -- decided by evan-wes on 2026-10-02 |
| L14 | should-fix | algorithm | `ba/solve.py`, `bundle.cpp` (`variance_factor`) | under a robust loss the numerator Σρ overstates σ̂₀² (2.5x on σ_f at 5% outliers, Huber); Σ w s pairs with the information used | fix (Σ w s) | fixed (f24d033): sum(w s) with the final IRLS weights -- decided by evan-wes on 2026-10-02 |
| L15 | note | algorithm | `docs/notes/bundle-adjustment.md` | COLMAP picks DENSE_SCHUR up to 50 images, so the slice's speed gap is not dense vs sparse; Power BA paraphrase | fix the note | fixed (3adfcd4) -- decided by evan-wes on 2026-10-02 |
| L16 | note | algorithm | live L2 test | Python stops on `step`, C++ on `gradient` at one start; both converge | waive: both converge to the same optimum; the live test rightly asserts convergence only | waived: both converge to the same optimum; the live test rightly asserts convergence only -- decided by evan-wes on 2026-10-02 |

### Hygiene review

## Hygiene review (Run 2)

**Scope:** `git diff e3c60fc...HEAD` (37 files). I read in full: `ba/{bundle,layout,solve}.py`, the `problem.py` diff, the `nls.py` and `arrays.py` diffs, `eval/{readers,reprojection,colmap_reference}.py`, `scripts/{check_layers,compare_ba_colmap}.py`, `generate_ba.py`, and the C++ diffs (`ba.cpp`, `bundle.cpp`, `layout.cpp`, `nls.hpp`, bindings). I read the test names plus the fixture, variance-factor, gauge and anchor tests. I did not review the docs notes or CHANGELOG, and I did not read the `ba.json` data.

**Summary:** The fixes are real and the new code is clean. Validation is centralised, `Damping` and the `_index_*` split read well, and the fixture, generator and test chain is sound. All Run 1 hygiene findings are resolved except H8, which is partly resolved. The remaining items are should-fix or note: a quadratic per-group loop that survives in three places, the unnamed `1e-12` tolerance in both cores, and a few readability and doc nits.

### Run 1 findings
- **H1** resolved. `check_problem` now checks lengths and index ranges first and runs `check_priors`/`check_anchors` last (`cpp/src/foundations/ba.cpp`). `test_ba.cpp` adds the short-`pt_idx` case.
- **H2** resolved. `BAProblem.__post_init__` runs `_validated` on every construction path. `test_construction_validates_however_built` covers it, and `generate_ba.problem_from` uses direct construction.
- **H3** resolved. `solve` is keyword-only, enums cross through named tables, and `codes()` is checked by `test_enum_tables_match_the_compiled_side`. One positional piece is left (see should-fix 5).
- **H4** resolved. `camera.CENTRE_INDICES` and `COLMAP_PIXEL_OFFSET` are each defined once and used by the readers, the reference and `compare_ba_colmap`.
- **H5** resolved. `_parse` turns malformed lines into `InvalidInputError` naming the file and line, and names are split with `maxsplit=9`. One wrong line number is noted below (should-fix 4).
- **H6** resolved. The inline import became `_pycolmap()` with a message pointing to the `colmap` extra, and `pycolmap` is now declared in `pyproject.toml`.
- **H7** resolved. `demo_slice.py` is deleted. `demo_bal.py` defaults to `Path.home()/datasets/bal` and downloads to `.part` before the rename.
- **H8** partly resolved. `rms` is validated, tested by hand, and raises on an empty mask. Its loop is now per intrinsics group, but BAL gives every camera its own group, so BAL is still O(C×M) (see should-fix 1).
- **H9** resolved. The `gauge` docstring now describes all five gauges and points at `ba.layout`. `solve`'s Raises is complete.
- **H10** resolved. `_members` does one sort and slices per group, and `_rays`, `_reprojection` and `_reprojection_kind` use it. The same per-group pattern survives elsewhere (should-fix 1).
- **H11** resolved. `_Damping`/`detail::Damping` carry named constants, and `Bundle` is split into `_index_*` helpers and `_add_camera`/`_add_point`/`_coupling`.
- **H12** resolved as recorded. The waivers are reasonable.

### Findings

#### [should-fix] Per-group masked loops are still O(groups × M), in three places, and `_members` is not shared
- **Where:** `src/slamlab/eval/reprojection.py:68-71` (`rms`), `src/slamlab/foundations/ba/layout.py` (`_unplaceable_anchors`, the `for g in np.unique(group): sel = group == g` loop), and `eval/colmap_reference.py:to_reconstruction`. In the last, `np.flatnonzero(problem.cam_idx == c)` runs per camera and `np.flatnonzero(problem.pt_idx == p)` runs per point, so it is O(C×M) and O(N×M).
- **Issue:** `bundle._members` fixes exactly this with one sort, but it is private to `bundle.py`. The other modules re-implement the masked loop.
- **Why it matters:** `read_bal` gives each camera its own group, so `rms` costs C×M comparisons on BAL. At ladybug size that is about 1.5M and fine; on the BAL problems with millions of observations it is billions. `to_reconstruction` is quadratic in points×observations.
- **Evidence:** the `rms` comment says "one projection per intrinsics group, not per camera", but the `group == g` mask is still computed once per group.
- **Suggested fix:** move `_members` to a shared place (`slamlab.foundations.arrays` or `ba.problem`) and use it in the three sites. In `to_reconstruction`, group observations once with `np.argsort` and `np.searchsorted`.

#### [should-fix] The tolerance `1e-12 * (1 + |origin|)` is an unnamed literal, repeated within each core
- **Where:** `layout.py` in `_confine_scale` and `_gauge`; `cpp/src/foundations/ba/layout.cpp` in `confine_scale` and `fix_gauge`. `kAnchorTolerance` and `ANCHOR_TOLERANCE` are named next to them, but this sibling is not.
- **Issue:** the same "two poses coincide" threshold is written twice per language with no name or justification.
- **Why it matters:** the two uses must agree (one decides whether scale is already pinned, the other whether a farthest pose exists). Changing one silently breaks the gauge logic.
- **Suggested fix:** add `COINCIDENT_TOLERANCE = 1e-12` (C++ `kCoincidentTolerance`) with a one-line justification, plus a small `_coincident(a, b)` helper.

#### [should-fix] `_gauge` / `fix_gauge` repeat what `_anchoring` computes
- **Where:** `layout.py` in `_gauge` (the `spread > tolerance` block) and `layout.cpp` in `fix_gauge`.
- **Issue:** the fixed-observed-poses-apart test duplicates part of `_spread` and `_anchoring`'s scale determination (`anchors.scale`). The explicit `similarity` gauge needs "fixed poses apart pin the scale" and `auto` needs "scale pinned", computed two ways.
- **Why it matters:** the two notions of "scale pinned" can drift apart, and the same logic exists in two forms per language.
- **Suggested fix:** let `fix_gauge` for `similarity` consult the poses-only subset through `_spread`. At minimum, comment why a poses-only test differs from `anchors.scale`.

#### [should-fix] Wrong line reported for a dangling point id, and closure-in-loop readability in the readers
- **Where:** `src/slamlab/eval/readers.py`: `read_colmap_text` (`_parse(images_path, image.line, partial(point_index.__getitem__, ...))`), and `_cameras` / `_images` (nested `parse`, `parse_header` and `parse_keypoints` defined in a loop with default-argument capture).
- **Issue:** `image.line` is the header's line, but a dangling point id sits on the keypoint line, one later. The error text promises "names the line" and names the wrong one. `images_path` is also rebuilt after `_images(root / "images.txt")` already used the path.
- **Why it matters:** misleading diagnostics, and the default-argument capture (`row: list[str] = row`) is the kind of trick that reads as a bug.
- **Suggested fix:** store `keypoint_line` in `_Image` and use it. Promote the parsers to module-level functions taking their arguments. Compute `images_path` once.

#### [should-fix] The result trace still crosses by column position
- **Where:** `src/slamlab/foundations/ba/solve.py:_result_from_backend` (`row[0]` to `row[6]`) against `trace_array` in `cpp/bindings/foundations/ba.cpp` (`kTraceColumns = 7`).
- **Issue:** this is the one remaining packed-by-position piece of H3. A reordered column fails silently (a swapped `cost` and `gradient_norm`) and `test_matches_fixture` does not compare the trace.
- **Suggested fix:** a `TRACE_COLUMNS` tuple of field names in `solve.py` (a test checks it against the C++ column order), or at least add the trace's final row to the fixture comparison.

#### [should-fix] `compare_ba_colmap.py`: unexplained constant and fragile indexing
- **Where:** `scripts/compare_ba_colmap.py:compare`: `tolerance=1e-6`, `ours.intrinsics_std[0, 0]`, `reference.intrinsics[0, 0]`.
- **Issue:** the COLMAP tolerance is a bare literal (the `ColmapOptions` default is `1e-10`) with no reason given. Reporting `[0, 0]` is NaN with `--refine none` and means only "group 0's first focal length" otherwise, which the docstring does not say. `compare` also builds the problem, runs both solvers and formats a row, so it does too much.
- **Suggested fix:** name the tolerance and state why. Say in the docstring and header that the table reports group 0. Split row formatting out of `compare`.

#### [should-fix] Stale module docstring in `colmap_reference.py`
- **Where:** `src/slamlab/eval/colmap_reference.py:11` ("pycolmap is a development dependency, imported only when these functions run").
- **Issue:** `pyproject.toml` now declares the optional `colmap` extra, and `_pycolmap` tells users `pip install slamlab[colmap]`.
- **Suggested fix:** say "an optional dependency (the `colmap` extra)".

#### [note] Smaller readability items
- `ba/bundle.py` has a module function `_members` and an instance attribute `self._members` (a list of tuples). The shadowing is confusing. Rename the attribute (`_by_group`).
- `_add_point(kind, point, ...)` and `_coupling(kind, point, ...)` take `point` as a parameter although it is always `kind.point`. The docstring says this is for the type checker. A `Kind` subtype or an `assert kind.point is not None` inside would be simpler.
- `_reprojection_kind` and `_depth_kind` repeat the rotation-part, `world[...]` and slot construction. They could share a helper like `_camera_side(j_world, world_pt)`.
- `layout.cpp::anchoring` names its unsubtracted matrix `centred`. Rename it `positions_m` or similar, since centring happens two lines later.
- `scripts/check_layers.py::_module_path` does not normalise `..` in quoted relative includes (`"../x.hpp"`), so such includes are silently unconstrained. `_may_import` rebuilds `depth` on every call. Neither matters today, but the docstring claims relative includes resolve.
- `generate_ba.py` says "so the file stays small", but `ba.json` is 272 KB and 17.8k lines because `indent=1` puts every number on its own line. Using compact separators for the numeric arrays would cut the review noise. The fixture's oracle is the Python reference and is documented as such, which is acceptable.
- `variance_factor` uses `2 cost / (m - n)`. With a robust loss, `cost` includes ρ, so σ̂₀² is biased low for outliers. `BAOptions.scale_deviations` does not say so; the algorithm reviewer owns whether this matters.

### Checked and fine
- `_Damping` and `detail::Damping`: named Nielsen constants, `accept`/`reject` mirror each other, and `solve` is shorter. The two cores agree.
- `_variance_factor` / `variance_factor` / `scale_deviations`: identical formula and NaN-without-redundancy rule in both cores. They are tested (`test_variance_factor_*`, `test_ba.cpp`), and the docstrings state m and n.
- Bundle split into `_index_*` helpers, `_add_camera`, `_add_point` and `_coupling`: each does one thing and is documented. The padding trick in `_coupling` is explained.
- `BAProblem.__post_init__` / `_validated`: one path for all construction, with messages that name the argument. The new `as_indices`, `as_mask`, `as_weights` and `allow_nan` helpers are documented and sit in `arrays.py`.
- `_anchoring` / `_resolve_gauge`: the Python and C++ versions are parallel, the minimality rule cites Triggs, and the gauge tests cover auto-motion-only, a single pose prior, and a scale gauge with no anchor.
- Bindings: `decode`, the name tables with `static_assert`s, and `codes()` are tidy. The NOLINT is justified in a comment.
- `check_layers`: `internal_orders` and `check_cpp_internal` are documented and tested (`tests/test_check_layers.py`), and unlisted modules are not constrained, as stated.
- Parity fixture: the generator is seeded and its tolerances carry a stated `reason`. `test_matches_fixture` covers every gauge, both methods, robust losses, degenerate drops, radial, priors and anchors, and asserts values.
- `demo_bal.py`: atomic download, a home-based default path, and `readers.read_bal`.
- `eval/reprojection.py` tests assert values, and the empty, mis-shaped and behind-camera cases are covered.

### Architecture review

## Architecture review

**Scope:** Run 2 re-review of `feature/ba-spine`. Fix range `e3c60fc...cda70e6` (37 files), read against the Run 1 report (`docs/reviews/2026-10-02-feature-ba-spine.md`) and the design doc. Read: `pyproject.toml` contracts, `scripts/check_layers.py` and `tests/test_check_layers.py`, `src/slamlab/foundations/{arrays,camera,nls}.py`, `ba/{problem,layout,solve}.py`, `src/slamlab/eval/{__init__,readers,colmap_reference}.py`, `scripts/{compare_ba_colmap,demo_bal}.py`, `cpp/bindings/foundations/ba.cpp`, `cpp/include/slamlab/foundations/{ba,nls,camera}.hpp`, `cpp/src/foundations/ba.cpp` and `ba/{bundle.cpp,layout.hpp}`, the whole C++ include graph of rung 0, `tests/fixtures/foundations/generate_ba.py` with the head of `ba.json`, the fixture and validation tests in `tests/foundations/test_ba.py`, and the design-doc and CHANGELOG diffs.

**Summary:** The fixes resolve all eleven Run 1 findings marked fixed, and the three deferrals are recorded where the report says. The slice's structure is now sound:
- the readers sit in `eval`, and the pycolmap boundary has a contract;
- options and results cross the binding by name;
- the enum tables are tested across languages;
- BA has a parity fixture with a stated policy;
- the variance factor and the `scale` gauge are in lock-step across both cores.

I found no must-fix. Three should-fix items remain:
- A2's guarantee ("any `BAProblem` is valid") can still be broken by changing a problem's arrays in place.
- The C++ internal-order checker is weaker than import-linter in ways its docstring claims it is not.
- `bearings_or_nan`, a camera capability, sits in the wrong module.

### Run 1 findings

- **A1** (BA parity fixture): **resolved.** `tests/fixtures/foundations/ba.json` has nine cases: `auto` resolving to each of similarity, first_pose, scale and none, an explicit similarity with Gauss-Newton, degenerate drops, radial, Huber, and every block kind. Each output has its own tolerance policy. `test_matches_fixture` runs every backend, including Python, against it. There are residual fixture-design notes below.
- **A2** (reference and C++ fail differently): **resolved for construction and `replace`**, through `BAProblem.__post_init__` and `test_construction_validates_however_built`. One way around it remains (in-place change of the arrays); see the should-fix below.
- **A3** (summary location, stale doc): **resolved / deferral recorded.** The design doc says where the summary lives "as built in PR 1" (around line 130). Its open items say it moves to `foundations.solve` in PR 2. The Status line is updated.
- **A4** (cross-language contract held by position): **resolved.** The binding takes keyword-only arguments (`nb::kw_only()`). `Implementation.solve` is keyword-only, and `test_backends._parameter_names` now reads `kwonlyargs`, so the Protocol test covers every option. Every crossing enum has explicit values. `codes()` is tested against `ENUM_TABLES`, and the range checks come from the name tables. There is a small residual note on how the tables are anchored.
- **A5** (pycolmap via importlib, no extra): **resolved.** pycolmap is a plain local import, there is a `colmap` extra, and the "Only eval may import pycolmap" contract exists. There are small residual notes below.
- **A6** (readers in rung 0): **resolved** in code: `slamlab.eval.readers`, and the eval docstring is rewritten. The design doc's package table (line 224, "Metrics, benchmarks, budget harness") does not record readers or references in `eval`, which A6 also asked for. See the note.
- **A7** (reifyscene yardstick in slamlab): **resolved as decided.** `compare_ba_colmap.py` is generic, and the slice invocation lives in `docs/notes/bundle-adjustment.md` as a recorded experiment. No code knows reifyscene's layout.
- **A8** (copied constants): **resolved.** There is one `COLMAP_PIXEL_OFFSET` (in `eval/readers.py`) and a new `camera.CENTRE_INDICES`. The readers take sizes from `camera.MODELS`.
- **A9** (validators in `arrays.py`): **resolved.** `as_indices`, `as_mask` and `as_weights` are in `arrays.py`, and `as_real(allow_nan=True)` handles `_depth_prior`.
- **A10** (internal order not applied to C++): **resolved as specified.** `check_cpp_internal` applies every sub-rung layers contract to includes. Its semantics differ from import-linter's; see the should-fix below.
- **A11** (batching in bindings): **deferral recorded** (design doc, "Deferred from PR 1's review panel", before milestone 7).
- **A12** (sparse reduced solver): **deferral recorded** (milestone 4). The record drops two points from the finding:
  - the solver belongs in **rung 0**, not `classic`;
  - C++ `ba::solve` is not yet parametrised on `ReducedSystemSolver`, so the C++ side has no "same seam" yet.

  It is worth adding one sentence.
- **A13** (order inside `ba`): **resolved.** The "Bundle adjustment's internal order" contract is `solve > bundle > layout | schur > problem`.
- **A14** (eval tests location): **resolved.** `tests/eval/test_readers.py` and `tests/eval/test_reprojection.py` exist.

### Findings

#### [should-fix] A validated `BAProblem` can still be changed in place, and `point_anchor` shares memory with the caller's array
- **Where:** `src/slamlab/foundations/ba/problem.py:96-111` (`__post_init__`) and `:413` (`_anchors` returns `np.ascontiguousarray(arr, dtype=np.int64)`, which does not copy an int64 contiguous input)
- **Issue:** The dataclass is frozen, but its arrays are writable. Every other validator returns a copy; `_anchors` does not. So a caller can make a validated problem invalid in two ways:
  - by writing to a field, e.g. `problem.pt_idx[0] = 99`;
  - by changing the array it passed as `point_anchor`.

  The solver trusts the problem, so the original A2 divergence comes back.
- **Why it matters:** `__post_init__`'s docstring promises "a solver can trust any `BAProblem` it is given". That is the A2 fix, and it does not fully hold.
- **Evidence:** I ran `PYTHONPATH=. conda run -n slamlab python -c ...` with `p = _scene('pinhole', seed=1)` from the generator:
  ```
  point_anchor aliased: True          # np.shares_memory(a, replace(p, point_anchor=a).point_anchor)
  python IndexError index 99 is out of bounds for axis 0 with size 24    # after q.pt_idx[0] = 99
  cpp InvalidInputError pt_idx: indices must lie in [0, 24)
  ```
- **Suggested fix:** In `__post_init__`, copy `point_anchor` and set `flags.writeable = False` on every validated array. If nothing writes to them, this costs nothing; if something does, it surfaces. Add the in-place case to `test_construction_validates_however_built`.

#### [should-fix] `check_layers.py`'s internal-order check is weaker than import-linter but says it matches it
- **Where:** `scripts/check_layers.py:22-28` (docstring: "not constrained, as in import-linter"; line 4: "so the three can never disagree") and `:232-247` (`_check_include`)
- **Issue:** There are three gaps between the C++ check and the import-linter contract it claims to mirror:
  1. **Direct includes only.** import-linter's layers contracts reject *chains*, including chains through modules no layer lists. The C++ check looks at one include at a time. So a C++-only helper that no contract lists (today `ba/common.hpp`, which has no Python counterpart) is a free bridge. For example, `schur.hpp → common.hpp → bundle.hpp` passes.
  2. **The facade exemption is the wrong way round.** The comment says the facade "(ba.hpp) is above its modules". Yet `schur.hpp`, `layout.hpp` and `bundle.hpp` all include `slamlab/foundations/ba.hpp`, which by that statement is an upward include, and the check skips it in both directions. The real correspondence is that C++ `ba.hpp` holds what Python splits between `problem` (the types, bottom layer) and the `solve` declaration (top layer).
  3. **No Rust check.** The internal-order check does not apply to Rust modules, and `internal_orders` silently replaces an earlier contract on the same container (`orders[...] = layers`).
- **Why it matters:** CLAUDE.md says the script holds C++ "to the same contracts as import-linter". Today's include graph is clean (I checked every `#include "..."` in `cpp/src/foundations` and `cpp/include/slamlab/foundations`), but the gaps will matter as C++-only helpers and Rust modules (se3, the edge VO) arrive.
- **Evidence:** In `_check_include`, `if len(here) <= depth or len(target) <= depth: continue` skips every include of the facade. There is no transitive closure over `_ANY_INCLUDE` matches. `rust/slamlab-foundations/src/` has `so3.rs` and `error.rs`, which nothing checks.
- **Suggested fix:** Do one of these two:
  - **Close the gaps:** follow includes through unlisted modules (a reachability walk per file), and map `ba.hpp` explicitly to `problem` in a table like `_CPP_MODULE_ALIASES`, so including it from inside `ba` is a downward include. Raise an error on duplicate containers.
  - **Narrow the claims:** state in the docstring and in CLAUDE.md that the check covers direct includes only, that the facade is exempt, and that it does not cover Rust.

  I recommend the first, because it is small.

#### [should-fix] `bearings_or_nan` is a camera capability placed in BA's layout module
- **Where:** `src/slamlab/foundations/ba/layout.py:196-212`, imported by `ba/bundle.py:28`
- **Issue:** "Unproject a batch; mark the pixels that cannot be undistorted" is a property of the camera model. C++ gets it from `camera::unproject`'s per-item `std::expected` (`ba/bundle.cpp:430`, `ba/layout.cpp:252`). Python's `camera.unproject` aborts the whole batch on the first failure, so BA builds its own per-pixel fallback around the exception. `bundle` then imports a camera utility from `layout`.
- **Why it matters:** PnP, RANSAC and the edge VO will need the same per-pixel answer, and each would either import it from `foundations.ba` or copy it. This is the Python face of A11 (per-item failure semantics live at the batch boundary).
- **Suggested fix:** Move it to `slamlab.foundations.camera`, either as a function or as an option on `unproject`, and import it in both BA modules. Alternatively, fold it into the A11 deferral record by name.

#### [should-fix] The explicit `"scale"` gauge error is tested only on the Python reference
- **Where:** `tests/foundations/test_ba.py:759-762` (`test_scale_gauge_needs_an_anchor`); C++ counterpart at `cpp/src/foundations/ba/layout.cpp:152`
- **Issue:** This PR adds a new `InvalidInput` path in both cores, but only the reference is tested against it. The fixture covers only successful solves.
- **Why it matters:** The design doc requires that every language fails the same way. Without the test, a change to the C++ check would go unnoticed.
- **Suggested fix:** Parametrise the test over `backends.BACKENDS`, with `require_function`, as `test_matches_fixture` does.

#### [note] The BA fixture is not self-describing: options and the exact-match policy live in the generator
- **Where:** `tests/fixtures/foundations/generate_ba.py:198-212` (`options_from`), `:57` (comment: "summary codes and counts compare exactly"); `tests/foundations/test_ba.py:21` imports `options_from` and `problem_from` from the generator
- The fixture stores only each case's overrides, e.g. `{"loss": ["huber", 2.0]}`:
  - The tight tolerances (`max_iterations=100`, `function_tolerance=1e-12`, `gradient_tolerance=1e-10`) and every other option default are supplied by Python code at test time.
  - The exact-match policy for codes, counts and `iterations` is stated in a comment, not in the JSON's `tolerance` block.

  Parity between backends holds, because both get the same `backend_arguments`. But the fixture is not the complete contract, so a non-Python consumer could not replay it.
- Two more points:
  - Exact `iterations` equality depends on a float threshold (the relative decrease against `function_tolerance`) landing the same way in both languages. It is reasonable to hold the line now; if it ever flakes under a compiler or FMA change, the policy should say so rather than loosen silently.
  - Suggestion: write the full option set (names, as `backend_arguments` builds it) into each case's input, and add a `"codes": {"comparison": "exact"}` entry to the tolerance block.

#### [note] Small A4/A5/A6 residuals
- `cpp/bindings/foundations/ba.cpp:44-58`: the name tables for model, gauge, method and loss are in the binding, not the core (only `Termination` has a core `nls::name`). The `static_assert`s check only that the *current* last enumerator plus 1 equals the table size. If an enumerator is appended in C++ only (for example, the planned time-cap termination), the build passes, `codes()` still matches Python, and Python fails with `IndexError` on the new code. A `kCount` sentinel, or a core `name()` for each enum, closes this.
- `src/slamlab/eval/colmap_reference.py:7`: the docstring says "pycolmap is a development dependency"; it is now the `colmap` extra.
- `pyproject.toml:36`: pycolmap is pinned both in `dev` and in `colmap`. `dev` could list `slamlab[colmap]`, so there is one pin.
- The pycolmap contract lists the forbidden *sources*, which is a deny-list: a new top-level package is not covered unless someone adds it. An allow-list is safer: `source_modules = ["slamlab"]` with `ignore_imports = ["slamlab.eval.colmap_reference -> pycolmap"]`.
- Design doc line 224: the `slamlab.eval` row should say "readers, external references, metrics, benchmarks, budget harness", to record A6's placement decision (TUM and EuRoC follow it at milestone 2).

### Checked and fine
- Dependency direction: `foundations` no longer references readers or pycolmap. `eval` imports only `foundations`. The scripts use public `eval`/`foundations` exports. Nothing in `src/` references reifyscene's layout.
- The binding is a thin type conversion: keyword-only arguments, float64/int64/bool arrays with `noconvert`, a dict of named results, and no algorithm logic. The `scale` gauge and `variance_factor` cross by name, and `codes()` is tested against `ENUM_TABLES`.
- Lock-step: `variance_factor` (`2 cost / (m - n)`, NaN without redundancy), `scale_deviations`, the `scale` gauge, the fixed-point and anchor rejection, and Nielsen's damping (`_Damping` / `detail::Damping`) all match between the Python reference and the C++ core. `make_layout` now returns `Result<Layout>` in C++, and raises `InvalidInputError` in Python.
- Error contract: C++ `check_problem` now checks lengths and indices before priors and anchors, which fixes H1's out-of-bounds read. The new rejections use `InvalidInput` in both languages.
- Validation once: `BAProblem.__post_init__` (via `_validated`) is the single Python check. The compiled core's re-check is documented in `ba.solve`'s Raises section.
- Fixture tolerance policy: one policy per output with a stated reason, applied through `conftest.assert_matches_fixture`. NaN is written as `null`. The generator refuses to write a case the reference did not converge on.
- The new import-linter contracts are well-formed: the `ba` order matches the actual imports (`bundle` imports `layout` and `schur`, `solve` imports all three), and the root config has `include_external_packages = true`.
- `tests/test_check_layers.py` covers upward, sibling, relative, angle-bracket, facade, unlisted-module and system-header cases.
- `compare_ba_colmap.py` is generic (`NAME=DIR` arguments); the reifyscene invocation is documentation only.
- The design doc's Status line, closed and open items, and deferral records (A3, A11, A12, L6, L7) are present and match the code.

One side effect: my evidence run (`conda run` in the repo) triggered the editable install's automatic C++ rebuild into the git-ignored `build/slamlab/`. No tracked files changed.

### Algorithm review

## Algorithm review

**Scope:** Fix range `e3c60fc...cda70e6` on feature/ba-spine, with the full PR (`0d9366f...cda70e6`) as context. I read these in full:
- `src/slamlab/foundations/ba/{layout,solve,bundle,problem}.py` (the diffs, plus all of `layout.py`)
- `cpp/src/foundations/ba/{layout,bundle}.cpp`, `cpp/src/foundations/ba.cpp`, `cpp/include/slamlab/foundations/{ba,nls}.hpp`
- `src/slamlab/foundations/nls.py` (`_Damping`)
- `src/slamlab/eval/colmap_reference.py`
- the new tests in `tests/foundations/test_ba.py`
- the diff of `docs/notes/bundle-adjustment.md`

References: Triggs et al. 1999 §9 (gauge), Förstner & Wrobel 2016 ch. 4 (variance factor), Huber, *Robust Statistics* (1981) for M-estimator scale, and COLMAP's `bundle_adjustment.cc` for solver selection. I ran small probes against the HEAD build with both backends. The scripts are in `/tmp/scratch/` (`run2_partial_anchor.py`, `run2_unobserved_ref.py`, `run2_variance_rows.py`, `run2_robust_factor.py`). Run them from the slamlab root with `PATH=<slamlab env>/bin:$PATH PYTHONPATH=. python <script>`. No repository files were modified.

**Summary:** Both cases Run 1 reported are fixed in both cores and covered by tests: the frozen pose under motion-only BA, and the free scale under a single pose prior. The new `scale` gauge is sound: under a full prior on one camera, it removes exactly the one null direction (scale about the reference centre). L2, L4, L5 and L10 are resolved, and the Nielsen damping refactor is behaviour-identical in both languages.

**One must-fix remains.** The new `auto` gauge sorts the anchoring into two yes/no flags, rigid motion and scale. Any partial anchoring therefore falls through to fixing a whole pose. I reproduced four noise-free cases where `auto` leaves cost between 4.7 and 29, while `none` reaches about 1e-21. This is the same kind of silent bias L1 was about.

The variance factor has two counting and numerator problems (should-fix), and the new state-of-the-art text has one inaccurate claim about COLMAP (note).

### Run 1 findings
- **L1 (must-fix): partly resolved.** Motion-only BA (`test_auto_gauge_motion_only`) and a single full prior (`test_auto_gauge_single_pose_prior`) now resolve correctly in both cores. My Run 1 probes now give: motion-only → `none`, cost 2.3e-20, pose error 0; one prior → `scale`, finite f_std, unobservable_focal = 0. Two gaps remain:
  - partial anchoring still over-constrains (new must-fix below);
  - a prior on an unobserved camera re-creates the original symptom (should-fix below).
- **L2: resolved.** Python `bearings_or_nan` gives NaN rays, so the step is rejected. At the start, both cores drop the point, and the drop rule matches (`layout.py:215-254`, `layout.cpp` `degenerate_points`). My Run 1 probe now converges to k1 = -0.0800 on both backends for k1 from -1.1 to 1.0. `test_anchor_pixels_outside_the_invertible_region` covers the mid-solve and start cases.
- **L3: implemented; two issues with the factor itself** (two should-fix findings below). The scaling, its option, the NaN for m ≤ n and the backend parity are correct. The slice table's σ_f values equal the old ones times σ̂₀ (for example 3.5 × 0.68 = 2.4 ≈ 2.3, and 902 × 0.27 = 244 ≈ 241).
- **L4: resolved.** `np.minimum.at` is used in `problem.py:474` and `layout.py:64`.
- **L5: resolved.** `_check` rejects priors, weights other than 1, fixed poses and fixed points. `refinement()` takes the masks from the problem and rejects mixed masks and a free principal point.
- **L6, L7: deferred.** The design-doc records exist; nothing to check.
- **L8: resolved.** The citations are accurate; one claim about COLMAP needs a correction (note below).
- **L9: resolved.** Sturm (CVPR 1997) and Kahl, Triggs & Åström (JMIV 2000) are cited correctly.
- **L10: resolved.** Both cores reject this in validation (`problem.py:409`, `ba.cpp` `check_anchors`, which now runs after the size checks).

### Findings

#### [must-fix] `auto` still over-constrains partial anchoring: it fixes a whole pose when the data pins only some gauge directions
- **Where:** `src/slamlab/foundations/ba/layout.py:101-143` (`_anchoring`, `_resolve_gauge`). Mirrored in `cpp/src/foundations/ba/layout.cpp` (`anchoring`, `resolve_gauge`).
- **Issue:** The anchoring is reduced to two flags, `rigid` and `scale`. Anything short of "rigid" (a fixed pose, a full prior, or three non-collinear known positions) resolves to `first_pose` or `similarity`. That fixes all six DoF of pose 0 at its starting value, even when priors or fixed points already pin some of those DoF. Cases that hit this:
  - one fixed point (pins 3 DoF);
  - two fixed points, or translation priors on two cameras (pin 6 DoF; only rotation about the baseline is free);
  - translation priors along a collinear track;
  - rotation-only (attitude) priors (pin 3 DoF).
- **Why it matters:**
  - Triggs' minimality is violated, and the result is silently biased: pose 0 stays at its perturbed start while the priors and fixed points pull elsewhere.
  - `auto` is the default, and these are ordinary inputs: a couple of control points, GNSS on a few frames, IMU attitude.
  - The docstrings (`layout.py:12`, `BAOptions.gauge`) and the note claim `auto` "removes exactly what the fixed variables and priors leave free". That is false for these inputs.
- **Evidence:** `run2_partial_anchor.py`, on the noise-free `_scene(cameras=5, points=60, perturb=0.02, seed=4, rotation=0.3)`. Priors and fixed points are at their true values; both backends give the same result.
  ```
  translation priors on 2 cams  auto -> first_pose gp=0  cost=4.747e+00 | none cost=7.2e-22
  rotation priors on all cams   auto -> similarity gp=0 sp=4 cost=9.577e+00 | none cost=1.0e-19
  one fixed point               auto -> similarity gp=0 sp=4 cost=5.362e+00 | none cost=4.6e-21
  two fixed points              auto -> first_pose gp=0  cost=2.936e+01 | none cost=2.6e-23
  ```
  Triggs et al. 1999, §9: gauge constraints must be minimal.
- **Suggested fix:** Compute the remaining gauge, not two flags.
  1. Stack, for every anchor, the derivative of the anchored quantity with respect to the 7 similarity generators (t, ω, s):
     - a known position X gives the rows `[I, −hat(X), X]`;
     - a fixed pose or full prior gives its centre rows plus `[0, I, 0]`;
     - a rotation-only prior gives `[0, I, 0]`;
     - metric depth gives `[0, 0, 1]`.
  2. Take the null space N of the stacked matrix. `auto` maps onto the existing gauges only when N is {} (`none`), scale about a reference (`scale`), the rigid motions (`first_pose`) or everything (`similarity`).
  3. For any other N, either generalise `_confine_scale` to project one pose's step off N's image (a QR/Householder complement of dimension 6 − d), or raise `InvalidInputError` asking for an explicit gauge.

  Add tests for one fixed point, two translation priors and rotation-only priors, in both cores.

#### [should-fix] A pose prior on an unobserved camera counts as an anchor and re-creates L1's symptom
- **Where:** `layout.py:111-127`. `translation` and `full` are not filtered by `observed`, while `fixed_cams` is. The C++ `anchoring` loop does the same.
- **Issue:** An unobserved camera is held fixed and is not connected to the scene. Its prior pins nothing about the observed cameras. Yet it becomes the `reference` (so `rigid=True`), and its prior centre counts as a known position.
- **Why it matters:** A camera becomes unobserved whenever all its observations are zero-weighted or dropped (behind the camera, or degenerate). `auto` then picks `scale`, which leaves the rigid gauge free. The reduced system is singular, and focal reads as unobservable: exactly what L1 fixed.
- **Evidence:** `run2_unobserved_ref.py`: `_scene(cameras=5, noise=0.5, seed=3, rotation=0.3)` with free focal, camera 4's observation weights set to 0, and a full prior on camera 4. Both backends:
  ```
  no prior                   gauge=similarity f_std=[1.409 1.997] unobservable_focal=0
  prior on unobserved cam 4  gauge=scale  sp=0 f_std=[inf inf]    unobservable_focal=1
  ```
- **Suggested fix:** Count priors only on observed cameras, as fixed poses already are (`& observed` on `translation` and `full`). Add the case above as a test.

#### [should-fix] The variance factor's m counts 6 rows for a half pose prior
- **Where:** `bundle.py` `residual_rows` (`POSE_DIM * self._p_cam.size`) and `bundle.cpp` `residual_rows`. `prior_cams` is every camera with *any* positive prior weight (`layout.py:273`).
- **Issue:** A translation-only or rotation-only prior has three zero-weight rows. They contribute no cost and no information, but they still add 3 to m. So the redundancy m − n is overstated and σ̂₀² is understated.
- **Why it matters:** In small problems, or problems with many priors, the deviations and the 1% focal rule shift. The partial priors that the note explicitly supports are exactly the affected case.
- **Evidence:** `run2_variance_rows.py`: 3 cameras, 6 points, translation-only priors on all cameras, gauge `none`.
  ```
  variance_factor reported 0.1473   (m=54, n=36)
  with m counting only weighted rows: 0.2947 (m=45, n=36)
  ```
- **Suggested fix:** Count `3 · (number of positive weights)` per prior camera, in both cores. Optionally, document that constant blocks (priors on fixed or unobserved poses) still count.

#### [should-fix] Under a robust loss, 2·cost = Σρ overstates σ̂₀² and is inconsistent with the information matrix used
- **Where:** `solve.py:_variance_factor` and `bundle.cpp` `variance_factor`; the note's "the residual sum of squares over the redundancy".
- **Issue:** The deviations come from `(Jᵀ W J)⁻¹` with the final IRLS weights. The numerator, though, is Σρ(sᵢ), not the IRLS-weighted Σ wᵢ sᵢ that pairs with that matrix:
  - Huber: for an outlier ρ = 2δ√s − δ², about twice w·s = δ√s;
  - Cauchy: ρ grows like log(s), while w·s is bounded by δ².

  Gross outliers therefore inflate σ̂₀², and through it σ_f and the `unobservable_focal` flag. Under the trivial loss the two numerators are identical, so all current tests pass.
- **Evidence:** `run2_robust_factor.py`: 0.5 px noise (true factor 0.25), a fraction of observations replaced by ±20 px outliers, δ = 1.

  | Outliers | Loss | Reported (2·cost) | Σ w s / (m − n) |
  |---|---|---|---|
  | 0% | Huber | 0.221 | 0.217 |
  | 5% | Huber | 1.547 | 0.904 |
  | 5% | Cauchy | 0.395 | 0.172 |
  | 15% | Cauchy | 0.776 | 0.231 |

  With 5% outliers under Huber, σ_f is inflated about 2.5×. Huber (1981, ch. 7) and Förstner & Wrobel (2016, §4.7) use a weighted or robust scale estimate for M-estimators, not Σρ.
- **Suggested fix:** Use Σ wᵢ sᵢ / (m − n) with the final IRLS weights in both cores, or a robust scale (MAD of the normalised residuals). State the choice in the `BAOptions.scale_deviations` and `variance_factor` docstrings and in the note. Add a known-answer test with a robust loss and outliers.

#### [note] COLMAP's linear solver depends on problem size; the slice timing attribution should be rechecked
- **Where:** `docs/notes/bundle-adjustment.md`, "Toward larger problems": "This is Ceres' `SPARSE_SCHUR`, and what COLMAP runs". Also the earlier "remaining gap … is the dense reduced system and single-threading".
- **Correction:** COLMAP's `bundle_adjustment.cc` chooses by image count:
  - `DENSE_SCHUR` up to 50 images;
  - `SPARSE_SCHUR` up to 1000;
  - beyond that, `ITERATIVE_SCHUR` with `SCHUR_JACOBI`.

  `colmap_reference.solve` does not override this. So on lab-coat (47 images) and judo (34 images), COLMAP also solved densely. The gaps there are not explained by "dense vs sparse"; multithreading and the cost of Jacobian evaluation are the likelier causes.
- Separately, the Power BA abstract claims to "challenge the state-of-the-art iterative methods" on BAL. "Faster solves than preconditioned CG" is a fair paraphrase only if the paper's tables show it. Consider quoting the abstract's wording.

#### [note] Termination differs between backends at one L2 start
- `_undistortion_scene(-1.0)`: Python stops on `step`, C++ on `gradient`. Both converge to k1 = −0.0800 with cost about 1e-25. The test asserts only `converged`, which is right for a live comparison. The fixture's exact termination policy does not include this case, so this is not a defect.

### Checked and fine
- **`scale` gauge with a free reference.** The confined pose's step is projected off its baseline to the (moving) reference centre, rebuilt at every state. The only Hessian null direction under one full prior is scale about the reference centre, so this removes exactly one DoF.
- **Known-position test.** The SVD rank test (`_spread`) is order-independent and matches between the cores. Its tolerance is relative to magnitude.
- **`first_pose` / `similarity`.** These paths are unchanged in behaviour. Two separated fixed poses still add no scale constraint. `scale` without a reference raises `InvalidInput` in both cores.
- **L2 drop at the start.** `bearings_or_nan` retries per pixel only after a batch failure. The start-time drop runs only on points not already degenerate, matching C++. NaN rays propagate to inf block costs, so the step is rejected.
- **Variance factor counting otherwise.** Anchor self-observations are excluded from m and an anchored point counts 1 in n. This gives the same redundancy as a plain point with all its observations (2k − 3). n is taken from the reduced coordinates (5 for the scale pose). m ≤ n gives NaN and no scaling. Scaled σ equals √σ̂₀² × unscaled σ in both backends.
- **Damping refactor.** `_Damping` / `detail::Damping` reproduce the previous Nielsen update (shrink by at most 1/3 via 1 − (2r − 1)³; ν reset on accept, doubled on reject) exactly.
- **First observation.** `np.minimum.at` gives the lowest kept observation in both `anchor_points` and `_ray_parallax`, matching the explicit C++ loops.
- **Note claims and citations.** Civera 2008, Engel 2018, Demmel 2021, Weber 2023, Agarwal 2010, Sturm 1997, Kahl/Triggs/Åström 2000 and Förstner & Wrobel 2016 are all correctly attributed. The slice's σ̂₀ values agree with RMS/√2 corrected for redundancy. "0.5 px noise under unit weights overstates σ by 2" is correct.

Sources:
- [Triggs et al., Bundle Adjustment -- A Modern Synthesis (1999)](https://link.springer.com/chapter/10.1007/3-540-44480-7_21)
- [Weber et al., Power Bundle Adjustment, arXiv:2204.12834](https://arxiv.org/abs/2204.12834)
- [COLMAP bundle_adjustment.cc solver selection (mirror)](https://huggingface.co/camenduru/ceres-solver-v1/blob/main/colmap/src/optim/bundle_adjustment.cc)
- [Ceres Solver FAQ: choosing a linear solver](https://ceres-solver.readthedocs.io/latest/solving_faqs.html)

## Run 3 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/ba-spine` |
| **Range** | `85b86fd...45019840772e7594fba4b1e2baded00e4bcb161e` (Run 2's fixes; full PR range `0d9366f...4501984` as context) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents (symlinked into reifyscene's `.claude/`) |
| **Layer 1** | all checks passed on the code at f24d033..b4196f7 (`scripts/check.sh --env slamlab`); later commits are docs and report only |

Run 2's findings: all resolved as recorded, except L11 (partly: see L17).

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L17 | must-fix | algorithm | `ba/bundle.py` (`_pose_error`), `bundle.cpp`, `gauge` rows | the translation half of a pose prior pins `R̄ᵀt`, not the camera centre: the gauge rows assume a known centre (rotation about the origin left free, σ_f inf), and a GNSS-style prior with identity rotation biases centres by up to 0.76 m | fix (decoupled residual `(C − C̄, log(R̄_wcᵀ R_wc))`) | pending |
| L18 | must-fix | algorithm | `ba/layout.py`, `gauge`, both cores | rank tolerance absolute in uncentred coordinates: scenes 5e4 m or more from the origin lose the gauge (σ_f inf; at 1e6 m the reference stops at cost 264 vs 41.7); regression | fix (normalised coordinates, relative tolerance) | pending |
| L19 | should-fix | algorithm | `gauge.plan`, `confinement` | a gauge the plan cannot remove is not reported, and the confinement then takes an arbitrary part of it (cores can differ) | fix (report the leftover dimension; assert the rebuilt rank) | pending |
| L20 | note | algorithm | BA note, `generate_ba.py` | "stiff translation priors converge slowly" may be the residual's near-null direction, not the problem | fix (re-measure after L17, correct the note and the fixture case) | pending |
| L21 | note | algorithm | `layout.py` | dropping constant prior blocks makes the reported cost depend on the gauge | fix (say so in `BASummary`) | pending |
| L22 | note | algorithm | variance factor | down-weighted rows count fully in m (σ̂₀² slightly low under Cauchy) | waive: standard, defensible; an effective redundancy is a refinement for later | pending |
| A21 | should-fix | architecture | `pyproject.toml` | the allow-list pycolmap contract lost indirect-import coverage (orchestration → colmap_reference → pycolmap passes) | fix (forbid orchestration → `eval.colmap_reference`) | pending |
| A22 | should-fix | architecture | `ba.cpp`, `problem.py` | "report-only gauge" encoded by position (`> kScale`, `GAUGES[:-1]`) (same as H28) | fix | pending |
| A23 | should-fix | architecture | fixture | explicit `first_pose` and `scale` (and `none`) have no cross-backend parity | fix (fixture cases) | pending |
| A24 | note | architecture | `check_layers.py` | the facade header's own includes are checked as `solve`; an unmapped facade is silently exempt | defer (design doc: when the next package facade arrives) | pending |
| A25 | note | architecture | `ba/layout.py`, `gauge.py` | Python splits the plan, C++ carries it whole; Python gauge helpers public | fix (carry `Plan`; private helpers) | pending |
| A26 | note | architecture | `ba/gauge` | the gauge is generic to pose-and-point problems | defer (design doc: lift to `foundations.gauge` with pose-graph optimisation) | pending |
| A27 | note | architecture | tests, `solve.py`, bindings | codes compared outside `assert_matches_fixture`; `_CONVERT` KeyError; two enum-name mechanisms; nls.hpp comment | fix the first two and the comment; defer the enum mechanism (with Rust's enums) | pending |
| H21 | should-fix | hygiene | `gauge.py`, `gauge.cpp` | tie-break `1e-12` unnamed, twice per core | fix | pending |
| H22 | should-fix | hygiene | `gauge.plan` | `plan()` long; tie-break names opaque | fix | pending |
| H23 | should-fix | hygiene | `gauge.py`, `layout.py` | `name` shadows and forces an alias | fix (rename `classify`) | pending |
| H24 | should-fix | hygiene | `gauge.py` | public functions without Parameters/frames | fix | pending |
| H25 | should-fix | hygiene | tests | no unit tests of the gauge primitives in either core | fix | pending |
| H26 | should-fix | hygiene | `bundle.cpp`, `nls.hpp`, `layout.cpp` | stale or garbled C++ comments | fix | pending |
| H27 | should-fix | hygiene | `layout.cpp` | `fix_gauge` mutates an out-parameter; `make_layout` long | fix (return a value; helpers) | pending |
| H28 | should-fix | hygiene | `problem.py`, `ba.cpp` | `GAUGES` serves two purposes (same as A22) | fix (with A22) | pending |
| H29 | should-fix | hygiene | `layout.py` | three `Layout` fields undocumented | fix | pending |
| H30 | should-fix | hygiene | `check_layers.py` | `check_cpp_internal` deeply nested | fix | pending |
| H31 | should-fix | hygiene | `compare_ba_colmap.py` | header and row formats written twice | fix | pending |
| H32 | should-fix | hygiene | `arrays.group_members` | no input checks or edge tests | fix | pending |
| H33 | note | hygiene | various | `TOLERANCE` doc; `RIGID_DIM`; the fixture regex; `confinement` signatures differ | fix the doc and `RIGID_DIM` (with L18), signatures (with A25); waive the regex (tested by the fixture's round trip) | pending |

### Hygiene review

## Hygiene review

**Scope:** Run 3 re-review, `85b86fd...45019840` on `feature/ba-spine`. I read `ba/gauge.py` and `ba/layout.py` in full, plus `gauge.{hpp,cpp}`, `layout.{hpp,cpp}`, the `Bundle.reduction`, `residual_rows` and variance-factor diffs in both cores, the `solve.py`/`problem.py`/`camera.py`/`arrays.py` diffs, `scripts/check_layers.py`, `eval/readers.py`, `scripts/compare_ba_colmap.py`, the fixture generator and the new tests. I did not rebuild anything.

**Summary:** All eight Run 2 hygiene findings are resolved. The new null-space gauge is correct in structure and the two cores match closely. What remains is should-fix: the tie-break epsilon is an unnamed literal in two places per core, `plan()` and C++ `fix_gauge`/`make_layout` are too long, two docstrings are stale or garbled, and `gauge.py` has no unit tests of its own. There are no must-fix items.

### Run 2 findings
- **H13** (shared `_members`, per-group masked loops): resolved. `arrays.group_members` is shared and tested, and `reprojection.py` and `colmap_reference.py` both use it. The O(K x M) loops are gone.
- **H14** (unnamed coincident-pose tolerance): resolved. The old gauge code is gone. New constants are named (see S1 for the one that is not).
- **H15** (scale-pinned computed two ways): resolved. One null-space test, `remaining()`, replaces both.
- **H16** (dangling point id reports header line; default-arg closures): resolved. `_Image.keypoint_line` is added, the parsers are module-level and bound with `partial`, and `test_readers.py` asserts `images.txt:3:`.
- **H17** (trace crosses by column position): resolved. The binding sends `trace_columns` and `_result_from_backend` reads by name, raising if they differ from `nls.Iteration`.
- **H18** (compare script): resolved. It uses `options.solver.function_tolerance`, the docstring and header say the columns are intrinsics group 0, and the work is split into `freed`, `compare` and `row`. S11 is a residual duplication.
- **H19** (pycolmap docstring): resolved.
- **H20** (note): resolved as decided. The `_by_group` rename and the compact fixture are in. `centred` went with the old code. The `..` includes are handled in `check_layers`.

### Findings

#### [should-fix] S1. The tie-break epsilon `1e-12` is an unnamed literal, twice per core
- **Where:** `src/slamlab/foundations/ba/gauge.py:191-193` and `:225`; `cpp/src/foundations/ba/gauge.cpp:102` and `:196-198`
- **Issue:** The relative `1e-12` that decides near-ties (in `plan()` and in `_first_largest`/`first_largest`) is written as a bare number four times. `TOLERANCE` is named, but this value is not.
- **Why it matters:** The cross-core agreement of the gauge depends on it. The cores can only change it in lockstep, but nothing says the four sites are one constant.
- **Evidence:** `values >= values.max() * (1.0 - 1e-12)`, `second > best_second + 1e-12 * abs(best_second)`, and the C++ equivalents.
- **Suggested fix:** Add `TIE_RTOL = 1e-12` in `gauge.py` and `kTieRtol` in `gauge.hpp`, each with a one-line justification (last-bit rounding between the two SVDs).

#### [should-fix] S2. `plan()` is too long and its tie-break names are opaque
- **Where:** `gauge.py:169-207` (`plan`, `_preference`) and `gauge.cpp:177-215`
- **Issue:** `plan()` mixes candidate scoring, selection, bookkeeping and null-space deflation in one loop with nesting depth 3. The score tuple is `(rank, second)`, where `second` is overloaded: `-cam` for a pose that sees all six directions, otherwise the smallest retained singular value. `_preference` hides this behind a one-line summary.
- **Why it matters:** This is the subtlest logic in the PR and the part most likely to be edited wrongly.
- **Suggested fix:** Extract `_best_candidate(...) -> (cam, rank, score)` in both cores. Rename `second`/`best_second` to `tie_break`, and `_preference` to `_tie_break`.

#### [should-fix] S3. `name` shadows and forces an import alias
- **Where:** `gauge.py:121`, `layout.py:32` (`from ...gauge import name as gauge_name`) and `layout.py:150` (`name = gauge_name(...)`); C++ `gauge.hpp:name(const MatrixXd&)`
- **Issue:** A free function called `name` needs an alias at its only caller. A local `name` sits next to the aliased import.
- **Suggested fix:** Rename it `classify` or `gauge_name` in both cores and drop the alias.

#### [should-fix] S4. Several public `gauge.py` functions have one-line docstrings with no Parameters section or frame statement
- **Where:** `gauge.py:105-112` (`remaining`), `:115-121` (`pose_image`), `:240-255` (`complement`), `:258-268` (`confinement`), `:124` (`name`)
- **Issue:** The spec asks for shape, dtype and frame. `centres` is stated as "world centres" only in `plan`. `pose_image` does not say the output rows are (centre, orientation) in the world frame. `complement` and `confinement` say nothing about which arguments are `T_world_cam` centres or the order of the returned list. Only `plan` has a numpy-style Parameters block.
- **Suggested fix:** Add Parameters and Returns blocks. For `centres`, write "ndarray, shape (C, 3), camera centres in the world frame". Say that `confinement` returns one (6, 6 - rank) basis per confined pose, in the plan's order.

#### [should-fix] S5. `gauge.py` has no unit tests; correctness is exercised only through full solves
- **Where:** there is no `tests/foundations/test_ba_gauge.py`, and no C++ test touches `gauge.cpp`
- **Issue:** The new behaviour tests are end-to-end and good: `test_auto_gauge_removes_only_what_is_free`, the unobserved-prior test and the variance-factor tests. The primitives are untested directly: `remaining()` dimensions for known anchor sets, `name()` for each of the five outputs, `complement()` orthonormality and its orthogonality to the normals, and `plan()` with no candidates or when no candidate sees any direction. In C++ the only gauge assertion is the single `gauge_pose == 0 && confined_pose == 3` line in `cpp/tests/foundations/test_ba.cpp`.
- **Why it matters:** A regression in the tie rules or the Householder helper would surface only as a fixture mismatch, with no pointer to the cause.
- **Suggested fix:** Add small value-asserting tests for the five primitives in Python, and a few C++ checks of `remaining`, `name` and `complement` (for example a fixed pose gives k=0, no anchors gives k=7, and two fixed points give rotation about their line, `partial`).

#### [should-fix] S6. Stale or garbled docs in C++
- **Where:**
  - `cpp/src/foundations/ba/bundle.cpp` `residual_rows`: the `@brief` still says "6 per pose prior", but the body now counts 3 per weighted half. The Python docstring was updated and the C++ one was not.
  - `cpp/include/slamlab/foundations/nls.hpp`, `Termination::kCount`: the comment is broken across lines ("...name tables\n/// are\n///        checked...").
  - `layout.cpp`, `fix_gauge` `@brief`: "...confined poses and name, and the gauge and confined poses." repeats itself and is unreadable.
- **Suggested fix:** Correct the three comments.

#### [should-fix] S7. C++ `fix_gauge` mutates an out-parameter and `make_layout` is about 70 lines; Python splits the same work
- **Where:** `layout.cpp:114-173` (`fix_gauge`, `Layout& layout`) and `:300-370` (`make_layout`); compare Python `_fix_gauge -> _Gauge` and `make_layout`
- **Issue:** The Python version returns a `_Gauge` value. The C++ version reads `layout.pose_free`, writes `counts.gauge` twice, `gauge_pose`, `confined_pose`, `gauge_anchors` and `gauge_plan`, and its docs list five side effects. `make_layout` inlines the observed-mask, intrinsics and pose-free loops.
- **Why it matters:** The parallel cores should have parallel structure so the review can compare them. The order dependence is easy to break: `pose_free` must be set before `fix_gauge`, and `prior_cams` is computed after it.
- **Suggested fix:** Return a `GaugeChoice {Gauge name; Mask pose_free; Anchors anchors; Plan plan;}` from `fix_gauge`, and lift the intrinsics/observed loops into helpers as `attach_priors` and `assign_points` already are.

#### [should-fix] S8. `GAUGES` serves two purposes, and the C++ guard relies on enum order
- **Where:** `problem.py:23` and `BAOptions.__post_init__` (`GAUGES[:-1]`, `self.gauge == "partial"`); `cpp/src/foundations/ba.cpp:167` (`options.gauge > Gauge::kScale`)
- **Issue:** One tuple is both the code table and the accepted set, with `"partial"` special-cased in the error text and by slicing off the last element. C++ hard-codes `kScale` as the last requestable value and hides `kPartial`'s position. A `kCount` value would also report the message "'partial' is reported", which is wrong.
- **Suggested fix:** Add `REQUESTABLE_GAUGES = GAUGES[:-1]` in Python. In C++ add a named `kLastRequestable` or a helper, and give `kCount` its own message.

#### [should-fix] S9. `Layout` leaves three fields undocumented
- **Where:** `layout.py:64-68` (`affine_free`, `gauge`, `summary`)
- **Issue:** The other fields have attribute docstrings. These three, including the shapes and meaning of `gauge` and `summary`, do not.
- **Suggested fix:** Add one-line attribute docs.

#### [should-fix] S10. `check_layers.check_cpp_internal` is deeply nested
- **Where:** `scripts/check_layers.py` in `check_cpp_internal` (for / for / while / for, nesting depth 5)
- **Issue:** The chain-following DFS is inlined inside the reporting loop and carries a `(target, via)` tuple state.
- **Suggested fix:** Extract `_reachable(graph, first, orders) -> Iterator[(target, via)]` and keep the loop as report formatting. The `chain`, `facade` and `parent-relative` cases are covered by the new tests, which are good.

#### [should-fix] S11. `compare_ba_colmap.py` has parallel format definitions
- **Where:** `scripts/compare_ba_colmap.py` (header f-string in `main`, row f-string in `row`)
- **Issue:** The column widths are written twice, in the header and the row, so a changed column must be edited in two places to stay aligned. The legend in the first `print` is also one 400-character string.
- **Suggested fix:** Define a `COLUMNS = [(title, width, spec), ...]` list and generate both the header and the rows from it.

#### [should-fix] S12. `group_members` has no input checks or edge-case tests
- **Where:** `src/slamlab/foundations/arrays.py` (`group_members`) and `test_validation.py::test_group_members_match_the_masks`
- **Issue:** The function is public in `arrays`. A group value outside `[0, n_groups)`, a negative one for example, is silently dropped and the other members are returned. The test covers empty groups but not empty input or an out-of-range value.
- **Suggested fix:** Either check `group.min() >= 0 and group.max() < n_groups` and raise an `InvalidInputError` that names the bad value, or document that the caller has validated. Add an empty-input test.

#### [note] N1. Small items
- The `TOLERANCE` docstring says "relative to the scene's extent", but `layout.py` uses `TOLERANCE * (1 + extent)`, and `name()` uses it as an absolute `atol`.
- `name()` compares `k == POSE_DIM` for "the rigid motions". Six is the dimension of the rigid group, not of a pose, so a `RIGID_DIM = 6` constant would read better.
- `generate_ba._compact` uses a regex whose character class includes the letters `nul`, to match JSON `null`. It works but is brittle.
- `confinement` takes `(confined, ranks)` in Python and a `Plan` in C++. This is harmless, and the parallel implementations are intentional.
- `camera.unproject_or_nan` falls back to a per-pixel Python loop only on failure. That is correct and documented.

### Checked and fine
- `gauge.py` and `gauge.cpp` match step for step: `rows`, `null_space`, `pose_image`, `pose_jacobian`, `canonical_span`, `complement` and `confinement`. I found no unexpected divergence, and the tie-break rule is documented as shared.
- The `gauge.py` module docstring states the conventions (generators `(t, ω, s)`, `T_cam_world` right-perturbation, Triggs section 9) clearly and consistently with the code.
- `Bundle.reduction` is a parallel block-diagonal assembly in both cores. `residual_rows` and the variance factor (Σ w·s over IRLS weights, with m counting weighted prior rows only) agree, and both are covered by value-asserting tests (`test_variance_factor_*`, which include a robust-loss case and a prior-row count).
- Enum `kCount` sentinels with `static_assert`s and the extra range checks give the bindings a build-time guarantee. The C++ `solve` check rejects a requested `partial`.
- `BAProblem` read-only copies are tested (`test_a_checked_problem_cannot_change`), and `BAOptions` rejects `"partial"` with a test.
- `eval/readers.py` parsers are now module-level, with line numbers asserted in the tests. `check_layers` chain, facade, parent-relative and duplicate-contract handling each have a test.
- The fixture generator names the `codes` tolerance policy and justifies the weak translation-prior weight in its docstring. The new cases have docstrings that state intent.
- No debug prints (the compare script's table output is its purpose), no stray TODOs, no commented-out code, and no hard-coded paths in the new code.

### Architecture review

## Architecture review

**Scope:** Run 3. I reviewed `git diff 85b86fd...4501984` (42 files) against the PR context `0d9366f...HEAD`. I read in full: `ba/gauge.py`, `ba/gauge.{hpp,cpp}`, `ba/layout.py`, the gauge part of `layout.cpp`, the `bundle`/`solve`/`problem` diffs, the binding diff, the enum headers, `camera.unproject_or_nan`, `arrays.group_members`, the `eval` diffs, `check_layers.py`, `test_check_layers.py`, `pyproject.toml`, `setup_slamlab.sh`, the generator and the fixture's structure. I probed two things in the scratchpad (the repo was not touched): the pycolmap contract, with grimp on an exported copy, and `check_cpp_internal` on a fake tree.

**Summary:** All six Run 2 findings (A15-A20) are resolved. The new `gauge` module sits in the right place in both cores. Python `layout`/`bundle` → `gauge` → `problem` and C++ `layout.hpp`/`bundle.cpp` → `gauge.hpp` → `ba.hpp`/`common.hpp` respect "solve > bundle > layout | schur > gauge > problem". The C++ `pinned`/`fix_gauge` mirror `_anchors`/`_fix_gauge` step for step. There are no must-fix findings. Three should-fixes:
- The allow-list pycolmap contract, which I recommended in Run 2, silently lost the indirect-import coverage the old deny-list had.
- The "partial is last / everything after kScale is report-only" rule is encoded by position in both cores, the same pattern A20 removed.
- The explicit `first_pose` and `scale` recipes have no cross-backend parity coverage, although the changelog says the fixture covers "every gauge".

### Run 2 findings
- **A15** (writable/aliased `BAProblem` arrays): **resolved.** `__post_init__` stores a read-only copy of every validated array (`problem.py:113-116`). `_validated` returns arrays only, so nothing non-array gets wrapped.
- **A16** (`check_layers.py` weaker than import-linter): **resolved.** The check now follows chains through unlisted modules, resolves `..`, maps the facade (`_CPP_FACADES`) and rejects duplicate containers. The Rust non-coverage is stated in the docstring and in CLAUDE.md. Two small residual gaps are below (note).
- **A17** (`bearings_or_nan` in `layout`): **resolved.** `camera.unproject_or_nan` takes `backend=`; `layout` and `bundle` both use it.
- **A18** (explicit `scale` error tested on the reference only): **resolved.** `test_scale_gauge_needs_an_anchor` is parametrised over `LOCKSTEP_BACKENDS` (`test_ba.py:773`).
- **A19** (fixture not self-describing): **resolved.** Every case stores `asdict(options)`, and `tolerance.codes` is an exact policy. One nit is below (note).
- **A20** (enum tables, docstring, pin, contract, design row): **resolved.** All enums have `kCount` sentinels with `static_assert`s; pycolmap is pinned once; the contract is an allow-list; the docstring and the design-doc `eval` row are fixed. The allow-list form has a side effect: see the first should-fix.

### Findings

#### [should-fix] The allow-list pycolmap contract no longer covers indirect imports through `colmap_reference`
- **Where:** `pyproject.toml:156-163` (`ignore_imports = ["slamlab.eval.colmap_reference -> pycolmap"]`)
- **Issue:** import-linter removes an ignored import from the graph before it checks chains. So, with the only edge to pycolmap ignored, every chain into pycolmap is gone. The old deny-list listed `slamlab.orchestration` as a source, so `orchestration → eval.colmap_reference → pycolmap` was an indirect violation. Under the new contract it passes. Direct imports are tighter than before (other `eval` modules may no longer import pycolmap), but the chain guarantee is lost. I suggested this exact form in Run 2 without noticing.
- **Why it matters:** The contract's comment says pycolmap is "imported by slamlab.eval alone", and the intent is that pipelines never depend on the optional reference. Today an orchestration pipeline could import `colmap_reference` and pass every check. It would fail at run time only where the `colmap` extra is missing.
- **Evidence:** I exported HEAD into the scratchpad, appended `import slamlab.eval.colmap_reference` to `orchestration/__init__.py` and built the graph with grimp, as import-linter does:
  ```
  chain: ('slamlab.orchestration', 'slamlab.eval.colmap_reference', 'pycolmap')
  after ignore: None
  ```
  `lint-imports` on the real tree reports `Only eval may import pycolmap KEPT (1 ignored import)`.
- **Suggested fix:** Keep the allow-list for direct imports, and add a forbidden contract with `source_modules = ["slamlab.orchestration"]` and `forbidden_modules = ["slamlab.eval.colmap_reference"]`. The ladder already keeps every other package from reaching `eval`. Alternatively, if pipelines may use the reference, say so in the contract's comment.

#### [should-fix] "Report-only" gauges are encoded by position in both cores
- **Where:** `cpp/src/foundations/ba.cpp:167` (`if (options.gauge > Gauge::kScale)`, message "'partial' is reported, not requested"); `src/slamlab/foundations/ba/problem.py:266-267` (`self.gauge == "partial"`, `choices = GAUGES[:-1]`)
- **Issue:** C++ treats every code after `kScale` as report-only. Python assumes `partial` is the last entry of `GAUGES`. Codes are append-only, so the next requestable gauge (for example a Sim(3)-about-a-point recipe) would go after `kPartial`. C++ would then reject it with a misleading message, and Python would list `partial` as a valid choice and omit the new one. A20's `kCount` work removed this kind of anchoring to a particular enumerator; these two checks bring it back.
- **Why it matters:** The two cores' error behaviour for a requested gauge would drift on the first extension, with no test catching it. The fixture only feeds valid gauges.
- **Suggested fix:** In C++, check `options.gauge == Gauge::kPartial || options.gauge >= Gauge::kCount`. In Python, use a `REQUESTABLE = tuple(g for g in GAUGES if g != "partial")` and build the message from it. A cleaner option is a separate report enum (`GaugeFound`) so that options cannot carry a report-only value. That is a bigger change and can wait until a second report-only value appears.

#### [should-fix] The explicit `first_pose` and `scale` recipes have no cross-backend parity
- **Where:** `tests/fixtures/foundations/ba.json`: the requested gauges across the 11 cases are only `auto` (10) and `similarity` (1). `tests/foundations/test_ba.py` has no `gauge="first_pose"` solve. Explicit `gauge="scale"` is exercised only on its error path (`:778`).
- **Issue:** `first_pose` has its own branch in both cores (`layout.py` `_fix_gauge`, `layout.cpp:133-136`). A successful explicit `scale` has its own anchor rule (`_anchors`/`pinned`: first fixed pose, else first full prior). None of these success paths is compared between Python and C++. CHANGELOG 0.2.0.0 says the fixture covers "every gauge".
- **Why it matters:** The design doc requires every compiled path to be held to the reference. These are public options whose two implementations could diverge silently. For example, C++'s `any_fixed` and Python's `np.any(fixed & observed)` are computed differently.
- **Suggested fix:** Add two fixture cases: explicit `first_pose`, and explicit `scale` with one full prior. Then either add explicit `none` too, or reword the changelog to "every gauge `auto` resolves to".

#### [note] `check_layers.py`: the facade header's own includes are checked as `solve`, and an unmapped facade is silently exempt
- **Where:** `scripts/check_layers.py` `_CPP_FACADES` and `_check_include`; `_include_graph` merges `cpp/include/slamlab/foundations/ba.hpp` and `cpp/src/foundations/ba.cpp` into one node, `("foundations", "ba")`.
- **Issue 1:** Because the two files share one node, the edges out of `ba.hpp` are checked as if `solve` were including. But `ba.hpp` holds the problem types, the bottom layer.
  - Probe: a fake tree where `ba.hpp` includes `slamlab/foundations/ba/gauge.hpp` gives `[]`. In Python, `problem → gauge` breaks the contract.
  - This cannot happen today, because only `cpp/include` is on the public include path (`CMakeLists.txt:25`), so the header cannot reach `src/` detail headers.
- **Issue 2:** A future package facade with no `_CPP_FACADES` entry "stays outside its container's rules" in both directions, with no message.
- **Suggested fix (when the next package arrives):**
  - Key the facade by file kind (`.hpp` → declared module, `.cpp` → implementing module).
  - Raise an error when a container with a contract has a facade file but no entry.

#### [note] Lock-step shape: Python `Layout` splits the plan; C++ carries it whole
- **Where:**
  - Python: `ba/layout.py` `Layout.confined`/`confined_ranks`, and `gauge.confinement(anchors, centres, confined, ranks)`.
  - C++: `layout.hpp` `Layout::gauge_plan`, and `confinement(anchors, centres, const Plan&)`.
- Python's `gauge` also exports `pose_image`, `complement`, `name` and others with no underscore, while C++ keeps `pose_image`/`complement` in an anonymous namespace. Both are harmless today. Carrying `gauge_plan: Plan` in Python and prefixing the internal helpers would make the two modules read one-to-one, as the rest of `ba` does.

#### [note] `gauge` is generic to any problem over poses and points
- **Where:** `src/slamlab/foundations/ba/gauge.py` depends only on `so3` and type aliases from `ba.problem`. The same holds in C++.
- When pose-graph optimisation or Sim(3) PGO lands (classic rung, loop closure), it needs the same null-space removal. Per the design doc ("shared components live in rung 0"), lift it to `slamlab.foundations.gauge` then, rather than importing `foundations.ba.gauge` from another module or copying it. Nothing to do now.

#### [note] Smaller residuals
- `tests/foundations/test_ba.py:131-132`: the `codes` policy is asserted (`comparison == "exact"`) and applied with `==`, not through `conftest.assert_matches_fixture` as CLAUDE.md says every policy is. The reason is that codes include strings. Either note the exception in the conftest docstring or let the helper accept a dict of scalars.
- `src/slamlab/foundations/ba/solve.py` `_CONVERT` is keyed on `str(field.type)`. That depends on `nls.py`'s `from __future__ import annotations` (line 54), which keeps annotations as strings. A non-`int`/`float`/`bool` field would raise `KeyError` rather than the intended "trace columns do not match" error.
- The enum name tables now come from two mechanisms: `Termination` from the core's `nls::name`, the other four from binding tables checked by `kCount`. Both are sound; pick one when Rust gains these enums.
- Out of lane (hygiene): clang-format split the `Termination::kCount` doc comment mid-sentence (`nls.hpp:136-138`: "tables\n/// are\n///        checked").

### Checked and fine
- **Layering:** `gauge` is below `layout | schur` and above `problem` in both languages. `schur` does not use it. `bundle → gauge` and `layout → gauge` are downward. `lint-imports` keeps all 9 contracts, and `check_layers` passes on the real tree. C++ `gauge.hpp → common.hpp` (unlisted, followed, includes nothing from slamlab) and `→ ba.hpp` (facade = `problem`) are allowed.
- **Lock-step:** `pinned` ≡ `_anchors`, including priors on observed poses only, the scale reference rule and the similarity rule. `fix_gauge` ≡ `_fix_gauge`, including the tolerance `kGaugeTolerance * (1 + extent)`. `plan`, `confinement`, `name`, the canonical span and the Householder complement follow the same algorithm and tie rules. `gauge_pose`/`confined_pose` are reported identically.
- **Binding boundary:** `trace_columns` crosses by name and Python checks them against `nls.Iteration`. `confined_pose` was renamed through the core, the binding, `COUNT_NAMES` and the Protocol docstring. The binding stays a type conversion.
- **Enum sentinels:** each core check rejects `kCount` (`camera::check_intrinsics`, `nls::check_options` including the block losses, `ba::solve` for `depth_loss` and `gauge`). The binding's `decode` already blocks out-of-table codes from Python. Switches handle `kCount` explicitly.
- **Read-only `BAProblem`:** the const-`ndarray` bindings accept it, and `replace` re-validates and re-copies.
- **`camera.unproject_or_nan`:** in the right module, has `backend=`, and uses no new compiled entry point, so no Protocol change is needed. Per-pixel fallback cost is covered by the A11 deferral.
- **`arrays.group_members`:** a fitting rung-0 utility, used by `ba`, `eval.reprojection` and `eval.colmap_reference`. Dependency direction is preserved.
- **`check_layers.py` semantics:** chain-following is sound. Any chain-only violation must pass through an unlisted module, and those are followed. Locations name the first include. Duplicate containers raise. The tests cover a chain, `..`, the facade and duplicate containers.
- **pycolmap packaging:** one pin in the `colmap` extra. `setup_slamlab.sh --unlocked` installs dev and colmap, and the lockfile already pins `pycolmap==4.2.1`. The `colmap_reference` docstring is accurate.
- **Fixture design:** the JSON is self-describing (full options, every policy with a reason). The two `partial` cases (rotation priors confine two poses; translation priors on two cameras) exercise multi-pose confinement across backends.
- **Design doc:** the `eval` row matches the placement. The A12 note places the sparse solver in rung 0 and records that the C++ seam has to be opened first.

Scratchpad probes (outside the repo): `/tmp/scratch/probe` (exported copy, edited for the pycolmap chain probe) and `.../scratchpad/fake` (the facade probe). No repository files were changed.

### Algorithm review

## Algorithm review (Run 3)

**Scope:** Fix range `85b86fd...4501984` on feature/ba-spine, with `0d9366f...HEAD` as context. I read these in full:
- `src/slamlab/foundations/ba/gauge.py` and `layout.py`
- `cpp/src/foundations/ba/gauge.cpp`
- the `bundle.py`, `solve.py`, `problem.py` and `schur.py` diffs
- the pose-prior residual and its Jacobian (`bundle.py` `_pose_error`, `_pose_prior_kind`)
- `generate_ba.py` and the new gauge, prior and variance tests in `tests/foundations/test_ba.py`
- the note's Priors, Gauge, Focal observability, Toward larger problems and What was learned sections

References: Triggs et al. 1999 §9 (gauge); Barfoot 2017 / Solà et al. 2018 for SE(3) perturbations; Förstner & Wrobel 2016 ch. 4; GTSAM `GPSFactor` and COLMAP's position prior for what a position prior conventionally pins.

All probes are in `/tmp/scratch/run3/`. Run them from the slamlab root with `PATH=<slamlab env>/bin:$PATH PYTHONPATH=. python <script>`. The three that reproduce the findings are also saved in a private scripts folder: `ba_trans_prior_rotation_leak.py`, `ba_far_origin.py` and `ba_centre_prior_fix.py`. No repository files were modified.

**Summary:** The null-space gauge is correct for what it models:
- The signs of `J_c` and `M` are right.
- The confinement meets the gauge orbit only at zero and covers the removed directions (I checked this analytically).
- Greedy removal is minimal, and whether it removes all of N does not depend on the order.
- The classic gauges reproduce the previous fixture to about 1e-14 relative.

Two must-fixes remain:
1. **The translation half of a pose prior does not pin the camera centre.** `log(T̄_cw⁻¹ T_cw)` pins `R̄ᵀt`. The gauge rows assume a known centre, so `auto` leaves a world rotation about the origin free. A GNSS-style prior (no attitude) also biases the result by tens of centimetres.
2. **The rank tolerance is absolute in uncentred coordinates.** Scenes 50 km or more from the world origin lose their gauge entirely. This is a regression: the previous code handled 1e6 m.

### Run 2 findings
- **L11 (must-fix): partly resolved.**
  - Fixed points, rotation-only priors, full priors, metric depth and the classic gauges are handled correctly, with tests in both cores.
  - Translation-only priors are not, because the residual does not pin what the gauge rows assume (new must-fix 1).
  - The fixture's "partial: translation priors on two cameras" case passes only because its priors carry the true rotations.
  - This is partly my own fault: my Run 2 suggestion listed "translation prior → known position `[I, -hat(X), X]`" without checking it against the residual.
- **L12: resolved.** Priors count only on observed poses (`layout.py:96-114`). The case is tested as `test_prior_on_an_unobserved_camera_pins_nothing`.
- **L13: resolved.** `residual_rows` counts `count_nonzero(_p_sqrt_w > 0)`, and `test_variance_factor_counts_only_weighted_prior_rows` checks it.
- **L14: resolved.** The numerator is `Σ ρ'(s) s` with the final weights, in both cores. `test_variance_factor_under_a_robust_loss` checks it, and the fixture's huber case moved by 0.44 relative, as expected.
- **L15: resolved.** The note gives the solver per image range (DENSE ≤ 50, SPARSE ≤ 1000) and quotes Power BA's abstract.
- **L16: waived.** Nothing to check.

### Findings

#### [must-fix] A translation-only pose prior pins `R̄ᵀ t`, not the camera centre, so the gauge removes the wrong directions and GNSS-style priors are biased
- **Where:**
  - the residual: `src/slamlab/foundations/ba/bundle.py` (`_pose_error`: `se3.log(se3.compose(T̄_world_cam, T_cam_world))`) and its C++ mirror;
  - the gauge rows that assume a known centre: `gauge.py:8-9`, `_rows`, `layout.py:111` (`centre_cams` includes translation-prior poses), and `gauge.cpp` `rows`;
  - the docs: the note's Priors ("from odometry, a previous solve or GNSS") and Gauge ("the centre of a pose with a translation prior", "translation priors on two cameras the rotation about their baseline").
- **Issue:**
  - `E = T̄_cw⁻¹ T_cw = [R̄ᵀR, R̄ᵀ(t − t̄)]`, so the translation half is `V⁻¹(θ_e) R̄ᵀ(t − t̄)`, with `t = −R C`.
  - A world rotation about the origin, `T_cw → T_cw G⁻¹` with `G = (R_g, 0)`, leaves `t` unchanged. At the prior mean the translation half is therefore exactly invariant under rotation about the world origin, while the centre moves.
  - Its true rows over `g` are about `[−I, 0, −C]` (no ω column), not `[I, −hat(C), C]`.
  - Full priors are unaffected: with the rotation rows present, the two row sets have the same span. Rotation-only priors are also correct.
- **Why it matters:**
  - **Gauge under-removed.** Translation priors on three non-collinear cameras resolve to `none`, but a 3-DoF rotation about the origin is still free. H is singular, `σ_f` is infinite and the focal length is flagged unobservable. With two cameras the gauge removes 1 of the 3 free directions.
  - **Bias.** A GNSS prior carries a centre and no attitude. If the caller fills the unused rotation with identity, the prior pins `−C̄` against `−R C` and pulls the cameras off by up to 0.76 m.
- **Evidence:**
  - `trans_prior_null.py`, noise-free, priors on cameras 0, 2 and 4, free focal. Both backends:
    ```
    python gauge none gp -1 cp -1 cost 7.934e-23 f_std [inf inf] unobs 1
    cpp    gauge none gp -1 cp -1 cost 7.940e-23 f_std [inf inf] unobs 1
    rotated about origin: translation half of prior error 2.26e-16   (centre moved 0.287 m)
    ```
  - `trans_prior_rotation_leak.py`: 0.5 px noise, translation weight 2500 on every camera at the true centres, rotation weight 0. Both backends give identical numbers.
    ```
    prior rotation = truth     gauge=none cost=42.54 centre error (m) [0.006 0.004 0.007 0.007 0.007]
    prior rotation = identity  gauge=none cost=508.8 centre error (m) [0.116 0.197 0.487 0.764 0.594]
    ```
  - `hessian_null.py`: the reduced H for "translation priors on two cameras" (the test case, gauge `partial`) has two eigenvalues near 1e-12 relative. Every other case's smallest is about 1e-9.
  - For comparison, GTSAM's `GPSFactor` error is `nTb.translation() − measured`, the body position in the navigation frame ([GPSFactor.cpp](https://docs.ros.org/en/api/gtsam/html/GPSFactor_8cpp_source.html)).
- **Suggested fix:**
  - Make the translation half pin the centre, in both cores. One option is a decoupled residual, `(C − C̄, log(R̄_wcᵀ R_wc))`. The smaller change is `e = log(T̄_wc⁻¹ T_wc) = log(T̄_cw T_cw⁻¹)`, whose translation half is `R̄_wcᵀ(C − C̄)` to first order, with Jacobian `−Jr⁻¹(e) Ad(T_cw)` for the right perturbation.
  - `centre_prior_fix.py` monkeypatches the second option into the Python reference. On noise-free data it reaches cost about 1e-24 in 10-59 iterations whether the stored rotation is the truth or identity. With the current residual and identity rotation, every weight runs to `max_iterations` with cost 0.24-1100.
  - Then:
    - add a test with translation-only priors whose stored rotation is identity, asserting that the solution does not depend on it;
    - add a test that the reduced H at the solution has no near-zero eigenvalue after `auto`;
    - regenerate the "partial: translation priors" fixture case;
    - correct the note.

#### [must-fix] The rank tolerance is absolute in uncentred generator coordinates: scenes far from the world origin lose the gauge (regression)
- **Where:**
  - `layout.py:147-149`: `extent = max |coordinate|`, `tolerance = 1e-9 · (1 + extent)`;
  - its use in `gauge.plan`, `gauge.remaining` and the C++ mirror (`layout.cpp`, `gauge.cpp`).
- **Issue:**
  - `g = (t, ω, s)` acts about the world origin. Far from it, the directions that distinguish rotation and scale are scaled by about 1/|C₀| in the orthonormal null basis.
  - After the first pose is fixed, N is the scale about C₀, the unit vector `(−C₀, 0, 1)/√(1+|C₀|²)`. Its image at another pose has norm `|C − C₀|/|C₀|`.
  - That is compared with `1e-9·|C₀|`, so the gauge is lost once `baseline < 1e-9·|C₀|²`. For example, 1.6 m at 40 km, or 1 cm at 3 km.
  - Even the first pose's generator `[[I, −hat(C), C], [0, I, 0]]` has smallest singular value about 1/|C|, so no pose is fixed at all.
- **Why it matters:**
  - Geo-referenced coordinates (UTM, ECEF, and COLMAP models aligned to GNSS) routinely sit at 1e5 to 6e6 m.
  - The gauge goes silently unremoved: the summary still says `similarity`, `σ_f` is infinite, the focal length is flagged unobservable, and at 1e6 m the Python reference stops at cost 264 against the optimum's 41.7.
  - The previous code handled all of these offsets.
- **Evidence:** `far_origin.py`: the same noisy scene shifted by `(d, 0.3d, 0)`, nothing anchored.
  ```
  HEAD:     1e4 python gauge=similarity gp=0  cp=4 f_std=[0.793 1.057] unobs=0
            5e4 python gauge=similarity gp=-1 cp=0 f_std=[inf inf]     unobs=1   (cpp the same)
            1e6 python gauge=similarity gp=-1 cp=4 cost=264.138 f_std=[inf inf]; cpp cp=0 cost=41.74
  85b86fd:  1e6 python gauge=similarity gp=0  cp=4 cost=41.7377 f_std=[0.793 1.057] unobs=0
  ```
  The `85b86fd` row was produced by `far_origin_old2.py` against `git archive 85b86fd`.
- **Suggested fix:**
  - Build every row and `pose_image` in normalised coordinates `X' = (X − X̄)/L`. Here X̄ is the mean of the observed centres and anchor points, and L is their RMS spread. Then compare singular values to a relative 1e-9.
  - A change of base point and unit is an invertible linear map of `g`. N's span transforms with it, so `M`, the plan and the confinement are unchanged in meaning. Only `name()` would need to read N back in world units, or test invariants that survive the map (k, a zero s-row, a zero ω-row).
  - Add a far-origin test in both cores: offset 1e6 must give the same gauge pose, confined pose and `σ_f` as offset 0.

#### [should-fix] A gauge the plan could not remove is not reported, and the confinement then picks an arbitrary part of it
- **Where:** `gauge.plan` (`if rank == 0: break` and `while ... and left`) and `gauge.confinement` (`_null(rows, GENERATORS − sum(ranks))`); the same in `gauge.cpp`.
- **Issue:**
  - When the free poses do not see all of N, the plan stops with part of N left, and nothing records the dimension left over.
  - `confinement` then rebuilds N with the planned dimension `sum(ranks)`. This takes an arbitrary `sum(ranks)`-dimensional subspace of a larger null space, which SVD fixes only up to rotation. That can differ from what the plan assigned, and it can differ between the cores.
- **Why it matters:** the failure is silent. The far-origin output above shows it: the summary says `similarity`, `gauge_pose = −1`, and the two backends confine different poses (`cp=4` against `cp=0` at 1e6). The error contract says failure must be observable.
- **Suggested fix:** compute N's dimension after the plan. If it is not zero, either raise `Degenerate`, naming the free dimensions, or report it, for example as a `free_gauge` count in the summary that tests assert is 0. Make `confinement` assert that its rebuilt rank equals the planned one.

#### [note] "Stiff translation priors converge slowly under any gauge" may describe the residual, not the problem
- **Where:** the note's "What was learned", and the comment on `_translation_priors` in `generate_ba.py`.
- **Observation:** the current residual leaves the rotation about the world origin pinned only at second order, through `V⁻¹(θ_e)` and away from the mean. A slow crawl, and cores that end 1e-3 apart, are the expected symptoms of such a near-null direction. With the centre residual from must-fix 1, `centre_prior_fix.py` converges at weights 1 to 1e4 regardless of the stored rotation. I did not reproduce the fixture seed's 40-100 iterations, so this is a pointer, not a verdict. Re-measure after the fix before keeping the lesson as written.

#### [note] Dropping constant prior blocks is right for the optimum and for σ̂₀²; it makes the reported cost depend on the gauge
- **Where:** `layout.py:249-254`.
- **Observation:**
  - A block on a pose that never moves adds a constant to the cost and nothing to `Jᵀ W J`. Dropping it changes neither the optimum nor the deviations, and it correctly keeps the block out of `Σ w s` and m.
  - Under `auto`, a pose the gauge fixes can never carry a prior: a fixed pose needs `dim N ≥ 6`, so no prior rows. Only explicit gauges, fixed poses and unobserved poses are affected.
  - As a result, `initial_cost` and `final_cost` for the same problem differ between, say, `gauge="similarity"` and `auto` when pose 0 has a prior. The note says so ("inflates neither σ̂₀² nor the reported cost"). Add a line to `BASummary` so nobody compares costs across gauges.

#### [note] Σ w s still counts heavily down-weighted rows fully in m
- `Σ w s / (m − n)` is a standard and defensible choice. Under Cauchy, though, outlier rows add almost nothing to the numerator but a full 2 to m, so σ̂₀² comes out slightly low: 0.23 against 0.25 at 15% outliers (Run 2's probe). An effective redundancy, `Σ w` rows minus n, is the usual refinement if it ever matters. No action needed for this rung.

### Checked and fine
- **Signs and conventions.** For `T_cw exp(δ)`, `dC = −ρ + hat(C)θ` and the world orientation changes by `−θ`, so `J_c = [[−I, hat(C)], [0, −I]]` is right. The world similarity moves a camera by `dC = t − hat(C)ω + sC` and `dR_wc = ω`, so `M` is right. The note's `a = (−b, b×C)` follows.
- **Transversality and minimality.** The constraint `Mᵀ J_c δ = 0` excludes every gauge direction the pose sees (`MᵀM n ≠ 0` whenever `M n ≠ 0`), and has codimension equal to `rank(M)`. The removed set after all candidates is `N ∩ ⋂ null(M_c)` in any order, so greedy order affects only which poses take the directions.
- **Degenerate placements.** A pose cannot see the scale about its own centre (`t + sC = 0` gives an image of 0). A candidate at that centre therefore gets rank 0 and is skipped, and the next pose that sees the direction takes it. Coincident-centre cases fall to point degeneracy (parallax), as before.
- **Holonomy.** The bases are rebuilt per state from the current anchor centres. The constraint holds to first order and drifts at second order. This is harmless to the optimum, because the cost is flat along the gauge, and only shifts which gauge the deviations are expressed in.
- **Determinism of the basis.** In `_canonical_span`, `P[:,k]/√P_kk` is a unit vector because P is idempotent, and the deflation stays a projector. The successive Householder complement stays orthonormal because each later normal lies in the remaining span. The 1e-12 relative tie-breaks match between the cores.
- **Classic gauges reproduce the old fixture.** `cmp_fixture.py` compares old against new `ba.json`:
  - similarity, scale, radial and Gauss-Newton: poses, points and intrinsics agree to ≤ 1.2e-14 relative, `intrinsics_std` to ≤ 6.7e-13, codes identical (`scale_pose` renamed `confined_pose`);
  - first_pose and none: bit-identical;
  - huber: differs only in `variance_factor` (L14);
  - every kind of block: differs only in `variance_factor` and `intrinsics_std` (L13: the half prior's 3 rows).
- **Other row types.** Fixed points `[I, −hat(X), X]`, rotation-only priors `[0, I, 0]` and metric depth `[0, 0, 1]` are right. A free-affine depth prior absorbs scale, so it is correctly not counted. Full priors give the same span as the true `log` rows.
- **`name()`.** The checks for `first_pose` (k = 6 with a zero s-row) and `scale` (k = 1 with a zero ω-row) are correct for any base point.
- **Note sections.** Gauge and Focal observability are accurate apart from the translation-prior claims above. Toward larger problems is fixed per L15. Förstner & Wrobel and Triggs are correctly cited.

Sources:
- [Triggs et al., Bundle Adjustment -- A Modern Synthesis (1999)](https://link.springer.com/chapter/10.1007/3-540-44480-7_21)
- [GTSAM GPSFactor.cpp (error = nTb.translation() − measured)](https://docs.ros.org/en/api/gtsam/html/GPSFactor_8cpp_source.html)
- [Solà, Deray & Atchuthan, A micro Lie theory for state estimation in robotics, arXiv:1812.01537](https://arxiv.org/abs/1812.01537)
