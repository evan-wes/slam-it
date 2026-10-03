# Review panel: `feature/five-point`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-03

| | |
|---|---|
| **Branch** | `feature/five-point` |
| **Range** | `8699f1f0693b29361a24c8c7f6816c9aa4f03a4f...eddc19ffc4932aedfc70e7c9871c4fb7831fa5fc` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`; 1211 Python tests passed, 337 skipped; ctest 15/15) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A1 | must-fix | architecture | `tests/fixtures/foundations/` | No parity fixture for `essential_5pt` or `refine_essential`; the `essential_ransac` and `initialize` fixtures' tolerance reasons describe the old 8-point default | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| L1 | must-fix | algorithm | `fivepoint` tests (Python, C++) | The documented rank-deficient sample (a repeated pair gives no solution) is untested | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| L2 | should-fix | algorithm | `fivepoint._polish` / `polish` | Nistér loses the true E on near-degenerate samples (18/300 at a 5% baseline vs Stewénius 3, PoseLib 35); the fixed 3-step polish has no convergence test; the note's precision is generic-geometry only | fix | fixed (4085638 polish, f5112b4 test); the default became Stewénius after the re-measure (4085638) -- decided by evan-wes, 2026-10-03 |
| A5 / L3 | should-fix | architecture, algorithm | `epipolar.polish_consensus` / `polish` | The refinement's `nls.Summary` is discarded; whether it ran, was kept or converged is invisible | fix | fixed (4085638) -- decided by evan-wes, 2026-10-03 |
| A4 | should-fix | architecture | `twoview._initialize`, `epipolar.polish_consensus` | E-RANSAC-then-refine is composed twice in Python and once in C++; `polish_consensus` is public with no C++ counterpart | fix | fixed (4085638) -- decided by evan-wes, 2026-10-03 |
| A3 | should-fix | architecture | `MINIMAL_SOLVERS`, `fivepoint.SOLVERS`, bindings | Integer code tables are not tested against the compiled ones; `fivepoint` has no `codes()` | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| A2 | should-fix | architecture | design doc Conventions, `twoview` binding | The solver choice crosses as a code in `essential_ransac` and by name in `twoview.Options` | fix (amend design doc) | fixed (4965af6, design doc) -- decided by evan-wes, 2026-10-03 |
| L4 | should-fix | algorithm | `fivepoint.real_roots` | Public, but NaN/inf coefficients silently return no roots | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| H1 | should-fix | hygiene | `epipolar.essential_estimator` | Two near-identical `PairEstimator` constructions | fix | fixed (4085638) -- decided by evan-wes, 2026-10-03 |
| H2 | should-fix | hygiene | `SampsonProblem.residuals` / `linearize` (Python, C++) | The Sampson formula is computed independently in both methods | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| H3 | should-fix | hygiene | `polish_consensus`, `refine_reference` | Unchecked public helpers don't state their preconditions | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| H4 | should-fix | hygiene | `fivepoint.solve_reference` | Any name other than "nister" silently runs Stewénius | fix | fixed (4085638) -- decided by evan-wes, 2026-10-03 |
| H5 | should-fix | hygiene | `cpp/tests/foundations/test_fivepoint.cpp` | No pure-rotation, six-pair or unknown-solver case natively | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| L6 | note | algorithm | `fivepoint._stewenius` | Stewénius could share the polish (1e-9 → Nistér's level; tighter parity) | fix (with L2) | fixed (4085638) -- decided by evan-wes, 2026-10-03 |
| A9 | note | architecture | `pyproject.toml` import-linter | Forbid importing test oracles (poselib, mpmath, manifpy, scipy) from `slamlab` by contract | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| H8 | note | hygiene | `fivepoint.py` docstring | Measured numbers in a docstring will go stale; point to the note | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| H6 | note | hygiene | `_nister` row pairs, action rows | Inline index literals; name them next to the orders | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| H7 | note | hygiene | `fivepoint.py`, C++ `polish` | Missing blank line; a full copy before `gather` | fix | fixed (f5112b4) -- decided by evan-wes, 2026-10-03 |
| A6 | note | architecture | `epipolar.py` (896 lines) | The refinement could be its own module; decide before PR 4 adds PnP refinement | defer (design doc open items, before PR 4) | deferred (design doc, deferred from PR 3) -- decided by evan-wes, 2026-10-03 |
| L5 | note | algorithm | `essential_estimator`, `polish_consensus` | LO's refits stay algebraic; current pipelines refine on Sampson inside LO (PoseLib, SupeRANSAC) | defer (design doc open items) | deferred (design doc, deferred from PR 3) -- decided by evan-wes, 2026-10-03 |
| A7 | note | architecture | `codes()` keys | `"termination"` means NLS in BA and RANSAC in epipolar | defer (with A8 of PR 2: generic dispatch) | deferred (design doc, deferred from PR 3) -- decided by evan-wes, 2026-10-03 |
| A8 | note | architecture | `ransac.run(extra=)` | Untyped keyword pass-through | waive: typed by the Implementation Protocol and checked by test_backends | waived as decided; then moot: the A4/A5 fix removed `ransac.run`'s pass-through (4085638) -- decided by evan-wes, 2026-10-03 |
| A10 | note | architecture | `solver` / `minimal_solver` / `essential_solver` | One concept, three parameter names | waive: each names what it is at its layer; the note maps them | waived: each names what it is at its layer; the note maps them -- decided by evan-wes, 2026-10-03 |
| L7 | note | algorithm | `REFINE_ITERATIONS`, nls tolerances | The refinement stops ~1e-4 from SciPy's optimum in E | waive: ~0.006°, negligible against 1 px noise; documented | waived: ~0.006°, negligible against 1 px noise; documented -- decided by evan-wes, 2026-10-03 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

Evidence the algorithm reviewer produced is saved at
`five_point_degeneracy_probe.py`. The reviewers' IDs are renumbered
in the table (by severity); the verbatim reviews below use their own headings.

### Hygiene review

**Scope:** `git diff 8699f1f...eddc19f`. I read these in full: `src/slamlab/foundations/fivepoint.py`, `cpp/src/foundations/fivepoint.cpp`, the new `epipolar` / `twoview` / `correspondences` code in Python and C++, `nls_crossing.hpp`, the epipolar and two-view bindings, and the new tests (`test_fivepoint.py`, the new tests in `test_epipolar.py`, `cpp/tests/foundations/test_fivepoint.cpp`). I read the C++ tests, `demo_two_view.py` and the other bindings only as diffs or excerpts. I did not read `docs/notes/five-point.md` or the CHANGELOG in detail, and I did not run anything.

**Summary:** This is good, production-grade work. Inputs are validated at every public boundary (finite, positive scale, rank check, solver codes in both languages). Tolerances are named and justified. Docstrings state shapes and conventions, and the tests assert on values, with independent oracles (PoseLib, scipy least squares, finite differences) and degenerate cases (pure rotation, bad input). I found no must-fix issues, only some small duplication and naming points.

#### [should-fix] Two near-identical `PairEstimator` constructions in `essential_estimator`
- **Where:** `src/slamlab/foundations/epipolar.py`, `essential_estimator` (the `if minimal_solver == "8pt"` branch and the fall-through).
- **Issue:** The two branches build the same `PairEstimator` and differ only in `sample_size` and the `minimal=` argument.
- **Why it matters:** A change to the error function or the refit has to be made twice. The jscpd check cannot see this.
- **Suggested fix:** Compute `sample_size` and `minimal` first (`minimal = None` for "8pt"), then construct once.

#### [should-fix] Sampson residual maths is written twice in the Python `SampsonProblem`
- **Where:** `src/slamlab/foundations/epipolar.py`, `SampsonProblem.residuals` and `SampsonProblem.linearize`. The C++ class has the same split.
- **Issue:** Both compute `line2`, `line1`, `c` and `g` independently. `linearize` recomputes what `residuals` returns, and `epipolar.sampson` is a third place with this formula.
- **Why it matters:** If the Sampson denominator is ever changed in one place (for example a guard for a zero denominator), the residual and its Jacobian silently disagree. The finite-difference test would catch it, but one helper is cleaner.
- **Suggested fix:** Add a private helper returning `(line2, line1, c, g)` and use it in both methods.

#### [should-fix] `polish_consensus` and `refine_reference` are public but unvalidated, and the docstrings do not say so
- **Where:** `src/slamlab/foundations/epipolar.py`, `polish_consensus` and `refine_reference`. `solve_reference` in `fivepoint.py` has the same gap.
- **Issue:** `polish_consensus` is public and imported by `twoview`, but its docstring does not say its bearings must already be validated. Only `refine_reference` says "unchecked", and its docstring does not say that E must be rank 2 or that z must be positive. `solve_reference`'s docstring does say it is unchecked.
- **Why it matters:** CLAUDE.md says each algorithm validates once, at its boundary. A reader cannot tell which entry points are safe to call directly.
- **Suggested fix:** Add "unchecked: the caller has validated" to `polish_consensus`, and state the preconditions in `refine_reference`.

#### [should-fix] `solve_reference` silently treats any non-"nister" string as Stewénius
- **Where:** `src/slamlab/foundations/fivepoint.py`, last line of `solve_reference`.
- **Issue:** `return _nister(...) if solver == "nister" else _stewenius(...)`. A typo reaching this function runs the wrong solver. The public wrapper validates the name, but `epipolar.essential_estimator` calls `solve_reference` directly with `minimal_solver`, and `twoview.Options` is the only check on that path.
- **Why it matters:** This is defensive-coding hygiene. The only real caller path is checked today, but the function is public.
- **Suggested fix:** Use an explicit `elif solver == "stewenius"` and raise `InvalidInputError` otherwise.

#### [should-fix] The test for `essential_estimator`'s unit-length contract is missing in the C++ tests
- **Where:** `cpp/tests/foundations/test_fivepoint.cpp`.
- **Issue:** The C++ test covers the true-E case, the real-roots case and four pairs rejected. It does not cover the no-baseline sample, an unknown solver code, or exactly-five-pairs (six pairs). Python covers all of these.
- **Why it matters:** The degenerate-sample drop (`kEssentialTolerance`) is a documented behaviour that exists only because of a past bug. It is tested in Python on both backends via `test_a_pure_rotation_returns_only_essential_matrices`. The C++ core's own ctest would not catch a regression there.
- **Suggested fix:** Add a pure-rotation case and a six-pair rejection to the C++ test. This is low priority because the Python test exercises the C++ backend.

#### [note] Inline magic literals in the Nistér setup
- **Where:** `fivepoint.py`, `_nister` (the row-pair tuple `((4, 5), (6, 7), (8, 9))` and the permutation signs). The C++ side has the same constants in `nister`, `_ACTION_ROWS` and `_ACTION_SHIFTS`.
- **Issue:** The indices are explained by a comment and by the module docstring's monomial order, so they are acceptable. A future reader must still map "4, 5" to "x²z, x²" by counting.
- **Suggested fix:** Optional. Name the pairs, for example `_NISTER_ELIMINATION_ROWS`, next to `NISTER_ORDER`.

#### [note] Cosmetic points
- **Where:** `fivepoint.py` has no blank line between `ROOT_TOLERANCE` and `ESSENTIAL_TOLERANCE`. In C++ `polish`, `Rows3(bearings1)` copies the full array just to `gather`.
- **Issue:** Both are harmless. The copy runs once per call, not in a hot loop.

#### [note] Anecdotal numbers in a module docstring
- **Where:** `fivepoint.py`, the Nistér paragraph ("on 300 exact random problems the worst error fell from 7e-4 …").
- **Issue:** These numbers are accurate but will go stale. They belong in `docs/notes/five-point.md`, which the PR adds, so the docstring could point there instead.

#### Checked and fine
- **Input validation:** Python `essential_5pt`, `refine_essential`, `essential_ransac` and `twoview.Options` all validate shapes, finiteness, solver names, scale and rank. The C++ core validates its own input independently and the binding decodes the solver code, so the checks are not shared and not skipped. Error messages name the bad value.
- **Error handling:** no swallowed exceptions, no bare `except`. The C++ `*nls::solve(...)` and `*decompose_essential(...)` dereferences are commented with why they cannot fail (non-strict solve, rank already checked).
- **Tolerances:** every one (`RANK_TOLERANCE`, `ESSENTIAL_TOLERANCE`, `REAL_TOLERANCE`, `ROOT_TOLERANCE`, `POLISH_STEPS`, `REFINE_ITERATIONS`) is named and justified in a docstring. No debug prints or TODOs.
- **Documentation:** module docstrings state the algorithm, the monomial ordering, the conventions (`E = [t]ₓ R`, singular values (1, 1, 0), right perturbation) and where the C++ core sits. Doxygen on every C++ function.
- **Binding contract:** `nls_crossing.hpp` is a clean extraction. The shared summary dict removed the BA-only duplication, and the trace column names are read by name on the Python side.
- **Within-language duplication:** none across the two solvers beyond the shared front end (`_null_space`, `_constraints`, `_gauss_jordan`, `_keep`), and `start_motion` pins the same start in both backends, with the reason given.
- **Tests:** both solvers on both backends; the solvers cross-checked against each other and PoseLib; the refinement against scipy and finite differences; degenerate cases (pure rotation, rank-deficient input, wrong pair count, unknown solver); RANSAC parity run for run for every solver and refine setting; `test_twoview` covers bad `essential_solver` in Python and in the C++ core.
- **Performance:** the pure-Python `_multiply` loops are in the reference only and run once per sample on 4x4x4 arrays. This is acceptable for a reference that the C++ core shadows.
- **Demo:** the `demo_two_view.py` flags are consistent with `Options`.

### Architecture review

**Scope:** `git diff 8699f1f...eddc19f` on `feature/five-point` (37 files, 5 commits: 703a022, fc47f72, 750c13f, fd120ee, eddc19f). I read every changed Python module, C++ header, core and binding, plus the neighbouring pieces they depend on: `backends.py`, `tests/test_backends.py`, the fixtures and `generate_twoview.py`, the import-linter contracts, and the design doc's "Languages" and "Conventions" sections.

**Summary:** The structure is sound. `fivepoint` is its own module on the right layer, with a mirrored C++ library file, binding and Protocol. The C++ core stays Python-free and returns `Result`. The shared `nls_crossing.hpp` / `nls.summary_from_backend` removes real duplication. There is one must-fix: the new compiled functions skip the parity-fixture rule, and the existing fixtures' tolerance reasons are now wrong because the defaults changed. The should-fixes are about how the solver choice crosses the bindings, and about the refinement's composition and summary, which differ between the two languages and are not observable.

#### [must-fix] No parity fixture for `essential_5pt` or `refine_essential`; the existing fixtures describe the old default
- **Where:** `tests/fixtures/foundations/` (no `fivepoint.json`; `epipolar.json` functions are `essential_8pt, fundamental_8pt, sampson_essential, sampson_fundamental, decompose_essential, essential_ransac`, no `refine_essential`); `tests/fixtures/foundations/generate_twoview.py:349,476`; `twoview.json` `initialize.tolerance.reason`.
- **Issue:** CLAUDE.md says every C++/Rust implementation has "a parity fixture in `tests/fixtures/<rung>/` that states its tolerance policy". The design doc adds that "the summary fields go into the parity fixtures". Two new compiled functions have no fixture. `fivepoint.essential_5pt` is checked only C++-against-Python at run time (`test_fivepoint.py::test_backends_agree`). `epipolar.refine_essential` and its `nls.Summary` are checked the same way. The PR also changes the defaults to Nistér plus refinement. The `essential_ransac` and `initialize` fixture cases now silently run that new path, because their inputs carry no solver or refine key. Their stated reasons still say "LO refits all clean inliers exactly" and "the 8-point fit and Lindstrom are exact to rounding", which no longer describe what runs. The fixtures also exercise only the default solver.
- **Why it matters:** The fixture is what Rust (PR 5, edge VO) will be held to without running the Python reference. It is also the only cross-language record of the 5-point solver's tolerance and the refinement summary's fields. A fixture whose policy text describes a different algorithm cannot tell anyone why its tolerance holds.
- **Evidence:** `git diff --stat 8699f1f...eddc19f -- tests` touches only three test files and no fixture. The `epipolar.json` `essential_ransac` case has input keys `['bearings1','bearings2','threshold','seed']`, so no solver is chosen.
- **Suggested fix:** Add a `fivepoint.json` generator with exact samples (true E among the solutions, unordered, up to sign, stated atol, both solvers); add `refine_essential` cases to `epipolar.json` (exact data from a perturbed start converging to the true E, plus summary fields); add `minimal_solver` and `refine` to the `essential_ransac` cases (at least "8pt"/off and "nister"/on); rewrite both tolerance reasons for the new default.

#### [should-fix] The same solver choice crosses the bindings two ways, and one diverges from the design doc
- **Where:** `src/slamlab/foundations/epipolar.py:855` (int code), `cpp/bindings/foundations/twoview.cpp:37,63` (name lookup), `src/slamlab/foundations/twoview.py:165`; design doc lines 195-210.
- **Issue:** `essential_ransac` passes `MINIMAL_SOLVERS.index(...)` as an integer. `twoview.Options.essential_solver` is a `str` that crosses inside the options dict by name, and the binding looks it up through `epipolar::name()`. The design doc says options cross "as scalars, with integer codes for enums", and that results carry "enums as integer codes and a `codes()` function giving their names". `docs/notes/five-point.md` records the split, but the design doc does not.
- **Why it matters:** That gives two conventions for one enum in one PR, and the next options dataclass with a choice field has no rule to follow.
- **Suggested fix:** Either change the code or change the doc. I recommend amending the design doc: a choice field inside an options dataclass crosses by name, mapped through the core's `name()`. `asdict()` produces exactly that, and `check_keys` plus a name lookup fail loudly. Direct enum arguments and results keep integer codes. If the owner prefers one rule, make `twoview` convert to the code instead.

#### [should-fix] The integer code tables are not tested against the compiled ones
- **Where:** `src/slamlab/foundations/epipolar.py:90` (`MINIMAL_SOLVERS`), `src/slamlab/foundations/fivepoint.py:71` (`SOLVERS`), `cpp/bindings/foundations/epipolar.cpp:90-98`, `cpp/bindings/foundations/fivepoint.cpp` (no `codes()`).
- **Issue:** The Python side sends `MINIMAL_SOLVERS.index(name)` and `SOLVERS.index(solver)`, and the C++ side `static_cast`s them to `MinimalSolver` and `fivepoint::Solver`. `epipolar.codes()["minimal_solver"]` is exported, but no test compares it with `MINIMAL_SOLVERS`. `fivepoint` has no `codes()` at all. BA has exactly this guard (`tests/foundations/test_ba.py:575-576`).
- **Why it matters:** If the order is swapped on one side, Nistér runs where Stewénius was asked for. Both return the same solutions to about 1e-7, so the behavioural tests would most likely pass.
- **Suggested fix:** Add `codes()` to the fivepoint binding and Protocol; one test per module asserting compiled `codes()` equals the Python tables; derive `MINIMAL_SOLVERS = (*fivepoint.SOLVERS, "8pt")`.

#### [should-fix] The E-RANSAC-then-refine step is assembled differently in the two languages
- **Where:** `src/slamlab/foundations/twoview.py:410-417` and `src/slamlab/foundations/epipolar.py:852-853` (Python composes it twice); `cpp/src/foundations/twoview.cpp:145-147` and `cpp/src/foundations/epipolar.cpp:401` (C++ composes it once, `polish` in an anonymous namespace).
- **Issue:** C++ `twoview` calls `epipolar::essential_ransac(..., options.essential_solver, options.refine)`, so the composition lives in one place and `polish` is private. Python `twoview._initialize` rebuilds it from parts. That duplicates `essential_ransac`'s Python branch and makes `polish_consensus` a public API that has no C++ counterpart.
- **Why it matters:** If the refine gating changes, it must change in two Python places. Only the lock-step tests would catch a miss. The public surface is also no longer mirrored across languages.
- **Suggested fix:** One Python reference function, e.g. `essential_ransac_reference(b1, b2, options, minimal_solver, refine)`, called by `essential_ransac`'s Python branch and `twoview._initialize`; make `polish_consensus` private.

#### [should-fix] The refinement's own solve summary is thrown away inside E-RANSAC
- **Where:** `src/slamlab/foundations/epipolar.py:769-802` (`polish_consensus`), `cpp/src/foundations/epipolar.cpp:401-430` (`polish`).
- **Issue:** With `refine=True`, E-RANSAC runs a Levenberg-Marquardt solve and then discards its `nls.Summary`. A caller cannot tell whether refinement ran, whether it was kept, or how it terminated.
- **Why it matters:** The design asks for failure to be observable, and here it is not. Whether the refinement helped on real data can only be seen by re-running without it.
- **Suggested fix:** Return the refinement's outcome next to the RANSAC summary (`refinement: nls.Summary | None` plus `refined: bool`), nested in an E-specific result rather than in the generic `ransac.Summary`, crossing as a nested `"refinement"` dict through `nls_summary`.

#### [note] The refinement grows `epipolar` instead of getting its own module
- **Where:** `src/slamlab/foundations/epipolar.py:505-802` (896 lines now); `epipolar.hpp` similarly.
- **Issue:** Manifold least squares on E is separate from the 8-point fits and decomposition; `tangent_basis` and the S² retraction are reusable primitives buried in `epipolar`. PR 4 adds pose refinement for PnP, so the pattern chosen here will be copied.
- **Suggested fix:** None needed now. Decide before PR 4 whether refinements get their own modules, and move the S² basis and retraction next to `so3` when a second user appears.

#### [note] The `codes()` key "termination" means different enums in different modules
- **Where:** `cpp/bindings/foundations/ba.cpp:82` (NLS), `cpp/bindings/foundations/epipolar.cpp:90-98` (RANSAC; `"nls_termination"` = NLS).
- **Issue:** If more solvers nest summaries, consider qualifying both keys consistently, e.g. `ransac_termination` and `nls_termination`.

#### [note] `ransac.run(extra=...)` is an untyped keyword pass-through
- **Where:** `src/slamlab/foundations/ransac.py:443,473`.
- **Issue:** Acceptable today: the `Implementation` Protocol types the compiled signature and `test_backends.py` checks it. A third caller would warrant a typed argument object.

#### [note] A contract for test-only oracles
- **Where:** `pyproject.toml:37-39` (poselib), alongside mpmath, scipy and manifpy.
- **Issue:** `src/` imports none of them today, but only comments guard it. A `forbidden` contract would turn the comment into a check; this matters most for poselib, a competing implementation of the same algorithm.

#### [note] One concept, three parameter names
- **Where:** `fivepoint.essential_5pt(solver=)`, `epipolar.essential_ransac(minimal_solver=)`, `twoview.Options.essential_solver`.
- **Issue:** Each name is defensible at its layer, but the note has to explain the mapping. Fine as is if intended; otherwise align on `minimal_solver` above `fivepoint`.

#### Checked and fine
- **Layering:** `fivepoint` imports only `arrays`, `correspondences`, `errors` and `backends`; its import-linter slot is right; `epipolar → fivepoint, nls, ransac, so3` respects the layers; C++ includes follow the same order.
- **No dependency on reifyscene, orchestration or eval**; no GPU or heavy dependency enters the CPU rung (poselib is dev-only).
- **Language split:** C++ is lock-step with Python; Rust is untouched, consistent with PR 5.
- **Cores build without Python:** native `test_fivepoint.cpp`; `refine_essential` returns `Result`.
- **Bindings convert types only**, one file per module, submodule mirrored.
- **Backend dispatch and Protocols** in place and picked up by `tests/test_backends.py`.
- **`nls_crossing.hpp` / `nls.summary_from_backend`:** a good extraction.
- **`PairEstimator(minimal=)`** and `positive_z` are minimal, mirrored extensions.
- **Design doc** status and open items updated; design note in place.
- **No shims, globals or import-time side effects.**

### Algorithm review

**Scope:** `git diff 8699f1f...eddc19f` (feature/five-point, PR 3 of milestone 1). I read these in full: `src/slamlab/foundations/fivepoint.py`, `cpp/src/foundations/fivepoint.cpp`, the Sampson refinement and E-RANSAC changes in `epipolar.py` and `epipolar.cpp`, the `twoview` wiring, `nls.summary_from_backend`, the new tests and `docs/notes/five-point.md`. I ran read-only experiments in the `slamlab` env, with PoseLib 2.0.5 as the oracle.

**Summary:** Both solvers follow their papers (Nistér: monomial order, pairing rows e−z·f, degree-10 det B(z), Sturm sequence, (x, y, 1) as a cross product; Stewénius: GrevLex order, the rows and shifts of the action matrix for x). The Sampson refinement is a clean 5-DOF problem on the manifold; its analytic Jacobian matches central differences to 1e-10, and the MSAC-guarded keep is sound. The main weakness is near-degenerate samples (small baseline, forward motion, the regime of the initialiser's 1° parallax gate): there Nistér as shipped loses the true E about 6x as often as Stewénius, half of it from the fixed 3-step polish, which has no convergence test; the note's precision claims hold only for generic geometry, and no test covers this regime. One must-fix: a documented degenerate path has no test.

#### [must-fix] Documented rank-deficient path ("five pairs of rank below five give no solution") is untested
- **Where:** `src/slamlab/foundations/fivepoint.py:127` (`_null_space`), the module docstring, and `cpp/src/foundations/fivepoint.cpp` (`null_space`). Tests: `tests/foundations/test_fivepoint.py`, `cpp/tests/foundations/test_fivepoint.cpp`.
- **Issue:** Neither suite feeds a rank-deficient sample, such as a repeated correspondence, to `essential_5pt`.
- **Why it matters:** This is the degenerate case RANSAC hits on duplicate matches. It is public, documented behaviour in both backends with no test pinning it.
- **Evidence:** The behaviour is correct today: `b1[4]=b1[3]; b2[4]=b2[3]` on a `sc.pairs(sc.scene(5, seed=0), pose=se3.exp([0.1,0,0,0.3,0.1,0]))` sample gives shape `(0, 3, 3)` for nister and stewenius, on python and cpp. No such case in either test file.
- **Suggested fix:** One parametrised test (both solvers, both backends) asserting `(0, 3, 3)` on a repeated pair, and the same in the C++ native test.

#### [should-fix] Nistér loses the true E on near-degenerate samples: fixed 3-step polish with no convergence test, and det B(z) pushes roots off the real axis
- **Where:** `fivepoint.py:78` (`POLISH_STEPS = 3`) and `:291` (`_polish`); `cpp/src/foundations/fivepoint.cpp:386` (`polish`). Claims in `docs/notes/five-point.md` and the module docstring.
- **Issue:** On exact samples with forward motion and a 5% baseline (depths 2–8, rotation σ = 0.02 rad), the true E is missing from the solutions in 18/300 (Nistér), 3/300 (Stewénius), 35/300 (PoseLib); at a 1% baseline 122, 90 and 133. Each Nistér miss traces to one of: (1) the real root, shifted ~1e-2 by forming det B(z), is still 1e-3 to 1e-4 away after three Gauss-Newton steps; (2) rounding turned the true real root into a near-real complex pair (`-4.0346+0.0068j` against true `z = -4.0256`), which Sturm correctly reports as no real root. Polishing to convergence fixes (1): at a 1e-6 threshold, misses go 22 → 12 (6 steps) → 11 (10) → 11 (20).
- **Why it matters:** The default solver gives up recall where two-view initialisation operates (forward motion, small parallax). It still beats PoseLib and RANSAC tolerates occasional misses, so not a correctness bug, but the note presents the polish as having recovered the precision without saying it applies only to generic geometry.
- **Evidence:** `five_point_degeneracy_probe.py`. Setting `ESSENTIAL_TOLERANCE` to 1e-3 or infinity left the miss counts unchanged.
- **Suggested fix:** Iterate the polish to a convergence criterion (step below ~1e-14 relative, or residual stalls) with a cap of ~10, keeping the result only if the residual dropped; optionally polish near-real complex roots too and let `_keep` reject junk; add a named hard-case test with a stated recall; qualify the note's precision numbers and record the near-degenerate recall; consider whether Stewénius should be the default (owner's call).

#### [should-fix] Whether the refinement was applied, and whether it converged, is invisible to E-RANSAC's caller
- **Where:** `epipolar.py:769–803` (`polish_consensus` drops `.summary`); `epipolar.cpp:401` (`polish`). `twoview._initialize` inherits it.
- **Issue:** The `nls.Summary` is thrown away; the returned `ransac.Summary` cannot say whether the refined E was kept or reverted.
- **Why it matters:** Iterative solvers report a solve summary by contract. With `refine=True` a run where LM hit its cap, or was rejected, looks identical to a successful one.
- **Suggested fix:** A field on `ransac.Summary` (or a wrapping result): `refinement: nls.Summary | None` plus `refined_kept: bool`, in both backends.

#### [should-fix] `real_roots` is public but accepts NaN or inf and silently returns no roots
- **Where:** `fivepoint.py:232–249`; the C++ `real_roots` (not bound) is the same code.
- **Issue:** No finiteness or shape validation: `real_roots([nan, 1.0])` returns `[]`; `real_roots([1.0, inf, 1.0])` returns `[]` with a RuntimeWarning.
- **Why it matters:** "No real roots" and "garbage in" are indistinguishable.
- **Suggested fix:** Validate a 1-D finite array and raise `InvalidInputError`; add a test.

#### [note] LO's refits stay algebraic; current pipelines put the Sampson LM inside or after LO
- PoseLib's LO-RANSAC refines relative pose on Sampson error inside LO; SupeRANSAC (Barath, arXiv:2506.04803, 2025) finishes E with LM over 5-DOF pose on MAGSAC++-weighted Sampson error with chirality-based selection. An inlier-reselect → refine loop of 2–3 rounds is the natural next step if the floor ratio matters later. Backlog, not this PR.

#### [note] Stewénius could share the polish, and there are better-conditioned Gröbner variants
- The same Gauss-Newton polish on Stewénius' solutions would take its ~1e-9 (~1e-5 near degeneracy) to Nistér's level and tighten the parity tolerance (1e-7). Literature: Byröd, Josephson & Åström, IJCV 84(3), 2009; Hartley & Li, TPAMI 34(12), 2012.

#### [note] The refinement stops on function tolerance about 1e-4 from SciPy's optimum
- About 0.006°, negligible against 1 px noise, and the note explains it. A tighter function tolerance for this 5-parameter problem would be nearly free if the refined E is later used as a reference.

#### Checked and fine
- **Nistér's formulation**, **Stewénius' action matrix**, **Demazure's constraints** (identical in Python and C++).
- **Sturm sequence:** Cauchy bound, counts on (a, b], multiple-root fallback, bisection all terminate; no hang on near-zero leading coefficient, NaN or inf.
- **The `_keep` filter** does not cost recall (identical misses at tolerance infinity) and removes pure-rotation junk.
- **Generic-geometry accuracy and parity** on 200 random exact samples each.
- **Sampson residual and Jacobian:** H&Z 11.4.3, signed; analytic vs numeric gradient difference 1.5e-10 (scale 0.63), Gauss-Newton Hessian 7.9e-10 (scale 16.8).
- **Retraction:** 5 DOF, right perturbation, deterministic tangent basis.
- **Cauchy scale and MSAC comparison** consistent with `ransac._score`.
- **Pure rotation** with 5-point and refine terminates, and the backends agree for all three solvers.
- **Input validation and seeding.**

Sources: [SupeRANSAC (arXiv:2506.04803)](https://arxiv.org/abs/2506.04803).
