---
name: review-panel
description: Run slamlab's review panel on the current branch -- layer-1 checks, then the hygiene, architecture and algorithm reviewers in parallel -- save the report to docs/reviews/, and present the findings so the user decides each action. Use before opening a PR for a PATCH-or-larger change, or with a reviewer name (hygiene | architecture | algorithm) to run one reviewer alone.
argument-hint: "[hygiene|architecture|algorithm] [--base <ref>]"
---

# /review-panel

slamlab reviews in two layers:

- **Layer 1, every PR:** deterministic checks in `scripts/check.sh`, also run by CI.
- **Layer 2, this panel:** three reviewer agents, for **PATCH-or-larger** changes (new
  algorithms, modules, rungs, language implementations, interface or boundary changes). Or
  run one reviewer alone when a smaller change touches its area.

Findings are **decided together with the user**. The panel never fixes, waives or dismisses
a finding on its own.

## Arguments

- No reviewer named: run all three.
- `hygiene`, `architecture` or `algorithm`: run only that reviewer. Several may be named.
- `--base <ref>`: the ref to diff against. Default: `origin/develop`.

## Steps

### 1. Establish the scope

- `git fetch origin`, then set `BASE_SHA` to `git rev-parse <base ref>` (default
  `origin/develop`) and `HEAD_SHA` to `git rev-parse HEAD`. Record the branch name and both
  SHAs.
- Refuse to run on a dirty tree: `git status --porcelain` must print nothing, since
  uncommitted changes would not be in the reviewed diff. Ask the user to commit first.
- `git diff --stat $BASE_SHA...$HEAD_SHA` gives the reviewers their scope. If the diff is
  empty, stop and say so.

### 2. Layer 1

- Run `scripts/check.sh --env slamlab`, or plain `scripts/check.sh` inside the activated
  env. Record its summary.
- **If any check fails, stop.** Report the failures. The panel reviews only code that passes
  layer 1, since the reviewers rely on it.

### 3. Run the reviewers in parallel

Spawn the selected reviewers **in a single message**, so they run concurrently:

| Reviewer | Agent (`.claude/agents/`) | Model |
|---|---|---|
| Hygiene | `hygiene-reviewer` | sonnet |
| Architecture | `architecture-reviewer` | opus |
| Algorithm | `algorithm-reviewer` | opus |

Give each the same brief: the repository path, `BASE_SHA` and `HEAD_SHA`, the branch name,
the `--stat` summary, and the instruction to return its review in the format of
`.claude/skills/review-panel/finding-format.md`.

The project agents are registered when the session runs inside the slamlab repo. In a
session started elsewhere they are not; then spawn a `general-purpose` agent whose prompt
tells it to read and follow its agent file, with that file's model as the model override.
Record which way each reviewer ran.

### 4. Write the report

Write `docs/reviews/YYYY-MM-DD-<branch-name-with-slashes-as-dashes>.md` from
`report-template.md` in this folder:

- the run metadata: date, branch, SHAs, reviewers and models, layer-1 summary;
- a findings table, one row per finding, numbered `H1, H2, …` (hygiene), `A1, …`
  (architecture) and `L1, …` (algorithm). Resolution is `pending` for every finding;
- each reviewer's full review, verbatim, under its own heading.

If a report for the same branch exists from an earlier run, append a new dated `## Run N`
section rather than overwriting it. The history of runs is part of the record.

### 5. Present the findings and decide together

Present in the conversation, not as a file dump:

1. A one-paragraph overall assessment.
2. **must-fix** findings first, then **should-fix**, then **notes**, each with:
   - its ID, reviewer and location;
   - the issue in plain terms: what is wrong and why it matters, for someone who has not
     read the code;
   - **your recommended action**: `fix`, `defer` (with where it will be tracked), `waive`
     (with the reason it is acceptable), or `dispute` (with your evidence that the finding is
     wrong);
   - for a fix, a one-line description of the change.
3. Ask the user to decide. Use a selectable prompt per must-fix finding, or one grouped
   prompt for straightforward ones. Should-fix and notes may be decided in bulk.

Never apply a fix, record a waiver or dismiss a finding before the user has decided. No
response means wait.

### 6. Record decisions and apply them

- Update each finding's resolution in the report: `fixed (<commit>)`, `deferred (<where>)`,
  `waived: <reason>`, or `disputed: <reason>`, plus `decided by <user> on <date>`.
- Apply the fixes the user chose, run layer 1 again, and re-run any reviewer whose area the
  fixes changed substantially. Append that run to the report.
- Commit the report with the branch. It ships in the PR.

### 7. Hand off to the PR

The PR body's "How this was built" section (`.github/pull_request_template.md`) links the
report and summarises the panel: reviewers and models, finding counts by severity, and how
each must-fix was resolved.
