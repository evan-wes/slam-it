---
name: slam-it-review-panel
description: Run the review panel on the current branch of the SLAM It! repository (from any session, normally reifyscene) -- layer-1 checks, then the hygiene, architecture and algorithm reviewers in parallel -- save the report to docs/reviews/, and present the findings so the user decides each action. Use once per PR, on the finished slice, before opening it -- never after every incremental change -- or with a reviewer name (hygiene | architecture | algorithm) to run one reviewer alone on a small PR.
argument-hint: "[hygiene|architecture|algorithm] [--base <ref>]"
---

# /slam-it-review-panel

SLAM It! reviews in two layers:

- **Layer 1, every PR:** deterministic checks in `scripts/check.sh`, also run by CI
  (`scripts/offline_ci.sh`, from a clean checkout).
- **Layer 2, this panel:** three reviewer agents, **once per PR, on the finished slice**,
  against the PR's stated goal. A PR is a slice of a milestone built as semantic commits;
  the panel never runs per commit. Or run one reviewer alone on a small PR that touches its
  area. Findings outside the PR's goal go to a backlog by default.

Findings are **decided together with the user**. The panel never fixes, waives or dismisses
a finding on its own.

## Where it runs

Sessions run from the reifyscene repository, so the panel always works on the SLAM It!
repository explicitly: `REPO`, the SLAM It! checkout. Run git as
`git -C "$REPO" ...` and tools from `$REPO`, and give every reviewer `REPO` in its brief.
Read `$REPO/CLAUDE.md` before starting. The agents and this skill are symlinked into
reifyscene's `.claude/` (git-ignored there), so they are registered in reifyscene sessions
while staying single-sourced here.

## Arguments

- No reviewer named: run all three.
- `hygiene`, `architecture` or `algorithm`: run only that reviewer. Several may be named.
- `--base <ref>`: the ref to diff against. Default: `origin/develop`.

## Steps

### 1. Establish the scope

- `git -C "$REPO" fetch origin`, then set `BASE_SHA` to `git rev-parse <base ref>` (default
  `origin/develop`) and `HEAD_SHA` to `git rev-parse HEAD`. Record the branch name and both
  SHAs.
- Refuse to run on a dirty tree: `git status --porcelain` must print nothing, since
  uncommitted changes would not be in the reviewed diff. Ask the user to commit first.
- `git diff --stat $BASE_SHA...$HEAD_SHA` gives the reviewers their scope. If the diff is
  empty, stop and say so.

### 2. Layer 1

- Run `scripts/check.sh --env slam-it`, or plain `scripts/check.sh` inside the activated
  env. Record its summary.
- **If any check fails, stop.** Report the failures. The panel reviews only code that passes
  layer 1, since the reviewers rely on it.

### 3. Run the reviewers in parallel

Spawn the selected reviewers **in a single message**, so they run concurrently:

| Reviewer | Agent (`.claude/agents/`) | Model |
|---|---|---|
| Hygiene | `slam-it-hygiene-reviewer` | sonnet |
| Architecture | `slam-it-architecture-reviewer` | opus |
| Algorithm | `slam-it-algorithm-reviewer` | opus |

Give each the same brief: `REPO` (the SLAM It! repository root), `BASE_SHA` and `HEAD_SHA`, the branch name,
the `--stat` summary, and the instruction to return its review in the format of
`.claude/skills/slam-it-review-panel/finding-format.md`, citing every path relative to the
repository root, never as an absolute or home-relative path.

If an agent type is not registered (the symlinks in reifyscene's `.claude/` are missing, or
the session started somewhere else), spawn a `general-purpose` agent whose prompt tells it to
read and follow `$REPO/.claude/agents/<agent>.md`, with that file's model as the model
override. Record which way each reviewer ran.

### 4. Write the report

Write `docs/reviews/YYYY-MM-DD-<branch-name-with-slashes-as-dashes>.md` from
`report-template.md` in this folder:

- the run metadata: date, branch, SHAs, reviewers and models, layer-1 summary;
- a findings table, one row per finding, numbered `H1, H2, …` (hygiene), `A1, …`
  (architecture) and `L1, …` (algorithm). Resolution is `pending` for every finding;
- each reviewer's full review, verbatim, under its own heading.

Every path in the report is relative to the repository root, never absolute or home-relative:
the report is committed with the PR.

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

The PR body's "How this was built" section (`$REPO/.github/pull_request_template.md`) links the
report and summarises the panel: reviewers and models, finding counts by severity, and how
each must-fix was resolved. Before asking for the merge, run `scripts/offline_ci.sh` on the
final branch head and record its result line in the PR's Verification section.
