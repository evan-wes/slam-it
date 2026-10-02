## Summary

<!-- What this PR does and why, in a few bullets. -->

-

## How this was built

<!--
slamlab is built through a human-directed agentic workflow: Claude Code agents write and
review the code; the owner makes the design decisions, decides every review finding, and
approves every merge. This section is the record of that process for this PR.
-->

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | <model> | <what it built> |
| Hygiene review | `hygiene-reviewer` | sonnet | <ran / not run: why> |
| Architecture review | `architecture-reviewer` | opus | <ran / not run: why> |
| Algorithm review | `algorithm-reviewer` | opus | <ran / not run: why> |

**Review panel:** <full panel / <reviewer> only / layer 1 only (MICRO change)>.
Report: [`docs/reviews/<file>.md`](../docs/reviews/<file>.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | | | | | |
| should-fix | | | | | |
| note | | | | | |

**Owner decisions**

<!-- Design choices the owner made, each finding the owner waived or deferred and why, and
     any approval given through a selectable prompt. -->

-

## Verification

<!-- What was run and what it showed. -->

- [ ] Layer 1: `scripts/check.sh` passes locally
- [ ] CI passes
-

## Version

`<old>` -> `<new>` (<MAJOR | MINOR | PATCH | MICRO>), CHANGELOG updated.
