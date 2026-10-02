# Reviewer output format

Every reviewer on the slamlab panel returns its review in exactly this shape, so the panel
can merge three reviews into one report.

## Severities

| Severity | Meaning |
|---|---|
| **must-fix** | Wrong, unsafe, or not production quality: a bug, a numerical fault, a broken architectural rule, missing input validation or error handling, an undocumented or untested public item. It cannot ship as is. |
| **should-fix** | Works but is weaker than it should be: unclear naming, a missed simplification, a test gap on a non-critical path, a design choice with a better standard alternative. |
| **note** | Worth knowing but not a defect: an observation, a future concern, a pointer to related work. |

**Not production quality is must-fix by default.** Code explicitly marked as a placeholder
(in its docstring or comment, e.g. the scaffold's `hat`) is held to correctness and
documentation, but its limited scope is a note, not a must-fix.

## Shape

```markdown
## <Reviewer name> review

**Scope:** <what you reviewed: files, commit range>
**Summary:** <two or three sentences: overall assessment>

### Findings

#### [must-fix] <short title>
- **Where:** `path/to/file.py:42` (and other locations, if the same issue recurs)
- **Issue:** <what is wrong, concretely>
- **Why it matters:** <the consequence: which input breaks it, what it costs>
- **Evidence:** <the code excerpt, a failing input, a command and its output, or a citation>
- **Suggested fix:** <what to change; brief>

#### [should-fix] <short title>
...

#### [note] <short title>
...

### Checked and fine
- <one line per area you examined and found sound, so the report shows coverage>
```

## Rules

- **Read-only.** Never edit, create, stage or commit files. You may run read-only commands
  (`git diff`, `git log`, tests, the layer-1 tools) to gather evidence.
- **Evidence over opinion.** Every must-fix carries evidence a reader can check: a quoted
  line, a reproducing input, tool output, or a citation.
- **Be specific about location.** `path:line`, never "somewhere in the solver".
- **No padding.** If an area is sound, say so in one line under "Checked and fine". Do not
  invent findings to fill space; an empty findings list is a valid review.
- **Stay in your lane.** Report what your role covers. Mention out-of-lane issues only if
  they are serious, as a `note` naming the reviewer who owns it.
