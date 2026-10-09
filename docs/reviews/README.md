# Review panel reports

One report per reviewed branch, written by `/slam-it-all-review-panel`
(`.claude/skills/slam-it-all-review-panel/`) and committed with the branch's PR. Each report records
the run (branch, commit range, reviewers and models, layer-1 result), every finding from the
hygiene, architecture and algorithm reviewers, and **how each finding was resolved and who
decided**. Later runs on the same branch append a new section.

Together they are the history of how the project's code was reviewed and why each decision went
the way it did.
