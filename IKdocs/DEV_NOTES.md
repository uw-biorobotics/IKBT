# IKBT Developer Notes

Rationale, measurements, and rejected alternatives that used to live in source comments.

The source files say **what the code does**. This file says **why it ended up that way** —
dated decisions, numbers someone measured once, and designs that were tried and abandoned.
Nothing here is needed to read or modify IKBT; it is here so the source can stay short.

For what to run and what each test asserts, see [TESTING.md](TESTING.md).
For release history, see [oldNews.md](oldNews.md).

---

## ikSolver.py

**`PERFORMANCE_OUTPUT`.** Measured across the robot set, 37–85% of a solve's wall clock is
inside `sp.simplify()`. The sympy meter prints a line per slow call, which is the only output
that can appear *during* a blocking simplify — `symbolic_loop`'s per-pass line cannot print
until the pass returns. Left off by default because in an ordinary run it buries the per-pass
progress lines it sits between.

**Per-robot debug blocks.** ikSolver.py used to end with roughly 200 lines of commented-out
per-robot debug setup. `build_default_bt()` now returns a `nodes` dict, so any node's `BHdebug`
can be set by name from a caller; the commented blocks were deleted.

**Solution-set finding is at version 3.** Version 2 was tree-based and went through
`R.notation_collections` / `matching.matching_func()`. That path is gone;
`create_solution_set()` replaced it, called exactly once by the `output_gen_full` leaf.

---

## ikbtfunctions/ik_driver.py

**Why the pipeline is a module of functions.** It used to run at module level inside
ikSolver.py, so importing any part of it triggered a full solve and a second front end
would have had to copy all of it.

**Why equation width is measured, not predicted** (`write_latex_fitted`). Two cheap proxies
were tried and both failed on real data:

- *Character count.* A 439-character equation overflows the page while a 2030-character one fits.
- *Operation count.* Brad's `th_3` is 25 operations with 12 in each `atan2` argument — under every
  threshold tried — and its typeset line is 144 pt too wide.

Width is a property of the typeset line, not of the expression, so `pdflatex` is asked. `Craig417`
needed three shortening passes: shortening one equation exposes another.

`slack_pt = 6.0` exists because Stanford has one box 3.7 pt over, about one character wide.
Restructuring a readable equation to win that back trades it for a folded one.

**Reporting must never cost the report.** The width loop degrades to pass 1 on any failure —
missing `pdflatex`, LaTeX error, timeout — because the pass 1 report is already written and
already correct, merely wide.
